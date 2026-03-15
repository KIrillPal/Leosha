from __future__ import annotations

from dataclasses import dataclass
from io import BytesIO

from ..interfaces import AlgorithmContext, InputState, OperationProfile
from ..models import ControlCommand, ControlMode, ManualInputState, TelemetryFrame
from ..services.friend_db import FriendDB
from ..services.vision_service import VisionService


class BaseControlProfile(OperationProfile):
    """Reusable base profile for all behavior modes."""

    @property
    def algorithm(self):
        # Backwards compatibility with legacy tests.
        return self

    def compute_command(
        self,
        context: AlgorithmContext,
        manual: ManualInputState,
        telemetry: TelemetryFrame,
        robot_config: dict | None = None,
    ) -> ControlCommand:
        state = InputState(
            manual=manual,
            telemetry=telemetry,
            robot_config=robot_config,
        )
        command = self.tick(context, state)
        self.post_tick(state)
        return command


class PauseProfile(BaseControlProfile):
    @property
    def mode(self) -> ControlMode:
        return ControlMode.PAUSE

    @property
    def title(self) -> str:
        return "Пауза"

    def tick(self, context: AlgorithmContext, input_state: InputState) -> ControlCommand:
        del context, input_state
        return ControlCommand(mode=self.mode)

    def get_ui_state(self) -> dict:
        return {"x": 0.0, "y": 0.0}


class TeleoperationProfile(BaseControlProfile):
    """Teleoperation profile: consumes ManualInputState and produces driving command."""

    def __init__(
        self,
        *,
        forward_throttle: float,
        backward_throttle: float,
        forward_fast_throttle: float,
        title: str,
        control_mode: ControlMode,
    ) -> None:
        self._forward_throttle = float(forward_throttle)
        self._backward_throttle = float(backward_throttle)
        self._forward_fast_throttle = float(forward_fast_throttle)
        self._title = title
        self._control_mode = control_mode
        self._head_pan = 0.0
        self._head_tilt = 0.0

    @property
    def mode(self) -> ControlMode:
        return self._control_mode

    @property
    def title(self) -> str:
        return self._title

    def tick(self, context: AlgorithmContext, input_state: InputState) -> ControlCommand:
        manual = input_state.manual
        telemetry = input_state.telemetry
        robot_config = input_state.robot_config

        if not manual.tracking_enabled:
            return ControlCommand(mode=self.mode, head_pan=self._head_pan, head_tilt=self._head_tilt)

        speed = 0.0
        steering = 0.0
        if manual.w and not manual.s:
            speed = self._forward_fast_throttle if manual.shift else self._forward_throttle
        elif manual.s and not manual.w:
            speed = self._backward_throttle
        # Нормализованное руление ±1.0; клиент мапит в диапазон по actuators.wheel (min/max/zero)
        if manual.a and not manual.d:
            steering = -1.0
        elif manual.d and not manual.a:
            steering = 1.0

        steering *= max(0.55, 1.0 - abs(telemetry.imu_yaw_rate) * 0.2)
        self._head_pan = max(-1.0, min(1.0, self._head_pan + manual.head_dx * context.head_sensitivity))
        self._head_tilt = max(-1.0, min(1.0, self._head_tilt + manual.head_dy * context.head_sensitivity))

        return ControlCommand(
            speed=speed,
            steering=steering,
            head_pan=self._head_pan,
            head_tilt=self._head_tilt,
            mode=self.mode,
        )

    def reset_runtime_state(self, manual: ManualInputState) -> None:
        manual.head_dx = 0.0
        manual.head_dy = 0.0
        self._head_pan = 0.0
        self._head_tilt = 0.0

    def post_tick(self, input_state: InputState) -> None:
        # Consume one-shot mouse deltas after applying them in current tick.
        input_state.manual.head_dx = 0.0
        input_state.manual.head_dy = 0.0

    def get_ui_state(self) -> dict:
        return {
            "x": float(self._head_pan),
            "y": float(self._head_tilt),
            "tracking_enabled": False,  # will be set by controller from ManualInputState
        }

    def save_state(self) -> dict:
        return {"head_pan": self._head_pan, "head_tilt": self._head_tilt}

    def restore_state(self, state: dict) -> None:
        self._head_pan = float(state["head_pan"])
        self._head_tilt = float(state["head_tilt"])


class TeleopSlamProfile(TeleoperationProfile):
    """Same teleop behavior as TeleoperationProfile, but marks SLAM as required."""

    def __init__(
        self,
        *,
        forward_throttle: float,
        backward_throttle: float,
        forward_fast_throttle: float,
    ) -> None:
        super().__init__(
            forward_throttle=forward_throttle,
            backward_throttle=backward_throttle,
            forward_fast_throttle=forward_fast_throttle,
            title="Телеуправление + SLAM",
            control_mode=ControlMode.TELEOP_SLAM,
        )

    @property
    def requires_slam(self) -> bool:
        return True


class StaringProfile(BaseControlProfile):
    def __init__(
        self,
        *,
        forward_throttle: float,
        backward_throttle: float,
        forward_fast_throttle: float,
        friend_embeddings_db: str,
        face_match_threshold: float,
        head_tracking_gain: float,
        head_tracking_deadzone: float,
        vision_service: VisionService,
    ) -> None:
        self._forward_throttle = float(forward_throttle)
        self._backward_throttle = float(backward_throttle)
        self._forward_fast_throttle = float(forward_fast_throttle)
        self._friend_db = FriendDB(friend_embeddings_db)
        self._face_match_threshold = float(face_match_threshold)
        self._head_tracking_gain = float(head_tracking_gain)
        self._head_tracking_deadzone = float(head_tracking_deadzone)
        self._vision = vision_service

        self._head_pan = 0.0
        self._head_tilt = 0.0
        self._staring_state = "manual"
        self._staring_target: str | None = None
        self._staring_track_id: int | None = None
        self._staring_debug: dict = {}
        self._staring_overlay: list[dict] = []

    @property
    def mode(self) -> ControlMode:
        return ControlMode.STARING

    @property
    def title(self) -> str:
        return "Слежение за знакомым лицом"

    def _drive_command(self, manual: ManualInputState, telemetry: TelemetryFrame) -> tuple[float, float]:
        speed = 0.0
        steering = 0.0
        if manual.w and not manual.s:
            speed = self._forward_fast_throttle if manual.shift else self._forward_throttle
        elif manual.s and not manual.w:
            speed = self._backward_throttle
        if manual.a and not manual.d:
            steering = -1.0
        elif manual.d and not manual.a:
            steering = 1.0
        steering *= max(0.55, 1.0 - abs(telemetry.imu_yaw_rate) * 0.2)
        return speed, steering

    @staticmethod
    def _bbox_area(bbox: list[float]) -> float:
        x1, y1, x2, y2 = [float(v) for v in bbox]
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)

    @staticmethod
    def _frame_size(camera_frame: bytes) -> tuple[int, int]:
        if not camera_frame:
            return 640, 480
        from PIL import Image

        img = Image.open(BytesIO(camera_frame))
        return int(img.width), int(img.height)

    def _track_head_by_bbox(self, bbox: list[float], frame_w: int, frame_h: int, dt: float) -> None:
        x1, y1, x2, y2 = [float(v) for v in bbox]
        cx = (x1 + x2) * 0.5
        cy = (y1 + y2) * 0.5
        err_x = ((cx / max(1.0, float(frame_w))) - 0.5) * 2.0
        err_y = ((cy / max(1.0, float(frame_h))) - 0.5) * 2.0
        if abs(err_x) < self._head_tracking_deadzone:
            err_x = 0.0
        if abs(err_y) < self._head_tracking_deadzone:
            err_y = 0.0
        alpha = self._head_tracking_gain * max(0.0, dt)
        self._head_pan = max(-1.0, min(1.0, self._head_pan - err_x * alpha))
        self._head_tilt = max(-1.0, min(1.0, self._head_tilt + err_y * alpha))

    def _manual_head(self, context: AlgorithmContext, manual: ManualInputState) -> None:
        self._head_pan = max(-1.0, min(1.0, self._head_pan + manual.head_dx * context.head_sensitivity))
        self._head_tilt = max(-1.0, min(1.0, self._head_tilt + manual.head_dy * context.head_sensitivity))

    def _best_visible_face(self, persons: list[dict]) -> dict | None:
        visible = []
        for person in persons:
            embedding = person["face_embedding"]
            if person["face_visible"] and isinstance(embedding, list) and embedding:
                visible.append(person)
        if not visible:
            return None
        return max(visible, key=lambda p: self._bbox_area(p["bbox"]))

    def _next_auto_name(self) -> str:
        names = set(self._friend_db.list_names())
        idx = 1
        while f"face_{idx}" in names:
            idx += 1
        return f"face_{idx}"

    def on_action(self, action: dict) -> None:
        action_type = str(action["type"])
        if action_type == "add_face":
            persons = self._vision.get_persons()
            candidate = self._best_visible_face(persons)
            if candidate is None:
                self._staring_debug["last_action"] = "add_face_skipped_no_visible_face"
                return
            name = str(action["name"]) if "name" in action else self._next_auto_name()
            self._friend_db.add(name, [float(v) for v in candidate["face_embedding"]])
            self._staring_debug["last_action"] = f"add_face:{name}"
            return
        if action_type == "remove_face":
            name = str(action["name"])
            self._friend_db.remove(name)
            self._staring_debug["last_action"] = f"remove_face:{name}"
            return
        if action_type == "list_faces":
            self._staring_debug["last_action"] = "list_faces"
            return

    def tick(self, context: AlgorithmContext, input_state: InputState) -> ControlCommand:
        manual = input_state.manual
        telemetry = input_state.telemetry

        if not manual.tracking_enabled:
            self._staring_state = "manual"
            self._staring_target = None
            self._staring_track_id = None
            self._staring_overlay = []
            return ControlCommand(mode=self.mode, head_pan=self._head_pan, head_tilt=self._head_tilt)

        speed, steering = self._drive_command(manual, telemetry)
        persons = self._vision.get_persons()

        matches: list[tuple[dict, str]] = []
        match_by_track_id: dict[int, str] = {}
        for person in persons:
            embedding = person["face_embedding"]
            if isinstance(embedding, list) and embedding:
                name = self._friend_db.match([float(v) for v in embedding], self._face_match_threshold)
                if name is not None:
                    matches.append((person, name))
                    match_by_track_id[int(person["track_id"])] = name

        overlay: list[dict] = []
        for person in persons:
            bbox = [float(v) for v in person["bbox"]]
            x1, y1, x2, y2 = bbox
            overlay.append(
                {
                    "track_id": int(person["track_id"]),
                    "bbox": bbox,
                    "keypoints": [[float(v) for v in kp] for kp in person["keypoints"]],
                    "head_yaw_deg": float(person["head_yaw_deg"]),
                    "face_visible": bool(person["face_visible"]),
                    "confidence": float(person["confidence"]),
                    "head_center": [0.5 * (x1 + x2), y1],
                    "matched_name": match_by_track_id[int(person["track_id"])] if int(person["track_id"]) in match_by_track_id else None,
                }
            )
        self._staring_overlay = overlay

        if matches:
            target_person, target_name = max(matches, key=lambda item: self._bbox_area(item[0]["bbox"]))
            frame_w, frame_h = self._frame_size(input_state.camera_frame)
            self._track_head_by_bbox(target_person["bbox"], frame_w, frame_h, input_state.dt)
            self._staring_state = "staring"
            self._staring_target = target_name
            self._staring_track_id = int(target_person["track_id"])
            self._staring_debug = {
                "matched_tracks": [int(item[0]["track_id"]) for item in matches],
                "target_track_id": int(target_person["track_id"]),
                "target_bbox": list(target_person["bbox"]),
                "vision_frame_id": self._vision.get_latest_frame_id(),
            }
        elif persons:
            target_person = persons[0]
            frame_w, frame_h = self._frame_size(input_state.camera_frame)
            self._track_head_by_bbox(target_person["bbox"], frame_w, frame_h, input_state.dt)
            self._staring_state = "staring_first_face_fallback"
            self._staring_target = None
            self._staring_track_id = int(target_person["track_id"])
            self._staring_debug = {
                "matched_tracks": [],
                "target_track_id": int(target_person["track_id"]),
                "target_bbox": list(target_person["bbox"]),
                "vision_frame_id": self._vision.get_latest_frame_id(),
            }
        else:
            self._manual_head(context, manual)
            self._staring_state = "manual"
            self._staring_target = None
            self._staring_track_id = None
            self._staring_debug = {
                "matched_tracks": [],
                "vision_frame_id": self._vision.get_latest_frame_id(),
            }

        return ControlCommand(
            speed=speed,
            steering=steering,
            head_pan=self._head_pan,
            head_tilt=self._head_tilt,
            mode=self.mode,
        )

    def reset_runtime_state(self, manual: ManualInputState) -> None:
        manual.head_dx = 0.0
        manual.head_dy = 0.0
        self._head_pan = 0.0
        self._head_tilt = 0.0
        self._staring_state = "manual"
        self._staring_target = None
        self._staring_track_id = None
        self._staring_overlay = []

    def post_tick(self, input_state: InputState) -> None:
        input_state.manual.head_dx = 0.0
        input_state.manual.head_dy = 0.0

    def get_ui_state(self) -> dict:
        return {
            "x": float(self._head_pan),
            "y": float(self._head_tilt),
            "staring_state": self._staring_state,
            "staring_target": self._staring_target,
            "staring_track_id": self._staring_track_id,
            "known_faces": self._friend_db.list_names(),
            "persons_count": len(self._vision.get_persons()),
            "staring_debug": dict(self._staring_debug),
            "staring_overlay": list(self._staring_overlay),
        }

    def save_state(self) -> dict:
        return {"head_pan": self._head_pan, "head_tilt": self._head_tilt}

    def restore_state(self, state: dict) -> None:
        self._head_pan = float(state["head_pan"])
        self._head_tilt = float(state["head_tilt"])


class AutonomyProfile1(BaseControlProfile):
    def __init__(self, **_: dict) -> None:
        self._target: dict = {}

    @property
    def mode(self) -> ControlMode:
        return ControlMode.AUTONOMY_PROFILE_1

    @property
    def title(self) -> str:
        return "Самоуправление: профиль 1 (заглушка)"

    def set_target(self, target: dict) -> None:
        self._target = dict(target)

    def on_action(self, action: dict) -> None:
        if action["type"] == "set_target" and isinstance(action["target"], dict):
            self.set_target(action["target"])

    def tick(self, context: AlgorithmContext, input_state: InputState) -> ControlCommand:
        del context, input_state
        return ControlCommand(mode=self.mode)

    def get_ui_state(self) -> dict:
        return {"x": 0.0, "y": 0.0}


@dataclass
class FollowingProfile(BaseControlProfile):
    """Behavior skeleton: follow known person with camera/lidar fusion.

    This profile is intentionally a non-functional skeleton. It documents how
    advanced behavior should be architected in this project.
    """

    friend_embeddings_db: str
    approach_distance_m: float
    max_head_tilt_deg: float
    average_human_height_m: float
    yolo_model: str

    def __post_init__(self) -> None:
        self._state = "idle"  # idle/searching/tracking/approaching/reached/lost
        self._target_id: str | None = None
        self._last_target_pose: tuple[float, float] | None = None
        self._debug: dict = {}

    @property
    def mode(self) -> ControlMode:
        # Reuse existing autonomy slot until dedicated enum is introduced.
        return ControlMode.AUTONOMY_PROFILE_1

    @property
    def title(self) -> str:
        return "Следование за человеком (скелет)"

    def on_activate(self, context: AlgorithmContext) -> None:
        del context
        self._state = "searching"

    def on_deactivate(self, context: AlgorithmContext) -> None:
        del context
        self._state = "idle"
        self._target_id = None
        self._last_target_pose = None

    def on_action(self, action: dict) -> None:
        action_type = str(action["type"])
        if action_type == "cancel_follow":
            self._target_id = None
            self._state = "searching"
        elif action_type == "set_target_id":
            target = action["target_id"]
            self._target_id = str(target) if target is not None else None
            self._state = "tracking" if self._target_id else "searching"

    def tick(self, context: AlgorithmContext, input_state: InputState) -> ControlCommand:
        del context
        # TODO(architecture): run detector (YOLO pose / face) on input_state.camera_frame.
        # TODO(architecture): compare face embedding against friend DB.
        # TODO(architecture): estimate person 2D point from lidar + average height.
        # TODO(architecture): drive robot toward target while centering face in camera.
        # TODO(architecture): stop when distance <= approach_distance_m
        #                    OR head_tilt reaches max_head_tilt_deg.
        # Expected dependencies:
        #   - vision service (detector + face embedder),
        #   - target tracker state machine,
        #   - planner/controller for (x,y) target in robot frame,
        #   - persistence via save_state/restore_state.
        # InputState may contain unrelated fields; profile ignores what it doesn't need.
        if not input_state.camera_frame:
            self._state = "searching"
        self._debug = {
            "manual_tracking_enabled": bool(input_state.manual.tracking_enabled),
            "slam_pose_available": input_state.slam_pose is not None,
            "lidar_available": input_state.lidar_scan is not None,
        }
        return ControlCommand(mode=self.mode)

    def get_ui_state(self) -> dict:
        return {
            "x": 0.0,
            "y": 0.0,
            "follow_state": self._state,
            "target_id": self._target_id,
            "last_target_pose": self._last_target_pose,
            "follow_debug": dict(self._debug),
        }

    def save_state(self) -> dict:
        return {
            "state": self._state,
            "target_id": self._target_id,
            "last_target_pose": self._last_target_pose,
        }

    def restore_state(self, state: dict) -> None:
        self._state = str(state["state"])
        target_id = state["target_id"]
        self._target_id = str(target_id) if target_id else None
        pose = state["last_target_pose"]
        if (
            isinstance(pose, (list, tuple))
            and len(pose) == 2
        ):
            self._last_target_pose = (float(pose[0]), float(pose[1]))
