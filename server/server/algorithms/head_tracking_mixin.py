from __future__ import annotations

import math
from io import BytesIO

from ..interfaces import AlgorithmContext, InputState
from ..models import ManualInputState
from ..services.friend_db import FriendDB
from ..services.vision_service import VisionService


class HeadTrackingMixin:
    """Reusable face-based head tracking for behavior profiles."""

    def _init_head_tracking(
        self,
        *,
        friend_embeddings_db: str,
        face_match_threshold: float,
        eye_confidence_threshold: float,
        head_tracking_gain: float,
        head_tracking_deadzone: float,
        bbox_target_x_frac: float,
        bbox_target_y_frac: float,
        latency_compensation_cap_frac: float,
        gyro_compensation_gain: float,
        gyro_compensation_neck_max_deg_fallback: float,
        stable_track_frames: int = 10,
        vision_service: VisionService,
    ) -> None:
        self._friend_db = FriendDB(friend_embeddings_db)
        self._face_match_threshold = float(face_match_threshold)
        self._eye_confidence_threshold = float(eye_confidence_threshold)
        self._head_tracking_gain = float(head_tracking_gain)
        self._head_tracking_deadzone = float(head_tracking_deadzone)
        self._bbox_target_x_frac = float(bbox_target_x_frac)
        self._bbox_target_y_frac = float(bbox_target_y_frac)
        self._latency_compensation_cap_frac = float(latency_compensation_cap_frac)
        self._gyro_compensation_gain = float(gyro_compensation_gain)
        self._gyro_compensation_neck_max_deg_fallback = float(gyro_compensation_neck_max_deg_fallback)
        self._stable_track_frames = max(1, int(stable_track_frames))
        self._vision = vision_service

        self._head_pan = 0.0
        self._head_tilt = 0.0
        self._tracking_state = "manual"
        self._tracking_target: str | None = None
        self._tracking_track_id: int | None = None
        self._tracking_debug: dict = {}
        self._tracking_overlay: list[dict] = []
        self._last_target_cx: float | None = None
        self._last_target_cy: float | None = None
        self._last_target_time: float | None = None
        self._track_presence_history: list[set[int]] = []
        self._track_presence_last_frame_id: int | None = None

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

    def _get_target_point(self, person: dict) -> tuple[float, float]:
        kps = person.get("keypoints") or []
        bbox = person["bbox"]
        x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
        if len(kps) >= 3:
            left_eye = kps[1] if len(kps) > 1 else []
            right_eye = kps[2] if len(kps) > 2 else []
            if (
                len(left_eye) >= 3
                and len(right_eye) >= 3
                and float(left_eye[2]) >= self._eye_confidence_threshold
                and float(right_eye[2]) >= self._eye_confidence_threshold
            ):
                cx = (float(left_eye[0]) + float(right_eye[0])) * 0.5
                cy = (float(left_eye[1]) + float(right_eye[1])) * 0.5
                return cx, cy
        cx = x1 + self._bbox_target_x_frac * (x2 - x1)
        cy = y1 + self._bbox_target_y_frac * (y2 - y1)
        return cx, cy

    def _extrapolate_target(
        self,
        cx: float,
        cy: float,
        frame_w: int,
        frame_h: int,
        dt: float,
        latency_ms: float,
        now: float,
    ) -> tuple[float, float]:
        prev_cx = self._last_target_cx
        prev_cy = self._last_target_cy
        self._last_target_cx = cx
        self._last_target_cy = cy
        self._last_target_time = now
        if latency_ms <= 0 or dt <= 0 or prev_cx is None or prev_cy is None:
            return cx, cy
        vx = (cx - prev_cx) / dt
        vy = (cy - prev_cy) / dt
        latency_sec = latency_ms / 1000.0
        dx = vx * latency_sec
        dy = vy * latency_sec
        max_dx = self._latency_compensation_cap_frac * max(1.0, float(frame_w))
        max_dy = self._latency_compensation_cap_frac * max(1.0, float(frame_h))
        dx = max(-max_dx, min(max_dx, dx))
        dy = max(-max_dy, min(max_dy, dy))
        cx_pred = max(0.0, min(float(frame_w), cx + dx))
        cy_pred = max(0.0, min(float(frame_h), cy + dy))
        return cx_pred, cy_pred

    def _track_head_by_point(self, cx: float, cy: float, frame_w: int, frame_h: int, dt: float) -> None:
        err_x = ((cx / max(1.0, float(frame_w))) - 0.5) * 2.0
        err_y = ((cy / max(1.0, float(frame_h))) - 0.5) * 2.0
        if abs(err_x) < self._head_tracking_deadzone:
            err_x = 0.0
        if abs(err_y) < self._head_tracking_deadzone:
            err_y = 0.0
        alpha = self._head_tracking_gain * max(0.0, dt)
        self._head_pan = max(-1.0, min(1.0, self._head_pan - err_x * alpha))
        self._head_tilt = max(-1.0, min(1.0, self._head_tilt - err_y * alpha))

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

    def handle_tracking_action(self, action: dict) -> bool:
        action_type = str(action["type"])
        if action_type == "add_face":
            persons = self._vision.get_persons()
            candidate = self._best_visible_face(persons)
            if candidate is None:
                self._tracking_debug["last_action"] = "add_face_skipped_no_visible_face"
                return True
            name = str(action["name"]) if "name" in action else self._next_auto_name()
            self._friend_db.add(name, [float(v) for v in candidate["face_embedding"]])
            self._tracking_debug["last_action"] = f"add_face:{name}"
            return True
        if action_type == "remove_face":
            name = str(action["name"])
            self._friend_db.remove(name)
            self._tracking_debug["last_action"] = f"remove_face:{name}"
            return True
        if action_type == "list_faces":
            self._tracking_debug["last_action"] = "list_faces"
            return True
        return False

    def _compensate_head_for_body_yaw(self, input_state: InputState) -> None:
        dt = input_state.dt
        if dt <= 0.0:
            return
        robot_cfg = input_state.robot_config
        if (
            robot_cfg is not None
            and "robot_geometry" in robot_cfg
            and "head" in robot_cfg["robot_geometry"]
        ):
            neck_max_deg = float(robot_cfg["robot_geometry"]["head"]["neck_max_deg"])
        else:
            neck_max_deg = self._gyro_compensation_neck_max_deg_fallback
        neck_range_rad = math.radians(max(1.0, neck_max_deg))
        delta_pan = -input_state.telemetry.imu_yaw_rate * dt * self._gyro_compensation_gain / neck_range_rad
        self._head_pan = max(-1.0, min(1.0, self._head_pan + delta_pan))

    def _update_track_presence(self, persons: list[dict], vision_frame_id: int) -> None:
        if self._track_presence_last_frame_id == vision_frame_id:
            return
        present = {int(person["track_id"]) for person in persons}
        self._track_presence_history.append(present)
        if len(self._track_presence_history) > self._stable_track_frames:
            self._track_presence_history = self._track_presence_history[-self._stable_track_frames :]
        self._track_presence_last_frame_id = vision_frame_id

    def _track_is_stable_candidate(self, track_id: int) -> bool:
        if self._tracking_track_id is not None and int(track_id) == int(self._tracking_track_id):
            return True
        if len(self._track_presence_history) < self._stable_track_frames:
            return False
        return all(int(track_id) in frame_tracks for frame_tracks in self._track_presence_history[-self._stable_track_frames :])

    def update_tracking(self, context: AlgorithmContext, input_state: InputState, *, allow_first_face_fallback: bool) -> bool:
        persons = self._vision.get_persons()
        vision_frame_id = int(self._vision.get_latest_frame_id())
        self._update_track_presence(persons, vision_frame_id)

        overlay: list[dict] = []
        matches: list[tuple[dict, str]] = []
        match_by_track_id: dict[int, str] = {}
        for person in persons:
            track_id = int(person["track_id"])
            is_stable = self._track_is_stable_candidate(track_id)
            bbox = [float(v) for v in person["bbox"]]
            x1, y1, x2, y2 = bbox
            embedding = person["face_embedding"]
            if is_stable and isinstance(embedding, list) and embedding:
                name = self._friend_db.match([float(v) for v in embedding], self._face_match_threshold)
                if name is not None:
                    matches.append((person, name))
                    match_by_track_id[track_id] = name
            overlay.append(
                {
                    "track_id": track_id,
                    "bbox": bbox,
                    "keypoints": [[float(v) for v in kp] for kp in person["keypoints"]],
                    "head_yaw_deg": float(person["head_yaw_deg"]),
                    "face_visible": bool(person["face_visible"]),
                    "confidence": float(person["confidence"]),
                    "track_is_stable_candidate": is_stable,
                    "head_center": [0.5 * (x1 + x2), y1],
                    "matched_name": (
                        match_by_track_id[track_id]
                        if track_id in match_by_track_id
                        else None
                    ),
                }
            )
        self._tracking_overlay = overlay

        if matches:
            target_person, target_name = max(matches, key=lambda item: self._bbox_area(item[0]["bbox"]))
            frame_w, frame_h = self._frame_size(input_state.camera_frame)
            cx, cy = self._get_target_point(target_person)
            cx, cy = self._extrapolate_target(
                cx,
                cy,
                frame_w,
                frame_h,
                input_state.dt,
                input_state.vision_latency_ms,
                input_state.timestamp,
            )
            self._track_head_by_point(cx, cy, frame_w, frame_h, input_state.dt)
            self._tracking_state = "staring"
            self._tracking_target = target_name
            self._tracking_track_id = int(target_person["track_id"])
            self._tracking_debug = {
                "matched_tracks": [int(item[0]["track_id"]) for item in matches],
                "target_track_id": int(target_person["track_id"]),
                "target_bbox": list(target_person["bbox"]),
                "vision_frame_id": vision_frame_id,
                "stable_track_frames": self._stable_track_frames,
            }
            return True

        stable_persons = [p for p in persons if self._track_is_stable_candidate(int(p["track_id"]))]
        if stable_persons and allow_first_face_fallback:
            target_person = stable_persons[0]
            frame_w, frame_h = self._frame_size(input_state.camera_frame)
            cx, cy = self._get_target_point(target_person)
            cx, cy = self._extrapolate_target(
                cx,
                cy,
                frame_w,
                frame_h,
                input_state.dt,
                input_state.vision_latency_ms,
                input_state.timestamp,
            )
            self._track_head_by_point(cx, cy, frame_w, frame_h, input_state.dt)
            self._tracking_state = "staring_first_face_fallback"
            self._tracking_target = None
            self._tracking_track_id = int(target_person["track_id"])
            self._tracking_debug = {
                "matched_tracks": [],
                "target_track_id": int(target_person["track_id"]),
                "target_bbox": list(target_person["bbox"]),
                "vision_frame_id": vision_frame_id,
                "stable_track_frames": self._stable_track_frames,
            }
            return True

        self._tracking_state = "manual"
        self._tracking_target = None
        self._tracking_track_id = None
        self._last_target_cx = None
        self._last_target_cy = None
        self._last_target_time = None
        self._tracking_debug = {
            "matched_tracks": [],
            "vision_frame_id": vision_frame_id,
            "stable_track_frames": self._stable_track_frames,
        }
        return False
