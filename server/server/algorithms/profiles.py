from __future__ import annotations

from dataclasses import dataclass

from ..interfaces import AlgorithmContext, InputState, OperationProfile
from ..models import ControlCommand, ControlMode, ManualInputState, TelemetryFrame


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
