from __future__ import annotations

from dataclasses import dataclass

from ..interfaces import AlgorithmContext, InputState
from ..models import ControlCommand, ControlMode
from .base_profile import BaseControlProfile


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
    """Behavior skeleton: follow known person with camera/lidar fusion."""

    friend_embeddings_db: str
    approach_distance_m: float
    max_head_tilt_deg: float
    average_human_height_m: float
    yolo_model: str

    def __post_init__(self) -> None:
        self._state = "idle"
        self._target_id: str | None = None
        self._last_target_pose: tuple[float, float] | None = None
        self._debug: dict = {}

    @property
    def mode(self) -> ControlMode:
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
        if isinstance(pose, (list, tuple)) and len(pose) == 2:
            self._last_target_pose = (float(pose[0]), float(pose[1]))
