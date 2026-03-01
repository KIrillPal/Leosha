from __future__ import annotations

from ..interfaces import ModeExecutor
from ..models import ActuatorCommand, OperatingMode, TeleoperationCommand


class TeleoperationExecutor(ModeExecutor):
    def __init__(self, head_pan_max_deg: float = 60.0, head_tilt_max_deg: float = 45.0) -> None:
        self._head_pan_max_deg = float(head_pan_max_deg)
        self._head_tilt_max_deg = float(head_tilt_max_deg)

    @property
    def mode(self) -> OperatingMode:
        return OperatingMode.TELEOPERATION

    def compute(self, state_snapshot: dict) -> ActuatorCommand:
        cmd: TeleoperationCommand | None = state_snapshot.get("teleop_cmd")
        if cmd is None:
            return ActuatorCommand()
        return ActuatorCommand(
            motor_throttle=max(-1.0, min(1.0, cmd.speed)),
            steering_throttle=max(-1.0, min(1.0, cmd.steering)),
            head_pan_angle=max(-1.0, min(1.0, cmd.head_pan)) * self._head_pan_max_deg,
            head_tilt_angle=max(-1.0, min(1.0, cmd.head_tilt)) * self._head_tilt_max_deg,
        )

