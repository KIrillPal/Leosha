from __future__ import annotations

import logging

from ..interfaces import ModeExecutor
from ..models import ActuatorCommand, OperatingMode, TeleoperationCommand

LOGGER = logging.getLogger(__name__)


class TeleoperationExecutor(ModeExecutor):
    def __init__(self, head_pan_max_deg: float = 60.0, head_tilt_max_deg: float = 45.0) -> None:
        self._head_pan_max_deg = float(head_pan_max_deg)
        self._head_tilt_max_deg = float(head_tilt_max_deg)
        self._log_count = 0

    @property
    def mode(self) -> OperatingMode:
        return OperatingMode.TELEOPERATION

    def compute(self, state_snapshot: dict) -> ActuatorCommand:
        cmd: TeleoperationCommand | None = state_snapshot.get("teleop_cmd")
        if cmd is None:
            return ActuatorCommand()
        result = ActuatorCommand(
            motor_throttle=max(-1.0, min(1.0, cmd.speed)),
            steering_throttle=max(-1.0, min(1.0, cmd.steering)),
            head_pan_angle=max(-1.0, min(1.0, cmd.head_pan)) * self._head_pan_max_deg,
            head_tilt_angle=max(-1.0, min(1.0, cmd.head_tilt)) * self._head_tilt_max_deg,
        )
        self._log_count += 1
        if self._log_count <= 5 or self._log_count % 300 == 0:
            LOGGER.info(
                "TELEOP apply #%d | throttle=%.3f steer=%.3f pan=%.1f° tilt=%.1f°",
                self._log_count, result.motor_throttle, result.steering_throttle,
                result.head_pan_angle, result.head_tilt_angle,
            )
        return result

