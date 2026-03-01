from __future__ import annotations

import math

from ..interfaces import ModeExecutor
from ..models import ActuatorCommand, OperatingMode, Pose2D, TrajectoryPoint


class AutonomyExecutor(ModeExecutor):
    """Локальный исполнитель траектории (упрощенный Pure Pursuit)."""

    def __init__(
        self,
        wheelbase_m: float = 0.24,
        lookahead_m: float = 0.35,
        max_steering_rad: float = 0.6,
        speed_to_throttle: float = 1.4,
        steering_to_throttle: float = 1.7,
    ) -> None:
        self._wheelbase = float(wheelbase_m)
        self._lookahead = float(lookahead_m)
        self._max_steering_rad = float(max_steering_rad)
        self._speed_to_throttle = float(speed_to_throttle)
        self._steering_to_throttle = float(steering_to_throttle)

    @property
    def mode(self) -> OperatingMode:
        return OperatingMode.AUTONOMY_PROFILE_1

    def compute(self, state_snapshot: dict) -> ActuatorCommand:
        pose: Pose2D = state_snapshot["pose"]
        trajectory: list[TrajectoryPoint] = state_snapshot.get("trajectory", [])
        autonomy_cmd = state_snapshot.get("autonomy_cmd")
        if not trajectory:
            return ActuatorCommand()
        target = self._pick_target(pose, trajectory)
        steering_rad = self._compute_steering(pose, target)
        speed = target.target_speed
        head_pan = autonomy_cmd.head_pan if autonomy_cmd else 0.0
        head_tilt = autonomy_cmd.head_tilt if autonomy_cmd else 0.0
        return ActuatorCommand(
            motor_throttle=max(-1.0, min(1.0, speed * self._speed_to_throttle)),
            steering_throttle=max(-1.0, min(1.0, steering_rad * self._steering_to_throttle)),
            head_pan_angle=max(-1.0, min(1.0, head_pan)) * 60.0,
            head_tilt_angle=max(-1.0, min(1.0, head_tilt)) * 45.0,
        )

    def _pick_target(self, pose: Pose2D, trajectory: list[TrajectoryPoint]) -> TrajectoryPoint:
        for point in trajectory:
            dist = math.hypot(point.x - pose.x, point.y - pose.y)
            if dist >= self._lookahead:
                return point
        return trajectory[-1]

    def _compute_steering(self, pose: Pose2D, target: TrajectoryPoint) -> float:
        dx = target.x - pose.x
        dy = target.y - pose.y
        local_x = dx * math.cos(pose.theta) + dy * math.sin(pose.theta)
        local_y = -dx * math.sin(pose.theta) + dy * math.cos(pose.theta)
        length = max(1e-6, math.hypot(local_x, local_y))
        curvature = 2.0 * local_y / (length * length)
        steering = math.atan(curvature * self._wheelbase)
        return max(-self._max_steering_rad, min(self._max_steering_rad, steering))

