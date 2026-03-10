from __future__ import annotations

import math
from dataclasses import dataclass

from .models import Pose2D, Twist2D, WheelOdometry


@dataclass
class EstimatorState:
    pose: Pose2D
    velocity: Twist2D


class SimpleStateEstimator:
    """Локальная быстрая оценка состояния (комплементарный фильтр)."""

    def __init__(self, alpha_gyro: float = 0.08) -> None:
        self._alpha = float(alpha_gyro)
        self._gyro_theta = 0.0

    def update(self, odometry: WheelOdometry, latest_gyro_z: float) -> EstimatorState:
        dt = max(0.01, 1.0 / 100.0)
        self._gyro_theta += latest_gyro_z * dt
        fused_theta = (1.0 - self._alpha) * odometry.pose.theta + self._alpha * self._gyro_theta
        pose = Pose2D(x=odometry.pose.x, y=odometry.pose.y, theta=fused_theta)
        velocity = Twist2D(
            linear=odometry.velocity.linear,
            angular=odometry.velocity.angular if odometry.velocity.angular else latest_gyro_z,
        )
        return EstimatorState(pose=pose, velocity=velocity)

    @staticmethod
    def integrate_ackermann(
        prev_pose: Pose2D,
        speed_mps: float,
        steering_angle: float,
        wheelbase_m: float,
        dt_s: float,
    ) -> Pose2D:
        if abs(steering_angle) < 1e-6:
            omega = 0.0
        else:
            radius = wheelbase_m / math.tan(steering_angle)
            omega = speed_mps / radius
        theta = prev_pose.theta + omega * dt_s
        x = prev_pose.x + speed_mps * math.cos(theta) * dt_s
        y = prev_pose.y + speed_mps * math.sin(theta) * dt_s
        return Pose2D(x=x, y=y, theta=theta)

