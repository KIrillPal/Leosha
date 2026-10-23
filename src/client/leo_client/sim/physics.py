"""Ackermann step. Throttle is converted back with the same calibration the client applied."""

from __future__ import annotations

import math

from leo_client.calibration import speed_from_throttle, steering_from_throttle
from leo_protocol.protocol import wrap_pi


def step_pose(
    x: float,
    y: float,
    yaw: float,
    motor_throttle: float,
    steering_throttle: float,
    dt: float,
    wheelbase_m: float,
    calibration: dict,
    slip: float,
) -> tuple[float, float, float, float, float]:
    speed = speed_from_throttle(motor_throttle, calibration)
    steer = steering_from_throttle(steering_throttle, calibration)
    speed *= 1.0 - slip
    x += speed * math.cos(yaw) * dt
    y += speed * math.sin(yaw) * dt
    if abs(wheelbase_m) > 1e-6:
        yaw = wrap_pi(yaw + speed * math.tan(steer) / wheelbase_m * dt)
    return x, y, yaw, speed, steer
