"""Small 2D pose filter. Motion sources contribute pose measurements; the pose itself is kept on reset."""

from __future__ import annotations

import math

import numpy as np


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class Fusion2D:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.covariance = np.eye(3) * 0.01

    def predict(self, dt: float, noise_xy: float, noise_yaw: float) -> None:
        self.covariance[0, 0] += noise_xy * dt
        self.covariance[1, 1] += noise_xy * dt
        self.covariance[2, 2] += noise_yaw * dt

    def predict_body(self, dx: float, dy: float, dyaw: float, var_xy: float, var_yaw: float) -> None:
        """Apply a body-frame motion in full. This is a prediction, not a fractional update."""
        cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
        self.x += cos_yaw * dx - sin_yaw * dy
        self.y += sin_yaw * dx + cos_yaw * dy
        self.yaw = wrap_pi(self.yaw + dyaw)
        self.covariance[0, 0] += var_xy
        self.covariance[1, 1] += var_xy
        self.covariance[2, 2] += var_yaw

    def update_pose(self, x: float, y: float, yaw: float, var_xy: float, var_yaw: float) -> None:
        state = np.array([self.x, self.y, self.yaw])
        innovation = np.array([x, y, wrap_pi(yaw)]) - state
        innovation[2] = wrap_pi(float(innovation[2]))
        measurement_noise = np.diag([var_xy, var_xy, var_yaw])
        gain = self.covariance @ np.linalg.inv(self.covariance + measurement_noise)
        updated = state + gain @ innovation
        self.x, self.y, self.yaw = (float(updated[0]), float(updated[1]), wrap_pi(float(updated[2])))
        self.covariance = (np.eye(3) - gain) @ self.covariance

    def forget_motion(self) -> None:
        """Drop matcher history trust. The pose stays, so the robot does not jump."""
        self.covariance = np.eye(3) * 0.05
