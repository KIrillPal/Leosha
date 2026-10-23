"""Motion sources for the odometry filter.

Kinematics and lidar scan matching are live. IMU, wheel encoders and visual
SLAM are called when their config flags are on, and return no measurement
until those sensors exist. A future source should return a world pose plus
variance; ``Tracker`` already applies it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from leo_server.nodes.odometry.fusion import wrap_pi
from leo_server.nodes.odometry.scan_matcher import match_scans, scan_points


@dataclass
class PoseMeasure:
    x: float
    y: float
    yaw: float
    var_xy: float
    var_yaw: float


def bicycle(x: float, y: float, yaw: float, speed: float, steer: float, dt: float, wheelbase: float) -> tuple[float, float, float]:
    yaw_rate = 0.0 if wheelbase == 0.0 else speed * math.tan(steer) / wheelbase
    return (
        x + speed * math.cos(yaw) * dt,
        y + speed * math.sin(yaw) * dt,
        wrap_pi(yaw + yaw_rate * dt),
    )


def kinematic_measure(x, y, yaw, speed, steer, dt, wheelbase, var_xy, var_yaw) -> PoseMeasure:
    nx, ny, nyaw = bicycle(x, y, yaw, speed, steer, dt, wheelbase)
    return PoseMeasure(nx, ny, nyaw, var_xy, var_yaw)


def body_delta(speed: float, steer: float, dt: float, wheelbase: float) -> tuple[float, float, float]:
    yaw_rate = 0.0 if wheelbase == 0.0 else speed * math.tan(steer) / wheelbase
    return speed * dt, 0.0, yaw_rate * dt


def scan_measure(x, y, yaw, previous, current, seed, cfg) -> PoseMeasure | None:
    motion = match_scans(
        previous,
        current,
        seed,
        int(cfg["scan_match_max_iters"]),
        float(cfg["scan_match_outlier_m"]),
    )
    if motion is None:
        return None
    dx, dy, dyaw = motion
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return PoseMeasure(
        x + cos_yaw * dx - sin_yaw * dy,
        y + sin_yaw * dx + cos_yaw * dy,
        wrap_pi(yaw + dyaw),
        float(cfg["scan_match_var_xy"]),
        float(cfg["scan_match_var_yaw"]),
    )


def measure_imu(gyro_z: float | None, x: float, y: float, yaw: float, dt: float, var_yaw: float) -> PoseMeasure | None:
    """Future: treat gyro_z as a yaw-rate measurement and return an updated yaw."""
    del gyro_z, x, y, yaw, dt, var_yaw
    return None


def measure_encoder(left_rad: float | None, right_rad: float | None, x: float, y: float, yaw: float, dt: float, var_xy: float) -> PoseMeasure | None:
    """Future: convert wheel ticks into a body-frame translation."""
    del left_rad, right_rad, x, y, yaw, dt, var_xy
    return None


def measure_visual(vo_delta, x: float, y: float, yaw: float, var_xy: float, var_yaw: float) -> PoseMeasure | None:
    """Future: compose a visual-odometry delta onto the current pose."""
    del vo_delta, x, y, yaw, var_xy, var_yaw
    return None


def prepare_scan(ranges, angle_min, increment, cfg):
    return scan_points(
        ranges,
        angle_min,
        increment,
        float(cfg["scan_match_min_range_m"]),
        float(cfg["scan_match_max_range_m"]),
        int(cfg["scan_match_max_points"]),
    )
