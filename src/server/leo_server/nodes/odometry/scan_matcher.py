"""Point-to-line ICP between two scans, seeded by the kinematic guess.

This is the lidar motion source. IMU, wheel encoders and visual SLAM plug in
beside it; they are not implemented yet.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial import cKDTree


def scan_points(ranges, angle_min: float, increment: float, min_range: float, max_range: float, max_points: int) -> np.ndarray:
    values = np.asarray(ranges, dtype=np.float64)
    valid = np.flatnonzero(np.isfinite(values) & (values >= min_range) & (values <= max_range))
    if len(valid) > max_points:
        valid = valid[:: max(1, len(valid) // max_points)][:max_points]
    angles = angle_min + valid.astype(np.float64) * increment
    distance = values[valid]
    return np.stack((distance * np.cos(angles), distance * np.sin(angles)), axis=1)


def _apply(points: np.ndarray, dx: float, dy: float, yaw: float) -> np.ndarray:
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    x = cos_yaw * points[:, 0] - sin_yaw * points[:, 1] + dx
    y = sin_yaw * points[:, 0] + cos_yaw * points[:, 1] + dy
    return np.stack((x, y), axis=1)


def match_scans(previous: np.ndarray, current: np.ndarray, seed: tuple[float, float, float], iters: int, outlier_m: float):
    """Return robot motion (dx, dy, dyaw) that maps the new scan onto the previous one."""
    if len(previous) < 10 or len(current) < 10:
        return None
    tree = cKDTree(previous)
    pose = np.array(seed, dtype=np.float64)
    for _ in range(max(1, iters)):
        moved = _apply(current, float(pose[0]), float(pose[1]), float(pose[2]))
        _dist, index = tree.query(moved, k=2)
        first = previous[index[:, 0]]
        second = previous[index[:, 1]]
        tangent = second - first
        length = np.linalg.norm(tangent, axis=1)
        gap = np.linalg.norm(moved - first, axis=1)
        good = (length > 1e-3) & (gap < outlier_m)
        if int(good.sum()) < 8:
            return None
        normal = np.stack((-tangent[good, 1], tangent[good, 0]), axis=1) / length[good, None]
        point = moved[good]
        origin = current[good]
        anchor = first[good]
        residual = np.sum(normal * (point - anchor), axis=1)
        yaw = float(pose[2])
        d_yaw_x = -math.sin(yaw) * origin[:, 0] - math.cos(yaw) * origin[:, 1]
        d_yaw_y = math.cos(yaw) * origin[:, 0] - math.sin(yaw) * origin[:, 1]
        jacobian = np.stack(
            (normal[:, 0], normal[:, 1], normal[:, 0] * d_yaw_x + normal[:, 1] * d_yaw_y),
            axis=1,
        )
        system = jacobian.T @ jacobian + np.eye(3) * 1e-6
        try:
            delta = np.linalg.solve(system, -(jacobian.T @ residual))
        except np.linalg.LinAlgError:
            return None
        pose = pose + delta
        if float(np.linalg.norm(delta)) < 1e-4:
            break
    return float(pose[0]), float(pose[1]), math.atan2(math.sin(float(pose[2])), math.cos(float(pose[2])))
