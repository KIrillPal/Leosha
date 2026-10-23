"""Resample a lidar revolution onto one fixed counter-clockwise ray grid."""

from __future__ import annotations

import math

import numpy as np

from leo_protocol.protocol import Scan


def _wrap_pi(angle: np.ndarray) -> np.ndarray:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def angle_in_sector(angle_deg: float, start_deg: float, end_deg: float) -> bool:
    if start_deg <= end_deg:
        return start_deg <= angle_deg <= end_deg
    return angle_deg >= start_deg or angle_deg <= end_deg


def apply_blind_zone(
    ranges: np.ndarray,
    angle_min: float,
    angle_increment: float,
    start_deg: float,
    end_deg: float,
    max_m: float,
) -> np.ndarray:
    out = np.array(ranges, dtype=np.float32, copy=True)
    for index, distance in enumerate(out):
        if not math.isfinite(float(distance)) or float(distance) >= max_m:
            continue
        angle = math.degrees(angle_min + angle_increment * index)
        if angle_in_sector(angle, start_deg, end_deg):
            out[index] = np.nan
    return out


def resample(
    angles_rad: np.ndarray,
    ranges: np.ndarray,
    *,
    num_rays: int,
    range_min: float,
    range_max: float,
    capture_time_ns: int,
    scan_time_sec: float,
    blind_start_deg: float,
    blind_end_deg: float,
    blind_max_m: float,
) -> Scan:
    """Bin raw points onto ``[-pi, pi)`` with a constant positive increment.

    The nearest raw point wins each bin. Empty bins stay NaN. slam_toolbox
    keeps the angle model of the first scan, so later scans must share it.
    """
    angle_min = -math.pi
    increment = (2.0 * math.pi) / float(num_rays)
    grid = np.full(num_rays, np.nan, dtype=np.float32)
    if len(angles_rad):
        wrapped = _wrap_pi(np.asarray(angles_rad, dtype=np.float64))
        bins = np.floor((wrapped - angle_min) / increment).astype(np.int64)
        bins = np.clip(bins, 0, num_rays - 1)
        distances = np.asarray(ranges, dtype=np.float32)
        for index, distance in zip(bins, distances):
            if math.isfinite(float(distance)) and range_min <= float(distance) <= range_max:
                grid[int(index)] = float(distance)
    grid = apply_blind_zone(grid, angle_min, increment, blind_start_deg, blind_end_deg, blind_max_m)
    return Scan(
        capture_time_ns=capture_time_ns,
        scan_time_sec=scan_time_sec,
        angle_min=angle_min,
        angle_increment=increment,
        range_min=range_min,
        range_max=range_max,
        ranges=grid,
    )
