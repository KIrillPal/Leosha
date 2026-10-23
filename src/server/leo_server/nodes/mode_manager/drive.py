"""Drive behaviours. They return speed in m/s and steering in radians."""

from __future__ import annotations

import math
from dataclasses import dataclass


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def keyboard_drive(keys, cfg: dict, yaw_rate: float, enabled: bool) -> tuple[float, float]:
    if not enabled:
        return 0.0, 0.0
    if keys.w and not keys.s:
        speed = float(cfg["fast_mps"] if keys.shift else cfg["forward_mps"])
    elif keys.s and not keys.w:
        speed = float(cfg["backward_mps"])
    else:
        speed = 0.0
    if keys.a and not keys.d:
        steer = -float(cfg["steering_rad"])
    elif keys.d and not keys.a:
        steer = float(cfg["steering_rad"])
    else:
        steer = 0.0
    damp = max(float(cfg["yaw_damping_min"]), 1.0 - abs(yaw_rate) * float(cfg["yaw_damping_gain"]))
    return speed, steer * damp


@dataclass
class Sectors:
    front: float
    left: float
    right: float
    rear: float


def sector_distances(scan, front_deg: float, side_deg: float, rear_deg: float) -> Sectors:
    front = left = right = rear = math.inf
    if scan is None:
        return Sectors(front, left, right, rear)
    for index, distance in enumerate(scan.ranges):
        if distance is None or not math.isfinite(float(distance)) or float(distance) <= 0.0:
            continue
        angle = math.degrees(scan.angle_min + scan.angle_increment * index)
        angle = ((angle + 180.0) % 360.0) - 180.0
        value = float(distance)
        if -front_deg <= angle <= front_deg:
            front = min(front, value)
        elif front_deg < angle <= side_deg:
            left = min(left, value)
        elif -side_deg <= angle < -front_deg:
            right = min(right, value)
        elif abs(angle) >= rear_deg:
            rear = min(rear, value)
    return Sectors(front, left, right, rear)


@dataclass
class FollowMemory:
    phase: str = "idle"
    reverse_since: float = 0.0


def follow_drive(pan_deg: float, tilt_deg: float, pan_limit: float, scan, yaw_rate: float, now: float, memory: FollowMemory, cfg: dict) -> tuple[float, float]:
    sectors = sector_distances(
        scan,
        float(cfg["front_sector_half_angle_deg"]),
        float(cfg["side_sector_outer_angle_deg"]),
        float(cfg["rear_sector_start_angle_deg"]),
    )
    if tilt_deg >= float(cfg["stop_head_tilt_deg"]):
        memory.phase = "reached"
        return 0.0, 0.0
    if cfg["stop_without_lidar"] and scan is None:
        memory.phase = "waiting_lidar"
        return 0.0, 0.0
    reversing = _update_reverse(memory, sectors, now, cfg)
    if memory.phase == "blocked":
        return 0.0, 0.0
    steer = _steer_from_head(pan_deg, pan_limit, yaw_rate, reversing, cfg)
    if reversing:
        memory.phase = "reversing"
        return float(cfg["backward_mps"]), steer
    if sectors.left < float(cfg["side_obstacle_distance_m"]):
        steer += float(cfg["side_steering_nudge_rad"])
    if sectors.right < float(cfg["side_obstacle_distance_m"]):
        steer -= float(cfg["side_steering_nudge_rad"])
    limit = float(cfg["max_steering_rad"])
    steer = _clamp(steer, limit)
    factor = max(float(cfg["forward_speed_min_factor"]), 1.0 - abs(steer) * float(cfg["forward_speed_steering_reduction_gain"]))
    memory.phase = "following"
    return float(cfg["forward_mps"]) * factor, steer


def _update_reverse(memory: FollowMemory, sectors: Sectors, now: float, cfg: dict) -> bool:
    reversing = memory.phase == "reversing"
    if reversing and now - memory.reverse_since >= float(cfg["reverse_timeout_sec"]):
        reversing = False
    if reversing and sectors.rear <= float(cfg["rear_clear_distance_m"]):
        memory.phase = "blocked"
        return False
    front_blocked = sectors.front <= float(cfg["front_obstacle_distance_m"])
    if not reversing and front_blocked:
        if sectors.rear > float(cfg["rear_clear_distance_m"]):
            memory.phase = "reversing"
            memory.reverse_since = now
            return True
        memory.phase = "blocked"
        return False
    return reversing


def _steer_from_head(pan_deg: float, pan_limit: float, yaw_rate: float, reverse: bool, cfg: dict) -> float:
    fraction = 0.0 if pan_limit == 0.0 else max(-1.0, min(1.0, pan_deg / pan_limit))
    steer = -fraction * float(cfg["steering_gain"])
    if abs(steer) < float(cfg["steering_deadzone_rad"]):
        steer = 0.0
    if reverse:
        steer *= float(cfg["reverse_steering_multiplier"])
    damp = max(
        float(cfg["steering_yaw_damping_min"]),
        1.0 - abs(yaw_rate) * float(cfg["steering_yaw_damping_gain"]),
    )
    return _clamp(steer * damp, float(cfg["max_steering_rad"]))
