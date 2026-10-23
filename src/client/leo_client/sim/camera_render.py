"""Pseudo-3D camera: one ray per column, walls as vertical strips, people as sprites."""

from __future__ import annotations

import math

import numpy as np

from leo_client.sim.lidar_sim import cast_ranges


def _person_positions(world: dict, now_s: float) -> list[tuple[float, float, float]]:
    placed = []
    for person in world["persons"]:
        travel = float(person["travel"])
        offset = math.sin(now_s * 0.4) * travel
        placed.append((float(person["x"]) + offset, float(person["y"]), float(person["radius"])))
    return placed


def render(pose: tuple[float, float, float], pan_deg: float, tilt_deg: float, sim_cfg: dict, now_s: float) -> np.ndarray:
    world = sim_cfg["world"]
    width = int(sim_cfg["camera_width"])
    height = int(sim_cfg["camera_height"])
    fov = math.radians(float(sim_cfg["camera_fov_deg"]))
    look = pose[2] + math.radians(pan_deg)
    horizon = int(height * 0.5 - tilt_deg * 1.2)
    horizon = max(0, min(height - 1, horizon))
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:horizon, :] = (70, 90, 110)
    image[horizon:, :] = (40, 40, 36)
    ranges = cast_ranges(
        pose[0],
        pose[1],
        look,
        width,
        12.0,
        world,
        _person_positions(world, now_s),
        angle_min=-fov / 2.0,
        angle_span=fov,
    )
    focal = (width / 2.0) / math.tan(fov / 2.0)
    depth = np.clip(ranges, 0.05, 12.0)
    column_h = np.clip((focal * 0.8 / depth).astype(np.int32), 1, height)
    shade = (180 - (np.arange(width) % 48)).astype(np.float32)
    color = shade * (1.0 - np.minimum(depth / 12.0, 0.85))
    rows = np.arange(height)[:, None]
    half = column_h // 2
    wall = (rows >= np.clip(horizon - half, 0, height)) & (rows < np.clip(horizon + half, 0, height))
    image[:, :, 0] = np.where(wall, color, image[:, :, 0])
    image[:, :, 1] = np.where(wall, color, image[:, :, 1])
    image[:, :, 2] = np.where(wall, color * 0.85, image[:, :, 2])
    _draw_people(image, pose, look, fov, horizon, sim_cfg, now_s)
    return image


def _draw_people(image, pose, look, fov, horizon, sim_cfg, now_s) -> None:
    height, width, _ = image.shape
    focal = (width / 2.0) / math.tan(fov / 2.0)
    for px, py, radius in _person_positions(sim_cfg["world"], now_s):
        dx = px - pose[0]
        dy = py - pose[1]
        dist = math.hypot(dx, dy)
        if dist < 0.3 or dist > 8.0:
            continue
        bearing = math.atan2(dy, dx) - look
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        if abs(bearing) > fov * 0.6:
            continue
        col = int(width / 2.0 + math.tan(bearing) * focal)
        sprite_h = int(min(height, focal * 1.6 / dist))
        sprite_w = max(2, int(sprite_h * 0.45))
        x0 = max(0, col - sprite_w // 2)
        x1 = min(width, col + sprite_w // 2)
        y0 = max(0, horizon - sprite_h)
        y1 = min(height, horizon + sprite_h // 5)
        image[y0:y1, x0:x1] = (40, 80, 200)
        head = max(y0, y0 + sprite_h // 5)
        image[y0:head, x0:x1] = (180, 160, 140)
        del radius
