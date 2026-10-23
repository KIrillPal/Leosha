"""Vectorized 2D ray cast against wall and box segments."""

from __future__ import annotations

import math

import numpy as np


def _segments(world: dict) -> np.ndarray:
    rows = list(world["walls"]) + list(world["boxes"])
    if not rows:
        return np.zeros((0, 4), dtype=np.float64)
    return np.asarray(rows, dtype=np.float64)


def cast_ranges(
    origin_x: float,
    origin_y: float,
    yaw: float,
    num_rays: int,
    range_max: float,
    world: dict,
    persons: list[tuple[float, float, float]] | None = None,
    angle_min: float = -math.pi,
    angle_span: float = 2.0 * math.pi,
) -> np.ndarray:
    """Return ranges for rays starting at ``angle_min`` in the robot frame."""
    segments = _segments(world)
    step = angle_span / float(num_rays)
    angles = angle_min + step * np.arange(num_rays, dtype=np.float64)
    world_angles = yaw + angles
    dx = np.cos(world_angles)
    dy = np.sin(world_angles)
    hit = np.full(num_rays, range_max, dtype=np.float64)
    if len(segments):
        x1, y1, x2, y2 = segments.T
        sx = x2 - x1
        sy = y2 - y1
        ax = x1[None, :] - origin_x
        ay = y1[None, :] - origin_y
        denom = dx[:, None] * sy[None, :] - dy[:, None] * sx[None, :]
        safe = np.where(np.abs(denom) < 1e-9, np.nan, denom)
        t = (ax * sy[None, :] - ay * sx[None, :]) / safe
        u = (ax * dy[:, None] - ay * dx[:, None]) / safe
        valid = (t > 1e-4) & (t < range_max) & (u >= 0.0) & (u <= 1.0)
        t = np.where(valid, t, np.inf)
        nearest = t.min(axis=1)
        finite = np.isfinite(nearest)
        hit[finite] = nearest[finite]
    for px, py, radius in persons or []:
        # Ray-circle: |origin + t d - center|^2 = r^2, smallest t > 0.
        fx = origin_x - px
        fy = origin_y - py
        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - radius * radius
        disc = b * b - 4.0 * c
        ok = disc > 0.0
        sqrt_disc = np.sqrt(np.maximum(disc, 0.0))
        t0 = (-b - sqrt_disc) / 2.0
        take = ok & (t0 > 1e-4) & (t0 < hit)
        hit[take] = t0[take]
    return hit.astype(np.float32)
