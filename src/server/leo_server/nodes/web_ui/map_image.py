"""Render a nav_msgs/OccupancyGrid as a small PNG without extra ROS code."""

from __future__ import annotations

import numpy as np


def grid_png(width: int, height: int, data: list[int]) -> bytes:
    import cv2

    cells = np.asarray(data, dtype=np.int16).reshape(height, width)
    image = np.full((height, width), 127, dtype=np.uint8)
    known = cells >= 0
    image[known] = np.clip(255 - cells[known] * 255 // 100, 0, 255).astype(np.uint8)
    ok, encoded = cv2.imencode(".png", np.flipud(image))
    if not ok:
        return b""
    return encoded.tobytes()
