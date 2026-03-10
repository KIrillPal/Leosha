from __future__ import annotations

import math
import struct
import zlib
from pathlib import Path


def write_png_rgb(path: Path, width: int, height: int, pixels: bytes) -> None:
    assert len(pixels) == width * height * 3
    path.parent.mkdir(parents=True, exist_ok=True)
    raw = bytearray()
    stride = width * 3
    for y in range(height):
        raw.append(0)
        start = y * stride
        raw.extend(pixels[start:start + stride])
    compressed = zlib.compress(bytes(raw), level=6)

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"IDAT", compressed) + chunk(b"IEND", b"")
    path.write_bytes(png)


def render_lidar_map_png(path: Path, ranges: list[float], angle_min: float, angle_inc: float) -> None:
    width, height = 256, 256
    cx, cy = width // 2, height // 2
    scale = 30.0
    img = bytearray([245] * (width * height * 3))

    def set_px(x: int, y: int, r: int, g: int, b: int) -> None:
        if x < 0 or y < 0 or x >= width or y >= height:
            return
        i = (y * width + x) * 3
        img[i] = r
        img[i + 1] = g
        img[i + 2] = b

    for dy in range(-2, 3):
        for dx in range(-2, 3):
            set_px(cx + dx, cy + dy, 30, 30, 200)

    for i, dist in enumerate(ranges):
        if not math.isfinite(dist) or dist <= 0.0:
            continue
        angle = angle_min + i * angle_inc
        x = int(cx + math.cos(angle) * dist * scale)
        y = int(cy - math.sin(angle) * dist * scale)
        set_px(x, y, 10, 10, 10)

    write_png_rgb(path, width, height, bytes(img))

