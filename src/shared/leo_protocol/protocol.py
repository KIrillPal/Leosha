"""Msgpack messages for one client session.

The handshake port is part of the wire contract so a robot that has no
config file can still open a session. Every other setting arrives in the
handshake reply from ``config/robot.yaml``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import msgpack
import numpy as np

# Session entry port. robot.yaml network.handshake_port must equal this.
HANDSHAKE_PORT = 5551

_PACK = {"use_bin_type": True}


@dataclass
class Hello:
    client_time_ns: int
    sim: bool
    world: str


@dataclass
class Command:
    seq: int
    server_time_ns: int
    speed_mps: float
    steering_rad: float
    head_pan_deg: float
    head_tilt_deg: float


@dataclass
class Scan:
    capture_time_ns: int
    scan_time_sec: float
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: np.ndarray


@dataclass
class Telemetry:
    seq: int
    client_time_ns: int
    scan: Scan | None
    jpeg: bytes
    frame_time_ns: int
    frame_width: int
    frame_height: int
    head_pan_deg: float
    head_tilt_deg: float
    steering_rad: float
    speed_mps: float
    gyro_z: float | None
    fps_loop: float
    fps_camera: float
    fps_lidar: float
    fps_telemetry: float
    ground_truth: tuple[float, float, float] | None


def pack_hello(hello: Hello) -> bytes:
    return msgpack.packb(
        {
            "type": "hello",
            "client_time_ns": int(hello.client_time_ns),
            "sim": bool(hello.sim),
            "world": hello.world,
        },
        **_PACK,
    )


def unpack_hello(payload: bytes) -> Hello:
    data = msgpack.unpackb(payload, raw=False)
    if data.get("type") != "hello":
        raise ValueError(f"expected hello, got {data.get('type')}")
    return Hello(
        client_time_ns=int(data["client_time_ns"]),
        sim=bool(data["sim"]),
        world=str(data["world"]),
    )


def pack_reply(
    server_time_ns: int,
    client_time_ns: int,
    client_cfg: dict,
    sim_cfg: dict | None,
    ports: dict,
) -> bytes:
    return msgpack.packb(
        {
            "type": "config",
            "server_time_ns": int(server_time_ns),
            "client_time_ns": int(client_time_ns),
            "client": client_cfg,
            "sim": sim_cfg,
            "ports": {
                "telemetry_port": int(ports["telemetry_port"]),
                "command_port": int(ports["command_port"]),
            },
        },
        **_PACK,
    )


def unpack_reply(payload: bytes) -> dict:
    data = msgpack.unpackb(payload, raw=False)
    if data.get("type") != "config":
        raise ValueError(data.get("error") or f"expected config, got {data.get('type')}")
    return data


def pack_error(message: str) -> bytes:
    return msgpack.packb({"type": "error", "error": message}, **_PACK)


def pack_command(command: Command) -> bytes:
    return msgpack.packb(
        {
            "seq": int(command.seq),
            "server_time_ns": int(command.server_time_ns),
            "speed_mps": float(command.speed_mps),
            "steering_rad": float(command.steering_rad),
            "head_pan_deg": float(command.head_pan_deg),
            "head_tilt_deg": float(command.head_tilt_deg),
        },
        **_PACK,
    )


def unpack_command(payload: bytes) -> Command:
    data = msgpack.unpackb(payload, raw=False)
    return Command(
        seq=int(data["seq"]),
        server_time_ns=int(data["server_time_ns"]),
        speed_mps=float(data["speed_mps"]),
        steering_rad=float(data["steering_rad"]),
        head_pan_deg=float(data["head_pan_deg"]),
        head_tilt_deg=float(data["head_tilt_deg"]),
    )


def _scan_dict(scan: Scan | None) -> dict | None:
    if scan is None:
        return None
    ranges = np.asarray(scan.ranges, dtype=np.float32)
    return {
        "capture_time_ns": int(scan.capture_time_ns),
        "scan_time_sec": float(scan.scan_time_sec),
        "angle_min": float(scan.angle_min),
        "angle_increment": float(scan.angle_increment),
        "range_min": float(scan.range_min),
        "range_max": float(scan.range_max),
        "ranges_f32": ranges.tobytes(),
    }


def pack_telemetry(packet: Telemetry) -> tuple[bytes, bytes]:
    truth = None
    if packet.ground_truth is not None:
        truth = [float(v) for v in packet.ground_truth]
    header = {
        "seq": int(packet.seq),
        "client_time_ns": int(packet.client_time_ns),
        "scan": _scan_dict(packet.scan),
        "frame_time_ns": int(packet.frame_time_ns),
        "frame_width": int(packet.frame_width),
        "frame_height": int(packet.frame_height),
        "head_pan_deg": float(packet.head_pan_deg),
        "head_tilt_deg": float(packet.head_tilt_deg),
        "steering_rad": float(packet.steering_rad),
        "speed_mps": float(packet.speed_mps),
        "gyro_z": None if packet.gyro_z is None else float(packet.gyro_z),
        "fps_loop": float(packet.fps_loop),
        "fps_camera": float(packet.fps_camera),
        "fps_lidar": float(packet.fps_lidar),
        "fps_telemetry": float(packet.fps_telemetry),
        "ground_truth": truth,
        "jpeg_bytes": len(packet.jpeg),
    }
    return msgpack.packb(header, **_PACK), bytes(packet.jpeg)


def _scan_from(data: dict | None) -> Scan | None:
    if not data:
        return None
    raw = data["ranges_f32"]
    if isinstance(raw, str):
        raw = raw.encode("latin1")
    ranges = np.frombuffer(bytes(raw), dtype=np.float32).copy()
    return Scan(
        capture_time_ns=int(data["capture_time_ns"]),
        scan_time_sec=float(data["scan_time_sec"]),
        angle_min=float(data["angle_min"]),
        angle_increment=float(data["angle_increment"]),
        range_min=float(data["range_min"]),
        range_max=float(data["range_max"]),
        ranges=ranges,
    )


def unpack_telemetry(header: bytes, jpeg: bytes) -> Telemetry:
    data = msgpack.unpackb(header, raw=False)
    truth = data["ground_truth"]
    ground = None if truth is None else (float(truth[0]), float(truth[1]), float(truth[2]))
    gyro = data["gyro_z"]
    return Telemetry(
        seq=int(data["seq"]),
        client_time_ns=int(data["client_time_ns"]),
        scan=_scan_from(data["scan"]),
        jpeg=bytes(jpeg),
        frame_time_ns=int(data["frame_time_ns"]),
        frame_width=int(data["frame_width"]),
        frame_height=int(data["frame_height"]),
        head_pan_deg=float(data["head_pan_deg"]),
        head_tilt_deg=float(data["head_tilt_deg"]),
        steering_rad=float(data["steering_rad"]),
        speed_mps=float(data["speed_mps"]),
        gyro_z=None if gyro is None else float(gyro),
        fps_loop=float(data["fps_loop"]),
        fps_camera=float(data["fps_camera"]),
        fps_lidar=float(data["fps_lidar"]),
        fps_telemetry=float(data["fps_telemetry"]),
        ground_truth=ground,
    )


def finite_ranges(ranges: np.ndarray) -> np.ndarray:
    """Replace non-finite ranges with NaN so ROS and the matcher agree."""
    out = np.asarray(ranges, dtype=np.float32)
    bad = ~np.isfinite(out)
    if bad.any():
        out = out.copy()
        out[bad] = np.nan
    return out


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
