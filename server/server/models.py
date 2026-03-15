from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from time import monotonic


class ControlMode(str, Enum):
    PAUSE = "pause"
    TELEOPERATION = "teleoperation"
    TELEOP_SLAM = "teleop_slam"
    AUTONOMY_PROFILE_1 = "autonomy_profile_1"
    STARING = "staring"


@dataclass
class ManualInputState:
    tracking_enabled: bool = False
    w: bool = False
    a: bool = False
    s: bool = False
    d: bool = False
    shift: bool = False
    ctrl: bool = False
    head_dx: float = 0.0
    head_dy: float = 0.0


@dataclass
class TelemetryFrame:
    timestamp: float = field(default_factory=monotonic)
    odom_x: float = 0.0
    odom_y: float = 0.0
    odom_yaw: float = 0.0
    odom_confidence: float = 0.0  # 0..1 from client; if < slam threshold use SLAM pose for TF
    speed_mps: float = 0.0
    steering_rad: float = 0.0
    imu_yaw_rate: float = 0.0
    frame_jpeg: bytes = b""
    timestamp_ns: int = 0
    last_scan: dict | None = None
    last_imu: dict | None = None


@dataclass
class SlamStats:
    """SLAM pipeline statistics for UI."""
    fps: float = 0.0
    latency_ms: float = 0.0
    scan_count: int = 0
    map_updates: int = 0
    pose_updates: int = 0
    status: str = "unknown"  # initializing, mapping, localized, degraded, failed


@dataclass
class ControlCommand:
    speed: float = 0.0
    steering: float = 0.0
    head_pan: float = 0.0
    head_tilt: float = 0.0
    mode: ControlMode = ControlMode.PAUSE


@dataclass
class RobotConnectionStatus:
    ip: str = "127.0.0.1"
    connected: bool = False
    last_ping_ms: float | None = None


@dataclass
class NetworkStats:
    rssi_dbm: float = -50.0
    latency_ms: float = 1.0
    tx_bytes_per_sec: float = 0.0
    rx_bytes_per_sec: float = 0.0
    tx_packets_per_sec: float = 0.0
    rx_packets_per_sec: float = 0.0
