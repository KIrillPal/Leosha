from __future__ import annotations

import io
import math
import random
import threading
from dataclasses import dataclass
from time import monotonic, sleep
from typing import Protocol

import numpy as np
from PIL import Image

from ..models import ControlCommand, NetworkStats, RobotConnectionStatus, TelemetryFrame


def _make_black_frame_jpeg(width: int = 320, height: int = 240) -> bytes:
    """Чёрный кадр как массив, сжатый в JPEG."""
    arr = np.zeros((height, width, 3), dtype=np.uint8)
    img = Image.fromarray(arr, mode="RGB")
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


class RobotClient(Protocol):
    def set_ip(self, ip: str) -> None: ...
    def ping(self) -> float | None: ...
    def get_status(self) -> RobotConnectionStatus: ...
    def send_command(self, command: ControlCommand) -> None: ...
    def get_latest_telemetry(self) -> TelemetryFrame: ...
    def get_latest_frame(self) -> bytes: ...
    def get_network_stats(self) -> NetworkStats: ...


@dataclass
class _Counters:
    tx_bytes: float = 0.0
    rx_bytes: float = 0.0
    tx_packets: float = 0.0
    rx_packets: float = 0.0
    last_t: float = monotonic()


class MockRobotClient:
    _FRAME_JPEG = _make_black_frame_jpeg()

    def __init__(self, ip: str = "127.0.0.1") -> None:
        self._status = RobotConnectionStatus(ip=ip, connected=True, last_ping_ms=1.2)
        self._last_command = ControlCommand()
        self._telemetry = TelemetryFrame(frame_jpeg=self._FRAME_JPEG)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._counters = _Counters()
        self._network = NetworkStats(latency_ms=1.5, rssi_dbm=-42.0)
        self._thread = threading.Thread(target=self._physics_loop, daemon=True)
        self._thread.start()

    def set_ip(self, ip: str) -> None:
        with self._lock:
            self._status.ip = ip

    def ping(self) -> float | None:
        with self._lock:
            if not self._status.connected:
                return None
            ping_ms = 1.2 + random.uniform(0.2, 2.0)
            self._status.last_ping_ms = ping_ms
            self._network.latency_ms = ping_ms
            return ping_ms

    def get_status(self) -> RobotConnectionStatus:
        with self._lock:
            return RobotConnectionStatus(**self._status.__dict__)

    def send_command(self, command: ControlCommand) -> None:
        with self._lock:
            self._last_command = command
            self._counters.tx_packets += 1
            self._counters.tx_bytes += 96.0

    def get_latest_telemetry(self) -> TelemetryFrame:
        with self._lock:
            self._counters.rx_packets += 1
            self._counters.rx_bytes += float(len(self._telemetry.frame_jpeg) + 320)
            return TelemetryFrame(**self._telemetry.__dict__)

    def get_latest_frame(self) -> bytes:
        with self._lock:
            return self._telemetry.frame_jpeg

    def get_network_stats(self) -> NetworkStats:
        with self._lock:
            now = monotonic()
            dt = max(1e-3, now - self._counters.last_t)
            self._network.tx_bytes_per_sec = self._counters.tx_bytes / dt
            self._network.rx_bytes_per_sec = self._counters.rx_bytes / dt
            self._network.tx_packets_per_sec = self._counters.tx_packets / dt
            self._network.rx_packets_per_sec = self._counters.rx_packets / dt
            self._counters = _Counters(last_t=now)
            self._network.rssi_dbm = max(-90.0, min(-30.0, self._network.rssi_dbm + random.uniform(-0.2, 0.2)))
            self._network.latency_ms = max(0.6, self._network.latency_ms + random.uniform(-0.1, 0.2))
            return NetworkStats(**self._network.__dict__)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)

    def _physics_loop(self) -> None:
        last_t = monotonic()
        while not self._stop.is_set():
            sleep(0.01)
            now = monotonic()
            dt = now - last_t
            last_t = now
            with self._lock:
                cmd = self._last_command
                yaw_rate = cmd.steering * cmd.speed * 2.2
                self._telemetry.imu_yaw_rate = yaw_rate
                self._telemetry.speed_mps = cmd.speed
                self._telemetry.steering_rad = cmd.steering
                self._telemetry.odom_yaw += yaw_rate * dt
                self._telemetry.odom_x += cmd.speed * math.cos(self._telemetry.odom_yaw) * dt
                self._telemetry.odom_y += cmd.speed * math.sin(self._telemetry.odom_yaw) * dt
                self._telemetry.timestamp = now
