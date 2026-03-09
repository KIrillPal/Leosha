from __future__ import annotations

import io
import logging
import math
import random
import threading
from dataclasses import dataclass
from time import monotonic, monotonic_ns, sleep
from typing import Protocol

import msgpack
import numpy as np
from PIL import Image

from ..models import ControlCommand, ControlMode, NetworkStats, RobotConnectionStatus, TelemetryFrame

LOGGER = logging.getLogger(__name__)


def _make_black_frame_jpeg(width: int = 320, height: int = 240) -> bytes:
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
    def get_latest_lidar_scan(self) -> dict | None: ...
    def get_network_stats(self) -> NetworkStats: ...


@dataclass
class _Counters:
    tx_bytes: float = 0.0
    rx_bytes: float = 0.0
    tx_packets: float = 0.0
    rx_packets: float = 0.0
    last_t: float = monotonic()


# ---------------------------------------------------------------------------
#  Mode mapping: client OperatingMode string ↔ server ControlMode
# ---------------------------------------------------------------------------
_CLIENT_MODE_TO_SERVER = {
    "pause": ControlMode.PAUSE,
    "teleoperation": ControlMode.TELEOPERATION,
    "autonomy_profile_1": ControlMode.AUTONOMY_PROFILE_1,
}


def _server_mode_to_client_str(mode: ControlMode) -> str:
    return mode.value


# ---------------------------------------------------------------------------
#  ZmqRobotClient — real ZeroMQ transport to the robot
# ---------------------------------------------------------------------------
class ZmqRobotClient:
    """Receives telemetry from the robot via ZMQ and sends commands back.

    Socket topology (current):
      server SUB  binds     telemetry_port   ← client PUB connects
      server PUB  connects  command_port     → client SUB binds
      server SUB  binds     report_port      ← client PUB connects
    """

    _BLACK_FRAME = _make_black_frame_jpeg()

    def __init__(
        self,
        ip: str,
        bind_address: str = "0.0.0.0",
        telemetry_port: int = 5550,
        command_port: int = 5552,
        report_port: int = 5553,
        recv_timeout_ms: int = 100,
        send_high_water_mark: int = 2,
        recv_high_water_mark: int = 1,
    ) -> None:
        import zmq

        self._zmq = zmq
        self._ctx = zmq.Context.instance()

        self._telemetry_sub = self._ctx.socket(zmq.SUB)
        self._telemetry_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self._telemetry_sub.setsockopt(zmq.RCVHWM, int(recv_high_water_mark))
        self._telemetry_sub.bind(f"tcp://{bind_address}:{telemetry_port}")

        self._command_pub = self._ctx.socket(zmq.PUB)
        self._command_pub.setsockopt(zmq.SNDHWM, max(100, int(send_high_water_mark)))
        self._command_pub.setsockopt(zmq.LINGER, 0)
        self._command_pub.connect(f"tcp://{ip}:{command_port}")

        self._report_sub = self._ctx.socket(zmq.SUB)
        self._report_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self._report_sub.setsockopt(zmq.RCVHWM, int(recv_high_water_mark))
        self._report_sub.setsockopt(zmq.CONFLATE, 1)
        self._report_sub.bind(f"tcp://{bind_address}:{report_port}")

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ip = ip
        self._connected = False
        self._last_recv_time: float = 0.0
        self._last_ping_ms: float | None = None
        self._telemetry = TelemetryFrame(frame_jpeg=self._BLACK_FRAME)
        self._latest_frame: bytes = self._BLACK_FRAME
        self._latest_lidar_scan: dict | None = None
        self._counters = _Counters()
        self._network = NetworkStats()
        self._cmd_seq = 0
        self._rx_total = 0
        self._tx_total = 0
        self._report_total = 0

        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

        LOGGER.info(
            "ZmqRobotClient started | robot=%s bind=%s telem_port=%d cmd->%s:%d report_port=%d",
            ip, bind_address, telemetry_port, ip, command_port, report_port,
        )

    # -- RobotClient protocol -----------------------------------------------

    def set_ip(self, ip: str) -> None:
        with self._lock:
            self._ip = ip
        LOGGER.info("ZmqRobotClient IP set to %s", ip)

    def ping(self) -> float | None:
        with self._lock:
            if not self._connected:
                return None
            return self._last_ping_ms

    def get_status(self) -> RobotConnectionStatus:
        with self._lock:
            return RobotConnectionStatus(
                ip=self._ip,
                connected=self._connected,
                last_ping_ms=self._last_ping_ms,
            )

    def send_command(self, command: ControlCommand) -> None:
        self._cmd_seq += 1
        payload = self._pack_server_packet(command)
        try:
            self._command_pub.send(payload, flags=self._zmq.NOBLOCK)
        except self._zmq.Again:
            LOGGER.warning("Command PUB send dropped (HWM reached) | seq=%d", self._cmd_seq)
        self._tx_total += 1
        if self._tx_total <= 3 or self._tx_total % 500 == 0:
            LOGGER.debug(
                "CMD TX #%d | mode=%s speed=%.3f steer=%.3f pan=%.2f tilt=%.2f",
                self._tx_total, command.mode.value, command.speed,
                command.steering, command.head_pan, command.head_tilt,
            )
        with self._lock:
            self._counters.tx_packets += 1
            self._counters.tx_bytes += len(payload)

    def get_latest_telemetry(self) -> TelemetryFrame:
        with self._lock:
            return TelemetryFrame(**self._telemetry.__dict__)

    def get_latest_frame(self) -> bytes:
        with self._lock:
            return self._latest_frame

    def get_latest_lidar_scan(self) -> dict | None:
        with self._lock:
            if self._latest_lidar_scan is None:
                return None
            return dict(self._latest_lidar_scan)

    def get_network_stats(self) -> NetworkStats:
        with self._lock:
            now = monotonic()
            dt = max(1e-3, now - self._counters.last_t)
            self._network.tx_bytes_per_sec = self._counters.tx_bytes / dt
            self._network.rx_bytes_per_sec = self._counters.rx_bytes / dt
            self._network.tx_packets_per_sec = self._counters.tx_packets / dt
            self._network.rx_packets_per_sec = self._counters.rx_packets / dt
            self._counters = _Counters(last_t=now)
            return NetworkStats(**self._network.__dict__)

    def close(self) -> None:
        self._stop.set()
        self._recv_thread.join(timeout=2.0)
        for sock in (self._telemetry_sub, self._command_pub, self._report_sub):
            try:
                sock.close(linger=0)
            except Exception:
                pass
        LOGGER.info("ZmqRobotClient stopped")

    # -- internal -----------------------------------------------------------

    def _recv_loop(self) -> None:
        """Background thread: receive telemetry multipart messages."""
        poller = self._zmq.Poller()
        poller.register(self._telemetry_sub, self._zmq.POLLIN)
        poller.register(self._report_sub, self._zmq.POLLIN)
        last_heartbeat = monotonic()

        while not self._stop.is_set():
            try:
                events = dict(poller.poll(timeout=50))
            except self._zmq.ZMQError:
                break

            if self._telemetry_sub in events:
                try:
                    parts = self._telemetry_sub.recv_multipart(flags=self._zmq.NOBLOCK)
                    if len(parts) >= 2:
                        self._handle_telemetry(parts[0], parts[1])
                except self._zmq.Again:
                    pass
                except Exception as exc:
                    LOGGER.warning("Telemetry recv error: %s", exc)

            if self._report_sub in events:
                try:
                    raw = self._report_sub.recv(flags=self._zmq.NOBLOCK)
                    report = msgpack.unpackb(raw, raw=False)
                    self._report_total += 1
                    LOGGER.warning(
                        "Client reports missing packets (#%d) | elapsed_ms=%.1f",
                        self._report_total, report.get("elapsed_ms", 0),
                    )
                except Exception:
                    pass

            now = monotonic()
            with self._lock:
                if self._last_recv_time > 0 and (now - self._last_recv_time) > 3.0:
                    self._connected = False
                    self._last_ping_ms = None

            if now - last_heartbeat >= 5.0:
                last_heartbeat = now
                LOGGER.debug(
                    "ZMQ heartbeat | rx_telemetry=%d tx_commands=%d client_reports=%d connected=%s",
                    self._rx_total, self._tx_total, self._report_total, self._connected,
                )

    def _handle_telemetry(self, header_raw: bytes, frame_jpeg: bytes) -> None:
        now = monotonic()
        header = msgpack.unpackb(header_raw, raw=False)

        odom = header.get("odometry", {})
        pose = odom.get("pose", {})
        vel = odom.get("velocity", {})
        imu_readings = header.get("imu_readings", [])
        imu_yaw_rate = imu_readings[-1]["gyro_z"] if imu_readings else 0.0

        with self._lock:
            self._telemetry.timestamp = now
            self._telemetry.odom_x = float(pose.get("x", 0.0))
            self._telemetry.odom_y = float(pose.get("y", 0.0))
            self._telemetry.odom_yaw = float(pose.get("theta", 0.0))
            self._telemetry.speed_mps = float(vel.get("linear", 0.0))
            self._telemetry.steering_rad = float(odom.get("steering_angle", 0.0))
            self._telemetry.imu_yaw_rate = float(imu_yaw_rate)
            self._telemetry.frame_jpeg = frame_jpeg
            self._latest_frame = frame_jpeg
            scan = header.get("scan")
            if isinstance(scan, dict):
                self._latest_lidar_scan = {
                    "timestamp_ns": int(scan.get("timestamp_ns", 0)),
                    "angle_min": float(scan.get("angle_min", 0.0)),
                    "angle_max": float(scan.get("angle_max", 0.0)),
                    "angle_increment": float(scan.get("angle_increment", 0.0)),
                    "range_min": float(scan.get("range_min", 0.0)),
                    "range_max": float(scan.get("range_max", 0.0)),
                    "ranges": list(scan.get("ranges", [])),
                    "intensities": list(scan.get("intensities", [])),
                }

            self._counters.rx_packets += 1
            self._counters.rx_bytes += len(header_raw) + len(frame_jpeg)
            self._rx_total += 1

            elapsed_ms = (now - self._last_recv_time) * 1000 if self._last_recv_time > 0 else 0.0
            self._last_recv_time = now
            self._connected = True
            self._last_ping_ms = elapsed_ms
            self._network.latency_ms = elapsed_ms
            self._network.rssi_dbm = float(header.get("wifi_rssi_dbm", -50.0))

        rx = self._rx_total
        if rx <= 3 or rx % 300 == 0:
            LOGGER.debug(
                "TELEM RX #%d | seq=%s mode=%s status=%s frame=%d bytes dt=%.1fms",
                rx, header.get("seq"), header.get("mode"), header.get("status"),
                len(frame_jpeg), elapsed_ms,
            )

    def _pack_server_packet(self, command: ControlCommand) -> bytes:
        payload = {
            "header": {
                "seq": self._cmd_seq,
                "timestamp_ns": monotonic_ns(),
                "mode": _server_mode_to_client_str(command.mode),
            },
            "teleop_cmd": {
                "speed": command.speed,
                "steering": command.steering,
                "head_pan": command.head_pan,
                "head_tilt": command.head_tilt,
            },
            "autonomy_cmd": None,
        }
        return msgpack.packb(payload, use_bin_type=True)


# ---------------------------------------------------------------------------
#  MockRobotClient — fake physics for testing without real hardware
# ---------------------------------------------------------------------------
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
        LOGGER.info("MockRobotClient started | ip=%s", ip)

    def set_ip(self, ip: str) -> None:
        with self._lock:
            self._status.ip = ip
        LOGGER.info("MockRobotClient IP set to %s", ip)

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

    def get_latest_lidar_scan(self) -> dict | None:
        return None

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
        LOGGER.info("MockRobotClient stopped")

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
