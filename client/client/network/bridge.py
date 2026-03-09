from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass
from time import monotonic_ns

from ..models import ServerPacket, TelemetryPacket
from .mock_server import MockServer
from .serialization import pack_missing_report, pack_telemetry_packet, unpack_server_packet

LOGGER = logging.getLogger(__name__)


@dataclass
class BridgeStats:
    tx_packets: int = 0
    rx_packets: int = 0
    dropped_rx: int = 0


class InMemoryBridge:
    """Тестовый мост client<->server без сокетов."""

    def __init__(self, server: MockServer) -> None:
        self._server = server
        self._stats = BridgeStats()
        self._telemetry_history: deque[TelemetryPacket] = deque(maxlen=128)
        LOGGER.info("InMemoryBridge initialized")

    @property
    def stats(self) -> BridgeStats:
        return self._stats

    @property
    def telemetry_history(self) -> list[TelemetryPacket]:
        return list(self._telemetry_history)

    def recv_packet(self) -> ServerPacket | None:
        packet = self._server.create_packet()
        if packet is None:
            self._stats.dropped_rx += 1
            return None
        self._stats.rx_packets += 1
        return packet

    def send_telemetry(self, packet: TelemetryPacket) -> None:
        self._stats.tx_packets += 1
        self._telemetry_history.append(packet)
        self._server.receive_telemetry(packet)

    def send_missing_packet_report(self, elapsed_ms: float) -> None:
        self._server.report_missing(elapsed_ms)


class ZmqBridge:
    """Реальный сетевой мост клиента через ZeroMQ."""

    def __init__(
        self,
        server_host: str,
        telemetry_port: int,
        command_port: int,
        report_port: int,
        recv_timeout_ms: int = 0,
        send_high_water_mark: int = 2,
        recv_high_water_mark: int = 1,
    ) -> None:
        try:
            import zmq
        except Exception as exc:
            raise RuntimeError("pyzmq не установлен, ZeroMQ мост недоступен") from exc
        self._zmq = zmq
        self._ctx = zmq.Context.instance()
        self._stats = BridgeStats()

        self._telemetry_pub = self._ctx.socket(zmq.PUB)
        self._telemetry_pub.setsockopt(zmq.SNDHWM, int(send_high_water_mark))
        self._telemetry_pub.setsockopt(zmq.IMMEDIATE, 1)
        self._telemetry_pub.connect(f"tcp://{server_host}:{int(telemetry_port)}")

        self._command_sub = self._ctx.socket(zmq.SUB)
        self._command_sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self._command_sub.setsockopt(zmq.RCVHWM, int(recv_high_water_mark))
        self._command_sub.setsockopt(zmq.CONFLATE, 1)
        self._command_sub.setsockopt(zmq.RCVTIMEO, int(recv_timeout_ms))
        self._command_sub.bind(f"tcp://0.0.0.0:{int(command_port)}")

        self._report_pub = self._ctx.socket(zmq.PUB)
        self._report_pub.setsockopt(zmq.SNDHWM, int(send_high_water_mark))
        self._report_pub.setsockopt(zmq.IMMEDIATE, 1)
        self._report_pub.connect(f"tcp://{server_host}:{int(report_port)}")
        LOGGER.info(
            "ZmqBridge connected | host=%s telemetry=%s command=%s report=%s",
            server_host,
            telemetry_port,
            command_port,
            report_port,
        )

    @property
    def stats(self) -> BridgeStats:
        return self._stats

    def recv_packet(self) -> ServerPacket | None:
        try:
            raw = self._command_sub.recv(flags=self._zmq.NOBLOCK)
        except self._zmq.Again:
            return None
        packet = unpack_server_packet(raw)
        self._stats.rx_packets += 1
        return packet

    def send_telemetry(self, packet: TelemetryPacket) -> None:
        header, frame = pack_telemetry_packet(packet)
        self._telemetry_pub.send(header, flags=self._zmq.SNDMORE)
        self._telemetry_pub.send(frame, copy=False)
        self._stats.tx_packets += 1

    def send_missing_packet_report(self, elapsed_ms: float) -> None:
        payload = pack_missing_report(elapsed_ms=elapsed_ms, timestamp_ns=monotonic_ns())
        self._report_pub.send(payload)

    def close(self) -> None:
        for sock in (self._telemetry_pub, self._command_sub, self._report_pub):
            try:
                sock.close(linger=0)
            except Exception:
                pass
        LOGGER.info("ZmqBridge sockets closed")

