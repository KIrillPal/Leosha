from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Protocol

from .models import (
    ActuatorFeedback,
    ActuatorCommand,
    OperatingMode,
    SensorTimingStats,
    ServerPacket,
    TelemetryPacket,
)


class ModeExecutor(ABC):
    @property
    @abstractmethod
    def mode(self) -> OperatingMode:
        raise NotImplementedError

    @abstractmethod
    def compute(self, state_snapshot: dict) -> ActuatorCommand:
        raise NotImplementedError


class ServerBridge(Protocol):
    def recv_packet(self) -> ServerPacket | None: ...
    def send_telemetry(self, packet: TelemetryPacket) -> None: ...
    def send_missing_packet_report(self, elapsed_ms: float) -> None: ...


class ActuatorDriver(Protocol):
    def apply(self, command: ActuatorCommand) -> None: ...
    def emergency_stop(self) -> None: ...
    def get_feedback_state(self) -> ActuatorFeedback: ...


class SensorStatsCollector(Protocol):
    def register(self, sensor_name: str, target_hz: float) -> None: ...
    def record(
        self,
        sensor_name: str,
        read_duration_ns: int,
        cycle_duration_ns: int,
        is_error: bool = False,
    ) -> None: ...
    def drain_all(self) -> dict[str, SensorTimingStats]: ...

