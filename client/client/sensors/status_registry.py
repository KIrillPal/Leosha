from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from ..models import SensorStatus


@dataclass
class SensorStatusRecord:
    enabled: bool
    active: bool
    healthy: bool
    failure_count: int
    last_update_ns: int
    last_error: str
    disabled_reason: str


class SensorStatusRegistry:
    """Потокобезопасное хранилище статусов сенсоров."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._items: dict[str, SensorStatusRecord] = {}

    def register(self, sensor_name: str, enabled: bool) -> None:
        with self._lock:
            self._items[sensor_name] = SensorStatusRecord(
                enabled=bool(enabled),
                active=False,
                healthy=bool(enabled),
                failure_count=0,
                last_update_ns=time.monotonic_ns(),
                last_error="",
                disabled_reason="" if enabled else "disabled_in_config",
            )

    def mark_ok(self, sensor_name: str) -> None:
        with self._lock:
            item = self._items[sensor_name]
            item.active = True
            item.healthy = True
            item.last_update_ns = time.monotonic_ns()
            item.last_error = ""

    def mark_failure(self, sensor_name: str, error: str) -> None:
        with self._lock:
            item = self._items[sensor_name]
            item.active = False
            item.healthy = False
            item.failure_count += 1
            item.last_update_ns = time.monotonic_ns()
            item.last_error = str(error)

    def mark_disabled(self, sensor_name: str, reason: str) -> None:
        with self._lock:
            item = self._items[sensor_name]
            item.enabled = False
            item.active = False
            item.healthy = False
            item.disabled_reason = reason
            item.last_update_ns = time.monotonic_ns()

    def snapshot(self) -> dict[str, SensorStatus]:
        with self._lock:
            out = {}
            for name, item in self._items.items():
                out[name] = SensorStatus(
                    sensor_name=name,
                    enabled=item.enabled,
                    active=item.active,
                    healthy=item.healthy,
                    failure_count=item.failure_count,
                    last_update_ns=item.last_update_ns,
                    last_error=item.last_error,
                    disabled_reason=item.disabled_reason,
                )
            return out

