from __future__ import annotations

import threading
from time import monotonic

from .models import RobotStatus


class Watchdog:
    """Контроль частоты прихода команд от сервера."""

    def __init__(self, warn_ms: float = 50.0, timeout_ms: float = 100.0, critical_ms: float = 500.0) -> None:
        self._warn_ms = float(warn_ms)
        self._timeout_ms = float(timeout_ms)
        self._critical_ms = float(critical_ms)
        self._lock = threading.Lock()
        self._last_feed = monotonic()

    def feed(self) -> None:
        with self._lock:
            self._last_feed = monotonic()

    def elapsed_ms(self) -> float:
        with self._lock:
            return (monotonic() - self._last_feed) * 1000.0

    def status(self) -> RobotStatus:
        elapsed = self.elapsed_ms()
        if elapsed > self._critical_ms:
            return RobotStatus.EMERGENCY_STOP
        if elapsed > self._timeout_ms:
            return RobotStatus.WAITING_FOR_SERVER
        if elapsed > self._warn_ms:
            return RobotStatus.DEGRADED
        return RobotStatus.RUNNING

    def deceleration_factor(self) -> float:
        elapsed = self.elapsed_ms()
        if elapsed <= self._timeout_ms:
            return 1.0
        if elapsed >= self._critical_ms:
            return 0.0
        span = self._critical_ms - self._timeout_ms
        return 1.0 - (elapsed - self._timeout_ms) / max(1e-6, span)

