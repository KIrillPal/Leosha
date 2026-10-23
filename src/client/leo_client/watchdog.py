"""Slow the robot when commands stop arriving, then cut drive. The head stays put."""

from __future__ import annotations


class Watchdog:
    def __init__(self, timeout_ms: float, critical_ms: float) -> None:
        if critical_ms <= timeout_ms:
            raise ValueError("critical_ms must be greater than timeout_ms")
        self._timeout_ms = float(timeout_ms)
        self._critical_ms = float(critical_ms)

    def speed_scale(self, elapsed_ms: float) -> float:
        if elapsed_ms <= self._timeout_ms:
            return 1.0
        if elapsed_ms >= self._critical_ms:
            return 0.0
        span = self._critical_ms - self._timeout_ms
        return 1.0 - (elapsed_ms - self._timeout_ms) / span

    def drive_cut(self, elapsed_ms: float) -> bool:
        return elapsed_ms >= self._critical_ms
