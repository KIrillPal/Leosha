from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

from .models import SensorTimingStats


@dataclass
class _Window:
    read_durations_us: list[float] = field(default_factory=list)
    cycle_durations_us: list[float] = field(default_factory=list)
    overrun_count: int = 0
    error_count: int = 0
    target_hz: float = 0.0
    window_start_ns: int = field(default_factory=time.monotonic_ns)


class StatsCollector:
    """Сбор статистики таймингов для передачи в телеметрию."""

    def __init__(self) -> None:
        self._windows: dict[str, _Window] = {}
        self._lock = threading.Lock()

    def register(self, sensor_name: str, target_hz: float) -> None:
        with self._lock:
            self._windows[sensor_name] = _Window(target_hz=float(target_hz))

    def record(
        self,
        sensor_name: str,
        read_duration_ns: int,
        cycle_duration_ns: int,
        is_error: bool = False,
    ) -> None:
        with self._lock:
            window = self._windows[sensor_name]
            read_us = float(read_duration_ns) / 1000.0
            cycle_us = float(cycle_duration_ns) / 1000.0
            window.read_durations_us.append(read_us)
            window.cycle_durations_us.append(cycle_us)
            if is_error:
                window.error_count += 1
            target_period_us = 1e6 / window.target_hz if window.target_hz > 0 else float("inf")
            if cycle_us > target_period_us * 1.1:
                window.overrun_count += 1

    def drain_all(self) -> dict[str, SensorTimingStats]:
        now_ns = time.monotonic_ns()
        out: dict[str, SensorTimingStats] = {}
        with self._lock:
            for name, window in self._windows.items():
                elapsed_s = max(1e-6, (now_ns - window.window_start_ns) / 1e9)
                reads = window.read_durations_us
                cycles = window.cycle_durations_us
                out[name] = SensorTimingStats(
                    sensor_name=name,
                    read_min_us=min(reads) if reads else 0.0,
                    read_max_us=max(reads) if reads else 0.0,
                    read_avg_us=(sum(reads) / len(reads)) if reads else 0.0,
                    read_last_us=reads[-1] if reads else 0.0,
                    cycle_min_us=min(cycles) if cycles else 0.0,
                    cycle_max_us=max(cycles) if cycles else 0.0,
                    cycle_avg_us=(sum(cycles) / len(cycles)) if cycles else 0.0,
                    cycle_count=len(reads),
                    overrun_count=window.overrun_count,
                    error_count=window.error_count,
                    actual_hz=len(reads) / elapsed_s,
                    target_hz=window.target_hz,
                )
                window.read_durations_us.clear()
                window.cycle_durations_us.clear()
                window.overrun_count = 0
                window.error_count = 0
                window.window_start_ns = now_ns
        return out

