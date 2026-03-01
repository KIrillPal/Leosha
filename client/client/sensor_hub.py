from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

from .models import ImuReading, LaserScan, WheelOdometry


@dataclass
class StampedValue:
    timestamp_ns: int
    value: object


@dataclass
class SensorSnapshot:
    captured_at_ns: int
    frame_jpeg: bytes
    scan: LaserScan | None
    imu_readings: list[ImuReading] = field(default_factory=list)
    odometry: WheelOdometry | None = None
    ultrasonic_range_m: float = -1.0


class SensorHub:
    """Потокобезопасное хранилище последних сенсорных данных."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frame = StampedValue(0, b"")
        self._scan: StampedValue | None = None
        self._imu: list[ImuReading] = []
        self._odometry: StampedValue | None = None
        self._ultrasonic = StampedValue(0, -1.0)

    def put_frame(self, frame_jpeg: bytes, timestamp_ns: int | None = None) -> None:
        ts = int(timestamp_ns or time.monotonic_ns())
        with self._lock:
            self._frame = StampedValue(ts, bytes(frame_jpeg))

    def put_scan(self, scan: LaserScan) -> None:
        with self._lock:
            self._scan = StampedValue(scan.timestamp_ns, scan)

    def append_imu(self, imu: ImuReading) -> None:
        with self._lock:
            self._imu.append(imu)
            if len(self._imu) > 128:
                self._imu.pop(0)

    def put_odometry(self, odometry: WheelOdometry) -> None:
        with self._lock:
            self._odometry = StampedValue(odometry.timestamp_ns, odometry)

    def put_ultrasonic(self, distance_m: float, timestamp_ns: int | None = None) -> None:
        ts = int(timestamp_ns or time.monotonic_ns())
        with self._lock:
            self._ultrasonic = StampedValue(ts, float(distance_m))

    def snapshot(self) -> SensorSnapshot:
        with self._lock:
            frame = bytes(self._frame.value)
            scan = self._scan.value if self._scan else None
            imu = list(self._imu)
            self._imu.clear()
            odometry = self._odometry.value if self._odometry else None
            ultrasonic = float(self._ultrasonic.value)
        return SensorSnapshot(
            captured_at_ns=time.monotonic_ns(),
            frame_jpeg=frame,
            scan=scan,
            imu_readings=imu,
            odometry=odometry,
            ultrasonic_range_m=ultrasonic,
        )

