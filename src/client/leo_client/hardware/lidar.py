"""YDLidar capture. Each revolution is resampled onto the fixed ROS ray grid."""

from __future__ import annotations

import logging
import math
import threading
import time

import numpy as np

from leo_client.hardware.scan_grid import resample
from leo_protocol.protocol import Scan

LOGGER = logging.getLogger("leo.client.lidar")


class Lidar:
    def __init__(self, cfg: dict) -> None:
        self._cfg = cfg
        self._lock = threading.Lock()
        self._scan: Scan | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if not self._cfg["enabled"]:
            return
        self._thread = threading.Thread(target=self._run, name="lidar", daemon=True)
        self._thread.start()

    def latest(self) -> Scan | None:
        with self._lock:
            return self._scan

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        import ydlidar

        ydlidar.os_init()
        port = "/dev/ydlidar"
        for _key, value in ydlidar.lidarPortList().items():
            port = value
        laser = ydlidar.CYdLidar()
        self._configure(ydlidar, laser, port)
        if not laser.initialize() or not laser.turnOn():
            raise RuntimeError(f"lidar failed to start on {port}")
        scan = ydlidar.LaserScan()
        period = 1.0 / float(self._cfg["scan_hz"])
        try:
            while not self._stop.is_set():
                started = time.monotonic()
                if laser.doProcessSimple(scan):
                    built = self._build(scan)
                    with self._lock:
                        self._scan = built
                time.sleep(max(0.0, period - (time.monotonic() - started)))
        finally:
            laser.turnOff()
            laser.disconnecting()

    def _configure(self, ydlidar, laser, port: str) -> None:
        cfg = self._cfg
        laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
        laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, int(cfg["baudrate"]))
        laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        laser.setlidaropt(ydlidar.LidarPropScanFrequency, float(cfg["scan_hz"]))
        laser.setlidaropt(ydlidar.LidarPropSampleRate, int(cfg["sample_rate"]))
        laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
        laser.setlidaropt(ydlidar.LidarPropMaxAngle, float(cfg["max_angle_deg"]))
        laser.setlidaropt(ydlidar.LidarPropMinAngle, float(cfg["min_angle_deg"]))
        laser.setlidaropt(ydlidar.LidarPropMaxRange, float(cfg["range_max_m"]))
        laser.setlidaropt(ydlidar.LidarPropMinRange, float(cfg["range_min_m"]))
        laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)

    def _build(self, scan_obj) -> Scan:
        cfg = self._cfg
        sign = -1.0 if cfg["invert_to_ccw"] else 1.0
        zero = math.radians(float(cfg["zero_angle_deg"]))
        points = list(scan_obj.points)
        angles = np.array([sign * float(point.angle) - zero for point in points], dtype=np.float64)
        ranges = np.array([float(point.range) for point in points], dtype=np.float32)
        return resample(
            angles,
            ranges,
            num_rays=int(cfg["num_rays"]),
            range_min=float(cfg["range_min_m"]),
            range_max=float(cfg["range_max_m"]),
            capture_time_ns=time.monotonic_ns(),
            scan_time_sec=1.0 / float(cfg["scan_hz"]),
            blind_start_deg=float(cfg["blind_start_deg"]),
            blind_end_deg=float(cfg["blind_end_deg"]),
            blind_max_m=float(cfg["blind_max_m"]),
        )
