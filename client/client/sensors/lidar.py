from __future__ import annotations

import logging
import math
import threading
from time import monotonic, monotonic_ns, sleep

from ..models import LaserScan
from .base import HealthcheckResult

LOGGER = logging.getLogger(__name__)


def _angle_in_sector(angle_deg: float, start_deg: float, end_deg: float) -> bool:
    """Проверка принадлежности угла сектору с поддержкой перехода через -180/180."""
    a = ((angle_deg + 180.0) % 360.0) - 180.0
    s = ((start_deg + 180.0) % 360.0) - 180.0
    e = ((end_deg + 180.0) % 360.0) - 180.0
    if s <= e:
        return s <= a <= e
    return a >= s or a <= e


class TMiniProPlusLidarThread(threading.Thread):
    """Поток реального лидара YDLidar T-mini Pro Plus через SDK ydlidar."""

    def __init__(self, lidar_cfg, geometry_cfg, sensor_hub, stats, statuses, stop_event) -> None:
        super().__init__(daemon=True)
        self._cfg = lidar_cfg
        self._geometry = geometry_cfg
        self._hub = sensor_hub
        self._stats = stats
        self._statuses = statuses
        self._stop_event = stop_event
        self._laser = None
        self._sdk = None
        self._failures = 0

    def healthcheck(self, timeout_sec: float | None = None) -> HealthcheckResult:
        sensor_name = "lidar"
        if not self._cfg.enabled:
            return HealthcheckResult(sensor_name=sensor_name, ok=False, latency_ms=0.0, reason="disabled_in_config")
        timeout = float(timeout_sec or self._cfg.healthcheck_timeout_sec)
        t_start = monotonic()
        laser = None
        try:
            import ydlidar  # type: ignore

            laser, selected_port = self._create_and_init_lidar(ydlidar)
            if not laser.turnOn():
                raise RuntimeError("ydlidar turnOn failed")
            last_scan = None
            while (monotonic() - t_start) < timeout:
                scan_obj = ydlidar.LaserScan()
                ok = laser.doProcessSimple(scan_obj)
                if ok and scan_obj.points:
                    last_scan = self._to_laserscan(scan_obj)
                    if len(last_scan.ranges) >= int(self._cfg.healthcheck_min_points):
                        return HealthcheckResult(
                            sensor_name=sensor_name,
                            ok=True,
                            latency_ms=(monotonic() - t_start) * 1000.0,
                            details={
                                "scan": last_scan,
                                "points": len(last_scan.ranges),
                                "port": selected_port,
                            },
                        )
                sleep(0.02)
            points = len(last_scan.ranges) if last_scan is not None else 0
            return HealthcheckResult(
                sensor_name=sensor_name,
                ok=False,
                latency_ms=(monotonic() - t_start) * 1000.0,
                reason="not_enough_points",
                details={"points": points},
            )
        except Exception as exc:
            return HealthcheckResult(
                sensor_name=sensor_name,
                ok=False,
                latency_ms=(monotonic() - t_start) * 1000.0,
                reason=str(exc),
            )
        finally:
            if laser is not None:
                try:
                    laser.turnOff()
                    laser.disconnecting()
                except Exception:
                    pass

    def run(self) -> None:
        name = "lidar"
        if not self._cfg.enabled:
            self._statuses.mark_disabled(name, "disabled_in_config")
            LOGGER.info("Lidar sensor disabled by config")
            return
        try:
            import ydlidar  # type: ignore

            self._sdk = ydlidar
            laser, _ = self._create_and_init_lidar(ydlidar)
            if not laser.turnOn():
                raise RuntimeError("ydlidar turnOn failed")
            self._laser = laser
        except Exception as exc:
            self._statuses.mark_failure(name, f"init_failed: {exc}")
            LOGGER.error("Lidar initialization failed: %s", exc)
            if self._cfg.fail_policy.auto_disable_on_fail:
                self._statuses.mark_disabled(name, "init_failed")
                LOGGER.warning("Lidar sensor auto-disabled after init failure")
            return

        period = 1.0 / max(1.0, float(self._cfg.scan_hz))
        while not self._stop_event.is_set():
            cycle_start = monotonic_ns()
            read_start = monotonic_ns()
            try:
                scan_obj = self._sdk.LaserScan()
                ok = self._laser.doProcessSimple(scan_obj)
                if not ok or not scan_obj.points:
                    raise RuntimeError("empty lidar scan")
                scan = self._to_laserscan(scan_obj)
                read_end = monotonic_ns()
                self._hub.put_scan(scan)
                self._statuses.mark_ok(name)
                self._failures = 0
                self._stats.record(name, read_end - read_start, monotonic_ns() - cycle_start)
            except Exception as exc:
                self._failures += 1
                self._statuses.mark_failure(name, str(exc))
                self._stats.record(name, monotonic_ns() - read_start, monotonic_ns() - cycle_start, is_error=True)
                if self._failures >= self._cfg.fail_policy.max_consecutive_failures and self._cfg.fail_policy.auto_disable_on_fail:
                    self._statuses.mark_disabled(name, "too_many_failures")
                    LOGGER.error("Lidar sensor auto-disabled after %d consecutive failures", self._failures)
                    break
                sleep(max(0.05, float(self._cfg.fail_policy.retry_interval_sec)))
                continue
            sleep(max(0.0, period - (monotonic_ns() - cycle_start) / 1e9))

        if self._laser is not None:
            try:
                self._laser.turnOff()
                self._laser.disconnecting()
            except Exception:
                pass
        LOGGER.info("Lidar sensor thread stopped")

    def _to_laserscan(self, scan_obj) -> LaserScan:
        # Sort by output angle so angle_min < angle_max and increment is positive (frontend expects first + i*inc).
        out_key = (lambda p: -float(p.angle)) if bool(self._cfg.invert_angle) else (lambda p: float(p.angle))
        points = sorted(scan_obj.points, key=out_key)
        if len(points) < 2:
            raise RuntimeError("not enough points")
        s = -1 if bool(self._cfg.invert_angle) else 1
        zero_rad = math.radians(float(self._cfg.zero_angle_deg))
        # Сдвиг нуля: на угле zero_angle_deg будет 0.
        angles = [s * float(p.angle) - zero_rad for p in points]
        ranges = [float(p.range) for p in points]
        intensities = [float(getattr(p, "intensity", 0.0)) for p in points]

        blind = self._geometry.lidar.blind_zone
        start_blind = float(blind.angle_start_deg)
        end_blind = float(blind.angle_end_deg)
        if bool(self._cfg.invert_angle):
            start_blind, end_blind = -end_blind, -start_blind
        filtered_ranges = []
        for angle_rad, dist in zip(angles, ranges):
            angle_deg = math.degrees(angle_rad)
            if _angle_in_sector(angle_deg, start_blind, end_blind) and dist < blind.max_distance_m:
                filtered_ranges.append(float(blind.max_distance_m))
            else:
                filtered_ranges.append(dist)

        diffs = [angles[i + 1] - angles[i] for i in range(len(angles) - 1)]
        diffs = [d for d in diffs if d > 0]
        angle_increment = sum(diffs) / len(diffs) if diffs else 0.01
        return LaserScan(
            timestamp_ns=monotonic_ns(),
            angle_min=angles[0],
            angle_max=angles[-1],
            angle_increment=angle_increment,
            range_min=float(self._cfg.range_min_m),
            range_max=float(self._cfg.range_max_m),
            ranges=filtered_ranges,
            intensities=intensities,
        )

    def _create_and_init_lidar(self, ydlidar):
        """Create CYdLidar, set options (same order as plot_tminiplus_test.py), initialize. Returns (laser, port)."""
        
        ydlidar.os_init()
        ports = ydlidar.lidarPortList()
        port = "/dev/ydlidar"
        for key, value in ports.items():
            port = value
            LOGGER.info("Found LiDAR port: %s", port)
        # Create laser and set options in exact order as plot_tminiplus_test.py (config values)
        laser = ydlidar.CYdLidar()
        laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
        laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, int(self._cfg.baudrate))
        laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        laser.setlidaropt(ydlidar.LidarPropScanFrequency, float(self._cfg.scan_hz))
        laser.setlidaropt(ydlidar.LidarPropSampleRate, 4)
        laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
        laser.setlidaropt(ydlidar.LidarPropMaxAngle, float(self._cfg.max_angle_deg))
        laser.setlidaropt(ydlidar.LidarPropMinAngle, float(self._cfg.min_angle_deg))
        laser.setlidaropt(ydlidar.LidarPropMaxRange, float(self._cfg.range_max_m))
        laser.setlidaropt(ydlidar.LidarPropMinRange, float(self._cfg.range_min_m))
        laser.setlidaropt(ydlidar.LidarPropIntenstiy, bool(self._cfg.intensity_enabled))
        if not laser.initialize():
            raise RuntimeError("ydlidar initialize failed")
        return laser, port
