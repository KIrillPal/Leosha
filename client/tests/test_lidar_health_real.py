from __future__ import annotations

from pathlib import Path

import pytest

from client.sensor_hub import SensorHub
from client.sensors.lidar_sensor import TMiniProPlusLidarThread
from client.sensors.status_registry import SensorStatusRegistry
from client.stats import StatsCollector

from ._png_utils import render_lidar_map_png


@pytest.mark.hardware
def test_lidar_healthcheck_real(require_hardware, hardware_config):
    cfg = hardware_config.sensors.lidar
    if not cfg.enabled:
        pytest.skip("lidar disabled in hardware config")
    statuses = SensorStatusRegistry()
    statuses.register("lidar", True)
    stats = StatsCollector()
    stats.register("lidar", cfg.scan_hz)
    thread = TMiniProPlusLidarThread(
        cfg,
        hardware_config.robot_geometry,
        SensorHub(),
        stats,
        statuses,
        stop_event=__import__("threading").Event(),
    )
    result = thread.healthcheck()
    assert result.ok, f"lidar healthcheck failed: {result.reason}"
    scan = result.details.get("scan")
    assert scan is not None
    assert len(scan.ranges) >= cfg.healthcheck_min_points

    artifact = Path(__file__).resolve().parent / "artifacts" / "lidar_map_real.png"
    render_lidar_map_png(artifact, scan.ranges, scan.angle_min, scan.angle_increment)
    assert artifact.exists()
    assert artifact.stat().st_size > 100

