from __future__ import annotations

import threading
from pathlib import Path

import pytest

from client.sensor_hub import SensorHub
from client.sensors.camera_sensor import CameraSensorThread
from client.sensors.status_registry import SensorStatusRegistry
from client.stats import StatsCollector

from ._png_utils import write_png_rgb


@pytest.mark.hardware
def test_camera_healthcheck_real(require_hardware, hardware_config):
    cfg = hardware_config.sensors.camera
    if not cfg.enabled:
        pytest.skip("camera disabled in hardware config")
    test_timeout = float(cfg.healthcheck_test_timeout_sec)
    statuses = SensorStatusRegistry()
    statuses.register("camera", True)
    stats = StatsCollector()
    stats.register("camera", cfg.fps)
    thread = CameraSensorThread(cfg, SensorHub(), stats, statuses, stop_event=threading.Event())
    result_holder = []

    def run_healthcheck():
        result_holder.append(thread.healthcheck(timeout_sec=float(cfg.healthcheck_timeout_sec)))

    t = threading.Thread(target=run_healthcheck, daemon=True)
    t.start()
    t.join(timeout=test_timeout)
    if not result_holder:
        pytest.fail(f"camera healthcheck did not finish within {test_timeout}s (init or capture blocked)")
    result = result_holder[0]
    assert result.ok, f"camera healthcheck failed: {result.reason}"
    frame_jpeg = result.details.get("frame_jpeg", b"")
    assert isinstance(frame_jpeg, (bytes, bytearray)) and len(frame_jpeg) > 16

    try:
        import cv2
        import numpy as np
    except Exception:
        pytest.skip("cv2/numpy required to decode camera jpeg artifact")
    arr = np.frombuffer(frame_jpeg, dtype=np.uint8)
    image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    assert image is not None and image.size > 0
    h, w = image.shape[:2]
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    artifact = Path(__file__).resolve().parent / "artifacts" / "camera_frame_real.png"
    write_png_rgb(artifact, w, h, rgb.tobytes())
    assert artifact.exists()
    assert artifact.stat().st_size > 100

