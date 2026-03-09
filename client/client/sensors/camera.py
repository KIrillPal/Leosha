from __future__ import annotations

import io
import logging
import threading
from time import monotonic, monotonic_ns, sleep

from .base import HealthcheckResult

LOGGER = logging.getLogger(__name__)

# Encode RGB numpy array to JPEG bytes (RGB order preserved for browser display).
def _encode_rgb_jpeg(frame_rgb, quality: int) -> bytes:
    from PIL import Image
    img = Image.fromarray(frame_rgb, mode="RGB")
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=quality)
    return buf.getvalue()

class CameraSensorThread(threading.Thread):
    """Поток реальной камеры IMX290 через picamera2."""

    def __init__(self, camera_cfg, sensor_hub, stats, statuses, stop_event) -> None:
        super().__init__(daemon=True)
        self._cfg = camera_cfg
        self._hub = sensor_hub
        self._stats = stats
        self._statuses = statuses
        self._stop_event = stop_event
        self._camera = None
        self._failures = 0

    def healthcheck(self, timeout_sec: float | None = None) -> HealthcheckResult:
        sensor_name = "camera"
        if not self._cfg.enabled:
            return HealthcheckResult(sensor_name=sensor_name, ok=False, latency_ms=0.0, reason="disabled_in_config")
        timeout = float(timeout_sec or self._cfg.healthcheck_timeout_sec)
        t_start = monotonic()
        camera = None
        try:
            from picamera2 import Picamera2

            tuning = (self._cfg.tuning_file or "").strip() or None
            if tuning:
                LOGGER.info("Camera healthcheck using tuning_file=%s", tuning)
            camera = Picamera2(tuning=tuning)
            config = camera.create_video_configuration(
                main={"size": tuple(self._cfg.resolution), "format": "RGB888"},
                buffer_count=2,
            )
            camera.configure(config)
            camera.set_controls(
                {
                    "ExposureTime": int(self._cfg.exposure_time),
                    "AnalogueGain": float(self._cfg.analogue_gain),
                    "AwbEnable": bool(self._cfg.awb_enable),
                    "AeEnable": bool(self._cfg.ae_enable),
                }
            )
            camera.start()
            last_jpeg = b""
            frames = 0
            while (monotonic() - t_start) < timeout:
                frame = camera.capture_array()
                last_jpeg = _encode_rgb_jpeg(frame, int(self._cfg.jpeg_quality))
                frames += 1
                if frames >= int(self._cfg.healthcheck_min_frames):
                    return HealthcheckResult(
                        sensor_name=sensor_name,
                        ok=True,
                        latency_ms=(monotonic() - t_start) * 1000.0,
                        details={"frame_jpeg": last_jpeg, "frames": frames, "resolution": tuple(self._cfg.resolution)},
                    )
            return HealthcheckResult(
                sensor_name=sensor_name,
                ok=False,
                latency_ms=(monotonic() - t_start) * 1000.0,
                reason="not_enough_frames",
                details={"frames": frames},
            )
        except IndexError as exc:
            return HealthcheckResult(
                sensor_name=sensor_name,
                ok=False,
                latency_ms=(monotonic() - t_start) * 1000.0,
                reason="no_camera_found (check /dev/media* permissions or connect camera)",
            )
        except Exception as exc:
            return HealthcheckResult(
                sensor_name=sensor_name,
                ok=False,
                latency_ms=(monotonic() - t_start) * 1000.0,
                reason=str(exc),
            )
        finally:
            if camera is not None:
                try:
                    camera.stop()
                except Exception:
                    pass

    def run(self) -> None:
        name = "camera"
        if not self._cfg.enabled:
            self._statuses.mark_disabled(name, "disabled_in_config")
            LOGGER.info("Camera sensor disabled by config")
            return
        try:
            from picamera2 import Picamera2

            tuning = (self._cfg.tuning_file or "").strip() or None
            if tuning:
                LOGGER.info("Camera using tuning_file=%s", tuning)
            self._camera = Picamera2(tuning=tuning)
            config = self._camera.create_video_configuration(
                main={"size": tuple(self._cfg.resolution), "format": "RGB888"},
                buffer_count=2,
            )
            self._camera.configure(config)
            self._camera.set_controls(
                {
                    "ExposureTime": int(self._cfg.exposure_time),
                    "AnalogueGain": float(self._cfg.analogue_gain),
                    "AwbEnable": bool(self._cfg.awb_enable),
                    "AeEnable": bool(self._cfg.ae_enable),
                }
            )
            self._camera.start()
        except Exception as exc:
            self._statuses.mark_failure(name, f"init_failed: {exc}")
            LOGGER.error("Camera initialization failed: %s", exc)
            if self._cfg.fail_policy.auto_disable_on_fail:
                self._statuses.mark_disabled(name, "init_failed")
                LOGGER.warning("Camera sensor auto-disabled after init failure")
            return

        period = 1.0 / max(1.0, float(self._cfg.fps))
        while not self._stop_event.is_set():
            cycle_start = monotonic_ns()
            read_start = monotonic_ns()
            try:
                frame = self._camera.capture_array()
                jpeg_bytes = _encode_rgb_jpeg(frame, int(self._cfg.jpeg_quality))
                read_end = monotonic_ns()
                self._hub.put_frame(jpeg_bytes, read_end)
                self._statuses.mark_ok(name)
                self._failures = 0
                self._stats.record(name, read_end - read_start, monotonic_ns() - cycle_start)
            except Exception as exc:
                self._failures += 1
                self._statuses.mark_failure(name, str(exc))
                self._stats.record(name, monotonic_ns() - read_start, monotonic_ns() - cycle_start, is_error=True)
                if self._failures >= self._cfg.fail_policy.max_consecutive_failures and self._cfg.fail_policy.auto_disable_on_fail:
                    self._statuses.mark_disabled(name, "too_many_failures")
                    LOGGER.error("Camera sensor auto-disabled after %d consecutive failures", self._failures)
                    break
                sleep(max(0.05, float(self._cfg.fail_policy.retry_interval_sec)))
                continue
            sleep(max(0.0, period - (monotonic_ns() - cycle_start) / 1e9))

        if self._camera is not None:
            try:
                self._camera.stop()
            except Exception:
                pass
        LOGGER.info("Camera sensor thread stopped")
