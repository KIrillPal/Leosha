"""Picamera2 capture thread. Frames are JPEG bytes plus a capture timestamp."""

from __future__ import annotations

import logging
import threading
import time

LOGGER = logging.getLogger("leo.client.camera")


class Camera:
    def __init__(self, cfg: dict) -> None:
        self._cfg = cfg
        self._lock = threading.Lock()
        self._jpeg = b""
        self._stamp_ns = 0
        self._width = int(cfg["width"])
        self._height = int(cfg["height"])
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if not self._cfg["enabled"]:
            return
        self._thread = threading.Thread(target=self._run, name="camera", daemon=True)
        self._thread.start()

    def latest(self) -> tuple[bytes, int, int, int]:
        with self._lock:
            return self._jpeg, self._stamp_ns, self._width, self._height

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        from picamera2 import Picamera2

        camera = Picamera2(tuning=self._cfg["tuning_file"] or None)
        config = camera.create_video_configuration(
            main={"size": (self._width, self._height), "format": "RGB888"},
            buffer_count=2,
        )
        camera.configure(config)
        camera.set_controls(
            {
                "ExposureTime": int(self._cfg["exposure_time"]),
                "AnalogueGain": float(self._cfg["analogue_gain"]),
                "AwbEnable": bool(self._cfg["awb_enable"]),
                "AeEnable": bool(self._cfg["ae_enable"]),
            }
        )
        camera.start()
        period = 1.0 / float(self._cfg["fps"])
        quality = int(self._cfg["jpeg_quality"])
        try:
            import cv2

            while not self._stop.is_set():
                started = time.monotonic()
                frame = camera.capture_array()
                ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
                if ok:
                    with self._lock:
                        self._jpeg = encoded.tobytes()
                        self._stamp_ns = time.monotonic_ns()
                time.sleep(max(0.0, period - (time.monotonic() - started)))
        except Exception:
            LOGGER.exception("camera stopped")
        finally:
            camera.stop()
