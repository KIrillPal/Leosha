"""Mock robot. The session drives it through the same servo calls as the real hardware."""

from __future__ import annotations

import math
import threading
import time

import numpy as np

from leo_client.calibration import pan_command_deg, throttle_from_speed, tilt_command_deg
from leo_client.hardware.scan_grid import apply_blind_zone
from leo_client.sim.camera_render import render
from leo_client.sim.lidar_sim import cast_ranges
from leo_client.sim.physics import step_pose
from leo_protocol.protocol import Scan


class _Body:
    def __init__(self, sim_cfg: dict, calibration: dict) -> None:
        world = sim_cfg["world"]
        self.cfg = sim_cfg
        self.calibration = calibration
        self.lock = threading.Lock()
        self.x = float(world["start_x"])
        self.y = float(world["start_y"])
        self.yaw = float(world["start_yaw"])
        self.motor = throttle_from_speed(0.0, calibration)
        self.steer = 0.0
        self.pan_servo = float(calibration["head_pan_zero_deg"])
        self.tilt_servo = float(calibration["head_tilt_zero_deg"])
        self.speed = 0.0
        self.stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="sim-physics", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self.stop.set()
        self._thread.join(timeout=1.0)

    def _run(self) -> None:
        hz = float(self.cfg["physics_hz"])
        dt = 1.0 / hz
        while not self.stop.is_set():
            started = time.monotonic()
            with self.lock:
                self.x, self.y, self.yaw, self.speed, _steer = step_pose(
                    self.x,
                    self.y,
                    self.yaw,
                    self.motor,
                    self.steer,
                    dt,
                    float(self.cfg["wheelbase_m"]),
                    self.calibration,
                    float(self.cfg["slip"]),
                )
            time.sleep(max(0.0, dt - (time.monotonic() - started)))

    def pose(self) -> tuple[float, float, float]:
        with self.lock:
            return self.x, self.y, self.yaw

    def look_deg(self) -> tuple[float, float]:
        with self.lock:
            pan = pan_command_deg(self.pan_servo, self.calibration)
            tilt = tilt_command_deg(self.tilt_servo, self.calibration)
        return pan, tilt


class SimServos:
    def __init__(self, body: _Body) -> None:
        self._body = body
        self.motor_throttle = body.motor
        self.steering_throttle = 0.0
        self.pan_servo_deg = body.pan_servo
        self.tilt_servo_deg = body.tilt_servo

    def start(self) -> None:
        return None

    def apply(self, motor_throttle: float, steering_throttle: float, pan_servo_deg: float, tilt_servo_deg: float) -> None:
        self.motor_throttle = float(motor_throttle)
        self.steering_throttle = float(steering_throttle)
        self.pan_servo_deg = float(pan_servo_deg)
        self.tilt_servo_deg = float(tilt_servo_deg)
        with self._body.lock:
            self._body.motor = self.motor_throttle
            self._body.steer = self.steering_throttle
            self._body.pan_servo = self.pan_servo_deg
            self._body.tilt_servo = self.tilt_servo_deg

    def ground_truth(self) -> tuple[float, float, float]:
        return self._body.pose()

    def close(self) -> None:
        return None


class SimLidar:
    def __init__(self, body: _Body) -> None:
        self._body = body
        self._lock = threading.Lock()
        self._scan: Scan | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="sim-lidar", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def latest(self) -> Scan | None:
        with self._lock:
            return self._scan

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)

    def _run(self) -> None:
        lidar = self._body.cfg["lidar"]
        period = 1.0 / float(lidar["scan_hz"])
        rays = int(lidar["num_rays"])
        while not self._stop.is_set():
            started = time.monotonic()
            x, y, yaw = self._body.pose()
            ranges = cast_ranges(x, y, yaw, rays, float(lidar["range_max_m"]), self._body.cfg["world"])
            noise = float(self._body.cfg["lidar_noise_m"])
            if noise:
                ranges = ranges + np.random.normal(0.0, noise, size=ranges.shape).astype(np.float32)
            angle_min = -math.pi
            increment = (2.0 * math.pi) / float(rays)
            ranges = apply_blind_zone(
                ranges,
                angle_min,
                increment,
                float(lidar["blind_start_deg"]),
                float(lidar["blind_end_deg"]),
                float(lidar["blind_max_m"]),
            )
            scan = Scan(
                capture_time_ns=time.monotonic_ns(),
                scan_time_sec=period,
                angle_min=angle_min,
                angle_increment=increment,
                range_min=float(lidar["range_min_m"]),
                range_max=float(lidar["range_max_m"]),
                ranges=ranges.astype(np.float32),
            )
            with self._lock:
                self._scan = scan
            time.sleep(max(0.0, period - (time.monotonic() - started)))


class SimCamera:
    def __init__(self, body: _Body) -> None:
        self._body = body
        self._lock = threading.Lock()
        self._jpeg = b""
        self._stamp_ns = 0
        self._width = int(body.cfg["camera_width"])
        self._height = int(body.cfg["camera_height"])
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="sim-camera", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def latest(self) -> tuple[bytes, int, int, int]:
        with self._lock:
            return self._jpeg, self._stamp_ns, self._width, self._height

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)

    def _run(self) -> None:
        import cv2

        period = 1.0 / float(self._body.cfg["camera_fps"])
        quality = int(self._body.cfg["jpeg_quality"])
        started_at = time.monotonic()
        while not self._stop.is_set():
            started = time.monotonic()
            pan, tilt = self._body.look_deg()
            frame = render(self._body.pose(), pan, tilt, self._body.cfg, time.monotonic() - started_at)
            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
            if ok:
                with self._lock:
                    self._jpeg = encoded.tobytes()
                    self._stamp_ns = time.monotonic_ns()
            time.sleep(max(0.0, period - (time.monotonic() - started)))


def build_sim(sim_cfg: dict, calibration: dict, show: bool):
    body = _Body(sim_cfg, calibration)
    if show:
        from leo_client.sim.view import start_view

        start_view(body)
    return SimCamera(body), SimLidar(body), SimServos(body), body
