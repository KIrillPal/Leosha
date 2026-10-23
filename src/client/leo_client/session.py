"""One session: handshake, then apply commands and publish sensors. No mode lives here."""

from __future__ import annotations

import logging
import threading
import time

from leo_client.calibration import (
    pan_command_deg,
    pan_servo_deg,
    steering_throttle,
    throttle_from_speed,
    tilt_command_deg,
    tilt_servo_deg,
)
from leo_client.link import ClientLink
from leo_client.rates import Rate
from leo_client.watchdog import Watchdog
from leo_protocol.protocol import Command, Hello, Telemetry

LOGGER = logging.getLogger("leo.client")


class _Route:
    def __init__(self, steps: list[dict]) -> None:
        self._steps = list(steps)
        self._index = 0
        self._remaining = float(steps[0]["duration_sec"]) if steps else 0.0

    def sample(self, dt: float) -> tuple[float, float]:
        self._remaining -= dt
        while self._index < len(self._steps) and self._remaining <= 0.0:
            self._index += 1
            if self._index < len(self._steps):
                self._remaining += float(self._steps[self._index]["duration_sec"])
        if self._index >= len(self._steps):
            return 0.0, 0.0
        step = self._steps[self._index]
        return float(step["speed_mps"]), float(step["steering_rad"])

    @property
    def done(self) -> bool:
        return self._index >= len(self._steps)


class Session:
    def __init__(self, link: ClientLink, client_cfg: dict, camera, lidar, servos, route: _Route | None) -> None:
        self._link = link
        self._cfg = client_cfg
        self._cal = client_cfg["calibration"]
        self._camera = camera
        self._lidar = lidar
        self._servos = servos
        self._route = route
        watchdog = client_cfg["watchdog"]
        self._watchdog = Watchdog(watchdog["timeout_ms"], watchdog["critical_ms"])
        self._stop = threading.Event()
        self._last_command_at = time.monotonic()
        self._have_command = False
        self._command_speed = 0.0
        self._command_steer = 0.0
        self._held_pan = 0.0
        self._held_tilt = 0.0
        self._applied_speed = 0.0
        self._applied_steer = 0.0
        self._seq = 0
        self._sent_scan_stamp = 0
        self._sent_frame_stamp = 0
        self._loop_rate = Rate()
        self._camera_rate = Rate()
        self._lidar_rate = Rate()
        self._telemetry_rate = Rate()
        self._lock = threading.Lock()

    def run(self, duration_sec: float = 0.0) -> None:
        self._camera.start()
        self._lidar.start()
        self._servos.start()
        command_thread = threading.Thread(target=self._command_loop, name="commands", daemon=True)
        telemetry_thread = threading.Thread(target=self._telemetry_loop, name="telemetry", daemon=True)
        command_thread.start()
        telemetry_thread.start()
        deadline = time.monotonic() + duration_sec if duration_sec > 0.0 else None
        try:
            while not self._stop.is_set():
                if deadline is not None and time.monotonic() >= deadline:
                    break
                if self._route is not None and self._route.done:
                    break
                time.sleep(0.05)
        finally:
            self._stop.set()
            command_thread.join(timeout=1.0)
            telemetry_thread.join(timeout=1.0)
            self._servos.close()
            self._camera.close()
            self._lidar.close()

    def _command_loop(self) -> None:
        period = 1.0 / float(self._cfg["command_hz"])
        neutral = throttle_from_speed(0.0, self._cal)
        while not self._stop.is_set():
            started = time.monotonic()
            command = self._link.poll_command()
            self._apply(command, period, neutral)
            self._loop_rate.tick()
            time.sleep(max(0.0, period - (time.monotonic() - started)))

    def _apply(self, command: Command | None, dt: float, neutral: float) -> None:
        if command is not None:
            self._last_command_at = time.monotonic()
            self._have_command = True
            self._command_speed = command.speed_mps
            self._command_steer = command.steering_rad
            self._held_pan = command.head_pan_deg
            self._held_tilt = command.head_tilt_deg
        if self._route is None and not self._have_command:
            self._servos.apply(neutral, 0.0, pan_servo_deg(0.0, self._cal), tilt_servo_deg(0.0, self._cal))
            return
        speed, steer = self._command_speed, self._command_steer
        pan, tilt = self._held_pan, self._held_tilt
        if self._route is not None:
            speed, steer = self._route.sample(dt)
        else:
            elapsed_ms = (time.monotonic() - self._last_command_at) * 1000.0
            if self._watchdog.drive_cut(elapsed_ms):
                speed, steer = 0.0, 0.0
            else:
                speed *= self._watchdog.speed_scale(elapsed_ms)
        self._applied_speed, self._applied_steer = speed, steer
        self._servos.apply(
            throttle_from_speed(speed, self._cal),
            steering_throttle(steer, self._cal),
            pan_servo_deg(pan, self._cal),
            tilt_servo_deg(tilt, self._cal),
        )

    def _telemetry_loop(self) -> None:
        period = 1.0 / float(self._cfg["telemetry_hz"])
        while not self._stop.is_set():
            started = time.monotonic()
            self._send()
            time.sleep(max(0.0, period - (time.monotonic() - started)))

    def _send(self) -> None:
        scan = self._lidar.latest()
        jpeg, frame_stamp, width, height = self._camera.latest()
        send_scan = scan if scan is not None and scan.capture_time_ns != self._sent_scan_stamp else None
        send_jpeg = jpeg if jpeg and frame_stamp != self._sent_frame_stamp else b""
        if send_scan is not None:
            self._sent_scan_stamp = send_scan.capture_time_ns
            self._lidar_rate.tick()
        if send_jpeg:
            self._sent_frame_stamp = frame_stamp
            self._camera_rate.tick()
        pan = pan_command_deg(self._servos.pan_servo_deg, self._cal)
        tilt = tilt_command_deg(self._servos.tilt_servo_deg, self._cal)
        truth = None
        ground = getattr(self._servos, "ground_truth", None)
        if ground is not None:
            truth = ground()
        self._seq += 1
        packet = Telemetry(
            seq=self._seq,
            client_time_ns=time.monotonic_ns(),
            scan=send_scan,
            jpeg=send_jpeg,
            frame_time_ns=frame_stamp,
            frame_width=width,
            frame_height=height,
            head_pan_deg=pan,
            head_tilt_deg=tilt,
            steering_rad=self._applied_steer,
            speed_mps=self._applied_speed,
            gyro_z=None,
            fps_loop=self._loop_rate.value,
            fps_camera=self._camera_rate.value,
            fps_lidar=self._lidar_rate.value,
            fps_telemetry=self._telemetry_rate.tick(),
            ground_truth=truth,
        )
        try:
            self._link.send(packet)
        except Exception:
            LOGGER.debug("telemetry dropped", exc_info=True)


def open_session(server: str, sim_world: str, show: bool, route: bool, duration_sec: float) -> None:
    link = ClientLink(server)
    try:
        reply = link.handshake(Hello(client_time_ns=time.monotonic_ns(), sim=bool(sim_world), world=sim_world))
        client_cfg = reply["client"]
        network_ports = reply["ports"]
        link.open_streams(int(network_ports["telemetry_port"]), int(network_ports["command_port"]))
        camera, lidar, servos, body = _devices(sim_world, reply.get("sim"), client_cfg, show)
        played = _Route(reply["sim"]["route"]) if route and reply.get("sim") else None
        if body is not None:
            body.start()
        try:
            Session(link, client_cfg, camera, lidar, servos, played).run(duration_sec)
        finally:
            if body is not None:
                body.close()
    finally:
        link.close()


def _devices(sim_world: str, sim_cfg: dict | None, client_cfg: dict, show: bool):
    if sim_world:
        if sim_cfg is None:
            raise RuntimeError("server did not send a sim world")
        from leo_client.sim.hardware import build_sim

        camera, lidar, servos, body = build_sim(sim_cfg, client_cfg["calibration"], show)
        return camera, lidar, servos, body
    from leo_client.hardware.camera import Camera
    from leo_client.hardware.lidar import Lidar
    from leo_client.hardware.servos import Servos

    return (
        Camera(client_cfg["camera"]),
        Lidar(client_cfg["lidar"]),
        Servos(client_cfg["servos"], client_cfg["calibration"]),
        None,
    )
