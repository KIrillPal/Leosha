"""One session: find the server, then apply commands and publish sensors.

Silence longer than ``waiting_timeout_sec`` is ``waiting``. The last successful
address is probed for ``known_ip_sec``. The whole search stays inside
``wait_limit_sec``. Every exit returns each actuator to its standard position
on a two-second sine ramp.
"""

from __future__ import annotations

import logging
import os
import threading
import time
from datetime import datetime
from pathlib import Path

from leo_client.addresses import AddressBook, default_book_path, plan_targets
from leo_protocol import console
from leo_protocol.duckdns import is_ipv4
from leo_client.calibration import (
    pan_command_deg,
    pan_servo_deg,
    steering_throttle,
    throttle_from_speed,
    tilt_command_deg,
    tilt_servo_deg,
)
from leo_client.link import ClientLink, local_ipv4_toward
from leo_client.modules import CAMERA, LIDAR, MOTOR, WHEEL_SERVO, format_modules, fresh
from leo_client.rates import Rate
from leo_client.watchdog import Watchdog
from leo_protocol.protocol import Command, Hello, Telemetry

LOGGER = logging.getLogger("leo.client")

_CONNECTED = "connected"
_WAITING = "waiting"


def link_timing(path: str | Path | None = None) -> tuple[float, float, float]:
    """Return the silence limit, the last-address probe, and the search budget."""
    if path:
        chosen = Path(path)
    else:
        env = os.environ.get("LEO_CONFIG", "").strip()
        chosen = Path(env) if env else Path(__file__).resolve().parents[3] / "config" / "robot.yaml"
    import yaml

    client = yaml.safe_load(chosen.read_text(encoding="utf-8"))["client"]
    waiting = float(client["waiting_timeout_sec"])
    known = float(client["known_ip_sec"])
    limit = float(client["wait_limit_sec"])
    if waiting <= 0 or known <= 0 or limit <= 0:
        raise ValueError("client search times must be positive")
    return waiting, known, limit


def attempt_timeouts(count: int, budget_sec: float, first_sec: float | None = None) -> list[float]:
    """Split ``budget_sec`` across ``count`` dials. The first dial can be shorter."""
    if count <= 0 or budget_sec <= 0:
        return []
    if first_sec is None:
        share = budget_sec / count
        return [share] * count
    first = min(float(first_sec), budget_sec)
    if count == 1:
        return [first]
    share = max(0.0, budget_sec - first) / (count - 1)
    return [first, *([share] * (count - 1))]


class LinkPhase:
    """Connected, then waiting, then a request to search again."""

    def __init__(self, waiting_timeout_sec: float, known_ip_sec: float, clock=time.monotonic) -> None:
        self.phase = _CONNECTED
        self._waiting_timeout_sec = float(waiting_timeout_sec)
        self._known_ip_sec = float(known_ip_sec)
        self._clock = clock
        self._last_heard = clock()
        self._waiting_since = 0.0

    def heard(self) -> str | None:
        now = self._clock()
        self._last_heard = now
        if self.phase != _WAITING:
            return None
        self.phase = _CONNECTED
        self._waiting_since = 0.0
        return "restored"

    def poll(self) -> str | None:
        now = self._clock()
        if self.phase == _CONNECTED and now - self._last_heard >= self._waiting_timeout_sec:
            self.phase = _WAITING
            self._waiting_since = now
            return "lost"
        if self.phase == _WAITING and now - self._waiting_since >= self._known_ip_sec:
            self.phase = "searching"
            return "search"
        return None


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


class LostWait:
    """One ``Lost connection`` line, rewritten in place while the search runs."""

    def __init__(self) -> None:
        self.host = ""
        self.active = False
        self._since = 0.0
        self._stamp: datetime | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def elapsed(self) -> float:
        if not self.active:
            return 0.0
        return time.monotonic() - self._since

    def start(self, host: str) -> None:
        if self.active:
            return
        self.host = host
        self._since = time.monotonic()
        self._stamp = datetime.now()
        self._stop.clear()
        self.active = True
        self._thread = threading.Thread(target=self._run, name="lost-wait", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if not self.active:
            return
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        console.close_line()
        self.active = False

    def _run(self) -> None:
        while True:
            elapsed = time.monotonic() - self._since
            console.rewrite(console.lost_line(elapsed, now=self._stamp))
            if self._stop.wait(0.1):
                return


class Session:
    def __init__(
        self,
        link: ClientLink,
        client_cfg: dict,
        camera,
        lidar,
        servos,
        route: _Route | None,
        server_ip: str = "",
        lost_wait: LostWait | None = None,
    ) -> None:
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
        self._phase = LinkPhase(float(client_cfg["waiting_timeout_sec"]), float(client_cfg["known_ip_sec"]))
        self._server_ip = server_ip
        self._lost_wait = lost_wait if lost_wait is not None else LostWait()
        self._search = False
        self._modules: dict[str, bool] = {}
        self._checked_at = 0.0

    def run(self, duration_sec: float = 0.0) -> str:
        command_thread = None
        telemetry_thread = None
        deadline = time.monotonic() + duration_sec if duration_sec > 0.0 else None
        try:
            self._camera.start()
            self._lidar.start()
            self._servos.start()
            self._report_modules()
            command_thread = threading.Thread(target=self._command_loop, name="commands", daemon=True)
            telemetry_thread = threading.Thread(target=self._telemetry_loop, name="telemetry", daemon=True)
            command_thread.start()
            telemetry_thread.start()
            while not self._stop.is_set():
                if deadline is not None and time.monotonic() >= deadline:
                    break
                if self._route is not None and self._route.done:
                    break
                time.sleep(0.05)
        finally:
            self._stop.set()
            if command_thread is not None:
                command_thread.join(timeout=1.0)
            if telemetry_thread is not None:
                telemetry_thread.join(timeout=1.0)
            self._servos.close()
            self._camera.close()
            self._lidar.close()
        return "search" if self._search else "done"

    def _command_loop(self) -> None:
        period = 1.0 / float(self._cfg["command_hz"])
        neutral = throttle_from_speed(0.0, self._cal)
        while not self._stop.is_set():
            started = time.monotonic()
            command = self._link.poll_command()
            self._watch_link(command)
            self._apply(command, period, neutral)
            self._loop_rate.tick()
            time.sleep(max(0.0, period - (time.monotonic() - started)))

    def _watch_link(self, command: Command | None) -> None:
        if command is not None:
            if self._phase.heard() == "restored":
                LOGGER.info("server link restored")
                self._lost_wait.stop()
                if self._server_ip:
                    console.write(console.connected_line(self._server_ip))
            return
        event = self._phase.poll()
        if event == "lost":
            LOGGER.warning("server link lost; waiting")
            self._lost_wait.start(self._server_ip)
            return
        if event == "search":
            LOGGER.warning("waiting expired; searching")
            self._search = True
            self._stop.set()

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
        actuators = self._servo_states()
        wheel_down = actuators.get(WHEEL_SERVO) is False
        if actuators.get(MOTOR) is False:
            speed = 0.0
        if wheel_down:
            steer = 0.0
        self._applied_speed, self._applied_steer = speed, steer
        steer_throttle = 0.0 if wheel_down else steering_throttle(steer, self._cal)
        self._servos.apply(
            throttle_from_speed(speed, self._cal),
            steer_throttle,
            pan_servo_deg(pan, self._cal),
            tilt_servo_deg(tilt, self._cal),
        )

    def _report_modules(self) -> None:
        self._modules = self._module_states()
        self._checked_at = time.monotonic()
        text = format_modules(self._modules)
        if text:
            console.write(text)

    def _refresh_modules(self, scan, jpeg: bytes, frame_stamp: int) -> None:
        if self._should_probe(scan, jpeg, frame_stamp):
            self._modules = self._module_states()
            self._checked_at = time.monotonic()
            return
        self._modules.update(self._servo_states())

    def _should_probe(self, scan, jpeg: bytes, frame_stamp: int) -> bool:
        if self._modules_due() or self._lidar_stale(scan) or self._camera_stale(jpeg, frame_stamp):
            return True
        return self._modules.get(LIDAR) is False or self._modules.get(CAMERA) is False

    def _modules_due(self) -> bool:
        interval = float(self._cfg["module_check_sec"])
        return time.monotonic() - self._checked_at >= interval

    def _module_states(self) -> dict[str, bool]:
        states: dict[str, bool] = {}
        if self._sensor_on(LIDAR):
            states[LIDAR] = bool(self._lidar.check())
        if self._sensor_on(CAMERA):
            states[CAMERA] = bool(self._camera.check())
        states.update(self._servo_states())
        return states

    def _servo_states(self) -> dict[str, bool]:
        check = getattr(self._servos, "check", None)
        if not callable(check):
            return {}
        states = check()
        if not isinstance(states, dict):
            return {}
        return {str(name): bool(ok) for name, ok in states.items()}

    def _sensor_on(self, name: str) -> bool:
        device = self._lidar if name == LIDAR else self._camera
        section = self._cfg.get(name, {})
        enabled = True if "enabled" not in section else bool(section["enabled"])
        return enabled and callable(getattr(device, "check", None))

    def _lidar_stale(self, scan) -> bool:
        if not self._sensor_on(LIDAR):
            return False
        stamp = 0 if scan is None else int(scan.capture_time_ns)
        period = 1.0 / float(self._cfg["lidar"]["scan_hz"])
        return not fresh(stamp, period)

    def _camera_stale(self, jpeg: bytes, frame_stamp: int) -> bool:
        if not self._sensor_on(CAMERA):
            return False
        if not jpeg:
            return True
        period = 1.0 / float(self._cfg["camera"]["fps"])
        return not fresh(int(frame_stamp), period)

    def _telemetry_loop(self) -> None:
        period = 1.0 / float(self._cfg["telemetry_hz"])
        while not self._stop.is_set():
            started = time.monotonic()
            self._send()
            time.sleep(max(0.0, period - (time.monotonic() - started)))

    def _send(self) -> None:
        scan = self._lidar.latest()
        jpeg, frame_stamp, width, height = self._camera.latest()
        self._refresh_modules(scan, jpeg, frame_stamp)
        if self._modules.get(LIDAR) is False:
            scan = None
        if self._modules.get(CAMERA) is False:
            jpeg = b""
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
            modules=dict(self._modules),
        )
        try:
            self._link.send(packet)
        except Exception:
            LOGGER.debug("telemetry dropped", exc_info=True)


def _hello(host: str, sim_world: str) -> Hello:
    return Hello(
        client_time_ns=time.monotonic_ns(),
        sim=bool(sim_world),
        world=sim_world,
        local_ip=local_ipv4_toward(host),
    )


def open_link(
    server: str,
    sim_world: str,
    addresses: str | None = None,
    timeout_ms: int = 3000,
    resolve=None,
    *,
    first_timeout_sec: float | None = None,
    search_budget_sec: float | None = None,
    skip_ip: str = "",
):
    chosen = server.strip()
    if chosen and is_ipv4(chosen):
        if chosen == skip_ip:
            raise RuntimeError(f"could not open a session ({chosen} already checked)")
        hello = _hello(chosen, sim_world)
        LOGGER.info("direct server %s, robot address %s", chosen, hello.local_ip or "unknown")
        dial_ms = _dial_ms(1, timeout_ms, first_timeout_sec, search_budget_sec)
        if not dial_ms:
            raise RuntimeError(f"could not open a session ({chosen} already checked)")
        link = ClientLink(chosen, dial_ms[0])
        try:
            return link, link.handshake(hello)
        except Exception:
            link.close()
            raise
    path = Path(addresses) if addresses else default_book_path()
    book = AddressBook.load(path)
    if resolve is None:
        attempts, host = plan_targets(server, book)
    else:
        attempts, host = plan_targets(server, book, resolve)
    if skip_ip:
        attempts = [item for item in attempts if item.ip != skip_ip]
    LOGGER.info("duckdns %s via %s (%s)", host, path, ", ".join(item.ip for item in attempts) or "none")
    errors: list[str] = []
    dial_ms = _dial_ms(len(attempts), timeout_ms, first_timeout_sec, search_budget_sec)
    for attempt, dial in zip(attempts, dial_ms):
        hello = _hello(attempt.ip, sim_world)
        LOGGER.info("robot address %s via %s", hello.local_ip or "unknown", attempt.ip)
        link = ClientLink(attempt.ip, dial)
        try:
            reply = link.handshake(hello)
        except Exception as exc:
            errors.append(f"{attempt.ip}: {exc}")
            LOGGER.warning("handshake %s failed: %s", attempt.ip, exc)
            link.close()
            continue
        try:
            book.remember(attempt.local_ip, attempt.global_ip, domain=host if not book.domain else None)
        except Exception:
            link.close()
            raise
        LOGGER.info("saved server local %s global %s to %s", attempt.local_ip, attempt.global_ip, book.path)
        return link, reply
    detail = "; ".join(errors) if errors else "no addresses"
    raise RuntimeError(f"could not open a session ({detail})")


def _dial_ms(count: int, timeout_ms: int, first_sec: float | None, budget_sec: float | None) -> list[int]:
    if count <= 0 or (budget_sec is not None and budget_sec <= 0):
        return []
    if budget_sec is None:
        return [int(timeout_ms)] * count
    seconds = attempt_timeouts(count, budget_sec, first_sec)
    return [max(1, int(round(item * 1000))) for item in seconds if item > 0]


def _pause_for_budget(started: float, budget_sec: float) -> None:
    pause = budget_sec - (time.monotonic() - started)
    if pause < 0.05:
        return
    console.write(console.no_server_line(pause))
    time.sleep(pause)


def open_session(server: str, sim_world: str, show: bool, route: bool, duration_sec: float, addresses: str | None = None) -> None:
    waiting_timeout, known_ip, wait_limit = link_timing()
    lost_wait = LostWait()
    while True:
        LOGGER.info("searching")
        started = time.monotonic()
        try:
            if lost_wait.active:
                remaining = max(0.0, wait_limit - lost_wait.elapsed)
                link, reply = open_link(
                    server,
                    sim_world,
                    addresses,
                    first_timeout_sec=None,
                    search_budget_sec=remaining,
                    skip_ip=lost_wait.host,
                )
            else:
                link, reply = open_link(
                    server,
                    sim_world,
                    addresses,
                    first_timeout_sec=known_ip,
                    search_budget_sec=wait_limit,
                )
            lost_wait.stop()
            console.write(console.connected_line(link.host))
        except Exception as exc:
            if lost_wait.active:
                LOGGER.warning("search failed (%s)", exc)
                remaining = wait_limit - lost_wait.elapsed
                if remaining > 0:
                    time.sleep(remaining)
                lost_wait.stop()
            else:
                LOGGER.warning("search failed (%s); retry inside %.0fs", exc, wait_limit)
                _pause_for_budget(started, wait_limit)
            continue
        reason = "done"
        try:
            client_cfg = dict(reply["client"])
            client_cfg.setdefault("waiting_timeout_sec", waiting_timeout)
            client_cfg.setdefault("known_ip_sec", known_ip)
            client_cfg.setdefault("wait_limit_sec", wait_limit)
            waiting_timeout = float(client_cfg["waiting_timeout_sec"])
            known_ip = float(client_cfg["known_ip_sec"])
            wait_limit = float(client_cfg["wait_limit_sec"])
            network_ports = reply["ports"]
            link.open_streams(int(network_ports["telemetry_port"]), int(network_ports["command_port"]))
            camera, lidar, servos, body = _devices(sim_world, reply.get("sim"), client_cfg, show)
            played = _Route(reply["sim"]["route"]) if route and reply.get("sim") else None
            if body is not None:
                body.start()
            try:
                reason = Session(
                    link, client_cfg, camera, lidar, servos, played, link.host, lost_wait
                ).run(duration_sec)
            finally:
                if body is not None:
                    body.close()
        finally:
            link.close()
        if reason != "search":
            lost_wait.stop()
            return


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
