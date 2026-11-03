import time
from pathlib import Path

import numpy as np

from leo_client.calibration import throttle_from_speed
from leo_client.hardware.camera import Camera
from leo_client.hardware.lidar import Lidar
from leo_client.hardware.servos import Servos
from leo_client.modules import (
    ACTUATORS,
    CAMERA,
    FACE_SERVO,
    LIDAR,
    MOTOR,
    NECK_SERVO,
    WHEEL_SERVO,
    format_modules,
    status_line,
)
from leo_client.session import Session
from leo_protocol.protocol import Scan
from leo_server.common.config import load_config

CFG = load_config(Path(__file__).resolve().parents[3] / "config" / "robot.yaml").client
CAL = CFG["calibration"]
NEUTRAL = throttle_from_speed(0.0, CAL)


def _scan(stamp_ns: int) -> Scan:
    ranges = np.array([1.0], dtype=np.float32)
    return Scan(stamp_ns, 0.1, -np.pi, 0.02, 0.02, 6.0, ranges)


def test_status_lines_color_ok_and_failed():
    assert status_line(LIDAR, True) == "\033[32mOK\033[0m lidar"
    assert status_line(NECK_SERVO, False) == "\033[31mFAILED\033[0m neck servo"
    text = format_modules({LIDAR: True, CAMERA: True, NECK_SERVO: False})
    assert text == "\n".join(
        (
            status_line(LIDAR, True),
            status_line(CAMERA, True),
            status_line(NECK_SERVO, False),
        )
    )


def test_lidar_check_follows_scan_age():
    lidar = Lidar(CFG["lidar"])
    assert lidar.check() is False
    lidar._scan = _scan(time.monotonic_ns())
    assert lidar.check() is True
    lidar._scan = _scan(time.monotonic_ns() - 10_000_000_000)
    assert lidar.check() is False


def test_camera_check_follows_frame_age():
    camera = Camera(CFG["camera"])
    assert camera.check() is False
    camera._jpeg = b"jpeg"
    camera._stamp_ns = time.monotonic_ns()
    assert camera.check() is True
    camera._stamp_ns = time.monotonic_ns() - 10_000_000_000
    assert camera.check() is False


def test_disabled_lidar_is_left_out_of_the_report():
    cfg = dict(CFG)
    cfg["lidar"] = dict(CFG["lidar"])
    cfg["lidar"]["enabled"] = False
    lidar = Lidar(cfg["lidar"])
    lidar._scan = _scan(time.monotonic_ns())
    session = Session(None, cfg, Camera(cfg["camera"]), lidar, _Board(), None)
    assert LIDAR not in session._module_states()


class _Throttle:
    def __init__(self) -> None:
        self.throttle = None


class _Angle:
    def __init__(self, fail: bool = False) -> None:
        self.angle = None
        self.fail = fail
        self.writes = 0

    def set_angle(self, value: float) -> None:
        self.writes += 1
        if self.fail:
            raise OSError("i2c")
        self.angle = value


class _Output:
    def __init__(self, angle: _Angle | None = None) -> None:
        self.throttle = None
        self._angle = angle or _Angle()

    @property
    def angle(self):
        return self._angle.angle

    @angle.setter
    def angle(self, value):
        self._angle.set_angle(value)


class _Kit:
    def __init__(self, neck: _Angle) -> None:
        self.continuous_servo = {0: _Throttle(), 1: _Throttle()}
        self.servo = {2: _Output(neck), 3: _Output()}


def test_a_neck_write_error_fails_only_that_channel():
    neck = _Angle(fail=True)
    board = Servos(CFG["servos"], CAL)
    board._kit = _Kit(neck)
    board.apply(0.4, -0.2, 90.0, 40.0)
    assert board.check()[NECK_SERVO] is False
    assert board.check()[MOTOR] is True
    assert board.check()[WHEEL_SERVO] is True
    assert board.check()[FACE_SERVO] is True
    assert board.pan_servo_deg == CAL["head_pan_zero_deg"]
    board.apply(0.1, 0.0, 10.0, 30.0)
    assert neck.writes == 1
    assert board._kit.continuous_servo[0].throttle == 0.1


def test_board_start_failure_marks_every_actuator(monkeypatch):
    def explode(_self) -> None:
        raise OSError("no board")

    monkeypatch.setattr(Servos, "_open", explode)
    board = Servos(CFG["servos"], CAL)
    board.start()
    assert board.check() == {name: False for name in ACTUATORS}
    assert board._kit is None


class _Board:
    def __init__(self, states: dict[str, bool] | None = None) -> None:
        self.states = states or {name: True for name in ACTUATORS}
        self.args = None
        self.pan_servo_deg = CAL["head_pan_zero_deg"]
        self.tilt_servo_deg = CAL["head_tilt_zero_deg"]

    def start(self) -> None:
        return None

    def close(self) -> None:
        return None

    def check(self) -> dict[str, bool]:
        return dict(self.states)

    def apply(self, motor, steer, pan, tilt) -> None:
        self.args = (motor, steer, pan, tilt)


class _Link:
    def __init__(self) -> None:
        self.packets = []

    def poll_command(self):
        return None

    def send(self, packet) -> None:
        self.packets.append(packet)


def test_failed_motor_and_wheel_are_commanded_to_stop():
    board = _Board({MOTOR: False, WHEEL_SERVO: False, NECK_SERVO: True, FACE_SERVO: True})
    session = Session(None, CFG, None, None, board, None)
    session._have_command = True
    session._command_speed = 0.4
    session._command_steer = 0.3
    session._apply(None, 0.01, NEUTRAL)
    assert session._applied_speed == 0.0
    assert session._applied_steer == 0.0
    assert board.args[0] == NEUTRAL
    assert board.args[1] == 0.0


def test_send_reports_flags_and_drops_a_stale_lidar():
    lidar = Lidar(CFG["lidar"])
    lidar._scan = _scan(1)
    camera = Camera(CFG["camera"])
    camera._jpeg = b"jpeg"
    camera._stamp_ns = time.monotonic_ns()
    board = _Board({MOTOR: True, WHEEL_SERVO: True, NECK_SERVO: False, FACE_SERVO: True})
    link = _Link()
    session = Session(link, CFG, camera, lidar, board, None)
    session._send()
    packet = link.packets[0]
    assert packet.modules[LIDAR] is False
    assert packet.modules[CAMERA] is True
    assert packet.modules[NECK_SERVO] is False
    assert packet.scan is None
    assert packet.jpeg == b"jpeg"


class _Sensor:
    def __init__(self, ok: bool, latest) -> None:
        self._ok = ok
        self._latest = latest

    def start(self) -> None:
        return None

    def close(self) -> None:
        return None

    def latest(self):
        return self._latest

    def check(self) -> bool:
        return self._ok


def test_startup_prints_colored_module_lines(capsys):
    camera = _Sensor(True, (b"", 0, 4, 2))
    lidar = _Sensor(False, None)
    board = _Board({MOTOR: True, WHEEL_SERVO: True, NECK_SERVO: False, FACE_SERVO: True})
    session = Session(_Link(), CFG, camera, lidar, board, None)
    assert session.run(duration_sec=0.05) == "done"
    text = capsys.readouterr().out
    assert status_line(LIDAR, False) in text
    assert status_line(CAMERA, True) in text
    assert status_line(NECK_SERVO, False) in text
