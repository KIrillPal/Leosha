import time
from pathlib import Path

from leo_client.calibration import throttle_from_speed
from leo_client.hardware.servos import PARK_HZ, PARK_SEC, Servos, ramp_outputs, sine_ramp
from leo_client.sim.hardware import SimServos, _Body
from leo_server.common.config import load_config

CFG = load_config(Path(__file__).resolve().parents[3] / "config" / "robot.yaml").client
CAL = CFG["calibration"]
NEUTRAL = throttle_from_speed(0.0, CAL)


class _Output:
    def __init__(self) -> None:
        self.throttle = None
        self.angle = None


class _Kit:
    def __init__(self) -> None:
        self.continuous_servo = {0: _Output(), 1: _Output()}
        self.servo = {2: _Output(), 3: _Output()}


def test_sine_ramp_accelerates_and_comes_to_rest():
    assert sine_ramp(0.0) == 0.0
    assert sine_ramp(1.0) == 1.0
    assert sine_ramp(0.5) == 0.5
    step = 0.01
    start = sine_ramp(step) - sine_ramp(0.0)
    middle = sine_ramp(0.5 + step) - sine_ramp(0.5)
    end = sine_ramp(1.0) - sine_ramp(1.0 - step)
    assert start < middle
    assert end < middle


def test_close_ramps_every_actuator_to_its_standard_pose(monkeypatch):
    samples = []

    def record(apply, start, target):
        samples.append((start, target))
        apply(*target)

    monkeypatch.setattr("leo_client.hardware.servos.ramp_outputs", record)
    board = Servos(CFG["servos"], CAL)
    board._kit = _Kit()
    board.motor_throttle = 0.4
    board.steering_throttle = -1.0
    board.pan_servo_deg = 90.0
    board.tilt_servo_deg = 40.0

    board.close()

    start, target = samples[0]
    assert start == (0.4, -1.0, 90.0, 40.0)
    assert target == (NEUTRAL, 0.0, CAL["head_pan_zero_deg"], CAL["head_tilt_zero_deg"])
    assert board._kit.continuous_servo[0].throttle == NEUTRAL
    assert board._kit.continuous_servo[1].throttle == 0.0
    assert board._kit.servo[2].angle == CAL["head_pan_zero_deg"]
    assert board._kit.servo[3].angle == CAL["head_tilt_zero_deg"]


def test_ramp_follows_the_sine_and_lasts_two_seconds():
    now = [0.0]
    interrupts = {"left": 1}
    samples = []

    def clock() -> float:
        return now[0]

    def fake_sleep(seconds: float) -> None:
        if interrupts["left"]:
            interrupts["left"] -= 1
            raise KeyboardInterrupt
        now[0] += seconds

    def apply(*values: float) -> None:
        samples.append(values)

    ramp_outputs(apply, (0.0, 10.0), (4.0, 0.0), sleep=fake_sleep, clock=clock)

    assert len(samples) == int(PARK_SEC * PARK_HZ) + 1
    assert samples[0] != (0.0, 10.0)
    halfway = samples[int(PARK_SEC * PARK_HZ) // 2 - 1]
    assert halfway[1] == 5.0
    assert samples[-1] == (4.0, 0.0)
    assert now[0] == PARK_SEC


def test_an_actuator_already_home_does_not_wait():
    applied = []
    slept = []

    ramp_outputs(lambda *values: applied.append(values), (NEUTRAL, 0.0), (NEUTRAL, 0.0), sleep=slept.append)

    assert applied == [(NEUTRAL, 0.0)]
    assert slept == []


def _instant_park(monkeypatch) -> None:
    import leo_client.hardware.servos as servos

    original = servos.ramp_outputs
    now = [0.0]

    def read_clock() -> float:
        return now[0]

    def fake_sleep(seconds: float) -> None:
        now[0] += seconds

    def fast(apply, start, target, sleep=None, clock=None):
        original(apply, start, target, sleep=fake_sleep, clock=read_clock)

    monkeypatch.setattr(servos, "ramp_outputs", fast)


def test_sim_close_waits_until_the_body_has_stopped(monkeypatch):
    _instant_park(monkeypatch)
    body = _Body(
        {
            "world": {"start_x": 0.0, "start_y": 0.0, "start_yaw": 0.0},
            "physics_hz": 100.0,
            "wheelbase_m": 0.16,
            "slip": 0.0,
        },
        CAL,
    )
    body.start()
    try:
        body.motor = 0.41
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and abs(body.speed) <= 1e-6:
            time.sleep(0.01)
        assert abs(body.speed) > 1e-6
        servos = SimServos(body)
        servos.pan_servo_deg = 90.0
        servos.tilt_servo_deg = 40.0

        servos.close()

        assert abs(body.speed) <= 1e-6
        assert body.motor == NEUTRAL
        assert body.steer == 0.0
        assert body.pan_servo == CAL["head_pan_zero_deg"]
        assert body.tilt_servo == CAL["head_tilt_zero_deg"]
    finally:
        body.close()
