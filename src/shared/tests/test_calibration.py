import pytest

from leo_client.calibration import (
    pan_command_deg,
    pan_servo_deg,
    speed_from_throttle,
    steering_from_throttle,
    steering_throttle,
    throttle_from_speed,
    tilt_servo_deg,
)
from leo_server.common.config import load_config
from pathlib import Path

CAL = load_config(Path(__file__).resolve().parents[3] / "config" / "robot.yaml").client["calibration"]


def test_zero_speed_is_neutral_throttle():
    assert throttle_from_speed(0.0, CAL) == pytest.approx(0.05)


def test_speed_round_trip():
    for speed in (-0.2, 0.0, 0.2, 0.4, 0.7):
        throttle = throttle_from_speed(speed, CAL)
        assert abs(speed_from_throttle(throttle, CAL) - speed) < 1e-6


def test_steering_invert_round_trip():
    throttle = steering_throttle(0.25, CAL)
    assert throttle < 0.0
    assert abs(steering_from_throttle(throttle, CAL) - 0.25) < 1e-6


def test_head_zero_command_is_the_servo_offset():
    assert pan_servo_deg(0.0, CAL) == CAL["head_pan_zero_deg"]
    assert tilt_servo_deg(0.0, CAL) == CAL["head_tilt_zero_deg"]
    assert pan_command_deg(pan_servo_deg(20.0, CAL), CAL) == 20.0
