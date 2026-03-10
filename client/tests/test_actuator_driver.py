from __future__ import annotations

import types

from client.actuators import Pca9685ActuatorDriver
from client.models import ActuatorCommand


def test_pca_driver_maps_commands_using_head_car_calibration(monkeypatch, client_config):
    cfg = client_config.actuators
    head_cfg = client_config.robot_geometry.head

    class _FakeContServo:
        def __init__(self):
            self.throttle = 0.0
            self.pulse = (0, 0)

        def set_pulse_width_range(self, mn, mx):
            self.pulse = (mn, mx)

    class _FakeServo:
        def __init__(self):
            self.angle = 0.0
            self.actuation_range = 180
            self.pulse = (0, 0)

        def set_pulse_width_range(self, mn, mx):
            self.pulse = (mn, mx)

    class _FakeKit:
        def __init__(self, channels, frequency):
            self.channels = channels
            self.frequency = frequency
            self.continuous_servo = [_FakeContServo() for _ in range(16)]
            self.servo = [_FakeServo() for _ in range(16)]

    fake_mod = types.SimpleNamespace(ServoKit=_FakeKit)
    monkeypatch.setitem(__import__("sys").modules, "adafruit_servokit", fake_mod)

    driver = Pca9685ActuatorDriver(cfg, head_cfg)
    driver.apply(ActuatorCommand(motor_throttle=0.3, steering_throttle=1.0, head_pan_angle=10.0, head_tilt_angle=5.0))

    # motor: direct throttle from server (clamped)
    expected_motor = 0.3
    assert abs(driver._motor.throttle - expected_motor) < 1e-6

    # wheel: right turn -> max_throttle then inverted
    assert abs(driver._wheel.throttle - (-cfg.wheel.max_throttle)) < 1e-6

    # neck/face: angle = zero + user_angle
    assert abs(driver._neck.angle - (cfg.neck.angle_zero + 10.0)) < 1e-6
    assert abs(driver._face.angle - (cfg.face.angle_zero + 5.0)) < 1e-6


def test_pca_driver_clamps_head_using_robot_geometry_limits(monkeypatch, client_config):
    cfg = client_config.actuators
    head_cfg = client_config.robot_geometry.head
    class _FakeContServo:
        def __init__(self):
            self.throttle = 0.0
            self.pulse = (0, 0)

        def set_pulse_width_range(self, mn, mx):
            self.pulse = (mn, mx)

    class _FakeServo:
        def __init__(self):
            self.angle = 0.0
            self.actuation_range = 180
            self.pulse = (0, 0)

        def set_pulse_width_range(self, mn, mx):
            self.pulse = (mn, mx)

    class _FakeKit:
        def __init__(self, channels, frequency):
            self.channels = channels
            self.frequency = frequency
            self.continuous_servo = [_FakeContServo() for _ in range(16)]
            self.servo = [_FakeServo() for _ in range(16)]

    fake_mod = types.SimpleNamespace(ServoKit=_FakeKit)
    monkeypatch.setitem(__import__("sys").modules, "adafruit_servokit", fake_mod)

    driver = Pca9685ActuatorDriver(cfg, head_cfg)
    driver.apply(
        ActuatorCommand(
            head_pan_angle=head_cfg.neck_max_deg + 100.0,
            head_tilt_angle=head_cfg.face_min_deg - 100.0,
        )
    )

    assert abs(driver._neck.angle - (cfg.neck.angle_zero + head_cfg.neck_max_deg)) < 1e-6
    assert abs(driver._face.angle - (cfg.face.angle_zero + head_cfg.face_min_deg)) < 1e-6

