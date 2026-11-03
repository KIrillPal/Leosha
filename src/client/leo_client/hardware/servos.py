"""PCA9685 output. The caller passes throttle and servo degrees, not ROS units."""

from __future__ import annotations

import logging
import math
import time

LOGGER = logging.getLogger("leo.client.servos")

# Every actuator returns to its standard output on one shared sine ramp.
PARK_SEC = 2.0
PARK_HZ = 50.0


class Servos:
    def __init__(self, cfg: dict, calibration: dict) -> None:
        self._cfg = cfg
        self._calibration = calibration
        self._kit = None
        self.motor_throttle = 0.0
        self.steering_throttle = 0.0
        self.pan_servo_deg = float(calibration["head_pan_zero_deg"])
        self.tilt_servo_deg = float(calibration["head_tilt_zero_deg"])

    def start(self) -> None:
        from adafruit_servokit import ServoKit

        self._kit = ServoKit(channels=int(self._cfg["pca_channels"]), frequency=int(self._cfg["pca_frequency"]))
        self._set_motor_pulses(self._cfg["motor_channel"], self._cfg["motor_pwm_min"], self._cfg["motor_pwm_max"])
        self._set_motor_pulses(self._cfg["wheel_channel"], self._cfg["wheel_pwm_min"], self._cfg["wheel_pwm_max"])
        self._set_servo(self._cfg["neck_channel"], self._cfg["neck_actuation_range"], self._cfg["neck_pwm_min"], self._cfg["neck_pwm_max"])
        self._set_servo(self._cfg["face_channel"], self._cfg["face_actuation_range"], self._cfg["face_pwm_min"], self._cfg["face_pwm_max"])
        self.apply(self._neutral_throttle(), 0.0, self.pan_servo_deg, self.tilt_servo_deg)

    def _neutral_throttle(self) -> float:
        from leo_client.calibration import throttle_from_speed

        return throttle_from_speed(0.0, self._calibration)

    def _set_motor_pulses(self, channel: int, low: int, high: int) -> None:
        servo = self._kit.continuous_servo[int(channel)]
        servo.set_pulse_width_range(int(low), int(high))

    def _set_servo(self, channel: int, actuation: int, low: int, high: int) -> None:
        servo = self._kit.servo[int(channel)]
        servo.actuation_range = int(actuation)
        if low > 0 and high > 0:
            servo.set_pulse_width_range(int(low), int(high))

    def apply(self, motor_throttle: float, steering_throttle: float, pan_servo_deg: float, tilt_servo_deg: float) -> None:
        self.motor_throttle = float(motor_throttle)
        self.steering_throttle = float(steering_throttle)
        self.pan_servo_deg = float(pan_servo_deg)
        self.tilt_servo_deg = float(tilt_servo_deg)
        if self._kit is None:
            return
        self._kit.continuous_servo[int(self._cfg["motor_channel"])].throttle = self.motor_throttle
        self._kit.continuous_servo[int(self._cfg["wheel_channel"])].throttle = self.steering_throttle
        self._kit.servo[int(self._cfg["neck_channel"])].angle = self.pan_servo_deg
        self._kit.servo[int(self._cfg["face_channel"])].angle = self.tilt_servo_deg

    def close(self) -> None:
        if self._kit is None:
            return
        start = (self.motor_throttle, self.steering_throttle, self.pan_servo_deg, self.tilt_servo_deg)
        try:
            ramp_outputs(self.apply, start, self.standard_pose())
        except Exception:
            LOGGER.exception("failed to park servos")

    def standard_pose(self) -> tuple[float, float, float, float]:
        return (
            self._neutral_throttle(),
            0.0,
            float(self._calibration["head_pan_zero_deg"]),
            float(self._calibration["head_tilt_zero_deg"]),
        )


def sine_ramp(fraction: float) -> float:
    """Move fraction for a sine acceleration that starts and ends at rest.

    Acceleration over the whole park is one period of a sine wave.
    """
    fraction = min(1.0, max(0.0, fraction))
    return fraction - math.sin(2.0 * math.pi * fraction) / (2.0 * math.pi)


def ramp_outputs(
    apply,
    start: tuple[float, ...],
    target: tuple[float, ...],
    sleep=time.sleep,
    clock=time.monotonic,
) -> None:
    """Bring every output to its target together, in ``PARK_SEC`` seconds."""
    if all(abs(left - right) <= 1e-6 for left, right in zip(start, target)):
        apply(*target)
        return
    steps = max(1, int(round(PARK_SEC * PARK_HZ)))
    origin = clock()
    for step in range(1, steps + 1):
        mix = sine_ramp(step / steps)
        blended = tuple(left + (right - left) * mix for left, right in zip(start, target))
        apply(*blended)
        _sleep_until(origin + PARK_SEC * step / steps, sleep, clock)
    apply(*target)


def _sleep_until(deadline: float, sleep, clock) -> None:
    """A stop signal does not end the park early."""
    while True:
        remaining = deadline - clock()
        if remaining <= 0.0:
            return
        try:
            sleep(remaining)
        except KeyboardInterrupt:
            continue
