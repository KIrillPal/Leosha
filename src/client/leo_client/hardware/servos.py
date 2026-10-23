"""PCA9685 output. The caller passes throttle and servo degrees, not ROS units."""

from __future__ import annotations

import logging

LOGGER = logging.getLogger("leo.client.servos")


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
        try:
            self.apply(self._neutral_throttle(), 0.0, self.pan_servo_deg, self.tilt_servo_deg)
        except Exception:
            LOGGER.exception("failed to park servos")
