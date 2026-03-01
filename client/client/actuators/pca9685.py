from __future__ import annotations

from dataclasses import dataclass

from ..models import ActuatorCommand, ActuatorFeedback


@dataclass
class MockActuatorState:
    motor_throttle: float = 0.0
    steering_throttle: float = 0.0
    head_pan_angle: float = 0.0
    head_tilt_angle: float = 0.0
    emergency_stop_count: int = 0


class MockActuatorDriver:
    """Убедительный mock драйвера приводов для тестов и локальной симуляции."""

    def __init__(self) -> None:
        self.state = MockActuatorState()
        self.history: list[ActuatorCommand] = []

    def apply(self, command: ActuatorCommand) -> None:
        self.state.motor_throttle = float(command.motor_throttle)
        self.state.steering_throttle = float(command.steering_throttle)
        self.state.head_pan_angle = float(command.head_pan_angle)
        self.state.head_tilt_angle = float(command.head_tilt_angle)
        self.history.append(command)

    def emergency_stop(self) -> None:
        self.state.emergency_stop_count += 1
        self.apply(ActuatorCommand())

    def get_feedback_state(self) -> ActuatorFeedback:
        return ActuatorFeedback(
            motor_throttle=self.state.motor_throttle,
            steering_throttle=self.state.steering_throttle,
        )


class Pca9685ActuatorDriver:
    """Реальный драйвер приводов на основе ServoKit/PCA9685.

    Логика преобразований синхронизирована с `code/head/modules/car.py` и `head.py`.
    """

    def __init__(self, cfg) -> None:
        self._cfg = cfg
        self._feedback = ActuatorFeedback()
        self._kit = None
        self._motor = None
        self._wheel = None
        self._neck = None
        self._face = None
        self._init_hw()

    def _init_hw(self) -> None:
        try:
            from adafruit_servokit import ServoKit
        except Exception as exc:
            raise RuntimeError("Не удалось импортировать adafruit_servokit") from exc

        self._kit = ServoKit(channels=int(self._cfg.pca.channels), frequency=int(self._cfg.pca.frequency))

        self._motor = self._kit.continuous_servo[int(self._cfg.motor.channel)]
        self._wheel = self._kit.continuous_servo[int(self._cfg.wheel.channel)]
        self._neck = self._kit.servo[int(self._cfg.neck.channel)]
        self._face = self._kit.servo[int(self._cfg.face.channel)]

        self._motor.set_pulse_width_range(int(self._cfg.motor.pwm_min_pulse), int(self._cfg.motor.pwm_max_pulse))
        self._wheel.set_pulse_width_range(int(self._cfg.wheel.pwm_min_pulse), int(self._cfg.wheel.pwm_max_pulse))

        self._neck.actuation_range = int(self._cfg.neck.actuation_range)
        if int(self._cfg.neck.pwm_min_pulse) > 0 and int(self._cfg.neck.pwm_max_pulse) > 0:
            self._neck.set_pulse_width_range(int(self._cfg.neck.pwm_min_pulse), int(self._cfg.neck.pwm_max_pulse))
        self._face.actuation_range = int(self._cfg.face.actuation_range)
        if int(self._cfg.face.pwm_min_pulse) > 0 and int(self._cfg.face.pwm_max_pulse) > 0:
            self._face.set_pulse_width_range(int(self._cfg.face.pwm_min_pulse), int(self._cfg.face.pwm_max_pulse))

        self.emergency_stop()
        self._set_head_angles(0.0, 0.0)

    @staticmethod
    def _clamp(v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, float(v)))

    def _speed_to_motor_throttle(self, speed_cmd: float) -> float:
        speed_cmd = self._clamp(speed_cmd, -1.0, 1.0)
        throttle = float(self._cfg.motor.zero_throttle) + speed_cmd * float(self._cfg.motor.speed_to_throttle_ratio)
        return self._clamp(throttle, -1.0, 1.0)

    def _steer_to_wheel_input(self, steer_cmd: float) -> float:
        steer_cmd = self._clamp(steer_cmd, -1.0, 1.0)
        zero = float(self._cfg.wheel.zero_throttle)
        if steer_cmd >= 0.0:
            val = zero + steer_cmd * (float(self._cfg.wheel.max_throttle) - zero)
        else:
            val = zero + (-steer_cmd) * (float(self._cfg.wheel.min_throttle) - zero)
        return self._clamp(val, float(self._cfg.wheel.min_throttle), float(self._cfg.wheel.max_throttle))

    def _set_head_angles(self, pan_deg: float, tilt_deg: float) -> None:
        # pan -> neck, tilt -> face
        pan_user = self._clamp(pan_deg, float(self._cfg.neck.angle_min), float(self._cfg.neck.angle_max))
        tilt_user = self._clamp(tilt_deg, float(self._cfg.face.angle_min), float(self._cfg.face.angle_max))
        self._neck.angle = float(self._cfg.neck.angle_zero) + pan_user
        self._face.angle = float(self._cfg.face.angle_zero) + tilt_user

    def apply(self, command: ActuatorCommand) -> None:
        motor = self._speed_to_motor_throttle(command.motor_throttle)
        wheel_input = self._steer_to_wheel_input(command.steering_throttle)
        wheel_throttle = -wheel_input if bool(self._cfg.wheel.invert) else wheel_input

        self._motor.throttle = motor
        self._wheel.throttle = wheel_throttle
        self._set_head_angles(command.head_pan_angle, command.head_tilt_angle)

        self._feedback.motor_throttle = float(command.motor_throttle)
        self._feedback.steering_throttle = float(command.steering_throttle)

    def emergency_stop(self) -> None:
        self._motor.throttle = self._speed_to_motor_throttle(0.0)
        zero_wheel_input = float(self._cfg.wheel.zero_throttle)
        self._wheel.throttle = -zero_wheel_input if bool(self._cfg.wheel.invert) else zero_wheel_input
        self._feedback.motor_throttle = 0.0
        self._feedback.steering_throttle = 0.0

    def get_feedback_state(self) -> ActuatorFeedback:
        return ActuatorFeedback(
            motor_throttle=self._feedback.motor_throttle,
            steering_throttle=self._feedback.steering_throttle,
        )

