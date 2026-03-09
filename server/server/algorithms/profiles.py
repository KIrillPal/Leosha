from __future__ import annotations

from dataclasses import dataclass, field

from ..interfaces import AlgorithmContext, AutonomyAlgorithm, ControlAlgorithm, OperationProfile
from ..models import ControlCommand, ControlMode, ManualInputState, TelemetryFrame


class PauseAlgorithm(ControlAlgorithm):
    @property
    def mode(self) -> ControlMode:
        return ControlMode.PAUSE

    @property
    def name(self) -> str:
        return "Пауза"

    def compute_command(
        self,
        context: AlgorithmContext,
        manual: ManualInputState,
        telemetry: TelemetryFrame,
    ) -> ControlCommand:
        return ControlCommand(mode=self.mode)


class SlamTeleoperationAlgorithm(ControlAlgorithm):
    def __init__(self) -> None:
        self._head_pan = 0.0
        self._head_tilt = 0.0

    @property
    def mode(self) -> ControlMode:
        return ControlMode.TELEOPERATION

    @property
    def name(self) -> str:
        return "SLAM + телеуправление"

    def compute_command(
        self,
        context: AlgorithmContext,
        manual: ManualInputState,
        telemetry: TelemetryFrame,
    ) -> ControlCommand:
        if not manual.tracking_enabled:
            return ControlCommand(mode=self.mode, head_pan=self._head_pan, head_tilt=self._head_tilt)

        speed = 0.0
        steering = 0.0
        speed_limit = context.max_speed_fast if manual.shift else context.max_speed_normal
        if manual.w and not manual.s:
            speed = speed_limit
        elif manual.s and not manual.w:
            speed = -speed_limit
        if manual.a and not manual.d:
            steering = -context.max_steering
        elif manual.d and not manual.a:
            steering = context.max_steering

        steering *= max(0.55, 1.0 - abs(telemetry.imu_yaw_rate) * 0.2)
        self._head_pan = max(-1.0, min(1.0, self._head_pan + manual.head_dx * context.head_sensitivity))
        self._head_tilt = max(-1.0, min(1.0, self._head_tilt + manual.head_dy * context.head_sensitivity))

        return ControlCommand(
            speed=speed,
            steering=steering,
            head_pan=self._head_pan,
            head_tilt=self._head_tilt,
            mode=self.mode,
        )


class PlaceholderAutonomyAlgorithm(AutonomyAlgorithm):
    def __init__(self) -> None:
        self._target: dict = {}

    @property
    def mode(self) -> ControlMode:
        return ControlMode.AUTONOMY_PROFILE_1

    @property
    def name(self) -> str:
        return "Самоуправление: профиль 1 (заглушка)"

    def set_target(self, target: dict) -> None:
        self._target = dict(target)

    def compute_command(
        self,
        context: AlgorithmContext,
        manual: ManualInputState,
        telemetry: TelemetryFrame,
    ) -> ControlCommand:
        return ControlCommand(mode=self.mode)


@dataclass
class PauseProfile(OperationProfile):
    _algorithm: ControlAlgorithm = field(default_factory=PauseAlgorithm)

    @property
    def mode(self) -> ControlMode:
        return ControlMode.PAUSE

    @property
    def title(self) -> str:
        return "Пауза"

    @property
    def algorithm(self) -> ControlAlgorithm:
        return self._algorithm


@dataclass
class TeleoperationProfile(OperationProfile):
    _algorithm: ControlAlgorithm = field(default_factory=SlamTeleoperationAlgorithm)

    @property
    def mode(self) -> ControlMode:
        return ControlMode.TELEOPERATION

    @property
    def title(self) -> str:
        return "Телеуправление (SLAM)"

    @property
    def algorithm(self) -> ControlAlgorithm:
        return self._algorithm


@dataclass
class AutonomyProfile1(OperationProfile):
    _algorithm: ControlAlgorithm = field(default_factory=PlaceholderAutonomyAlgorithm)

    @property
    def mode(self) -> ControlMode:
        return ControlMode.AUTONOMY_PROFILE_1

    @property
    def title(self) -> str:
        return "Самоуправление: профиль 1"

    @property
    def algorithm(self) -> ControlAlgorithm:
        return self._algorithm
