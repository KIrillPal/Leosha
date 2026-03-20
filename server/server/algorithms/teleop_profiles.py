from __future__ import annotations

from ..interfaces import AlgorithmContext, InputState
from ..models import ControlCommand, ControlMode, ManualInputState, TelemetryFrame
from .base_profile import BaseControlProfile


class PauseProfile(BaseControlProfile):
    @property
    def mode(self) -> ControlMode:
        return ControlMode.PAUSE

    @property
    def title(self) -> str:
        return "Пауза"

    def tick(self, context: AlgorithmContext, input_state: InputState) -> ControlCommand:
        del context, input_state
        return ControlCommand(mode=self.mode)

    def get_ui_state(self) -> dict:
        return {"x": 0.0, "y": 0.0}


class TeleoperationProfile(BaseControlProfile):
    """Teleoperation profile: consumes ManualInputState and produces driving command."""

    def __init__(
        self,
        *,
        forward_throttle: float,
        backward_throttle: float,
        forward_fast_throttle: float,
        title: str,
        control_mode: ControlMode,
    ) -> None:
        self._forward_throttle = float(forward_throttle)
        self._backward_throttle = float(backward_throttle)
        self._forward_fast_throttle = float(forward_fast_throttle)
        self._title = title
        self._control_mode = control_mode
        self._head_pan = 0.0
        self._head_tilt = 0.0

    @property
    def mode(self) -> ControlMode:
        return self._control_mode

    @property
    def title(self) -> str:
        return self._title

    def tick(self, context: AlgorithmContext, input_state: InputState) -> ControlCommand:
        manual = input_state.manual
        telemetry = input_state.telemetry

        if not manual.tracking_enabled:
            return ControlCommand(mode=self.mode, head_pan=self._head_pan, head_tilt=self._head_tilt)

        speed = 0.0
        steering = 0.0
        if manual.w and not manual.s:
            speed = self._forward_fast_throttle if manual.shift else self._forward_throttle
        elif manual.s and not manual.w:
            speed = self._backward_throttle
        if manual.a and not manual.d:
            steering = -1.0
        elif manual.d and not manual.a:
            steering = 1.0

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

    def reset_runtime_state(self, manual: ManualInputState) -> None:
        manual.head_dx = 0.0
        manual.head_dy = 0.0
        self._head_pan = 0.0
        self._head_tilt = 0.0

    def post_tick(self, input_state: InputState) -> None:
        input_state.manual.head_dx = 0.0
        input_state.manual.head_dy = 0.0

    def get_ui_state(self) -> dict:
        return {
            "x": float(self._head_pan),
            "y": float(self._head_tilt),
            "tracking_enabled": False,
        }

    def save_state(self) -> dict:
        return {"head_pan": self._head_pan, "head_tilt": self._head_tilt}

    def restore_state(self, state: dict) -> None:
        self._head_pan = float(state["head_pan"])
        self._head_tilt = float(state["head_tilt"])


class TeleopSlamProfile(TeleoperationProfile):
    """Same teleop behavior as TeleoperationProfile, but marks SLAM as required."""

    def __init__(
        self,
        *,
        forward_throttle: float,
        backward_throttle: float,
        forward_fast_throttle: float,
    ) -> None:
        super().__init__(
            forward_throttle=forward_throttle,
            backward_throttle=backward_throttle,
            forward_fast_throttle=forward_fast_throttle,
            title="Телеуправление + SLAM",
            control_mode=ControlMode.TELEOP_SLAM,
        )

    @property
    def requires_slam(self) -> bool:
        return True
