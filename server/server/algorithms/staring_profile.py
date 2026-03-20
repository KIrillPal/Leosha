from __future__ import annotations

from ..interfaces import AlgorithmContext, InputState
from ..models import ControlCommand, ControlMode, ManualInputState, TelemetryFrame
from ..services.vision_service import VisionService
from .base_profile import BaseControlProfile
from .head_tracking_mixin import HeadTrackingMixin


class StaringProfile(HeadTrackingMixin, BaseControlProfile):
    def __init__(
        self,
        *,
        forward_throttle: float,
        backward_throttle: float,
        forward_fast_throttle: float,
        friend_embeddings_db: str,
        face_match_threshold: float,
        eye_confidence_threshold: float,
        head_tracking_gain: float,
        head_tracking_deadzone: float,
        bbox_target_x_frac: float = 0.5,
        bbox_target_y_frac: float = 0.9,
        latency_compensation_cap_frac: float = 0.5,
        gyro_compensation_gain: float = 0.0,
        gyro_compensation_neck_max_deg_fallback: float = 135.0,
        vision_service: VisionService,
    ) -> None:
        self._forward_throttle = float(forward_throttle)
        self._backward_throttle = float(backward_throttle)
        self._forward_fast_throttle = float(forward_fast_throttle)
        self._init_head_tracking(
            friend_embeddings_db=friend_embeddings_db,
            face_match_threshold=face_match_threshold,
            eye_confidence_threshold=eye_confidence_threshold,
            head_tracking_gain=head_tracking_gain,
            head_tracking_deadzone=head_tracking_deadzone,
            bbox_target_x_frac=bbox_target_x_frac,
            bbox_target_y_frac=bbox_target_y_frac,
            latency_compensation_cap_frac=latency_compensation_cap_frac,
            gyro_compensation_gain=gyro_compensation_gain,
            gyro_compensation_neck_max_deg_fallback=gyro_compensation_neck_max_deg_fallback,
            vision_service=vision_service,
        )

    @property
    def mode(self) -> ControlMode:
        return ControlMode.STARING

    @property
    def title(self) -> str:
        return "Слежение за знакомым лицом"

    def _drive_command(self, manual: ManualInputState, telemetry: TelemetryFrame) -> tuple[float, float]:
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
        return speed, steering

    def on_action(self, action: dict) -> None:
        self.handle_tracking_action(action)

    def tick(self, context: AlgorithmContext, input_state: InputState) -> ControlCommand:
        manual = input_state.manual
        telemetry = input_state.telemetry
        speed, steering = self._drive_command(manual, telemetry)

        if not manual.tracking_enabled:
            self._tracking_state = "manual"
            self._tracking_target = None
            self._tracking_track_id = None
            self._last_target_cx = None
            self._last_target_cy = None
            self._last_target_time = None
        else:
            if self._gyro_compensation_gain != 0.0:
                self._compensate_head_for_body_yaw(input_state)
            has_target = self.update_tracking(context, input_state, allow_first_face_fallback=True)
            if not has_target:
                self._manual_head(context, manual)

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
        self._tracking_state = "manual"
        self._tracking_target = None
        self._tracking_track_id = None
        self._last_target_cx = None
        self._last_target_cy = None
        self._last_target_time = None
        self._tracking_overlay = []

    def post_tick(self, input_state: InputState) -> None:
        input_state.manual.head_dx = 0.0
        input_state.manual.head_dy = 0.0

    def get_ui_state(self) -> dict:
        return {
            "x": float(self._head_pan),
            "y": float(self._head_tilt),
            "staring_state": self._tracking_state,
            "staring_target": self._tracking_target,
            "staring_track_id": self._tracking_track_id,
            "known_faces": self._friend_db.list_names(),
            "persons_count": len(self._vision.get_persons()),
            "staring_debug": dict(self._tracking_debug),
            "staring_overlay": list(self._tracking_overlay),
        }

    def save_state(self) -> dict:
        return {"head_pan": self._head_pan, "head_tilt": self._head_tilt}

    def restore_state(self, state: dict) -> None:
        self._head_pan = float(state["head_pan"])
        self._head_tilt = float(state["head_tilt"])
