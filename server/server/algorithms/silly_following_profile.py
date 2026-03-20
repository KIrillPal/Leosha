from __future__ import annotations

import math
from dataclasses import dataclass

from ..interfaces import AlgorithmContext, InputState
from ..models import ControlCommand, ControlMode, ManualInputState
from ..services.vision_service import VisionService
from .base_profile import BaseControlProfile
from .head_tracking_mixin import HeadTrackingMixin


@dataclass
class _LidarSectors:
    front_min_m: float
    left_min_m: float
    right_min_m: float
    rear_min_m: float


class SillyFollowingProfile(HeadTrackingMixin, BaseControlProfile):
    """Head tracking + reactive following without SLAM/global map."""

    def __init__(
        self,
        *,
        forward_throttle: float,
        backward_throttle: float,
        friend_embeddings_db: str,
        face_match_threshold: float,
        eye_confidence_threshold: float,
        head_tracking_gain: float,
        head_tracking_deadzone: float,
        bbox_target_x_frac: float,
        bbox_target_y_frac: float,
        latency_compensation_cap_frac: float,
        gyro_compensation_gain: float,
        gyro_compensation_neck_max_deg_fallback: float,
        steering_gain: float,
        steering_deadzone: float,
        steering_yaw_damping_gain: float,
        steering_yaw_damping_min: float,
        front_obstacle_distance_m: float,
        side_obstacle_distance_m: float,
        rear_clear_distance_m: float,
        side_steering_nudge: float,
        reverse_timeout_sec: float,
        reverse_steering_multiplier: float,
        forward_speed_steering_reduction_gain: float,
        forward_speed_min_factor: float,
        stop_head_tilt_deg: float,
        head_tilt_deg_fallback_scale: float,
        stop_without_lidar: bool,
        front_sector_half_angle_deg: float,
        side_sector_outer_angle_deg: float,
        rear_sector_start_angle_deg: float,
        vision_service: VisionService,
    ) -> None:
        self._forward_throttle = float(forward_throttle)
        self._backward_throttle = float(backward_throttle)
        self._steering_gain = float(steering_gain)
        self._steering_deadzone = float(steering_deadzone)
        self._steering_yaw_damping_gain = float(steering_yaw_damping_gain)
        self._steering_yaw_damping_min = float(steering_yaw_damping_min)
        self._front_obstacle_distance_m = float(front_obstacle_distance_m)
        self._side_obstacle_distance_m = float(side_obstacle_distance_m)
        self._rear_clear_distance_m = float(rear_clear_distance_m)
        self._side_steering_nudge = float(side_steering_nudge)
        self._reverse_timeout_sec = float(reverse_timeout_sec)
        self._reverse_steering_multiplier = float(reverse_steering_multiplier)
        self._forward_speed_steering_reduction_gain = float(forward_speed_steering_reduction_gain)
        self._forward_speed_min_factor = float(forward_speed_min_factor)
        self._stop_head_tilt_deg = float(stop_head_tilt_deg)
        self._head_tilt_deg_fallback_scale = float(head_tilt_deg_fallback_scale)
        self._stop_without_lidar = bool(stop_without_lidar)
        self._front_sector_half_angle_deg = float(front_sector_half_angle_deg)
        self._side_sector_outer_angle_deg = float(side_sector_outer_angle_deg)
        self._rear_sector_start_angle_deg = float(rear_sector_start_angle_deg)
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
        self._state = "idle"
        self._reverse_started_at = 0.0
        self._last_sectors = _LidarSectors(
            front_min_m=float("inf"),
            left_min_m=float("inf"),
            right_min_m=float("inf"),
            rear_min_m=float("inf"),
        )
        self._last_speed = 0.0
        self._last_steering = 0.0

    @property
    def mode(self) -> ControlMode:
        return ControlMode.SILLY_FOLLOWING

    @property
    def title(self) -> str:
        return "Слежение и реактивное следование"

    def on_action(self, action: dict) -> None:
        self.handle_tracking_action(action)

    def _tilt_axis_to_deg(self, input_state: InputState) -> float:
        cfg = input_state.robot_config
        if cfg is not None and "robot_geometry" in cfg and "head" in cfg["robot_geometry"]:
            max_up_deg = float(cfg["robot_geometry"]["head"]["face_max_deg"])
            return self._head_tilt * max_up_deg
        return self._head_tilt * self._head_tilt_deg_fallback_scale

    def _angle_deg(self, angle_min: float, angle_increment: float, index: int) -> float:
        return math.degrees(angle_min + angle_increment * index)

    def _analyze_lidar(self, scan: dict | None) -> _LidarSectors:
        if scan is None:
            return _LidarSectors(
                front_min_m=float("inf"),
                left_min_m=float("inf"),
                right_min_m=float("inf"),
                rear_min_m=float("inf"),
            )
        ranges = scan["ranges"]
        angle_min = float(scan["angle_min"])
        angle_increment = float(scan["angle_increment"])
        front = float("inf")
        left = float("inf")
        right = float("inf")
        rear = float("inf")
        for i, value in enumerate(ranges):
            if value is None:
                continue
            distance = float(value)
            if not math.isfinite(distance) or distance <= 0.0:
                continue
            angle_deg = self._angle_deg(angle_min, angle_increment, i)
            if -self._front_sector_half_angle_deg <= angle_deg <= self._front_sector_half_angle_deg:
                front = min(front, distance)
            elif self._front_sector_half_angle_deg < angle_deg <= self._side_sector_outer_angle_deg:
                left = min(left, distance)
            elif -self._side_sector_outer_angle_deg <= angle_deg < -self._front_sector_half_angle_deg:
                right = min(right, distance)
            elif abs(angle_deg) >= self._rear_sector_start_angle_deg:
                rear = min(rear, distance)
        return _LidarSectors(front, left, right, rear)

    def _base_steering(self, telemetry_yaw_rate: float, reverse: bool) -> float:
        steering = -self._head_pan * self._steering_gain
        if abs(steering) < self._steering_deadzone:
            steering = 0.0
        if reverse:
            steering *= self._reverse_steering_multiplier
        damping = max(
            self._steering_yaw_damping_min,
            1.0 - abs(telemetry_yaw_rate) * self._steering_yaw_damping_gain,
        )
        steering *= damping
        return max(-1.0, min(1.0, steering))

    def _apply_side_nudge(self, steering: float, sectors: _LidarSectors) -> float:
        result = steering
        if sectors.left_min_m < self._side_obstacle_distance_m:
            result += self._side_steering_nudge
        if sectors.right_min_m < self._side_obstacle_distance_m:
            result -= self._side_steering_nudge
        return max(-1.0, min(1.0, result))

    def _enter_reverse(self, now: float) -> None:
        self._state = "reversing"
        self._reverse_started_at = now

    def tick(self, context: AlgorithmContext, input_state: InputState) -> ControlCommand:
        self._compensate_head_for_body_yaw(input_state)
        has_target = False
        if input_state.manual.tracking_enabled:
            has_target = self.update_tracking(context, input_state, allow_first_face_fallback=True)
            if not has_target:
                self._manual_head(context, input_state.manual)
        else:
            self._state = "idle"
            self._tracking_state = "manual"
            self._tracking_target = None
            self._tracking_track_id = None

        sectors = self._analyze_lidar(input_state.lidar_scan)
        self._last_sectors = sectors
        tilt_deg = self._tilt_axis_to_deg(input_state)

        if not input_state.manual.tracking_enabled or not has_target:
            self._state = "idle"
            self._last_speed = 0.0
            self._last_steering = 0.0
            return ControlCommand(
                speed=0.0,
                steering=0.0,
                head_pan=self._head_pan,
                head_tilt=self._head_tilt,
                mode=self.mode,
            )

        if self._stop_without_lidar and input_state.lidar_scan is None:
            self._state = "waiting_lidar"
            self._last_speed = 0.0
            self._last_steering = 0.0
            return ControlCommand(
                speed=0.0,
                steering=0.0,
                head_pan=self._head_pan,
                head_tilt=self._head_tilt,
                mode=self.mode,
            )

        if tilt_deg >= self._stop_head_tilt_deg:
            self._state = "reached"
            self._last_speed = 0.0
            self._last_steering = 0.0
            return ControlCommand(
                speed=0.0,
                steering=0.0,
                head_pan=self._head_pan,
                head_tilt=self._head_tilt,
                mode=self.mode,
            )

        now = input_state.timestamp
        reversing = self._state == "reversing"
        if reversing:
            reverse_elapsed = max(0.0, now - self._reverse_started_at)
            if reverse_elapsed >= self._reverse_timeout_sec:
                self._state = "following"
                reversing = False
            elif sectors.rear_min_m <= self._rear_clear_distance_m:
                self._state = "blocked"
                self._last_speed = 0.0
                self._last_steering = 0.0
                return ControlCommand(
                    speed=0.0,
                    steering=0.0,
                    head_pan=self._head_pan,
                    head_tilt=self._head_tilt,
                    mode=self.mode,
                )

        if not reversing and sectors.front_min_m <= self._front_obstacle_distance_m:
            if sectors.rear_min_m > self._rear_clear_distance_m:
                self._enter_reverse(now)
                reversing = True
            else:
                self._state = "blocked"
                self._last_speed = 0.0
                self._last_steering = 0.0
                return ControlCommand(
                    speed=0.0,
                    steering=0.0,
                    head_pan=self._head_pan,
                    head_tilt=self._head_tilt,
                    mode=self.mode,
                )

        if reversing:
            steering = self._base_steering(input_state.telemetry.imu_yaw_rate, reverse=True)
            speed = self._backward_throttle
            self._state = "reversing"
        else:
            steering = self._base_steering(input_state.telemetry.imu_yaw_rate, reverse=False)
            steering = self._apply_side_nudge(steering, sectors)
            speed_factor = max(
                self._forward_speed_min_factor,
                1.0 - abs(steering) * self._forward_speed_steering_reduction_gain,
            )
            speed = self._forward_throttle * speed_factor
            self._state = "following"

        self._last_speed = speed
        self._last_steering = steering
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
        self._state = "idle"
        self._tracking_state = "manual"
        self._tracking_target = None
        self._tracking_track_id = None
        self._tracking_overlay = []
        self._last_speed = 0.0
        self._last_steering = 0.0
        self._reverse_started_at = 0.0

    def post_tick(self, input_state: InputState) -> None:
        input_state.manual.head_dx = 0.0
        input_state.manual.head_dy = 0.0

    def get_ui_state(self) -> dict:
        head_tilt_deg = self._head_tilt * self._head_tilt_deg_fallback_scale
        return {
            "x": float(self._head_pan),
            "y": float(self._head_tilt),
            "follow_state": self._state,
            "staring_state": self._tracking_state,
            "staring_target": self._tracking_target,
            "staring_track_id": self._tracking_track_id,
            "known_faces": self._friend_db.list_names(),
            "persons_count": len(self._vision.get_persons()),
            "staring_debug": dict(self._tracking_debug),
            "staring_overlay": list(self._tracking_overlay),
            "lidar_sectors": {
                "front_min_m": self._last_sectors.front_min_m,
                "left_min_m": self._last_sectors.left_min_m,
                "right_min_m": self._last_sectors.right_min_m,
                "rear_min_m": self._last_sectors.rear_min_m,
            },
            "follow_debug": {
                "speed": self._last_speed,
                "steering": self._last_steering,
                "head_tilt_deg": head_tilt_deg,
            },
        }

    def save_state(self) -> dict:
        return {"head_pan": self._head_pan, "head_tilt": self._head_tilt}

    def restore_state(self, state: dict) -> None:
        self._head_pan = float(state["head_pan"])
        self._head_tilt = float(state["head_tilt"])
