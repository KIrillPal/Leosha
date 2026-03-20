from __future__ import annotations

import math
import random
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
        search_timeout_sec: float = 3.0,
        search_head_speed_axis_per_sec: float = 0.35,
        search_dwell_sec: float = 1.0,
        search_min_target_delta_axis: float = 0.25,
        search_pan_min_deg: float | None = None,
        search_pan_max_deg: float | None = None,
        search_tilt_min_axis: float = -1.0,
        search_tilt_max_axis: float = 1.0,
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
        self._search_timeout_sec = max(0.0, float(search_timeout_sec))
        self._search_head_speed_axis_per_sec = max(1e-3, float(search_head_speed_axis_per_sec))
        self._search_dwell_sec = max(0.0, float(search_dwell_sec))
        self._search_min_target_delta_axis = max(0.0, float(search_min_target_delta_axis))
        self._search_pan_min_deg = float(search_pan_min_deg) if search_pan_min_deg is not None else None
        self._search_pan_max_deg = float(search_pan_max_deg) if search_pan_max_deg is not None else None
        self._search_tilt_min_axis = max(-1.0, min(1.0, float(search_tilt_min_axis)))
        self._search_tilt_max_axis = max(-1.0, min(1.0, float(search_tilt_max_axis)))
        if self._search_tilt_min_axis > self._search_tilt_max_axis:
            self._search_tilt_min_axis, self._search_tilt_max_axis = (
                self._search_tilt_max_axis,
                self._search_tilt_min_axis,
            )
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
        self._search_lost_since: float | None = None
        self._search_target_pan_deg: float | None = None
        self._search_target_tilt: float | None = None
        self._search_prev_pan_deg: float | None = None
        self._search_prev_tilt: float | None = None
        self._search_dwell_until: float = 0.0

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
        # Lidar raw angles are not guaranteed to be wrapped to [-pi, pi].
        # Normalize degrees to [-180, 180] so sector checks are consistent.
        raw = math.degrees(angle_min + angle_increment * index)
        return ((raw + 180.0) % 360.0) - 180.0

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

    def _reset_search(self) -> None:
        self._search_lost_since = None
        self._search_target_pan_deg = None
        self._search_target_tilt = None
        self._search_prev_pan_deg = None
        self._search_prev_tilt = None
        self._search_dwell_until = 0.0

    def _head_pan_limits_deg(self, input_state: InputState) -> tuple[float, float]:
        cfg = input_state.robot_config
        if (
            cfg is not None
            and "robot_geometry" in cfg
            and "head" in cfg["robot_geometry"]
            and "neck_min_deg" in cfg["robot_geometry"]["head"]
            and "neck_max_deg" in cfg["robot_geometry"]["head"]
        ):
            min_deg = float(cfg["robot_geometry"]["head"]["neck_min_deg"])
            max_deg = float(cfg["robot_geometry"]["head"]["neck_max_deg"])
        else:
            max_deg = float(self._gyro_compensation_neck_max_deg_fallback)
            min_deg = -max_deg
        if min_deg > max_deg:
            min_deg, max_deg = max_deg, min_deg
        return min_deg, max_deg

    def _pan_axis_to_deg(self, pan_axis: float, input_state: InputState) -> float:
        min_deg, max_deg = self._head_pan_limits_deg(input_state)
        axis = max(-1.0, min(1.0, pan_axis))
        if axis >= 0.0:
            return axis * max_deg
        return axis * abs(min_deg)

    def _pan_deg_to_axis(self, pan_deg: float, input_state: InputState) -> float:
        min_deg, max_deg = self._head_pan_limits_deg(input_state)
        if pan_deg >= 0.0:
            denom = max(1e-6, max_deg)
            return max(-1.0, min(1.0, pan_deg / denom))
        denom = max(1e-6, abs(min_deg))
        return max(-1.0, min(1.0, pan_deg / denom))

    def _search_pan_range_deg(self, input_state: InputState) -> tuple[float, float]:
        hw_min, hw_max = self._head_pan_limits_deg(input_state)
        cfg_min = hw_min if self._search_pan_min_deg is None else self._search_pan_min_deg
        cfg_max = hw_max if self._search_pan_max_deg is None else self._search_pan_max_deg
        pan_min = max(hw_min, min(hw_max, cfg_min))
        pan_max = max(hw_min, min(hw_max, cfg_max))
        if pan_min > pan_max:
            pan_min, pan_max = pan_max, pan_min
        return pan_min, pan_max

    def _search_pick_next_target(self, input_state: InputState) -> tuple[float, float]:
        pan_min_deg, pan_max_deg = self._search_pan_range_deg(input_state)
        prev_pan = self._search_prev_pan_deg
        prev_tilt = self._search_prev_tilt
        if prev_pan is None or prev_tilt is None:
            return (
                random.uniform(pan_min_deg, pan_max_deg),
                random.uniform(self._search_tilt_min_axis, self._search_tilt_max_axis),
            )
        max_attempts = 16
        candidate = (prev_pan, prev_tilt)
        for _ in range(max_attempts):
            pan = random.uniform(pan_min_deg, pan_max_deg)
            tilt = random.uniform(self._search_tilt_min_axis, self._search_tilt_max_axis)
            pan_delta_axis = abs(self._pan_deg_to_axis(pan, input_state) - self._pan_deg_to_axis(prev_pan, input_state))
            if math.hypot(pan_delta_axis, tilt - prev_tilt) >= self._search_min_target_delta_axis:
                return (pan, tilt)
            candidate = (pan, tilt)
        # If range is too small for the requested delta, still continue with the best found sample.
        return candidate

    def _move_axis_towards(self, current: float, target: float, max_step: float) -> tuple[float, bool]:
        delta = target - current
        if abs(delta) <= max_step:
            return target, True
        step = max_step if delta > 0.0 else -max_step
        return current + step, False

    def _run_searching_head_motion(self, input_state: InputState) -> None:
        now = input_state.timestamp
        if self._search_target_pan_deg is None or self._search_target_tilt is None:
            self._search_target_pan_deg, self._search_target_tilt = self._search_pick_next_target(input_state)

        if self._search_dwell_until > now:
            return

        max_step = self._search_head_speed_axis_per_sec * max(0.0, input_state.dt)
        target_pan_axis = self._pan_deg_to_axis(self._search_target_pan_deg, input_state)
        next_pan, reached_pan = self._move_axis_towards(
            self._head_pan,
            target_pan_axis,
            max_step,
        )
        next_tilt, reached_tilt = self._move_axis_towards(
            self._head_tilt,
            self._search_target_tilt,
            max_step,
        )
        self._head_pan = max(-1.0, min(1.0, next_pan))
        self._head_tilt = max(-1.0, min(1.0, next_tilt))
        if reached_pan and reached_tilt:
            self._search_prev_pan_deg = self._search_target_pan_deg
            self._search_prev_tilt = self._search_target_tilt
            self._search_target_pan_deg = None
            self._search_target_tilt = None
            self._search_dwell_until = now + self._search_dwell_sec

    def tick(self, context: AlgorithmContext, input_state: InputState) -> ControlCommand:
        self._compensate_head_for_body_yaw(input_state)
        has_target = False
        if input_state.manual.tracking_enabled:
            has_target = self.update_tracking(context, input_state, allow_first_face_fallback=True)
            if not has_target:
                self._manual_head(context, input_state.manual)
            else:
                self._reset_search()
        else:
            self._state = "idle"
            self._tracking_state = "manual"
            self._tracking_target = None
            self._tracking_track_id = None
            self._reset_search()

        sectors = self._analyze_lidar(input_state.lidar_scan)
        self._last_sectors = sectors
        tilt_deg = self._tilt_axis_to_deg(input_state)

        if not input_state.manual.tracking_enabled or not has_target:
            if input_state.manual.tracking_enabled:
                now = input_state.timestamp
                if self._search_lost_since is None:
                    self._search_lost_since = now
                lost_for = max(0.0, now - self._search_lost_since)
                if lost_for >= self._search_timeout_sec:
                    self._state = "searching"
                    self._run_searching_head_motion(input_state)
                else:
                    self._state = "idle"
            else:
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
        self._reset_search()

    def post_tick(self, input_state: InputState) -> None:
        input_state.manual.head_dx = 0.0
        input_state.manual.head_dy = 0.0

    def get_ui_state(self) -> dict:
        head_tilt_deg = self._head_tilt * self._head_tilt_deg_fallback_scale

        def _json_safe_dist(v: float) -> float | None:
            # Ensure UI JSON remains valid: Infinity/NaN are not valid JSON values.
            if not math.isfinite(v):
                return None
            return float(v)

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
                "front_min_m": _json_safe_dist(self._last_sectors.front_min_m),
                "left_min_m": _json_safe_dist(self._last_sectors.left_min_m),
                "right_min_m": _json_safe_dist(self._last_sectors.right_min_m),
                "rear_min_m": _json_safe_dist(self._last_sectors.rear_min_m),
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
