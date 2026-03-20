from __future__ import annotations

import math
import random

from server.algorithms.silly_following_profile import SillyFollowingProfile
from server.interfaces import AlgorithmContext, InputState
from server.models import ControlMode, ManualInputState, TelemetryFrame


class MockVisionService:
    def __init__(self) -> None:
        self._persons: list[dict] = []
        self._frame_id = 0

    def set_persons(self, persons: list[dict], frame_id: int = 1) -> None:
        self._persons = [dict(p) for p in persons]
        self._frame_id = frame_id

    def get_persons(self) -> list[dict]:
        return [dict(p) for p in self._persons]

    def get_latest_frame_id(self) -> int:
        return self._frame_id


def _person(track_id: int, bbox: list[float], emb: list[float]) -> dict:
    return {
        "track_id": track_id,
        "bbox": bbox,
        "keypoints": [],
        "head_yaw_deg": 0.0,
        "face_visible": True,
        "face_embedding": emb,
        "confidence": 0.9,
    }


def _scan(*, front: float, left: float, right: float, rear: float) -> dict:
    angle_min = -math.pi
    angle_increment = math.radians(1.0)
    ranges: list[float] = []
    for deg in range(-180, 181):
        value = float("inf")
        if -30 <= deg <= 30:
            value = front
        elif 30 < deg <= 90:
            value = left
        elif -90 <= deg < -30:
            value = right
        elif abs(deg) >= 150:
            value = rear
        ranges.append(value)
    return {
        "timestamp_ns": 0,
        "angle_min": angle_min,
        "angle_max": math.pi,
        "angle_increment": angle_increment,
        "range_min": 0.02,
        "range_max": 6.0,
        "ranges": ranges,
        "intensities": [0.0] * len(ranges),
    }


def _build_profile(tmp_path, vision: MockVisionService) -> SillyFollowingProfile:
    return SillyFollowingProfile(
        forward_throttle=0.25,
        backward_throttle=-0.13,
        friend_embeddings_db=str(tmp_path / "friends.db"),
        face_match_threshold=0.5,
        eye_confidence_threshold=0.3,
        head_tracking_gain=1.8,
        head_tracking_deadzone=0.05,
        bbox_target_x_frac=0.5,
        bbox_target_y_frac=0.1,
        latency_compensation_cap_frac=0.3,
        gyro_compensation_gain=1.0,
        gyro_compensation_neck_max_deg_fallback=135.0,
        steering_gain=1.0,
        steering_deadzone=0.03,
        steering_yaw_damping_gain=0.2,
        steering_yaw_damping_min=0.55,
        front_obstacle_distance_m=0.35,
        side_obstacle_distance_m=0.25,
        rear_clear_distance_m=0.3,
        side_steering_nudge=0.25,
        reverse_timeout_sec=1.5,
        reverse_steering_multiplier=-1.0,
        forward_speed_steering_reduction_gain=0.4,
        forward_speed_min_factor=0.5,
        stop_head_tilt_deg=30.0,
        head_tilt_deg_fallback_scale=75.0,
        stop_without_lidar=True,
        front_sector_half_angle_deg=30.0,
        side_sector_outer_angle_deg=90.0,
        rear_sector_start_angle_deg=150.0,
        search_timeout_sec=3.0,
        search_head_speed_axis_per_sec=0.35,
        search_dwell_sec=1.0,
        search_min_target_delta_axis=0.25,
        search_pan_min_deg=-135.0,
        search_pan_max_deg=135.0,
        search_tilt_min_axis=-1.0,
        search_tilt_max_axis=1.0,
        stable_track_frames=1,
        vision_service=vision,
    )


def test_silly_following_moves_and_steers_to_target(tmp_path):
    vision = MockVisionService()
    profile = _build_profile(tmp_path, vision)
    vision.set_persons([_person(1, [420, 120, 620, 380], [1.0] * 128)], frame_id=10)
    profile.on_action({"type": "add_face", "name": "Kir"})
    cmd = profile.tick(
        AlgorithmContext(head_sensitivity=-0.002),
        InputState(
            manual=ManualInputState(tracking_enabled=True),
            telemetry=TelemetryFrame(),
            lidar_scan=_scan(front=2.0, left=2.0, right=2.0, rear=2.0),
            dt=0.1,
            timestamp=1.0,
        ),
    )
    ui = profile.get_ui_state()
    assert cmd.mode == ControlMode.SILLY_FOLLOWING
    assert cmd.speed > 0.0
    assert cmd.steering > 0.0
    assert ui["follow_state"] == "following"
    assert ui["staring_target"] == "Kir"


def test_silly_following_reverses_and_inverts_steering_when_front_blocked(tmp_path):
    vision = MockVisionService()
    profile = _build_profile(tmp_path, vision)
    vision.set_persons([_person(1, [420, 120, 620, 380], [1.0] * 128)], frame_id=10)
    profile.on_action({"type": "add_face", "name": "Kir"})

    forward_cmd = profile.tick(
        AlgorithmContext(head_sensitivity=-0.002),
        InputState(
            manual=ManualInputState(tracking_enabled=True),
            telemetry=TelemetryFrame(),
            lidar_scan=_scan(front=2.0, left=2.0, right=2.0, rear=2.0),
            dt=0.1,
            timestamp=1.0,
        ),
    )
    reverse_cmd = profile.tick(
        AlgorithmContext(head_sensitivity=-0.002),
        InputState(
            manual=ManualInputState(tracking_enabled=True),
            telemetry=TelemetryFrame(),
            lidar_scan=_scan(front=0.2, left=2.0, right=2.0, rear=2.0),
            dt=0.1,
            timestamp=1.1,
        ),
    )
    assert forward_cmd.steering > 0.0
    assert reverse_cmd.speed < 0.0
    assert reverse_cmd.steering < 0.0
    assert profile.get_ui_state()["follow_state"] == "reversing"


def test_silly_following_stops_when_front_and_rear_blocked(tmp_path):
    vision = MockVisionService()
    profile = _build_profile(tmp_path, vision)
    vision.set_persons([_person(1, [320, 120, 520, 380], [1.0] * 128)], frame_id=10)
    cmd = profile.tick(
        AlgorithmContext(head_sensitivity=-0.002),
        InputState(
            manual=ManualInputState(tracking_enabled=True),
            telemetry=TelemetryFrame(),
            lidar_scan=_scan(front=0.2, left=2.0, right=2.0, rear=0.2),
            dt=0.1,
            timestamp=1.0,
        ),
    )
    assert cmd.speed == 0.0
    assert profile.get_ui_state()["follow_state"] == "blocked"


def test_silly_following_stops_when_head_tilt_reaches_threshold(tmp_path):
    vision = MockVisionService()
    profile = _build_profile(tmp_path, vision)
    vision.set_persons([_person(1, [260, 0, 380, 100], [1.0] * 128)], frame_id=10)
    cmd = profile.tick(
        AlgorithmContext(head_sensitivity=-0.002),
        InputState(
            manual=ManualInputState(tracking_enabled=True),
            telemetry=TelemetryFrame(),
            lidar_scan=_scan(front=2.0, left=2.0, right=2.0, rear=2.0),
            dt=0.3,
            timestamp=1.0,
        ),
    )
    assert cmd.speed == 0.0
    assert profile.get_ui_state()["follow_state"] == "reached"


def test_silly_following_gyro_compensates_head_between_vision_frames(tmp_path):
    vision = MockVisionService()
    profile = _build_profile(tmp_path, vision)
    profile._head_pan = 0.3
    cmd = profile.tick(
        AlgorithmContext(head_sensitivity=-0.002),
        InputState(
            manual=ManualInputState(tracking_enabled=True),
            telemetry=TelemetryFrame(imu_yaw_rate=1.0),
            lidar_scan=_scan(front=2.0, left=2.0, right=2.0, rear=2.0),
            dt=0.1,
            timestamp=1.0,
        ),
    )
    assert cmd.head_pan < 0.3


def test_silly_following_applies_side_nudge(tmp_path):
    vision = MockVisionService()
    profile = _build_profile(tmp_path, vision)
    vision.set_persons([_person(1, [270, 150, 370, 390], [1.0] * 128)], frame_id=10)
    cmd = profile.tick(
        AlgorithmContext(head_sensitivity=-0.002),
        InputState(
            manual=ManualInputState(tracking_enabled=True),
            telemetry=TelemetryFrame(),
            lidar_scan=_scan(front=2.0, left=0.15, right=2.0, rear=2.0),
            dt=0.1,
            timestamp=1.0,
        ),
    )
    assert cmd.speed > 0.0
    assert cmd.steering > 0.0


def test_silly_following_enters_searching_and_moves_head_after_timeout(tmp_path):
    vision = MockVisionService()
    profile = SillyFollowingProfile(
        forward_throttle=0.25,
        backward_throttle=-0.13,
        friend_embeddings_db=str(tmp_path / "friends.db"),
        face_match_threshold=0.5,
        eye_confidence_threshold=0.3,
        head_tracking_gain=1.8,
        head_tracking_deadzone=0.05,
        bbox_target_x_frac=0.5,
        bbox_target_y_frac=0.1,
        latency_compensation_cap_frac=0.3,
        gyro_compensation_gain=1.0,
        gyro_compensation_neck_max_deg_fallback=135.0,
        steering_gain=1.0,
        steering_deadzone=0.03,
        steering_yaw_damping_gain=0.2,
        steering_yaw_damping_min=0.55,
        front_obstacle_distance_m=0.35,
        side_obstacle_distance_m=0.25,
        rear_clear_distance_m=0.3,
        side_steering_nudge=0.25,
        reverse_timeout_sec=1.5,
        reverse_steering_multiplier=-1.0,
        forward_speed_steering_reduction_gain=0.4,
        forward_speed_min_factor=0.5,
        stop_head_tilt_deg=30.0,
        head_tilt_deg_fallback_scale=75.0,
        stop_without_lidar=True,
        front_sector_half_angle_deg=30.0,
        side_sector_outer_angle_deg=90.0,
        rear_sector_start_angle_deg=150.0,
        search_timeout_sec=0.2,
        search_head_speed_axis_per_sec=0.5,
        search_dwell_sec=0.05,
        search_min_target_delta_axis=0.1,
        search_pan_min_deg=40.0,
        search_pan_max_deg=60.0,
        search_tilt_min_axis=-0.6,
        search_tilt_max_axis=-0.4,
        stable_track_frames=1,
        vision_service=vision,
    )

    random.seed(7)

    # No face in frame -> after timeout profile starts scanning.
    first = profile.tick(
        AlgorithmContext(head_sensitivity=-0.002),
        InputState(
            manual=ManualInputState(tracking_enabled=True),
            telemetry=TelemetryFrame(),
            lidar_scan=_scan(front=2.0, left=2.0, right=2.0, rear=2.0),
            dt=0.1,
            timestamp=1.0,
        ),
    )
    assert first.speed == 0.0
    assert profile.get_ui_state()["follow_state"] == "idle"

    second = profile.tick(
        AlgorithmContext(head_sensitivity=-0.002),
        InputState(
            manual=ManualInputState(tracking_enabled=True),
            telemetry=TelemetryFrame(),
            lidar_scan=_scan(front=2.0, left=2.0, right=2.0, rear=2.0),
            dt=0.2,
            timestamp=1.21,
        ),
    )
    assert second.speed == 0.0
    assert profile.get_ui_state()["follow_state"] == "searching"
    assert profile._head_pan != 0.0 or profile._head_tilt != 0.0
