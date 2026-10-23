import math

import numpy as np

from leo_server.nodes.odometry.fusion import Fusion2D
from leo_server.nodes.odometry.scan_matcher import match_scans, scan_points
from leo_server.nodes.odometry.sources import measure_encoder, measure_imu, measure_visual
from leo_server.nodes.odometry.tracker import Tracker
from leo_server.nodes.robot_bridge.clock_sync import ClockSync


def test_clock_sync_keeps_the_smallest_delay():
    clock = ClockSync()
    clock.observe(1_000, 5_000)
    clock.observe(2_000, 9_000)
    assert clock.offset_ns == 4_000
    assert clock.to_ros_ns(2_000) == 6_000


def test_clock_sync_follows_a_forward_wall_clock_step():
    clock = ClockSync()
    for tick in range(15):
        clock.observe(tick, 1_000_000_000 + tick)
    assert clock.offset_ns == 1_000_000_000

    for tick in range(15, 20):
        clock.observe(tick, 4_000_000_000 + tick)

    assert clock.offset_ns == 4_000_000_000
    assert clock.to_ros_ns(20) == 4_000_000_020


def test_clock_sync_ignores_one_late_packet_and_one_early_glitch():
    clock = ClockSync()
    for tick in range(15):
        clock.observe(tick, 1_000_000_000 + tick)
    clock.observe(100, 1_000_000_000 + 100 + 5_000_000_000)
    clock.observe(200, 1_000_000_000 + 200 - 5_000_000_000)
    assert clock.offset_ns == 1_000_000_000


def test_icp_recovers_a_forward_shift():
    previous = np.stack([np.full(20, 2.0), np.linspace(-1.0, 1.0, 20)], axis=1)
    current = previous - np.array([0.2, 0.0])
    motion = match_scans(previous, current, (0.0, 0.0, 0.0), 15, 0.5)
    assert motion is not None
    assert abs(motion[0] - 0.2) < 0.03
    assert abs(motion[1]) < 0.03


def test_scan_points_drop_nan_and_the_blind_zone():
    ranges = np.array([np.nan, 1.0, 0.01, 8.0])
    points = scan_points(ranges, -math.pi, 0.1, 0.08, 6.0, 10)
    assert len(points) == 1


def test_fusion_keeps_pose_when_memory_is_reset():
    fusion = Fusion2D()
    fusion.update_pose(1.0, 2.0, 0.4, 1e-9, 1e-9)
    fusion.forget_motion()
    assert abs(fusion.x - 1.0) < 1e-6
    assert abs(fusion.y - 2.0) < 1e-6


def test_future_sources_add_no_measurement():
    assert measure_imu(0.1, 0, 0, 0, 0.1, 0.01) is None
    assert measure_encoder(0.1, 0.1, 0, 0, 0, 0.1, 0.01) is None
    assert measure_visual((0.1, 0, 0), 0, 0, 0, 0.01, 0.01) is None


def test_tracker_moves_with_applied_speed():
    cfg = {
        "process_noise_xy": 0.0,
        "process_noise_yaw": 0.0,
        "kinematic_enabled": True,
        "kinematic_var_xy": 0.0001,
        "kinematic_var_yaw": 0.0001,
        "scan_match_enabled": False,
        "scan_match_var_xy": 0.01,
        "scan_match_var_yaw": 0.01,
        "scan_match_max_points": 10,
        "scan_match_max_iters": 2,
        "scan_match_max_range_m": 6,
        "scan_match_min_range_m": 0.05,
        "scan_match_outlier_m": 0.3,
        "imu_enabled": True,
        "imu_var_yaw": 0.01,
        "encoder_enabled": True,
        "encoder_var_xy": 0.01,
        "visual_enabled": True,
        "visual_var_xy": 0.01,
        "visual_var_yaw": 0.01,
    }
    tracker = Tracker(cfg, 0.16)
    tracker.set_applied(0.4, 0.0)
    for _ in range(10):
        tracker.step(0.1)
    assert tracker.fusion.x > 0.3
