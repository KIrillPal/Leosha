import math
from pathlib import Path

import numpy as np

from leo_client.hardware.scan_grid import apply_blind_zone
from leo_client.sim.lidar_sim import cast_ranges
from leo_client.sim.physics import step_pose
from leo_server.common.config import load_config
from leo_server.nodes.odometry.tracker import Tracker

ROOT = Path(__file__).resolve().parents[3]
CFG = load_config(ROOT / "config" / "robot.yaml")


def test_forward_ray_hits_the_far_wall():
    world = CFG.sim["worlds"]["room"]
    rays = 360
    ranges = cast_ranges(1.5, 1.2, 0.0, rays, 8.0, world)
    assert abs(float(ranges[rays // 2]) - (7.8 - 1.5)) < 0.15


def test_scripted_route_odometry_stays_within_the_limit():
    sim = CFG.sim_payload("room")
    world = sim["world"]
    cal = sim["calibration"]
    x, y, yaw = world["start_x"], world["start_y"], world["start_yaw"]
    tracker = Tracker(CFG.odometry, sim["wheelbase_m"])
    tracker.fusion.x, tracker.fusion.y, tracker.fusion.yaw = x, y, yaw
    tracker._anchor = (x, y, yaw)
    dt = 0.1
    rays = int(sim["lidar"]["num_rays"])
    errors = []
    for step in sim["route"]:
        ticks = int(round(step["duration_sec"] / dt))
        for _ in range(ticks):
            from leo_client.calibration import steering_throttle, throttle_from_speed

            motor = throttle_from_speed(step["speed_mps"], cal)
            steer = steering_throttle(step["steering_rad"], cal)
            x, y, yaw, _speed, _steer = step_pose(x, y, yaw, motor, steer, dt, sim["wheelbase_m"], cal, sim["slip"])
            tracker.set_applied(step["speed_mps"], step["steering_rad"])
            tracker.step(dt)
            ranges = cast_ranges(x, y, yaw, rays, float(sim["lidar"]["range_max_m"]), world)
            ranges = apply_blind_zone(
                ranges,
                -math.pi,
                2.0 * math.pi / rays,
                sim["lidar"]["blind_start_deg"],
                sim["lidar"]["blind_end_deg"],
                sim["lidar"]["blind_max_m"],
            )
            tracker.observe_scan(ranges, -math.pi, 2.0 * math.pi / rays, dt)
            errors.append(math.hypot(tracker.fusion.x - x, tracker.fusion.y - y))
    ate = float(np.mean(errors))
    assert ate < float(CFG.sim["route_ate_limit_m"]), ate
