from __future__ import annotations

import copy
import math

from client.sensors.lidar import TMiniProPlusLidarThread, _angle_in_sector


class _Point:
    def __init__(self, angle: float, dist: float, intensity: float = 100.0) -> None:
        self.angle = angle
        self.range = dist
        self.intensity = intensity


class _Scan:
    def __init__(self, points: list[_Point]) -> None:
        self.points = points


def _make_thread(
    client_config,
    *,
    invert_angle: bool,
    blind_start_deg: float,
    blind_end_deg: float,
    max_distance_m: float,
):
    cfg = copy.deepcopy(client_config)
    cfg.sensors.lidar.invert_angle = invert_angle
    cfg.robot_geometry.lidar.blind_zone.angle_start_deg = blind_start_deg
    cfg.robot_geometry.lidar.blind_zone.angle_end_deg = blind_end_deg
    cfg.robot_geometry.lidar.blind_zone.max_distance_m = max_distance_m
    return TMiniProPlusLidarThread(cfg.sensors.lidar, cfg.robot_geometry, None, None, None, None)


def test_angle_in_sector_handles_wraparound():
    assert _angle_in_sector(179.0, 170.0, -170.0) is True
    assert _angle_in_sector(-179.0, 170.0, -170.0) is True
    assert _angle_in_sector(160.0, 170.0, -170.0) is False


def test_lidar_blind_zone_filters_near_points_without_inversion(client_config):
    thread = _make_thread(
        client_config,
        invert_angle=False,
        blind_start_deg=-15.0,
        blind_end_deg=15.0,
        max_distance_m=0.5,
    )
    scan = thread._to_laserscan(
        _Scan(
            [
                _Point(-0.6, 1.0),
                _Point(-0.2, 0.2),
                _Point(0.1, 0.3),
                _Point(0.5, 1.1),
            ]
        )
    )

    assert scan.angle_min < scan.angle_max
    assert scan.angle_increment > 0.0
    assert scan.ranges[0] == 1.0
    assert math.isinf(scan.ranges[1])
    assert math.isinf(scan.ranges[2])
    assert scan.ranges[3] == 1.1


def test_lidar_blind_zone_uses_inverted_output_angles(client_config):
    thread = _make_thread(
        client_config,
        invert_angle=True,
        blind_start_deg=60.0,
        blind_end_deg=120.0,
        max_distance_m=0.5,
    )
    scan = thread._to_laserscan(
        _Scan(
            [
                _Point(math.pi / 2, 0.2),
                _Point(0.0, 1.0),
                _Point(-math.pi / 2, 0.2),
            ]
        )
    )

    assert scan.angle_min < scan.angle_max
    assert scan.angle_increment > 0.0
    assert scan.ranges[0] == 0.2
    assert scan.ranges[1] == 1.0
    assert math.isinf(scan.ranges[2])


def test_lidar_blind_zone_respects_distance_threshold_and_wraparound(client_config):
    thread = _make_thread(
        client_config,
        invert_angle=False,
        blind_start_deg=170.0,
        blind_end_deg=-170.0,
        max_distance_m=0.5,
    )
    scan = thread._to_laserscan(
        _Scan(
            [
                _Point(math.radians(-179.0), 0.8),
                _Point(math.radians(160.0), 0.2),
                _Point(math.radians(179.0), 0.2),
            ]
        )
    )

    assert scan.ranges[0] == 0.8
    assert scan.ranges[1] == 0.2
    assert math.isinf(scan.ranges[2])
