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
    cfg.sensors.lidar.zero_angle_deg = 0.0  # выходной угол = сырой, чтобы слепая зона в градусах совпадала с углами точек
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
    # Слепая зона помечается NaN (нет измерения)
    assert math.isnan(scan.ranges[1])
    assert math.isnan(scan.ranges[2])
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
    # При invert_angle сортировка ставит -90° первым → он в слепой зоне 60..120 (в выходных -120..-60), помечается NaN
    assert math.isnan(scan.ranges[0])
    assert scan.ranges[1] == 1.0
    assert scan.ranges[2] == 0.2


def test_lidar_binning_when_expected_rays_mismatch(client_config):
    """При expected_num_rays != 0 и другом числе лучей применяется бининг; слепая зона остаётся NaN."""
    cfg = copy.deepcopy(client_config)
    cfg.sensors.lidar.invert_angle = False
    cfg.sensors.lidar.zero_angle_deg = 0.0
    cfg.sensors.lidar.expected_num_rays = 360
    cfg.robot_geometry.lidar.blind_zone.angle_start_deg = -10.0
    cfg.robot_geometry.lidar.blind_zone.angle_end_deg = 10.0
    cfg.robot_geometry.lidar.blind_zone.max_distance_m = 0.5
    thread = TMiniProPlusLidarThread(cfg.sensors.lidar, cfg.robot_geometry, None, None, None, None)
    # Скан из 4 точек (не 360) — должен пройти бининг до 360
    points = [
        _Point(math.radians(-20), 1.0),
        _Point(math.radians(0), 0.2),   # слепая зона → NaN
        _Point(math.radians(5), 0.3),   # слепая зона → NaN
        _Point(math.radians(20), 2.0),
    ]
    scan = thread._to_laserscan(_Scan(points))
    assert len(scan.ranges) == 360
    assert scan.angle_increment == (scan.angle_max - scan.angle_min) / 359
    # В слепой зоне должны быть NaN
    assert any(math.isnan(v) for v in scan.ranges)
    # Часть лучей валидные
    assert any(not math.isnan(v) and v > 0 for v in scan.ranges)


def test_lidar_warns_when_ray_count_deviation_over_10_percent(client_config):
    """При отклонении числа лучей от expected_num_rays более чем на 10% пишется warning (один раз)."""
    from unittest.mock import patch
    cfg = copy.deepcopy(client_config)
    cfg.sensors.lidar.expected_num_rays = 100
    cfg.sensors.lidar.zero_angle_deg = 0.0
    cfg.robot_geometry.lidar.blind_zone.angle_start_deg = -180.0
    cfg.robot_geometry.lidar.blind_zone.angle_end_deg = 180.0
    thread = TMiniProPlusLidarThread(cfg.sensors.lidar, cfg.robot_geometry, None, None, None, None)
    points = [_Point(math.radians(-90 + i * 60), 1.0) for i in range(4)]
    with patch("client.sensors.lidar.LOGGER") as mock_log:
        thread._to_laserscan(_Scan(points))
        mock_log.warning.assert_called_once()
        fmt = mock_log.warning.call_args[0][0]
        args = mock_log.warning.call_args[0][1:]
        assert "deviates from config" in fmt
        assert args[1] == 4  # n_actual
        assert args[2] == 100  # n_expected
        mock_log.warning.reset_mock()
        thread._to_laserscan(_Scan(points))
        mock_log.warning.assert_not_called()


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
    # Слепая зона помечается NaN
    assert math.isnan(scan.ranges[2])
