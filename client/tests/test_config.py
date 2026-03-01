from __future__ import annotations


def test_config_contains_robot_geometry_and_sensor_settings(client_config):
    geom = client_config.robot_geometry
    assert geom.body_size.length > 0
    assert geom.wheel.radius > 0
    assert geom.lidar.position.z >= 0
    assert geom.lidar.blind_zone.max_distance_m > 0
    assert geom.head.neck_max_deg > geom.head.neck_min_deg
    assert client_config.sensors.camera.fail_policy.max_consecutive_failures >= 1
    assert client_config.sensors.lidar.port.startswith("/dev/")

