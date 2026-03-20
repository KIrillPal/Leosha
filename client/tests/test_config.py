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
    assert len(client_config.network.server_hosts) >= 1
    assert client_config.network.server_host == client_config.network.server_hosts[0]


def test_head_limits_are_only_loaded_from_robot_geometry(client_config):
    assert not hasattr(client_config.actuators.neck, "angle_min")
    assert not hasattr(client_config.actuators.neck, "angle_max")
    assert not hasattr(client_config.actuators.face, "angle_min")
    assert not hasattr(client_config.actuators.face, "angle_max")
    assert client_config.robot_geometry.head.neck_min_deg < 0.0
    assert client_config.robot_geometry.head.face_max_deg > 0.0

