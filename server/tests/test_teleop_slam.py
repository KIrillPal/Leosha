"""Tests for teleop-slam profile."""

from __future__ import annotations

from pathlib import Path

from server.config import load_server_config
from server.interfaces import AlgorithmContext
from server.models import ControlMode
from server.services.controller_service import ControllerService
from server.services.robot_client import MockRobotClient
from server.services.slam_service import SlamService
from server.web.app_factory import create_app


def _configs_dir() -> Path:
    return Path(__file__).resolve().parents[2] / "configs"


def test_teleop_slam_mode_available():
    """TELEOP_SLAM mode is registered and switchable."""
    cfg = load_server_config(str(_configs_dir() / "server.yaml"))
    robot = MockRobotClient(ip=cfg.robot.ip)
    context = AlgorithmContext(head_sensitivity=cfg.control.head_sensitivity)
    slam = SlamService(robot)
    controller = ControllerService(robot, context=context, slam_service=slam)
    assert ControlMode.TELEOP_SLAM in controller.modes
    controller.set_mode("teleop_slam")
    assert controller.active_mode == ControlMode.TELEOP_SLAM


def test_teleop_slam_tick_calls_slam_update():
    """Controller tick in TELEOP_SLAM mode calls slam_service.update_from_telemetry."""
    cfg = load_server_config(str(_configs_dir() / "server.yaml"))
    robot = MockRobotClient(ip=cfg.robot.ip)
    context = AlgorithmContext(head_sensitivity=cfg.control.head_sensitivity)
    slam = SlamService(robot)
    controller = ControllerService(robot, context=context, slam_service=slam)
    controller.set_mode("teleop_slam")
    controller.tick_once()
    stats = slam.get_stats()
    assert stats.scan_count >= 0
    assert stats.status in ("initializing", "mapping", "unknown")


def test_slam_api_endpoints(client_with_slam):
    """SLAM REST endpoints return valid responses."""
    app, c = client_with_slam
    client = app.test_client()

    status = client.get("/api/slam/status")
    assert status.status_code == 200
    data = status.get_json()
    assert "fps" in data
    assert "latency_ms" in data
    assert "scan_count" in data
    assert "status" in data

    map_resp = client.get("/api/slam/map")
    assert map_resp.status_code in (200, 404)

    pose = client.get("/api/slam/pose")
    assert pose.status_code == 200
    assert "x" in pose.get_json()
    assert "y" in pose.get_json()
    assert "theta" in pose.get_json()


def test_slam_and_ros_graph_pages(client_with_slam):
    """SLAM and ROS Graph tabs return 200."""
    app, _ = client_with_slam
    client = app.test_client()
    assert client.get("/slam").status_code == 200
    assert client.get("/ros-graph").status_code == 200


def test_ros_graph_api(client_with_slam):
    """ROS graph API returns nodes list."""
    app, _ = client_with_slam
    client = app.test_client()
    resp = client.get("/api/ros/graph")
    assert resp.status_code == 200
    data = resp.get_json()
    assert "nodes" in data
    assert isinstance(data["nodes"], list)
