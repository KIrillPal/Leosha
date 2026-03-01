from __future__ import annotations

from pathlib import Path

import pytest

from server.config import load_server_config
from server.interfaces import AlgorithmContext
from server.services.controller_service import ControllerService
from server.services.robot_client import MockRobotClient
from server.web.app_factory import create_app


@pytest.fixture()
def runtime():
    cfg_path = Path(__file__).resolve().parents[1] / "config" / "server.yaml"
    cfg = load_server_config(cfg_path)
    robot = MockRobotClient(ip=cfg.robot.ip)
    context = AlgorithmContext(
        max_speed_normal=cfg.control.max_speed_normal,
        max_speed_fast=cfg.control.max_speed_fast,
        max_steering=cfg.control.max_steering,
        head_sensitivity=cfg.control.head_sensitivity,
    )
    controller = ControllerService(robot, context=context)
    app = create_app(controller, robot, cfg)
    app.config.update(TESTING=True)
    yield app, controller, robot
    controller.stop_command_loop()
    robot.close()


@pytest.fixture()
def client(runtime):
    app, _, _ = runtime
    return app.test_client()
