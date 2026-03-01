from __future__ import annotations

import argparse
from pathlib import Path

from .config import load_server_config
from .interfaces import AlgorithmContext
from .ros_node import Ros2ServerBridge
from .services.controller_service import ControllerService
from .services.robot_client import MockRobotClient
from .web.app_factory import create_app


def default_config_path() -> str:
    return str(Path(__file__).resolve().parent.parent / "config" / "server.yaml")


def build_runtime(config_path: str):
    cfg = load_server_config(config_path)
    robot = MockRobotClient(ip=cfg.robot.ip)
    context = AlgorithmContext(
        max_speed_normal=cfg.control.max_speed_normal,
        max_speed_fast=cfg.control.max_speed_fast,
        max_steering=cfg.control.max_steering,
        head_sensitivity=cfg.control.head_sensitivity,
    )
    controller = ControllerService(robot, context=context)
    app = create_app(controller, robot, cfg)
    return cfg, app, controller, robot


def main() -> None:
    parser = argparse.ArgumentParser(description="Server runtime (ROS2 package)")
    parser.add_argument("--config", default=default_config_path(), help="Путь к YAML конфигу")
    args, _ = parser.parse_known_args()  # unknown args (e.g. --ros-args) передаются launch'ем

    cfg, app, controller, robot = build_runtime(args.config)
    ros_bridge = Ros2ServerBridge()
    ros_bridge.start()
    controller.start_command_loop(cfg.app.command_hz)

    try:
        app.run(host=cfg.app.host, port=cfg.app.port, debug=False, threaded=True)
    finally:
        controller.stop_command_loop()
        robot.close()
        ros_bridge.stop()


if __name__ == "__main__":
    main()
