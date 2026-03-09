from __future__ import annotations

import argparse
import logging
from pathlib import Path

from .config import load_server_config
from .interfaces import AlgorithmContext
from .logging_setup import configure_pipeline_logging
from .ros_node import Ros2ServerBridge
from .services.controller_service import ControllerService
from .services.robot_client import MockRobotClient, ZmqRobotClient
from .web.app_factory import create_app

LOGGER = logging.getLogger(__name__)


def _configs_dir() -> Path:
    return Path(__file__).resolve().parents[2] / "configs"


def default_config_path() -> str:
    return str(_configs_dir() / "server.yaml")


def _create_robot_client(cfg, transport: str):
    """Instantiate the correct RobotClient backend based on --transport flag or config."""
    backend = transport or cfg.robot.backend
    if backend == "zmq":
        return ZmqRobotClient(
            ip=cfg.robot.ip,
            bind_address=cfg.network.bind_address,
            telemetry_port=cfg.network.telemetry_port,
            command_port=cfg.network.command_port,
            report_port=cfg.network.report_port,
            recv_timeout_ms=cfg.network.recv_timeout_ms,
            send_high_water_mark=cfg.network.send_high_water_mark,
            recv_high_water_mark=cfg.network.recv_high_water_mark,
        )
    return MockRobotClient(ip=cfg.robot.ip)


def build_runtime(config_path: str, transport: str = ""):
    cfg = load_server_config(config_path)
    robot = _create_robot_client(cfg, transport)
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
    log_path = configure_pipeline_logging("server_pipeline")
    parser = argparse.ArgumentParser(description="Server runtime (ROS2 package)")
    parser.add_argument("--config", default=default_config_path(), help="Путь к YAML конфигу")
    parser.add_argument(
        "--transport",
        choices=["mock", "zmq"],
        default="",
        help="Транспорт до робота: mock (без сети) или zmq (реальный ZeroMQ). "
             "Если не указан — берётся из конфига (robot.backend).",
    )
    args, _ = parser.parse_known_args()
    LOGGER.info(
        "Starting server pipeline | config=%s transport=%s log_file=%s",
        args.config, args.transport or "(from config)", log_path,
    )

    cfg, app, controller, robot = build_runtime(args.config, args.transport)
    ros_bridge = Ros2ServerBridge()
    ros_bridge.start()
    controller.start_command_loop(cfg.app.command_hz)

    try:
        app.run(host=cfg.app.host, port=cfg.app.port, debug=False, threaded=True)
    finally:
        controller.stop_command_loop()
        robot.close()
        ros_bridge.stop()
        LOGGER.info("Server pipeline stopped")


if __name__ == "__main__":
    main()
