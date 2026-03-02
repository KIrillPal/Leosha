from __future__ import annotations

import argparse
import logging
from pathlib import Path
from time import sleep

from .actuators import MockActuatorDriver, Pca9685ActuatorDriver
from .config import load_client_config
from .logging_setup import configure_pipeline_logging
from .network import InMemoryBridge, MockServer, ZmqBridge
from .runtime import ClientRuntime

LOGGER = logging.getLogger(__name__)


def _configs_dir() -> Path:
    """Папка configs рядом с client: code/configs (из code/client/client/ на 2 уровня вверх = code)."""
    return Path(__file__).resolve().parents[2] / "configs"


def default_config_path() -> str:
    return str(_configs_dir() / "client.yaml")


def main() -> None:
    log_path = configure_pipeline_logging("client_pipeline")
    parser = argparse.ArgumentParser(description="Leosha client runtime")
    parser.add_argument("--config", default=default_config_path(), help="Путь к YAML конфигу")
    parser.add_argument("--duration-sec", type=float, default=5.0, help="Длительность демо запуска")
    parser.add_argument(
        "--transport",
        choices=["mock", "zmq"],
        default="zmq",
        help="Транспорт до сервера: mock или реальный zmq",
    )
    args = parser.parse_args()
    LOGGER.info("Starting client pipeline | config=%s transport=%s log_file=%s", args.config, args.transport, log_path)

    config = load_client_config(args.config)
    if args.transport == "mock":
        mock_server = MockServer()
        bridge = InMemoryBridge(mock_server)
    else:
        bridge = ZmqBridge(
            server_host=config.network.server_host,
            telemetry_port=config.network.telemetry_port,
            command_port=config.network.command_port,
            report_port=config.network.report_port,
            recv_timeout_ms=config.network.recv_timeout_ms,
            send_high_water_mark=config.network.send_high_water_mark,
            recv_high_water_mark=config.network.recv_high_water_mark,
        )
    if str(config.actuators.backend).lower() == "pca9685":
        actuators = Pca9685ActuatorDriver(config.actuators)
    else:
        actuators = MockActuatorDriver()
    runtime = ClientRuntime(bridge=bridge, actuator_driver=actuators, config=config)
    runtime.start()
    try:
        sleep(max(0.1, args.duration_sec))
    finally:
        runtime.stop()
        LOGGER.info("Client pipeline stopped")


if __name__ == "__main__":
    main()

