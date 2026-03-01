from __future__ import annotations

import argparse
from pathlib import Path
from time import sleep

from .actuators import MockActuatorDriver, Pca9685ActuatorDriver
from .config import load_client_config
from .network import InMemoryBridge, MockServer, ZmqBridge
from .runtime import ClientRuntime


def default_config_path() -> str:
    return str(Path(__file__).resolve().parent.parent / "config" / "client.yaml")


def main() -> None:
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


if __name__ == "__main__":
    main()

