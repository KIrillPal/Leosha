from __future__ import annotations

from pathlib import Path
import os

import pytest

from client.actuators import MockActuatorDriver
from client.config import load_client_config
from client.network import InMemoryBridge, MockServer
from client.network.mock_server import MockServerConfig
from client.runtime import ClientRuntime


@pytest.fixture
def client_config():
    cfg_path = Path(__file__).resolve().parent.parent / "config" / "client.yaml"
    return load_client_config(str(cfg_path))


@pytest.fixture
def hardware_config():
    cfg_env = os.getenv("LEOSHA_HARDWARE_CONFIG", "")
    if cfg_env:
        cfg_path = Path(cfg_env)
    else:
        cfg_path = Path(__file__).resolve().parent.parent / "config" / "client.hardware.yaml"
    return load_client_config(str(cfg_path))


@pytest.fixture
def require_hardware():
    if os.getenv("LEOSHA_RUN_HARDWARE_TESTS", "0") != "1":
        pytest.skip("hardware tests disabled (set LEOSHA_RUN_HARDWARE_TESTS=1)")


@pytest.fixture
def teleop_runtime(client_config):
    server = MockServer(MockServerConfig())
    bridge = InMemoryBridge(server)
    actuators = MockActuatorDriver()
    runtime = ClientRuntime(bridge=bridge, actuator_driver=actuators, config=client_config)
    return runtime, bridge, server, actuators

