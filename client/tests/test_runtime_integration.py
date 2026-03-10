from __future__ import annotations

import time

from client.models import OperatingMode, RobotStatus
from client.network.mock_server import MockServerConfig


def test_runtime_sends_telemetry_and_applies_commands(teleop_runtime):
    runtime, bridge, server, actuators = teleop_runtime
    runtime.start()
    time.sleep(0.25)
    runtime.stop()
    assert bridge.stats.tx_packets > 0
    assert bridge.stats.rx_packets > 0
    assert len(bridge.telemetry_history) > 0
    assert len(actuators.history) > 0
    assert bridge.telemetry_history[-1].sensor_timing
    assert bridge.telemetry_history[-1].sensor_status


def test_runtime_emergency_stop_when_packets_missing(client_config):
    from client.actuators import MockActuatorDriver
    from client.network import InMemoryBridge, MockServer
    from client.runtime import ClientRuntime

    cfg = MockServerConfig(mode=OperatingMode.TELEOPERATION, drop_every_nth=1)
    server = MockServer(cfg)
    bridge = InMemoryBridge(server)
    actuators = MockActuatorDriver()
    runtime = ClientRuntime(bridge=bridge, actuator_driver=actuators, config=client_config)
    runtime.watchdog = runtime.watchdog.__class__(warn_ms=1.0, timeout_ms=5.0, critical_ms=20.0)
    runtime.start()
    time.sleep(0.08)
    runtime.stop()
    assert actuators.state.emergency_stop_count > 0
    assert len(server.missing_reports) > 0
    assert runtime.state.snapshot()["status"] in (
        RobotStatus.WAITING_FOR_SERVER,
        RobotStatus.EMERGENCY_STOP,
    )


def test_runtime_accepts_autonomy_mode(client_config):
    from client.actuators import MockActuatorDriver
    from client.network import InMemoryBridge, MockServer
    from client.runtime import ClientRuntime

    cfg = MockServerConfig(mode=OperatingMode.AUTONOMY_PROFILE_1)
    server = MockServer(cfg)
    bridge = InMemoryBridge(server)
    actuators = MockActuatorDriver()
    runtime = ClientRuntime(bridge=bridge, actuator_driver=actuators, config=client_config)
    runtime.start()
    time.sleep(0.2)
    runtime.stop()
    snap = runtime.state.snapshot()
    assert snap["mode"] == OperatingMode.AUTONOMY_PROFILE_1
    assert len(snap["trajectory"]) > 0


def test_camera_can_fail_without_crashing_client(client_config):
    from client.actuators import MockActuatorDriver
    from client.network import InMemoryBridge, MockServer
    from client.runtime import ClientRuntime

    client_config.sensors.camera.enabled = True
    client_config.sensors.camera.fail_policy.max_consecutive_failures = 1
    client_config.sensors.camera.fail_policy.auto_disable_on_fail = True
    client_config.sensors.camera.tuning_file = "/definitely/missing/imx290.json"
    server = MockServer(MockServerConfig(mode=OperatingMode.TELEOPERATION))
    bridge = InMemoryBridge(server)
    actuators = MockActuatorDriver()
    runtime = ClientRuntime(bridge=bridge, actuator_driver=actuators, config=client_config)
    runtime.start()
    time.sleep(0.15)
    runtime.stop()
    camera_status = runtime.sensor_statuses.snapshot()["camera"]
    assert camera_status.sensor_name == "camera"
    assert bridge.stats.tx_packets > 0

