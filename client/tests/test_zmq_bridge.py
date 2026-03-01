from __future__ import annotations

import socket
import time

import pytest

from client.models import (
    ImuReading,
    OperatingMode,
    Pose2D,
    RobotStatus,
    SensorStatus,
    SensorTimingStats,
    ServerPacket,
    ServerPacketHeader,
    TelemetryPacket,
    TeleoperationCommand,
    Twist2D,
    WheelOdometry,
)
from client.network.bridge import ZmqBridge
from client.network.serialization import pack_server_packet

zmq = pytest.importorskip("zmq", reason="pyzmq is required for ZeroMQ test")


def _free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def test_zmq_bridge_send_and_receive():
    telemetry_port = _free_port()
    command_port = _free_port()
    report_port = _free_port()

    ctx = zmq.Context.instance()

    telemetry_sub = ctx.socket(zmq.SUB)
    telemetry_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    telemetry_sub.bind(f"tcp://127.0.0.1:{telemetry_port}")

    command_pub = ctx.socket(zmq.PUB)
    command_pub.bind(f"tcp://127.0.0.1:{command_port}")

    report_sub = ctx.socket(zmq.SUB)
    report_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    report_sub.bind(f"tcp://127.0.0.1:{report_port}")

    bridge = ZmqBridge(
        server_host="127.0.0.1",
        telemetry_port=telemetry_port,
        command_port=command_port,
        report_port=report_port,
    )
    time.sleep(0.2)

    outgoing = ServerPacket(
        header=ServerPacketHeader(seq=7, timestamp_ns=123, mode=OperatingMode.TELEOPERATION),
        teleop_cmd=TeleoperationCommand(speed=0.2, steering=-0.1, head_pan=0.0, head_tilt=0.0),
    )
    command_pub.send(pack_server_packet(outgoing))
    incoming = None
    for _ in range(20):
        time.sleep(0.02)
        incoming = bridge.recv_packet()
        if incoming is not None:
            break
    assert incoming is not None
    assert incoming.header.seq == 7

    telemetry = TelemetryPacket(
        seq=1,
        timestamp_ns=999,
        mode=OperatingMode.TELEOPERATION,
        status=RobotStatus.RUNNING,
        frame_jpeg=b"\xff\xd8\xff\xd9",
        scan=None,
        imu_readings=[ImuReading(1, 0.0, 0.0, 9.81, 0.0, 0.0, 0.1)],
        odometry=WheelOdometry(1, Pose2D(), Twist2D(), 0.0),
        ultrasonic_range_m=1.2,
        battery_voltage=11.8,
        cpu_temp_c=50.0,
        wifi_rssi_dbm=-40.0,
        sensor_status={
            "camera": SensorStatus("camera", False, False, False),
        },
        sensor_timing={
            "camera": SensorTimingStats("camera"),
        },
    )
    poller = zmq.Poller()
    poller.register(telemetry_sub, zmq.POLLIN)
    got_telemetry = False
    for _ in range(30):
        bridge.send_telemetry(telemetry)
        events = dict(poller.poll(timeout=100))
        if telemetry_sub in events:
            got_telemetry = True
            break
    assert got_telemetry
    parts = telemetry_sub.recv_multipart(flags=zmq.NOBLOCK)
    assert len(parts) == 2
    assert parts[1] == b"\xff\xd8\xff\xd9"

    poller.unregister(telemetry_sub)
    poller.register(report_sub, zmq.POLLIN)
    got_report = False
    for _ in range(20):
        bridge.send_missing_packet_report(123.0)
        events = dict(poller.poll(timeout=100))
        if report_sub in events:
            got_report = True
            break
    assert got_report
    report_raw = report_sub.recv(flags=zmq.NOBLOCK)
    assert report_raw

    bridge.close()
    telemetry_sub.close(0)
    command_pub.close(0)
    report_sub.close(0)

