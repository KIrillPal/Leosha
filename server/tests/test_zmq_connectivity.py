"""Connectivity test: verifies that a real robot client is reachable and streaming telemetry.

Usage:
    pytest tests/test_zmq_connectivity.py --robot-ip 192.168.31.232 -v
    pytest tests/test_zmq_connectivity.py --robot-ip 192.168.31.232 --telemetry-port 5550 -v

The test binds ZMQ SUB on the specified ports, waits for telemetry multipart
messages from the robot, and asserts that:
  1. At least one telemetry packet arrives within the timeout.
  2. The packet contains a valid msgpack header + JPEG frame.
  3. Multiple packets arrive steadily (the client is alive, not a single fluke).
"""
from __future__ import annotations

import time

import msgpack
import pytest
import zmq


@pytest.fixture(scope="module")
def robot_ip(request):
    ip = request.config.getoption("--robot-ip")
    if not ip:
        pytest.skip("--robot-ip not provided, skipping connectivity tests")
    return ip


@pytest.fixture(scope="module")
def telemetry_port(request):
    return int(request.config.getoption("--telemetry-port"))


@pytest.fixture(scope="module")
def command_port(request):
    return int(request.config.getoption("--command-port"))


@pytest.fixture(scope="module")
def report_port(request):
    return int(request.config.getoption("--report-port"))


@pytest.fixture(scope="module")
def wait_sec(request):
    return float(request.config.getoption("--wait-sec"))


@pytest.fixture(scope="module")
def min_packets(request):
    return int(request.config.getoption("--min-packets"))


@pytest.fixture(scope="module")
def zmq_telemetry_sub(telemetry_port):
    """Bind a ZMQ SUB socket to receive telemetry from the robot."""
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    sock.setsockopt(zmq.RCVHWM, 5)
    sock.bind(f"tcp://0.0.0.0:{telemetry_port}")
    yield sock
    sock.close(linger=0)
    ctx.term()


@pytest.fixture(scope="module")
def zmq_command_pub(command_port):
    """Bind a ZMQ PUB socket to send commands to the robot."""
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 2)
    sock.bind(f"tcp://0.0.0.0:{command_port}")
    yield sock
    sock.close(linger=0)
    ctx.term()


@pytest.fixture(scope="module")
def zmq_report_sub(report_port):
    """Bind a ZMQ SUB socket to receive missing-packet reports from the robot."""
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    sock.setsockopt(zmq.RCVHWM, 5)
    sock.bind(f"tcp://0.0.0.0:{report_port}")
    yield sock
    sock.close(linger=0)
    ctx.term()


def _recv_telemetry_packet(sock: zmq.Socket, timeout_ms: int = 2000):
    """Try to receive one multipart telemetry packet within timeout."""
    if sock.poll(timeout_ms, zmq.POLLIN):
        parts = sock.recv_multipart(flags=zmq.NOBLOCK)
        return parts
    return None


class TestClientConnectivity:
    """Suite of tests verifying the robot client is alive and streaming."""

    def test_first_packet_arrives(self, zmq_telemetry_sub, zmq_command_pub, robot_ip, wait_sec):
        """The robot must send at least one telemetry packet within the wait window."""
        deadline = time.monotonic() + wait_sec
        packet = None
        while time.monotonic() < deadline:
            packet = _recv_telemetry_packet(zmq_telemetry_sub, timeout_ms=1000)
            if packet is not None:
                break
        assert packet is not None, (
            f"No telemetry received from {robot_ip} within {wait_sec}s. "
            "Check that the client is running and configured to connect to this server."
        )
        assert len(packet) >= 2, "Telemetry must be multipart: [header, frame_jpeg]"

    def test_packet_has_valid_header(self, zmq_telemetry_sub, zmq_command_pub, robot_ip, wait_sec):
        """Each telemetry packet header must be valid msgpack with expected fields."""
        deadline = time.monotonic() + wait_sec
        packet = None
        while time.monotonic() < deadline:
            packet = _recv_telemetry_packet(zmq_telemetry_sub, timeout_ms=1000)
            if packet is not None:
                break
        assert packet is not None, f"No packet from {robot_ip}"

        header_raw, frame_jpeg = packet[0], packet[1]
        header = msgpack.unpackb(header_raw, raw=False)

        assert "seq" in header, "Header missing 'seq'"
        assert "timestamp_ns" in header, "Header missing 'timestamp_ns'"
        assert "mode" in header, "Header missing 'mode'"
        assert "status" in header, "Header missing 'status'"
        assert isinstance(header["seq"], int), "seq must be int"
        assert isinstance(header["timestamp_ns"], int), "timestamp_ns must be int"

    def test_frame_is_jpeg(self, zmq_telemetry_sub, zmq_command_pub, robot_ip, wait_sec):
        """Frame payload must start with JPEG magic bytes (FFD8)."""
        deadline = time.monotonic() + wait_sec
        packet = None
        while time.monotonic() < deadline:
            packet = _recv_telemetry_packet(zmq_telemetry_sub, timeout_ms=1000)
            if packet is not None:
                break
        assert packet is not None, f"No packet from {robot_ip}"

        frame_jpeg = packet[1]
        frame_size = len(frame_jpeg)
        assert frame_size > 0, "Frame is empty"
        if frame_size > 2:
            assert frame_jpeg[:2] == b"\xff\xd8", (
                f"Frame does not start with JPEG SOI marker (got {frame_jpeg[:2].hex()})"
            )

    def test_steady_stream(self, zmq_telemetry_sub, zmq_command_pub, robot_ip, wait_sec, min_packets):
        """Multiple packets must arrive within a reasonable window — client is continuously alive."""
        received = []
        deadline = time.monotonic() + wait_sec
        while time.monotonic() < deadline and len(received) < min_packets:
            packet = _recv_telemetry_packet(zmq_telemetry_sub, timeout_ms=1000)
            if packet is not None:
                header = msgpack.unpackb(packet[0], raw=False)
                received.append(header)

        assert len(received) >= min_packets, (
            f"Expected at least {min_packets} packets from {robot_ip}, "
            f"got {len(received)} within {wait_sec}s"
        )

        seqs = [h["seq"] for h in received]
        assert seqs == sorted(seqs), f"Sequence numbers not monotonically increasing: {seqs}"

    def test_sequence_incrementing(self, zmq_telemetry_sub, zmq_command_pub, robot_ip, wait_sec, min_packets):
        """Sequence numbers must be strictly increasing (no duplicates or resets)."""
        received = []
        deadline = time.monotonic() + wait_sec
        while time.monotonic() < deadline and len(received) < min_packets:
            packet = _recv_telemetry_packet(zmq_telemetry_sub, timeout_ms=1000)
            if packet is not None:
                header = msgpack.unpackb(packet[0], raw=False)
                received.append(header["seq"])

        assert len(received) >= 2, f"Need at least 2 packets, got {len(received)}"
        for i in range(1, len(received)):
            assert received[i] > received[i - 1], (
                f"seq not strictly increasing: {received[i-1]} -> {received[i]}"
            )

    def test_telemetry_contains_sensor_data(self, zmq_telemetry_sub, zmq_command_pub, robot_ip, wait_sec):
        """Telemetry header should contain odometry and sensor fields."""
        deadline = time.monotonic() + wait_sec
        packet = None
        while time.monotonic() < deadline:
            packet = _recv_telemetry_packet(zmq_telemetry_sub, timeout_ms=1000)
            if packet is not None:
                break
        assert packet is not None, f"No packet from {robot_ip}"

        header = msgpack.unpackb(packet[0], raw=False)
        assert "odometry" in header, "Header missing 'odometry'"
        odom = header["odometry"]
        assert "pose" in odom, "Odometry missing 'pose'"
        assert "velocity" in odom, "Odometry missing 'velocity'"
