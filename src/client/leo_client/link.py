"""ZeroMQ sockets for one robot session.

Telemetry: client PUB connects, server SUB binds.
Commands: client SUB binds, server PUB connects.
Handshake: client REQ connects, server REP binds.
"""

from __future__ import annotations

import socket

import zmq

from leo_protocol.duckdns import is_ipv4
from leo_protocol.protocol import HANDSHAKE_PORT, Command, Hello, Telemetry, pack_hello, pack_telemetry, unpack_command, unpack_reply


def local_ipv4_toward(host: str) -> str:
    """Return the source IPv4 used to reach ``host``."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((host, 9))
        address = probe.getsockname()[0]
    except OSError:
        return ""
    finally:
        probe.close()
    if is_ipv4(address):
        return address
    return ""


class ClientLink:
    def __init__(self, server_host: str, timeout_ms: int = 3000) -> None:
        self._host = server_host
        self._ctx = zmq.Context()
        self._hello = self._ctx.socket(zmq.REQ)
        self._hello.setsockopt(zmq.RCVTIMEO, int(timeout_ms))
        self._hello.setsockopt(zmq.SNDTIMEO, int(timeout_ms))
        self._hello.setsockopt(zmq.LINGER, 0)
        self._hello.connect(f"tcp://{server_host}:{HANDSHAKE_PORT}")
        self._telemetry = None
        self._commands = None

    @property
    def host(self) -> str:
        return self._host

    def handshake(self, hello: Hello) -> dict:
        self._hello.send(pack_hello(hello))
        return unpack_reply(self._hello.recv())

    def open_streams(self, telemetry_port: int, command_port: int) -> None:
        self._telemetry = self._ctx.socket(zmq.PUB)
        self._telemetry.setsockopt(zmq.SNDHWM, 4)
        self._telemetry.setsockopt(zmq.LINGER, 0)
        self._telemetry.connect(f"tcp://{self._host}:{telemetry_port}")
        self._commands = self._ctx.socket(zmq.SUB)
        self._commands.setsockopt(zmq.SUBSCRIBE, b"")
        self._commands.setsockopt(zmq.CONFLATE, 1)
        self._commands.setsockopt(zmq.RCVHWM, 1)
        self._commands.setsockopt(zmq.LINGER, 0)
        self._commands.bind(f"tcp://0.0.0.0:{command_port}")

    def send(self, packet: Telemetry) -> None:
        if self._telemetry is None:
            raise RuntimeError("streams are not open")
        header, jpeg = pack_telemetry(packet)
        self._telemetry.send_multipart((header, jpeg), flags=zmq.NOBLOCK)

    def poll_command(self) -> Command | None:
        if self._commands is None:
            return None
        try:
            payload = self._commands.recv(flags=zmq.NOBLOCK)
        except zmq.Again:
            return None
        return unpack_command(payload)

    def close(self) -> None:
        for socket in (self._hello, self._telemetry, self._commands):
            if socket is not None:
                socket.close(linger=0)
        self._ctx.term()
