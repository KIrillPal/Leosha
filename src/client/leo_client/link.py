"""ZeroMQ sockets for one robot session.

Telemetry: client PUB connects, server SUB binds.
Commands: client SUB binds, server PUB connects.
Handshake: client REQ connects, server REP binds.
"""

from __future__ import annotations

import zmq

from leo_protocol.protocol import HANDSHAKE_PORT, Command, Hello, Telemetry, pack_hello, pack_telemetry, unpack_command, unpack_reply


class ClientLink:
    def __init__(self, server_host: str) -> None:
        self._host = server_host
        self._ctx = zmq.Context()
        self._hello = self._ctx.socket(zmq.REQ)
        self._hello.setsockopt(zmq.RCVTIMEO, 3000)
        self._hello.setsockopt(zmq.SNDTIMEO, 3000)
        self._hello.setsockopt(zmq.LINGER, 0)
        self._hello.connect(f"tcp://{server_host}:{HANDSHAKE_PORT}")
        self._telemetry = None
        self._commands = None

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
