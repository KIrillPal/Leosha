"""Server side of the client session."""

from __future__ import annotations

import threading
import time

import zmq

from leo_protocol.protocol import Telemetry, pack_command, pack_error, pack_reply, unpack_hello, unpack_telemetry


class ServerLink:
    def __init__(self, bind: str, telemetry_port: int, handshake_port: int, command_port: int, robot_ip: str) -> None:
        self._ctx = zmq.Context()
        self._telemetry = self._ctx.socket(zmq.SUB)
        self._telemetry.setsockopt(zmq.SUBSCRIBE, b"")
        self._telemetry.setsockopt(zmq.RCVHWM, 20)
        self._telemetry.setsockopt(zmq.LINGER, 0)
        self._telemetry.bind(f"tcp://{bind}:{telemetry_port}")
        self._commands = self._ctx.socket(zmq.PUB)
        self._commands.setsockopt(zmq.SNDHWM, 4)
        self._commands.setsockopt(zmq.LINGER, 0)
        self._command_port = int(command_port)
        self._robot_ip = robot_ip
        self._commands.connect(f"tcp://{robot_ip}:{command_port}")
        self._handshake_addr = f"tcp://{bind}:{handshake_port}"
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._next_ip: str | None = None

    def start(self, handler) -> None:
        self._thread = threading.Thread(target=self._handshake_loop, args=(handler,), name="handshake", daemon=True)
        self._thread.start()

    def _handshake_loop(self, handler) -> None:
        socket = self._ctx.socket(zmq.REP)
        socket.setsockopt(zmq.RCVTIMEO, 500)
        socket.setsockopt(zmq.LINGER, 0)
        socket.bind(self._handshake_addr)
        while not self._stop.is_set():
            try:
                payload = socket.recv()
            except zmq.Again:
                continue
            try:
                hello = unpack_hello(payload)
                socket.send(handler(hello, time.time_ns()))
            except Exception as exc:
                socket.send(pack_error(str(exc)))
        socket.close(linger=0)

    def set_robot_ip(self, robot_ip: str) -> None:
        self._next_ip = robot_ip

    def _apply_ip(self) -> None:
        robot_ip = self._next_ip
        if robot_ip is None or robot_ip == self._robot_ip:
            self._next_ip = None
            return
        self._commands.disconnect(f"tcp://{self._robot_ip}:{self._command_port}")
        self._robot_ip = robot_ip
        self._commands.connect(f"tcp://{robot_ip}:{self._command_port}")
        self._next_ip = None

    def send_command(self, command) -> None:
        self._apply_ip()
        self._commands.send(pack_command(command), flags=zmq.NOBLOCK)

    def poll(self) -> Telemetry | None:
        try:
            header, jpeg = self._telemetry.recv_multipart(flags=zmq.NOBLOCK)
        except zmq.Again:
            return None
        return unpack_telemetry(header, jpeg)

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._telemetry.close(linger=0)
        self._commands.close(linger=0)
        self._ctx.term()


def reply_bytes(server_time_ns: int, hello, client_cfg: dict, sim_cfg: dict | None, ports: dict) -> bytes:
    return pack_reply(server_time_ns, hello.client_time_ns, client_cfg, sim_cfg, ports)
