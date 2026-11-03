"""Server side of the client session."""

from __future__ import annotations

import threading
import time

import zmq

from leo_protocol.duckdns import is_ipv4
from leo_protocol.protocol import Telemetry, pack_command, pack_error, pack_reply, unpack_hello, unpack_telemetry


class ServerLink:
    def __init__(self, bind: str, telemetry_port: int, handshake_port: int, command_port: int, robot_ip: str) -> None:
        self._ctx = zmq.Context()
        self._telemetry = self._ctx.socket(zmq.SUB)
        self._telemetry.setsockopt(zmq.SUBSCRIBE, b"")
        self._telemetry.setsockopt(zmq.RCVHWM, 20)
        self._telemetry.setsockopt(zmq.LINGER, 0)
        self._telemetry.bind(f"tcp://{bind}:{telemetry_port}")
        self._commands = None
        self._command_port = int(command_port)
        self._robot_ip = ""
        self._lock = threading.Lock()
        self._retarget(robot_ip.strip())
        self._handshake_addr = f"tcp://{bind}:{handshake_port}"
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.last_peer = ""

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
                frame = socket.recv(copy=False)
            except zmq.Again:
                continue
            self._remember_peer(frame)
            try:
                hello = unpack_hello(frame.bytes)
                socket.send(handler(hello, time.time_ns()))
            except Exception as exc:
                socket.send(pack_error(str(exc)))
        socket.close(linger=0)

    @property
    def robot_ip(self) -> str:
        return self._robot_ip

    def set_robot_ip(self, robot_ip: str) -> None:
        """Point the command socket at ``robot_ip`` before the next command goes out."""
        robot_ip = robot_ip.strip()
        if not robot_ip:
            return
        with self._lock:
            if robot_ip == self._robot_ip:
                return
            self._retarget(robot_ip)

    def _retarget(self, robot_ip: str) -> None:
        fresh = self._ctx.socket(zmq.PUB)
        fresh.setsockopt(zmq.SNDHWM, 4)
        fresh.setsockopt(zmq.LINGER, 0)
        fresh.setsockopt(zmq.RECONNECT_IVL, 100)
        fresh.setsockopt(zmq.RECONNECT_IVL_MAX, 1000)
        if robot_ip:
            fresh.connect(f"tcp://{robot_ip}:{self._command_port}")
        previous = self._commands
        self._commands = fresh
        self._robot_ip = robot_ip
        if previous is not None:
            previous.close(linger=0)

    def _remember_peer(self, frame) -> None:
        ip = peer_ipv4(frame)
        if ip:
            self.last_peer = ip

    def send_command(self, command) -> None:
        with self._lock:
            self._commands.send(pack_command(command), flags=zmq.NOBLOCK)

    def poll(self) -> Telemetry | None:
        try:
            frames = self._telemetry.recv_multipart(flags=zmq.NOBLOCK, copy=False)
        except zmq.Again:
            return None
        self._remember_peer(frames[0])
        return unpack_telemetry(frames[0].bytes, frames[1].bytes)

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        with self._lock:
            self._telemetry.close(linger=0)
            self._commands.close(linger=0)
        self._ctx.term()


def peer_ipv4(frame) -> str:
    """Return the IPv4 of the peer that sent this frame."""
    try:
        raw = frame.get("Peer-Address")
    except Exception:
        return ""
    if not raw:
        return ""
    text = raw.decode("utf-8", errors="replace") if isinstance(raw, (bytes, bytearray)) else str(raw)
    return ipv4_host(text)


def ipv4_host(endpoint: str) -> str:
    """Return an IPv4 address from a ZMQ peer endpoint, including IPv4-mapped IPv6."""
    host = endpoint.strip()
    if "://" in host:
        host = host.split("://", 1)[1]
    if host.startswith("["):
        end = host.find("]")
        host = host[1:end] if end > 1 else ""
    elif host.count(":") == 1:
        host = host.rsplit(":", 1)[0]
    host = host.strip().lower()
    if host.startswith("::ffff:"):
        host = host[7:]
        if host.count(":") == 1:
            host = host.rsplit(":", 1)[0]
    if is_ipv4(host):
        return host
    return ""


def reply_bytes(server_time_ns: int, hello, client_cfg: dict, sim_cfg: dict | None, ports: dict) -> bytes:
    return pack_reply(server_time_ns, hello.client_time_ns, client_cfg, sim_cfg, ports)
