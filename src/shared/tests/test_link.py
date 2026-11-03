import threading
import time
from pathlib import Path

from leo_client.link import ClientLink, local_ipv4_toward
from leo_protocol.protocol import Command, Hello
from leo_server.common.config import load_config
from leo_server.nodes.robot_bridge.link import ServerLink, peer_ipv4, reply_bytes

CFG = load_config(Path(__file__).resolve().parents[3] / "config" / "robot.yaml")


def test_local_address_toward_loopback():
    assert local_ipv4_toward("127.0.0.1") == "127.0.0.1"


def test_peer_address_keeps_the_ipv4_host():
    class _Frame:
        def __init__(self, value):
            self._value = value

        def get(self, key):
            assert key == "Peer-Address"
            return self._value

    assert peer_ipv4(_Frame("tcp://192.168.31.71:40000")) == "192.168.31.71"
    assert peer_ipv4(_Frame(b"tcp://192.168.31.232:40000")) == "192.168.31.232"
    assert peer_ipv4(_Frame("tcp://[::ffff:192.168.31.6]:40000")) == "192.168.31.6"
    assert peer_ipv4(_Frame("tcp://[::ffff:192.168.31.6]:40000".encode())) == "192.168.31.6"
    assert peer_ipv4(_FrameEmpty()) == ""


class _FrameEmpty:
    def get(self, key):
        return None


def test_session_handshake_delivers_config_and_a_command():
    network = CFG.network
    server = ServerLink(
        "127.0.0.1",
        network["telemetry_port"],
        network["handshake_port"],
        network["command_port"],
        "10.1.1.1",
    )

    def handler(hello, server_time_ns):
        sim = CFG.sim_payload(hello.world) if hello.sim else None
        return reply_bytes(server_time_ns, hello, CFG.client_payload(), sim, network)

    server.start(handler)
    time.sleep(0.15)
    try:
        client = ClientLink("127.0.0.1")
        reply = client.handshake(Hello(time.monotonic_ns(), True, "room"))
        assert reply["client"]["calibration"]["steering_invert"] is True
        assert reply["sim"]["world"]["start_x"] == CFG.sim["worlds"]["room"]["start_x"]
        assert server.last_peer == "127.0.0.1"
        server.set_robot_ip(server.last_peer)
        assert server.robot_ip == "127.0.0.1"
        client.open_streams(network["telemetry_port"], network["command_port"])
        time.sleep(0.2)
        found = None
        for _ in range(30):
            server.send_command(Command(4, 5, 0.4, 0.1, 12.0, -3.0))
            found = client.poll_command()
            if found is not None:
                break
            time.sleep(0.03)
        assert found is not None
        assert found.speed_mps == 0.4
        assert found.head_pan_deg == 12.0
        client.close()
    finally:
        server.close()
