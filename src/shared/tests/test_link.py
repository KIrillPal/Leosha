import threading
import time
from pathlib import Path

from leo_client.link import ClientLink
from leo_protocol.protocol import Command, Hello
from leo_server.common.config import load_config
from leo_server.nodes.robot_bridge.link import ServerLink, reply_bytes

CFG = load_config(Path(__file__).resolve().parents[3] / "config" / "robot.yaml")


def test_session_handshake_delivers_config_and_a_command():
    network = CFG.network
    server = ServerLink(
        "127.0.0.1",
        network["telemetry_port"],
        network["handshake_port"],
        network["command_port"],
        "127.0.0.1",
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
