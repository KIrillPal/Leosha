from leo_server.nodes.robot_bridge.node import RobotBridge


def test_a_quiet_robot_is_announced_once(monkeypatch):
    written = []
    monkeypatch.setattr("leo_server.nodes.robot_bridge.node.console.write", written.append)
    now = [0.0]
    monkeypatch.setattr("leo_server.nodes.robot_bridge.node.time.monotonic", lambda: now[0])
    bridge = RobotBridge.__new__(RobotBridge)
    bridge._connected_ip = "100.98.9.234"
    bridge._last_reply_at = 0.0
    bridge._reply_timeout_sec = 2.0

    now[0] = 2.0
    bridge._note_silence()
    assert written == []

    now[0] = 2.01
    bridge._note_silence()
    bridge._note_silence()
    assert bridge._connected_ip == ""
    assert len(written) == 1
    assert "disconnected" in written[0]

    bridge._hear("100.98.9.234")
    bridge._hear("100.98.9.234")
    assert bridge._connected_ip == "100.98.9.234"
    assert sum("connected from" in line for line in written) == 1
