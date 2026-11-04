from leo_server.nodes.robot_bridge.node import RobotBridge


def test_a_quiet_robot_is_announced_once(monkeypatch):
    written = []
    monkeypatch.setattr("leo_server.nodes.robot_bridge.node.console.write", written.append)
    now = [0.0]
    monkeypatch.setattr("leo_server.nodes.robot_bridge.node.time.monotonic", lambda: now[0])
    bridge = RobotBridge.__new__(RobotBridge)
    bridge._connected_ip = "100.98.9.234"
    bridge._last_reply_at = 0.0
    bridge._silence_sec = 1.5

    now[0] = 1.4
    bridge._note_silence()
    assert written == []

    now[0] = 1.5
    bridge._note_silence()
    bridge._note_silence()
    assert bridge._connected_ip == ""
    assert len(written) == 1
    assert "100.98.9.234" in written[0]
    assert "disconnected" in written[0]
