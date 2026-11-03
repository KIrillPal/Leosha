import time
from pathlib import Path

from leo_server.nodes.monitor.app import create_monitor_app
from leo_server.nodes.monitor.graph import Edge, build_view
from leo_server.nodes.web_ui.app import UiState, create_app, robot_status


def test_graph_marks_a_quiet_node_stale_and_keeps_client_fps_separate():
    view = build_view(
        {"robot_bridge": {"fps": 40, "stamp": time.time() - 10}},
        [Edge("/scan", "robot_bridge", "odometry", 9.5)],
        {"system_fps": 28.0, "client": {"loop": 100, "camera": 15, "lidar": 10}, "latency_ms": 12},
        time.time(),
        2.5,
    )
    bridge = next(node for node in view["nodes"] if node["name"] == "robot_bridge")
    assert bridge["stale"] is True
    assert view["system_fps"] == 28.0
    assert view["client"]["loop"] == 100
    assert view["edges"][0]["hz"] == 9.5


def test_monitor_serves_the_graph_at_the_site_root():
    app = create_monitor_app("<html>graph</html>", lambda: {"nodes": []})
    client = app.test_client()
    assert client.get("/").status_code == 200
    assert client.get("/graph").status_code == 200
    assert b"graph" in client.get("/").data
    assert client.get("/api/graph").get_json() == {"nodes": []}


def test_monitor_page_matches_the_site_and_the_architecture_layout():
    page = Path(__file__).resolve().parents[1] / "leo_server" / "nodes" / "monitor" / "page.html"
    app = create_monitor_app(page.read_text(encoding="utf-8"), lambda: {"nodes": [], "edges": [], "client": {}}, 8080)
    html = app.test_client().get("/").data
    assert b"topbar" in html
    assert b"tabs" in html
    assert b"tab active" in html
    assert b"Node graph" in html
    assert b":8080/teleop" in html
    for name in (
        b"leo_client",
        b"robot_bridge",
        b"odometry",
        b"vision",
        b"mode_manager",
        b"slam_toolbox",
        b"web_ui",
        b"monitor",
    ):
        assert name in html


def test_settings_page_keeps_the_known_robot_report():
    state = UiState(["pause"], "10.0.0.1")
    state.note_reply(0.1, {"lidar": True, "camera": False})
    html = create_app(state, lambda *_args: None).test_client().get("/").data
    assert b'id="robotStatus">Connected' in html
    assert b"lidar" in html
    assert b"camera" in html
    assert b">ok<" in html
    assert b">failed<" in html


def test_settings_renders_the_live_robot_ip():
    state = UiState(["pause"], "192.168.31.6")
    html = create_app(state, lambda *_args: None).test_client().get("/").data
    assert b'value="192.168.31.6"' in html
    assert b"data.robot_ip" in html
    assert b"robotStatus" in html
    assert b"Modules" in html


def test_robot_status_waits_then_connects_then_goes_quiet():
    assert robot_status(None, 2.0) == "Waiting"
    assert robot_status(2.0, 2.0) == "Connected"
    assert robot_status(2.01, 2.0) == "Not responding"


def test_status_reports_modules_in_display_order():
    state = UiState(["pause"], "10.0.0.1", reply_timeout_sec=2.0)
    state.note_reply(0.1, {"face servo": False, "lidar": True, "camera": True})
    payload = create_app(state, lambda *_args: None).test_client().get("/api/status").get_json()
    assert payload["robot_status"] == "Connected"
    assert payload["modules"] == [
        {"name": "lidar", "ok": True},
        {"name": "camera", "ok": True},
        {"name": "face servo", "ok": False},
    ]
    state.note_reply(3.0, {"lidar": True})
    assert state.status_payload()["robot_status"] == "Not responding"


def test_status_reports_the_robot_ip_held_in_memory():
    state = UiState(["pause"], "10.0.0.1")
    client = create_app(state, lambda *_args: None).test_client()
    assert client.get("/api/status").get_json()["robot_ip"] == "10.0.0.1"
    state.robot_ip = "192.168.31.71"
    assert client.get("/api/status").get_json()["robot_ip"] == "192.168.31.71"


def test_web_pages_only_publish():
    sent = []
    state = UiState(["pause", "teleoperation"], "127.0.0.1")
    app = create_app(state, lambda kind, payload: sent.append((kind, payload)))
    client = app.test_client()
    home = client.get("/")
    assert home.status_code == 200
    assert b"topbar" in home.data
    assert b"tab active" in home.data
    teleop = client.get("/teleop")
    assert teleop.status_code == 200
    assert b"requestPointerLock" in teleop.data
    assert b"Escape" in teleop.data
    assert client.post("/api/mode", json={"mode": "teleoperation"}).status_code == 200
    assert client.post("/api/input", json={"w": True}).status_code == 200
    assert ("mode", "teleoperation") in sent
    assert sent[-1][0] == "input"
