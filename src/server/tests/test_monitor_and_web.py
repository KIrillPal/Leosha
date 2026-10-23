import time
from pathlib import Path

from leo_server.nodes.monitor.app import create_monitor_app
from leo_server.nodes.monitor.graph import Edge, build_view
from leo_server.nodes.web_ui.app import UiState, create_app


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
