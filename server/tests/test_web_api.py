def test_tabs_available(client):
    assert client.get("/settings").status_code == 200
    assert client.get("/teleop").status_code == 200
    assert client.get("/network").status_code == 200
    assert client.get("/lidar").status_code == 200
    assert client.get("/slam").status_code == 200
    assert client.get("/ros-graph").status_code == 200


def test_settings_and_mode_switch(client):
    response = client.get("/api/settings")
    assert response.status_code == 200
    data = response.get_json()
    assert "available_modes" in data

    set_mode = client.post("/api/mode", json={"mode": "teleoperation"})
    assert set_mode.status_code == 200
    assert set_mode.get_json()["mode"] == "teleoperation"


def test_ping_and_network_stats(client):
    ping = client.post("/api/ping")
    assert ping.status_code == 200
    assert "connected" in ping.get_json()

    stats = client.get("/api/network/stats")
    assert stats.status_code == 200
    payload = stats.get_json()
    assert "latency_ms" in payload
    assert "tx_bytes_per_sec" in payload


def test_teleop_keyboard_and_status(client):
    client.post("/api/tracking", json={"tracking": True})
    press = client.post("/api/keyboard", json={"key": "w", "state": True, "action": "press"})
    assert press.status_code == 200
    payload = press.get_json()
    assert payload["success"] is True

    status = client.get("/api/status").get_json()
    assert status["tracking_enabled"] is True
    assert status["key_states"]["w"] is True


def test_video_feed_mjpeg_boundary(client):
    response = client.get("/video_feed")
    assert response.status_code == 200
    first_chunk = next(response.response)
    assert b"--frame" in first_chunk
