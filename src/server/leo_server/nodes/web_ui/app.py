"""Flask pages. They publish ROS messages through callbacks and never tick the controller."""

from __future__ import annotations

import json

from flask import Flask, Response, jsonify, render_template, request


def create_app(state, publish) -> Flask:
    app = Flask(__name__, template_folder="templates")

    @app.context_processor
    def _ports():
        path = request.path
        tab = "settings" if path == "/" else path.strip("/").split("/")[0]
        return {"monitor_port": state.monitor_port, "active_tab": tab}

    @app.get("/")
    def home():
        return render_template("settings.html", modes=state.modes, mode=state.mode)

    @app.get("/teleop")
    def teleop():
        return render_template("teleop.html")

    @app.get("/lidar")
    def lidar():
        return render_template("lidar.html")

    @app.get("/slam")
    def slam():
        return render_template("slam.html")

    @app.get("/network")
    def network():
        return render_template("network.html")

    @app.post("/api/mode")
    def set_mode():
        mode = request.json["mode"]
        publish("mode", mode)
        return jsonify({"mode": mode})

    @app.post("/api/input")
    def input_state():
        publish("input", request.json or {})
        return jsonify({"ok": True})

    @app.post("/api/ip")
    def set_ip():
        publish("ip", request.json["ip"])
        return jsonify({"ok": True})

    @app.get("/api/status")
    def status():
        with state.lock:
            return jsonify({"mode": state.mode, "ui": state.ui, "stats": state.stats, "robot_ip": state.robot_ip})

    @app.get("/api/lidar")
    def lidar_data():
        with state.lock:
            scan = state.scan
        if scan is None:
            return jsonify({"ranges": [], "angle_min": 0, "angle_increment": 0})
        return jsonify({"ranges": scan["ranges"], "angle_min": scan["angle_min"], "angle_increment": scan["angle_increment"]})

    @app.get("/api/map")
    def map_image():
        with state.lock:
            png = state.map_png
        return Response(png, mimetype="image/png")

    @app.get("/video_feed")
    def video():
        return Response(_frames(state), mimetype="multipart/x-mixed-replace; boundary=frame")

    return app


def _frames(state):
    import time

    while True:
        with state.lock:
            jpeg = state.jpeg
        if jpeg:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
        time.sleep(0.05)


class UiState:
    def __init__(self, modes: list[str], robot_ip: str) -> None:
        import threading

        self.lock = threading.Lock()
        self.modes = list(modes)
        self.mode = "pause"
        self.ui: dict = {}
        self.stats: dict = {}
        self.scan = None
        self.jpeg = b""
        self.map_png = b""
        self.robot_ip = robot_ip
        self.monitor_port = 8081

    def snapshot(self) -> str:
        with self.lock:
            return json.dumps({"mode": self.mode})
