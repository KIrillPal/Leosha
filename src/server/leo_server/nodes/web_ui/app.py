"""Flask pages. They publish ROS messages through callbacks and never tick the controller."""

from __future__ import annotations

import json
import time

from flask import Flask, Response, jsonify, render_template, request

_MODULE_ORDER = ("lidar", "camera", "motor", "wheel servo", "neck servo", "face servo")


def robot_status(reply_age_sec: float | None, reply_timeout_sec: float) -> str:
    """English link status. Silence longer than the timeout is not a reply."""
    if reply_age_sec is None:
        return "Waiting"
    if reply_age_sec > reply_timeout_sec:
        return "Not responding"
    return "Connected"


def ordered_modules(modules: dict) -> list[dict]:
    """Return module rows in a stable order."""
    rank = {name: index for index, name in enumerate(_MODULE_ORDER)}
    rows = [(str(name), bool(ok)) for name, ok in modules.items()]
    rows.sort(key=lambda item: (rank.get(item[0], len(rank)), item[0]))
    return [{"name": name, "ok": ok} for name, ok in rows]


def create_app(state, publish) -> Flask:
    app = Flask(__name__, template_folder="templates")

    @app.context_processor
    def _ports():
        path = request.path
        tab = "settings" if path == "/" else path.strip("/").split("/")[0]
        return {"monitor_port": state.monitor_port, "active_tab": tab}

    @app.get("/")
    def home():
        with state.lock:
            mode = state.mode
            robot_ip = state.robot_ip
        return render_template("settings.html", modes=state.modes, mode=mode, robot_ip=robot_ip)

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
        return jsonify(state.status_payload())

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
    def __init__(self, modes: list[str], robot_ip: str, reply_timeout_sec: float = 2.0) -> None:
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
        self.reply_timeout_sec = float(reply_timeout_sec)
        self.reply_age_sec: float | None = None
        self.reply_noted_at: float | None = None
        self.modules: dict[str, bool] = {}

    def note_reply(self, reply_age_sec: float | None, modules: dict) -> None:
        """Remember a robot-status sample and the time it was observed."""
        with self.lock:
            self.reply_age_sec = None if reply_age_sec is None else float(reply_age_sec)
            self.reply_noted_at = None if reply_age_sec is None else time.monotonic()
            self.modules = {str(name): bool(ok) for name, ok in modules.items()}

    def status_payload(self) -> dict:
        with self.lock:
            age = self._reply_age(time.monotonic())
            return {
                "mode": self.mode,
                "ui": self.ui,
                "stats": self.stats,
                "robot_ip": self.robot_ip,
                "robot_status": robot_status(age, self.reply_timeout_sec),
                "modules": ordered_modules(self.modules),
            }

    def _reply_age(self, now: float) -> float | None:
        if self.reply_age_sec is None or self.reply_noted_at is None:
            return None
        return self.reply_age_sec + (now - self.reply_noted_at)

    def snapshot(self) -> str:
        with self.lock:
            return json.dumps({"mode": self.mode})
