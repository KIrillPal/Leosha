from __future__ import annotations

import logging
import os
from time import sleep

from flask import Flask, Response, jsonify, redirect, render_template, request, url_for

from ..config import ServerConfig
from ..models import ControlMode
from ..services.controller_service import ControllerService
from ..services.robot_client import RobotClient

LOGGER = logging.getLogger(__name__)


def create_app(controller: ControllerService, robot_client: RobotClient, config: ServerConfig) -> Flask:
    try:
        from ament_index_python.packages import get_package_share_directory
        _share = get_package_share_directory("server")
        _share_templates = os.path.join(_share, "templates")
        template_folder = _share_templates if os.path.exists(_share_templates) else "templates"
    except Exception:
        template_folder = "templates"
    app = Flask(__name__, template_folder=template_folder)
    LOGGER.info("Web app created | template_folder=%s", template_folder)

    @app.get("/")
    def index():
        return redirect(url_for("settings_page"))

    @app.get("/settings")
    def settings_page():
        return render_template("settings.html", active_tab="settings")

    @app.get("/teleop")
    def teleop_page():
        return render_template("teleop.html", active_tab="teleop")

    @app.get("/network")
    def network_page():
        return render_template("network.html", active_tab="network", grafana_url=config.network.grafana_url)

    @app.get("/lidar")
    def lidar_page():
        return render_template("lidar.html", active_tab="lidar")

    @app.get("/video_feed")
    def video_feed():
        def generate():
            while True:
                frame = robot_client.get_latest_frame()
                if not frame:
                    sleep(0.05)
                    continue
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
                sleep(1.0 / 30.0)

        return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.get("/api/settings")
    def get_settings():
        status = robot_client.get_status()
        return jsonify({
            "ip": status.ip,
            "connected": status.connected,
            "last_ping_ms": status.last_ping_ms,
            "mode": controller.active_mode.value,
            "available_modes": controller.modes,
        })

    @app.post("/api/settings/ip")
    def set_robot_ip():
        data = request.get_json(silent=True) or {}
        ip = str(data.get("ip", "")).strip()
        if not ip:
            return jsonify({"success": False, "error": "IP не задан"}), 400
        robot_client.set_ip(ip)
        LOGGER.info("Robot IP updated to %s", ip)
        return jsonify({"success": True, "ip": ip})

    @app.post("/api/ping")
    def ping_robot():
        ping_ms = robot_client.ping()
        if ping_ms is None:
            return jsonify({"success": False, "connected": False, "ping_ms": None})
        return jsonify({"success": True, "connected": True, "ping_ms": round(ping_ms, 2)})

    @app.post("/api/mode")
    def set_mode():
        data = request.get_json(silent=True) or {}
        mode_raw = str(data.get("mode", ControlMode.PAUSE.value))
        try:
            mode = controller.set_mode(mode_raw)
            return jsonify({"success": True, "mode": mode.value})
        except ValueError:
            LOGGER.warning("Rejected unknown mode: %s", mode_raw)
            return jsonify({"success": False, "error": f"Неизвестный режим: {mode_raw}"}), 400

    @app.post("/api/position")
    def set_position():
        data = request.get_json(silent=True) or {}
        controller.apply_mouse_delta(float(data.get("dx", 0.0)), float(data.get("dy", 0.0)))
        controller.tick_once()
        status = controller.get_ui_status()
        return jsonify({"success": True, "x": status["x"], "y": status["y"]})

    @app.post("/api/reset")
    def reset_position():
        controller.reset_head_state()
        controller.tick_once()
        return jsonify({"success": True, "x": 0, "y": 0})

    @app.get("/api/status")
    def get_status():
        return jsonify(controller.get_ui_status())

    @app.post("/api/tracking")
    def set_tracking():
        data = request.get_json(silent=True) or {}
        tracking = bool(data.get("tracking", False))
        controller.set_tracking(tracking)
        controller.tick_once()
        return jsonify({"success": True, "tracking": tracking})

    @app.post("/api/keyboard")
    def handle_keyboard():
        data = request.get_json(silent=True) or {}
        key = str(data.get("key", "")).lower()
        state = bool(data.get("state", False))
        controller.apply_keyboard(key, state)
        controller.tick_once()
        return jsonify({"success": True, "key": key, "state": state})

    @app.post("/api/car/control")
    def car_control():
        data = request.get_json(silent=True) or {}
        speed = float(data.get("speed", 0.0))
        steering = float(data.get("steering", 0.0))
        controller.set_tracking(True)
        controller.apply_keyboard("w", speed > 0.0)
        controller.apply_keyboard("s", speed < 0.0)
        controller.apply_keyboard("a", steering < 0.0)
        controller.apply_keyboard("d", steering > 0.0)
        return jsonify({"success": True, "mode": controller.tick_once().mode.value})

    @app.post("/api/car/stop")
    def car_stop():
        for key in ("w", "a", "s", "d"):
            controller.apply_keyboard(key, False)
        return jsonify({"success": True, "mode": controller.tick_once().mode.value})

    @app.post("/api/capture")
    def capture_image():
        return jsonify({"success": True, "message": "Capture mocked on server side"})

    @app.get("/api/network/stats")
    def get_network_stats():
        stats = robot_client.get_network_stats()
        parts = robot_client.get_avg_packet_parts()
        return jsonify({
            "rssi_dbm": round(stats.rssi_dbm, 2),
            "latency_ms": round(stats.latency_ms, 2),
            "tx_bytes_per_sec": round(stats.tx_bytes_per_sec, 2),
            "rx_bytes_per_sec": round(stats.rx_bytes_per_sec, 2),
            "tx_packets_per_sec": round(stats.tx_packets_per_sec, 2),
            "rx_packets_per_sec": round(stats.rx_packets_per_sec, 2),
            "avg_packet_parts": {k: round(v, 1) for k, v in parts.items()},
        })

    @app.get("/api/lidar")
    def get_lidar_scan():
        scan = robot_client.get_latest_lidar_scan()
        if scan is None:
            return jsonify({"success": False, "scan": None})
        invert_angle = False
        try:
            cfg = robot_client.get_robot_config()
            if cfg:
                lidar_cfg = (cfg.get("sensors") or {}).get("lidar") or {}
                invert_angle = bool(lidar_cfg.get("invert_angle", False))
        except Exception:
            invert_angle = False
        return jsonify({"success": True, "scan": scan, "invert_angle": invert_angle})

    return app
