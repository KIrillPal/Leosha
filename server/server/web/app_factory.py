from __future__ import annotations

import logging
from time import sleep

from flask import Flask, Response, jsonify, redirect, render_template, request, url_for

from ..config import ServerConfig
from ..services.controller_service import ControllerService
from ..services.robot_client import RobotClient

LOGGER = logging.getLogger(__name__)


def create_app(
    controller: ControllerService,
    robot_client: RobotClient,
    config: ServerConfig,
    slam_service=None,
    ros_graph=None,
) -> Flask:
    template_folder = "templates"
    app = Flask(__name__, template_folder=template_folder)
    LOGGER.info("Web app created | template_folder=%s", template_folder)

    def _require_json() -> dict:
        data = request.get_json(silent=False)
        if not isinstance(data, dict):
            raise ValueError("JSON body must be an object")
        return data

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

    @app.get("/slam")
    def slam_page():
        return render_template("slam.html", active_tab="slam")

    @app.get("/ros-graph")
    def ros_graph_page():
        return render_template("ros_graph.html", active_tab="ros_graph")

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
        data = _require_json()
        ip = str(data["ip"]).strip()
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
        data = _require_json()
        mode_raw = str(data["mode"])
        try:
            mode = controller.set_mode(mode_raw)
            return jsonify({"success": True, "mode": mode.value})
        except ValueError:
            LOGGER.warning("Rejected unknown mode: %s", mode_raw)
            return jsonify({"success": False, "error": f"Неизвестный режим: {mode_raw}"}), 400

    @app.post("/api/position")
    def set_position():
        data = _require_json()
        controller.apply_mouse_delta(float(data["dx"]), float(data["dy"]))
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
        data = _require_json()
        tracking = bool(data["tracking"])
        controller.set_tracking(tracking)
        controller.tick_once()
        return jsonify({"success": True, "tracking": tracking})

    @app.post("/api/keyboard")
    def handle_keyboard():
        data = _require_json()
        key = str(data["key"]).lower()
        state = bool(data["state"])
        controller.apply_keyboard(key, state)
        controller.tick_once()
        return jsonify({"success": True, "key": key, "state": state})

    @app.post("/api/car/control")
    def car_control():
        data = _require_json()
        speed = float(data["speed"])
        steering = float(data["steering"])
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

    @app.get("/api/slam/status")
    def get_slam_status():
        if slam_service is None:
            return jsonify({"success": False, "error": "SLAM service not available"})
        stats = slam_service.get_stats()
        return jsonify({
            "success": True,
            "fps": stats.fps,
            "latency_ms": stats.latency_ms,
            "scan_count": stats.scan_count,
            "map_updates": stats.map_updates,
            "pose_updates": stats.pose_updates,
            "status": stats.status,
        })

    @app.get("/api/slam/map")
    def get_slam_map():
        if slam_service is None:
            return Response(b"", status=404)
        png = slam_service.get_map_png()
        if png is None:
            return Response(b"", status=404)
        return Response(png, mimetype="image/png")

    @app.get("/api/slam/pose")
    def get_slam_pose():
        if slam_service is None:
            return jsonify({"success": False})
        x, y, theta = slam_service.get_pose()
        meta = slam_service.get_map_meta()
        cmd = controller.get_ui_status()["command"]
        return jsonify({
            "success": True,
            "x": x, "y": y, "theta": theta,
            "head_pan_deg": cmd["head_pan_deg"],
            "map_meta": {
                "resolution": meta.resolution,
                "origin_x": meta.origin_x,
                "origin_y": meta.origin_y,
                "width": meta.width,
                "height": meta.height,
            },
        })

    @app.get("/api/ros/graph")
    def get_ros_graph():
        if ros_graph is None:
            return jsonify({"success": False, "nodes": []})
        nodes = ros_graph.get_graph()
        return jsonify({"success": True, "nodes": nodes})

    def _scan_for_json(scan: dict) -> dict:
        """Copy scan and replace float('nan') with None so JSON serialization succeeds (frontend skips non-finite)."""
        import math
        out = dict(scan)
        if "ranges" in out and isinstance(out["ranges"], (list, tuple)):
            out["ranges"] = [None if isinstance(v, float) and math.isnan(v) else v for v in out["ranges"]]
        if "intensities" in out and isinstance(out["intensities"], (list, tuple)):
            out["intensities"] = [None if isinstance(v, float) and math.isnan(v) else v for v in out["intensities"]]
        return out

    @app.get("/api/lidar")
    def get_lidar_scan():
        scan = robot_client.get_latest_lidar_scan()
        if scan is None:
            return jsonify({"success": False, "scan": None})
        cfg = robot_client.get_robot_config()
        if cfg is None:
            raise ValueError("robot_config is required for lidar endpoint")
        invert_angle = bool(cfg["sensors"]["lidar"]["invert_angle"])
        return jsonify({"success": True, "scan": _scan_for_json(scan), "invert_angle": invert_angle})

    return app
