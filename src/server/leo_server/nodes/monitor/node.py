"""Live node graph. Topic rates are counted from raw subscriptions, without deserializing."""

from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from leo_server.common.config import config_from_env
from leo_server.common.heartbeat import Heartbeat
from leo_server.nodes.monitor.app import create_monitor_app
from leo_server.nodes.monitor.graph import Edge, build_view

try:
    from rosidl_runtime_py.utilities import get_message
except ImportError:  # pragma: no cover - only inside a sourced ROS env
    get_message = None


class Monitor(Node):
    def __init__(self, config) -> None:
        super().__init__("monitor")
        self._stale = float(config.monitor["stale_sec"])
        self._heartbeats: dict[str, dict] = {}
        self._counts: dict[str, int] = {}
        self._hz: dict[str, float] = {}
        self._subs = {}
        self._stats: dict = {}
        self._lock = threading.Lock()
        self._heartbeat = Heartbeat(self)
        self.create_subscription(String, "/leo/heartbeat", self._on_heartbeat, 50)
        self.create_subscription(String, "/leo/system_stats", self._on_stats, 10)
        self.create_timer(1.0, self._measure)
        self.create_timer(1.0, self._heartbeat.tick)
        page = Path(__file__).with_name("page.html").read_text(encoding="utf-8")
        self._start_page(config, page)

    def _snapshot(self) -> dict:
        with self._lock:
            return build_view(dict(self._heartbeats), self._edges(), dict(self._stats), time.time(), self._stale)

    def _start_page(self, config, page: str) -> None:
        app = create_monitor_app(page, self._snapshot, int(config.web_ui["port"]))
        thread = threading.Thread(
            target=lambda: app.run(host=config.monitor["host"], port=int(config.monitor["port"]), threaded=True, use_reloader=False),
            name="monitor-web",
            daemon=True,
        )
        thread.start()

    def _on_heartbeat(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        payload["stamp"] = time.time()
        with self._lock:
            self._heartbeats[str(payload.get("node"))] = payload

    def _on_stats(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._stats = payload

    def _measure(self) -> None:
        if get_message is None:
            return
        seen = set()
        for name, types in self.get_topic_names_and_types():
            if not types or name in self._subs or name.startswith("/rosout"):
                if name in self._subs:
                    seen.add(name)
                continue
            try:
                msg_type = get_message(types[0])
            except (AttributeError, ModuleNotFoundError, ValueError):
                continue
            self._counts[name] = 0
            self._subs[name] = self.create_subscription(msg_type, name, self._counter(name), qos_profile_sensor_data, raw=True)
            seen.add(name)
        with self._lock:
            for name, count in list(self._counts.items()):
                self._hz[name] = float(count)
                self._counts[name] = 0

    def _counter(self, topic: str):
        def _count(_payload) -> None:
            self._counts[topic] = self._counts.get(topic, 0) + 1

        return _count

    def _edges(self) -> list[Edge]:
        edges = []
        for name, _types in self.get_topic_names_and_types():
            pubs = self.get_publishers_info_by_topic(name)
            subs = self.get_subscriptions_info_by_topic(name)
            hz = self._hz.get(name, 0.0)
            for pub in pubs:
                for sub in subs:
                    if pub.node_name == sub.node_name:
                        continue
                    edges.append(Edge(name, pub.node_name, sub.node_name, hz))
        return edges


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=None)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Monitor(config_from_env(args.config))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
