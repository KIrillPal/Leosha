"""Turn heartbeats, topic edges and system stats into the graph the page draws."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Edge:
    topic: str
    publisher: str
    subscriber: str
    hz: float


def build_view(heartbeats: dict, edges: list[Edge], stats: dict, now: float, stale_sec: float) -> dict:
    nodes = {}
    for name, beat in heartbeats.items():
        age = now - float(beat.get("stamp", 0.0))
        nodes[name] = {
            "name": name,
            "fps": beat.get("fps"),
            "stale": age > stale_sec,
            "source": "heartbeat",
        }
    for edge in edges:
        nodes.setdefault(edge.publisher, {"name": edge.publisher, "fps": None, "stale": False, "source": "graph"})
        nodes.setdefault(edge.subscriber, {"name": edge.subscriber, "fps": None, "stale": False, "source": "graph"})
        if nodes[edge.publisher]["fps"] is None and edge.hz:
            nodes[edge.publisher]["fps"] = round(edge.hz, 2)
    return {
        "nodes": list(nodes.values()),
        "edges": [
            {"topic": edge.topic, "source": edge.publisher, "target": edge.subscriber, "hz": round(edge.hz, 2)}
            for edge in edges
        ],
        "system_fps": stats.get("system_fps"),
        "client": stats.get("client") or {},
        "latency_ms": stats.get("latency_ms"),
    }
