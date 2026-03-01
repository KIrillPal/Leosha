from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass
class AppSection:
    host: str
    port: int
    command_hz: float


@dataclass
class RobotSection:
    ip: str


@dataclass
class ControlSection:
    max_speed_normal: float
    max_speed_fast: float
    max_steering: float
    head_sensitivity: float


@dataclass
class NetworkSection:
    grafana_url: str


@dataclass
class ServerConfig:
    app: AppSection
    robot: RobotSection
    control: ControlSection
    network: NetworkSection


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_server_config(path: str | Path) -> ServerConfig:
    default_path = Path(__file__).resolve().parent.parent / "config" / "server.yaml"
    with default_path.open("r", encoding="utf-8") as f:
        defaults = yaml.safe_load(f) or {}

    custom_path = Path(path)
    if custom_path != default_path and custom_path.exists():
        with custom_path.open("r", encoding="utf-8") as f:
            loaded = yaml.safe_load(f) or {}
        data = _deep_merge(defaults, loaded)
    else:
        data = defaults

    return ServerConfig(
        app=AppSection(**data["app"]),
        robot=RobotSection(**data["robot"]),
        control=ControlSection(**data["control"]),
        network=NetworkSection(**data["network"]),
    )
