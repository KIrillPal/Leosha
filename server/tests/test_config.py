from pathlib import Path

from server.config import load_server_config


def _configs_dir() -> Path:
    return Path(__file__).resolve().parents[2] / "configs"


def test_yaml_config_loads():
    path = _configs_dir() / "server.yaml"
    cfg = load_server_config(path)
    assert cfg.app.port > 0
    assert cfg.robot.ip
    assert isinstance(cfg.control.head_sensitivity, (int, float))
