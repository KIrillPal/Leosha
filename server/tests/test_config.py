from pathlib import Path

from server.config import load_server_config


def test_yaml_config_loads():
    path = Path(__file__).resolve().parents[1] / "config" / "server.yaml"
    cfg = load_server_config(path)
    assert cfg.app.port > 0
    assert cfg.robot.ip
    assert cfg.control.max_speed_fast >= cfg.control.max_speed_normal
