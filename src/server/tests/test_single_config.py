from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def test_only_robot_yaml_holds_settings():
    found = []
    for path in ROOT.rglob("*"):
        if not path.is_file():
            continue
        if path.suffix.lower() not in {".yaml", ".yml"}:
            continue
        if any(part in {".git", "logs", "__pycache__"} for part in path.parts):
            continue
        found.append(path.relative_to(ROOT).as_posix())
    assert found == ["config/robot.yaml", "environment.yml"] or set(found) <= {
        "config/robot.yaml",
        "environment.yml",
    }
    assert "config/robot.yaml" in found


def test_loader_has_no_numeric_defaults():
    text = (ROOT / "src" / "server" / "leo_server" / "common" / "config.py").read_text(encoding="utf-8")
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("def ") and "=" in stripped and any(ch.isdigit() for ch in stripped):
            raise AssertionError(f"numeric default in loader: {stripped}")
