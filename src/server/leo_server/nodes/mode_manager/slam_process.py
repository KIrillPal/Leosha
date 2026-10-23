"""Start and stop slam_toolbox. Humble has no map-reset service, so a new process is a new map."""

from __future__ import annotations

import signal
import subprocess
from pathlib import Path

import yaml


class SlamProcess:
    def __init__(self, params: dict, work_dir: Path) -> None:
        self._params = dict(params)
        self._path = Path(work_dir) / "slam_toolbox_runtime.yaml"
        self._proc: subprocess.Popen | None = None

    def restart(self) -> None:
        self.stop()
        self._path.parent.mkdir(parents=True, exist_ok=True)
        payload = {"slam_toolbox": {"ros__parameters": self._params}}
        self._path.write_text(yaml.safe_dump(payload), encoding="utf-8")
        self._proc = subprocess.Popen(
            [
                "ros2",
                "run",
                "slam_toolbox",
                "async_slam_toolbox_node",
                "--ros-args",
                "-r",
                "__node:=slam_toolbox",
                "--params-file",
                str(self._path),
            ]
        )

    def stop(self) -> None:
        proc = self._proc
        self._proc = None
        if proc is None or proc.poll() is not None:
            return
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None
