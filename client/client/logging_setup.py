from __future__ import annotations

import logging
import os
import sys
from datetime import datetime
from pathlib import Path


class _ColorFormatter(logging.Formatter):
    _RESET = "\033[0m"
    _COLORS = {
        logging.DEBUG: "\033[36m",  # cyan
        logging.INFO: "\033[32m",  # green
        logging.WARNING: "\033[33m",  # yellow
        logging.ERROR: "\033[31m",  # red
        logging.CRITICAL: "\033[35m",  # magenta
    }

    def format(self, record: logging.LogRecord) -> str:
        color = self._COLORS.get(record.levelno, "")
        base = super().format(record)
        if not color:
            return base
        return f"{color}{base}{self._RESET}"


def _is_tty() -> bool:
    return bool(getattr(sys.stderr, "isatty", lambda: False)())


def configure_pipeline_logging(pipeline_name: str, level: int = logging.INFO) -> Path:
    root = logging.getLogger()
    if getattr(root, "_leosha_logging_configured", False):
        return getattr(root, "_leosha_last_log_file")

    logs_dir = Path(__file__).resolve().parents[2] / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = logs_dir / f"{pipeline_name}_{ts}.log"

    root.handlers.clear()
    root.setLevel(logging.DEBUG)

    datefmt = "%Y-%m-%d %H:%M:%S"
    console_format = "%(asctime)s | %(levelname)-8s | %(name)s | %(message)s"
    file_format = "%(asctime)s.%(msecs)03d | %(levelname)-8s | %(name)s | %(message)s"

    console = logging.StreamHandler()
    console.setLevel(level)
    if _is_tty() and os.getenv("NO_COLOR") is None:
        console.setFormatter(_ColorFormatter(console_format, datefmt=datefmt))
    else:
        console.setFormatter(logging.Formatter(console_format, datefmt=datefmt))

    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(logging.Formatter(file_format, datefmt=datefmt))

    root.addHandler(console)
    root.addHandler(file_handler)

    root._leosha_logging_configured = True
    root._leosha_last_log_file = log_path

    logging.getLogger(__name__).info("Logging configured: terminal + file=%s", log_path)
    return log_path
