"""One short colored line on the operator terminal.

File logs stay on their existing handlers. This module writes only to a
terminal, so a process whose stderr is a launch pipe does not copy the
line into ``*-stderr.log``.

>>> "192.168.1.10" in connected_line("192.168.1.10")
True
"""

from __future__ import annotations

import os
import sys
import traceback
from datetime import datetime

WHITE = "\033[37m"
GREEN = "\033[32m"
BRIGHT_GREEN = "\033[92m"
YELLOW = "\033[33m"
BRIGHT_YELLOW = "\033[93m"
RED = "\033[31m"
RESET = "\033[0m"

_TRACEBACK = "Traceback (most recent call last):"
_STDERR_TAIL = 65536


def stamp(now: datetime | None = None) -> str:
    """Return ``HH:MM:SS,mmm`` for ``now`` or the current local time."""
    moment = datetime.now() if now is None else now
    return f"{moment:%H:%M:%S},{moment.microsecond // 1000:03d}"


def duration(seconds: float) -> str:
    """Return a short second count such as ``30s`` or ``1.5s``."""
    value = float(seconds)
    if value.is_integer():
        return f"{int(value)}s"
    text = f"{value:.3f}".rstrip("0").rstrip(".")
    return f"{text}s"


def connected_line(ip: str, *, now: datetime | None = None) -> str:
    """Return the green line a robot prints after it reaches ``ip``."""
    body = f"{GREEN}Connected to server {BRIGHT_GREEN}{ip}{RESET}"
    return _line(body, now)


def robot_connected_line(ip: str, *, now: datetime | None = None) -> str:
    """Return the green line the server prints when a robot connects."""
    body = f"{GREEN}Robot is connected from {BRIGHT_GREEN}{ip}{RESET}"
    return _line(body, now)


def lost_line(seconds: float, *, now: datetime | None = None) -> str:
    """Return the yellow line printed while a lost link is still waited on."""
    body = f"{YELLOW}Lost connection. Waiting for {BRIGHT_YELLOW}{duration(seconds)}{RESET}"
    return _line(body, now)


def no_server_line(seconds: float, *, now: datetime | None = None) -> str:
    """Return the yellow line printed when the next search has to wait."""
    body = f"{YELLOW}No server. Waiting for {BRIGHT_YELLOW}{duration(seconds)}{RESET}"
    return _line(body, now)


def module_line(name: str, ok: bool, *, now: datetime | None = None) -> str:
    """Return one module line. The module name carries the color."""
    if ok:
        body = f"{GREEN}{name}{RESET} ok"
    else:
        body = f"{RED}{name}{RESET} failed"
    return _line(body, now)


def died_line(name: str, *, now: datetime | None = None) -> str:
    """Return the red line for a process that exited on its own."""
    return _line(f"{RED}{name}{RESET} died", now)


def shutdown_line(*, now: datetime | None = None) -> str:
    """Return the plain line for an orderly stop."""
    return _line("shutdown", now)


def initializing_line(*, now: datetime | None = None) -> str:
    """Return the plain line printed when the server package starts."""
    return _line("Initializing ROS2 package", now)


def logs_line(path: str, *, now: datetime | None = None) -> str:
    """Return the plain line that names the server log file."""
    return _line(f"Logs {path}", now)


def http_url(host: str, port: int) -> str:
    """Return a browser URL. A wildcard bind is shown as localhost."""
    shown = (host or "").strip()
    if shown in {"", "0.0.0.0", "::"}:
        shown = "127.0.0.1"
    return f"http://{shown}:{int(port)}"


def server_ready_line(url: str, *, now: datetime | None = None) -> str:
    """Return the green line printed when the web server is listening."""
    return _line(f"{GREEN}Server is ready on {BRIGHT_GREEN}{url}{RESET}", now)


def failure_line(text: str, *, now: datetime | None = None) -> str:
    """Return a red line for a failure that is not a traceback."""
    return _line(f"{RED}{text}{RESET}", now)


def warning_line(text: str, *, now: datetime | None = None) -> str:
    """Return a yellow line for a failure the process can retry."""
    return _line(f"{YELLOW}{text}{RESET}", now)


def short_name(process_name: str) -> str:
    """Drop the ``-N`` suffix launch adds to a process name."""
    stem, _dash, suffix = process_name.rpartition("-")
    if stem and suffix.isdigit():
        return stem
    return process_name


def remember_output(previous: str, chunk: str) -> str:
    """Keep the latest traceback, or the tail of ``previous`` plus ``chunk``."""
    text = previous + chunk
    mark = text.rfind(_TRACEBACK)
    if mark >= 0:
        return text[mark:]
    if len(text) > _STDERR_TAIL:
        return text[-_STDERR_TAIL:]
    return text


def traceback_text(captured: str) -> str:
    """Return the raw traceback inside ``captured``, or an empty string."""
    mark = captured.rfind(_TRACEBACK)
    if mark < 0:
        return ""
    return captured[mark:].rstrip("\n") + "\n"


def exception_text(exc: BaseException) -> str:
    """Return ``exc`` as the interpreter would print it."""
    lines = traceback.format_exception(type(exc), exc, exc.__traceback__)
    return "".join(lines)


def report_exception(text: str, exc: BaseException) -> None:
    """Print one red line, then ``exc`` with no color and no time prefix."""
    write(failure_line(text))
    write_raw(exception_text(exc))


def write(text: str) -> None:
    """Print one already formatted console line."""
    _emit(text)


def write_raw(text: str) -> None:
    """Print ``text`` exactly, aside from a missing final newline."""
    _emit(text)


def _line(message: str, now: datetime | None) -> str:
    return f"{WHITE}{stamp(now)}{RESET} {message}"


def _emit(text: str) -> None:
    if not text or _suppressed():
        return
    opened = _sink()
    if opened is None:
        return
    stream, owned = opened
    payload = text if text.endswith("\n") else text + "\n"
    try:
        stream.write(payload)
        stream.flush()
    except OSError:
        return
    finally:
        if owned:
            stream.close()


def _suppressed() -> bool:
    return bool(os.environ.get("PYTEST_CURRENT_TEST"))


def _sink():
    """Open the operator terminal.

    A node started by launch has a pipe for stderr, so the line must go to a
    terminal device. ``setsid`` drops the controlling terminal, and then
    ``/dev/tty`` no longer opens. ``LEO_TTY`` is that same device, saved by
    the launcher before the new session starts.
    """
    if sys.stderr.isatty():
        return sys.stderr, False
    for path in (os.environ.get("LEO_TTY", ""), "/dev/tty"):
        if not path:
            continue
        try:
            return open(path, "w", encoding="utf-8", errors="replace"), True
        except OSError:
            continue
    return None
