from datetime import datetime

from leo_protocol.console import (
    BRIGHT_GREEN,
    BRIGHT_YELLOW,
    GREEN,
    RED,
    RESET,
    WHITE,
    YELLOW,
    connected_line,
    died_line,
    duration,
    exception_text,
    http_url,
    initializing_line,
    logs_line,
    lost_line,
    no_server_line,
    remember_output,
    report_exception,
    robot_connected_line,
    server_ready_line,
    short_name,
    shutdown_line,
    traceback_text,
    write,
    write_raw,
)

WHEN = datetime(2026, 11, 3, 18, 32, 1, 123000)


def test_lines_start_with_a_white_clock():
    line = connected_line("192.168.1.10", now=WHEN)
    assert line.startswith(f"{WHITE}18:32:01,123{RESET} ")
    assert f"{GREEN}Connected to server {BRIGHT_GREEN}192.168.1.10{RESET}" in line


def test_server_startup_lines():
    started = initializing_line(now=WHEN)
    assert started.endswith(" Initializing ROS2 package")
    assert RED not in started
    assert GREEN not in started
    logs = logs_line("/home/kir/.ros/log/run/launch.log", now=WHEN)
    assert logs.endswith(" Logs /home/kir/.ros/log/run/launch.log")
    assert RED not in logs
    ready = server_ready_line(http_url("0.0.0.0", 8080), now=WHEN)
    assert f"{GREEN}Server is ready on {BRIGHT_GREEN}http://127.0.0.1:8080{RESET}" in ready
    assert http_url("192.168.1.10", 8080) == "http://192.168.1.10:8080"


def test_waiting_lines_highlight_the_duration():
    assert duration(30) == "30s"
    assert duration(1.5) == "1.5s"
    lost = lost_line(30, now=WHEN)
    assert f"{YELLOW}Lost connection. Waiting for {BRIGHT_YELLOW}30s{RESET}" in lost
    waiting = no_server_line(1.5, now=WHEN)
    assert f"{YELLOW}No server. Waiting for {BRIGHT_YELLOW}1.5s{RESET}" in waiting


def test_robot_death_and_shutdown_use_one_style():
    arrived = robot_connected_line("10.0.0.8", now=WHEN)
    assert f"{GREEN}Robot is connected from {BRIGHT_GREEN}10.0.0.8{RESET}" in arrived
    assert f"{RED}odometry{RESET} died" in died_line("odometry", now=WHEN)
    stopped = shutdown_line(now=WHEN)
    assert stopped.endswith(" shutdown")
    assert RED not in stopped
    assert YELLOW not in stopped
    assert GREEN not in stopped


def test_process_names_drop_the_launch_suffix():
    assert short_name("odometry-3") == "odometry"
    assert short_name("static_transform_publisher-7") == "static_transform_publisher"
    assert short_name("tf_laser") == "tf_laser"


def test_traceback_text_is_raw_and_uncolored():
    captured = "info line\nTraceback (most recent call last):\n  File \"x.py\", line 1, in f\nRuntimeError: boom\n"
    raw = traceback_text(captured)
    assert raw == "Traceback (most recent call last):\n  File \"x.py\", line 1, in f\nRuntimeError: boom\n"
    assert "\033[" not in raw
    assert "info line" not in raw
    try:
        raise RuntimeError("boom")
    except RuntimeError as exc:
        text = exception_text(exc)
    assert text.startswith("Traceback (most recent call last):\n")
    assert "\033[" not in text
    assert "RuntimeError: boom" in text


def test_stderr_tail_keeps_the_latest_traceback_only():
    noise = "x" * 70000
    kept = remember_output("", noise)
    assert len(kept) == 65536
    assert kept == noise[-65536:]
    traced = remember_output(kept, "\nTraceback (most recent call last):\nError: boom\n")
    assert traced.startswith("Traceback (most recent call last):")
    assert not traced.startswith("x")


def test_saved_terminal_is_used_when_stderr_is_a_pipe(monkeypatch, tmp_path):
    import leo_protocol.console as console

    target = tmp_path / "tty"
    monkeypatch.setenv("LEO_TTY", str(target))
    monkeypatch.setattr(console.sys.stderr, "isatty", lambda: False)
    opened = console._sink()
    assert opened is not None
    stream, owned = opened
    assert owned is True
    stream.write("ready\n")
    stream.close()
    assert target.read_text() == "ready\n"


def test_console_is_silent_during_pytest(monkeypatch):
    def _forbidden():
        raise AssertionError("console opened a stream during pytest")

    monkeypatch.setattr("leo_protocol.console._sink", _forbidden)
    write(connected_line("192.168.1.10", now=WHEN))
    write_raw("Traceback (most recent call last):\n")
    try:
        raise RuntimeError("boom")
    except RuntimeError as exc:
        report_exception("camera stopped", exc)
