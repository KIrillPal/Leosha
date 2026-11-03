from pathlib import Path

from leo_client.calibration import pan_servo_deg, throttle_from_speed
from leo_client.session import LinkPhase, Session
from leo_server.common.config import load_config

CFG = load_config(Path(__file__).resolve().parents[3] / "config" / "robot.yaml").client


class _Servos:
    def __init__(self):
        self.args = None

    def apply(self, motor, steer, pan, tilt):
        self.args = (motor, steer, pan, tilt)


def test_silence_waits_then_asks_for_a_new_search():
    now = [0.0]
    lost = CFG["waiting_timeout_sec"]
    limit = CFG["wait_limit_sec"]
    phase = LinkPhase(lost, limit, clock=lambda: now[0])
    assert phase.poll() is None
    now[0] = lost
    assert phase.poll() == "lost"
    assert phase.phase == "waiting"
    now[0] = lost + limit - 0.1
    assert phase.poll() is None
    now[0] = lost + limit
    assert phase.poll() == "search"
    assert phase.phase == "searching"


def test_a_command_during_waiting_restores_the_link():
    now = [0.0]
    lost = CFG["waiting_timeout_sec"]
    phase = LinkPhase(lost, CFG["wait_limit_sec"], clock=lambda: now[0])
    now[0] = lost
    assert phase.poll() == "lost"
    now[0] = 2.0
    assert phase.heard() == "restored"
    assert phase.phase == "connected"
    now[0] = 2.0 + lost - 0.1
    assert phase.poll() is None


class _Device:
    def __init__(self, name: str, order: list[str]):
        self._name = name
        self._order = order

    def start(self) -> None:
        return None

    def close(self) -> None:
        self._order.append(self._name)

    def latest(self):
        if self._name == "lidar":
            return None
        return b"", 0, 0, 0


class _Link:
    def poll_command(self):
        return None

    def send(self, _packet) -> None:
        return None


def test_ending_the_session_parks_servos_before_the_sensors_stop():
    order = []
    servos = _Servos()
    servos.pan_servo_deg = CFG["calibration"]["head_pan_zero_deg"]
    servos.tilt_servo_deg = CFG["calibration"]["head_tilt_zero_deg"]
    servos.motor_throttle = 0.0
    servos.steering_throttle = 0.0

    def close() -> None:
        order.append("servos")

    servos.start = lambda: None
    servos.close = close
    session = Session(_Link(), CFG, _Device("camera", order), _Device("lidar", order), servos, None)

    assert session.run(duration_sec=0.05) == "done"
    assert order == ["servos", "camera", "lidar"]


def test_actuators_stay_neutral_until_a_command_arrives():
    servos = _Servos()
    session = Session(None, CFG, None, None, servos, None)
    session._apply(None, 0.01, throttle_from_speed(0.0, CFG["calibration"]))
    motor, steer, pan, tilt = servos.args
    assert motor == throttle_from_speed(0.0, CFG["calibration"])
    assert steer == 0.0
    assert pan == pan_servo_deg(0.0, CFG["calibration"])
    assert tilt != 0.0
