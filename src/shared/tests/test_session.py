from pathlib import Path

from leo_client.calibration import pan_servo_deg, throttle_from_speed
from leo_client.session import Session
from leo_server.common.config import load_config

CFG = load_config(Path(__file__).resolve().parents[3] / "config" / "robot.yaml").client


class _Servos:
    def __init__(self):
        self.args = None

    def apply(self, motor, steer, pan, tilt):
        self.args = (motor, steer, pan, tilt)


def test_actuators_stay_neutral_until_a_command_arrives():
    servos = _Servos()
    session = Session(None, CFG, None, None, servos, None)
    session._apply(None, 0.01, throttle_from_speed(0.0, CFG["calibration"]))
    motor, steer, pan, tilt = servos.args
    assert motor == throttle_from_speed(0.0, CFG["calibration"])
    assert steer == 0.0
    assert pan == pan_servo_deg(0.0, CFG["calibration"])
    assert tilt != 0.0
