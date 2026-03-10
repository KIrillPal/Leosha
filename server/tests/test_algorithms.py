from server.algorithms.profiles import AutonomyProfile1, PauseProfile, TeleoperationProfile
from server.interfaces import AlgorithmContext
from server.models import ControlMode, ManualInputState, TelemetryFrame


def test_pause_profile_returns_zero_command():
    profile = PauseProfile()
    context = AlgorithmContext(0.002)
    command = profile.algorithm.compute_command(context, ManualInputState(), TelemetryFrame())
    assert command.mode == ControlMode.PAUSE
    assert command.speed == 0.0
    assert command.steering == 0.0


def test_teleoperation_profile_generates_forward_and_right_command():
    profile = TeleoperationProfile()
    manual = ManualInputState(tracking_enabled=True, w=True, d=True)
    telemetry = TelemetryFrame(imu_yaw_rate=0.2)

    context = AlgorithmContext(0.002)
    command = profile.algorithm.compute_command(context, manual, telemetry)
    assert command.mode == ControlMode.TELEOPERATION
    assert command.speed > 0.0
    assert command.steering > 0.0


def test_autonomy_profile_is_stub_with_zero_output():
    profile = AutonomyProfile1()
    context = AlgorithmContext(0.002)
    command = profile.algorithm.compute_command(context, ManualInputState(), TelemetryFrame())
    assert command.mode == ControlMode.AUTONOMY_PROFILE_1
    assert command.speed == 0.0
    assert command.steering == 0.0
