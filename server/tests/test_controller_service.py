from server.models import ControlMode


def test_controller_mode_switch(runtime):
    _, controller, _ = runtime
    assert controller.active_mode == ControlMode.PAUSE
    controller.set_mode("teleoperation")
    assert controller.active_mode == ControlMode.TELEOPERATION
    controller.set_mode("teleop_slam")
    assert controller.active_mode == ControlMode.TELEOP_SLAM
    controller.set_mode("autonomy_profile_1")
    assert controller.active_mode == ControlMode.AUTONOMY_PROFILE_1


def test_controller_tick_sends_command(runtime):
    _, controller, _ = runtime
    controller.set_mode("teleoperation")
    controller.set_tracking(True)
    controller.apply_keyboard("w", True)
    command = controller.tick_once()
    assert command.mode == ControlMode.TELEOPERATION
    assert command.speed > 0.0
