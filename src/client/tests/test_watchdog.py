from leo_client.watchdog import Watchdog


def test_speed_ramps_down_before_the_cut():
    dog = Watchdog(100.0, 500.0)
    assert dog.speed_scale(50.0) == 1.0
    assert dog.speed_scale(300.0) == 0.5
    assert dog.speed_scale(500.0) == 0.0
    assert dog.drive_cut(499.0) is False
    assert dog.drive_cut(500.0) is True
