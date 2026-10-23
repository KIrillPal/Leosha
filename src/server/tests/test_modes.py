from pathlib import Path

from leo_server.common.config import load_config
from leo_server.nodes.mode_manager.modes import Keys, ModeController, ScanView, Sense

ROOT = Path(__file__).resolve().parents[3]
CFG = load_config(ROOT / "config" / "robot.yaml")


def _controller(tmp_path):
    return ModeController({"client": CFG.client, "modes": CFG.modes}, tmp_path / "friends.json")


def _sense(**kwargs):
    base = dict(dt=0.1, now=1.0, keys=Keys())
    base.update(kwargs)
    return Sense(**base)


def test_mode_switch_restarts_slam_and_keeps_the_head(tmp_path):
    robot = _controller(tmp_path)
    robot.set_measured_joints(30.0, 12.0)
    robot.set_mode("teleop_slam")
    assert robot.uses_slam()
    assert robot.consume_slam_restart() is True
    robot.set_mode("teleoperation")
    assert robot.uses_slam() is False
    assert robot.consume_slam_restart() is False
    robot.set_mode("teleop_slam")
    assert robot.consume_slam_restart() is True
    command = robot.tick(_sense())
    assert abs(command.head_pan_deg - 30.0) < 1e-6
    assert abs(command.head_tilt_deg - 12.0) < 1e-6
    assert command.speed_mps == 0.0


def test_reset_head_returns_the_teleop_head_to_zero(tmp_path):
    robot = _controller(tmp_path)
    robot.set_measured_joints(20.0, 15.0)
    robot.set_mode("teleoperation")
    command = robot.tick(_sense(keys=Keys(tracking=True), action={"type": "reset_head"}))
    assert command.head_pan_deg == 0.0
    assert command.head_tilt_deg == 0.0


def test_mouse_right_pans_with_the_pointer(tmp_path):
    robot = _controller(tmp_path)
    robot.set_measured_joints(0.0, 15.0)
    robot.set_mode("teleoperation")
    command = robot.tick(_sense(keys=Keys(tracking=True, head_dx=10.0)))
    assert command.head_pan_deg < 0.0


def test_mouse_down_lowers_the_head(tmp_path):
    robot = _controller(tmp_path)
    robot.set_measured_joints(0.0, 15.0)
    robot.set_mode("teleoperation")
    command = robot.tick(_sense(keys=Keys(tracking=True, head_dy=10.0)))
    assert command.head_tilt_deg < 15.0


def test_staring_falls_back_to_the_first_stable_person(tmp_path):
    robot = _controller(tmp_path)
    robot.set_mode("staring")
    person = {
        "track_id": 7,
        "bbox": [0, 0, 100, 200],
        "keypoints": [],
        "face_embedding": [],
        "face_visible": True,
        "confidence": 0.9,
    }
    sense = _sense(keys=Keys(tracking=True, w=True), persons=[person], frame_id=1, frame_w=640, frame_h=480)
    robot.tick(sense)
    sense.frame_id = 2
    command = robot.tick(sense)
    # stable_track_frames is 10, so one repeat is not enough
    assert robot.ui_state["staring_state"] == "manual"
    assert command.speed_mps > 0.0


def test_staring_tracks_after_enough_frames(tmp_path):
    robot = _controller(tmp_path)
    robot.set_mode("staring")
    person = {"track_id": 4, "bbox": [300, 0, 400, 80], "keypoints": [], "face_embedding": [], "face_visible": True, "confidence": 0.9}
    need = CFG.modes["tracking"]["stable_track_frames"]
    command = None
    for frame in range(need):
        command = robot.tick(_sense(keys=Keys(tracking=True), persons=[person], frame_id=frame, frame_w=200, frame_h=100))
    assert robot.ui_state["staring_state"] == "staring_first_face_fallback"
    assert command.head_pan_deg != 0.0


def test_silly_reverses_from_a_front_obstacle_and_stops_when_blocked(tmp_path):
    robot = _controller(tmp_path)
    robot.set_mode("silly_following")
    person = {"track_id": 1, "bbox": [90, 40, 110, 60], "keypoints": [], "face_embedding": [1.0, 0.0], "face_visible": True, "confidence": 0.9}
    need = CFG.modes["tracking"]["stable_track_frames"]
    ranges = [3.0] * 360
    ranges[180] = 0.05  # angle 0, in front
    scan = ScanView(-3.14159265, (2 * 3.14159265) / 360, ranges)
    for frame in range(need):
        robot.tick(_sense(keys=Keys(tracking=True), persons=[person], frame_id=frame, scan=scan, frame_w=200, frame_h=100))
    command = robot.tick(_sense(keys=Keys(tracking=True), persons=[person], frame_id=need, scan=scan, frame_w=200, frame_h=100, now=5.0))
    assert command.speed_mps < 0.0
    ranges[0] = 0.05  # behind
    blocked = ScanView(scan.angle_min, scan.angle_increment, ranges)
    command = robot.tick(_sense(keys=Keys(tracking=True), persons=[person], frame_id=need + 1, scan=blocked, frame_w=200, frame_h=100, now=5.1))
    assert command.speed_mps == 0.0
    assert robot.ui_state["follow_state"] == "blocked"


def test_autonomy_target_is_forgotten_on_the_next_mode(tmp_path):
    robot = _controller(tmp_path)
    robot.set_mode("autonomy_profile_1")
    robot.tick(_sense(action={"type": "set_target", "target": {"x": 1}}))
    assert robot.ui_state["target"] == {"x": 1}
    robot.set_mode("pause")
    robot.set_mode("autonomy_profile_1")
    robot.tick(_sense())
    assert robot.ui_state["target"] is None


def test_keys_do_not_survive_a_mode_switch(tmp_path):
    robot = _controller(tmp_path)
    robot.set_mode("teleoperation")
    robot.tick(_sense(keys=Keys(tracking=True, w=True)))
    assert robot.ui_state["mode"] == "teleoperation"
    robot.set_mode("pause")
    command = robot.tick(_sense(keys=Keys(tracking=True, w=True)))
    assert command.speed_mps == 0.0
