from __future__ import annotations

from server.algorithms.profiles import StaringProfile
from server.interfaces import AlgorithmContext, InputState
from server.models import ControlMode, ManualInputState, TelemetryFrame


class MockVisionService:
    def __init__(self) -> None:
        self._persons: list[dict] = []
        self._frame_id = 0

    def set_persons(self, persons: list[dict], frame_id: int = 1) -> None:
        self._persons = [dict(p) for p in persons]
        self._frame_id = frame_id

    def get_persons(self) -> list[dict]:
        return [dict(p) for p in self._persons]

    def get_latest_frame_id(self) -> int:
        return self._frame_id


def _person(track_id: int, bbox: list[float], emb: list[float]) -> dict:
    return {
        "track_id": track_id,
        "bbox": bbox,
        "keypoints": [],
        "head_yaw_deg": 0.0,
        "face_visible": True,
        "face_embedding": emb,
        "confidence": 0.9,
    }


def test_staring_profile_falls_back_to_manual_when_no_match(tmp_path):
    vision = MockVisionService()
    profile = StaringProfile(
        forward_throttle=0.3,
        backward_throttle=-0.15,
        forward_fast_throttle=0.5,
        friend_embeddings_db=str(tmp_path / "friends.db"),
        face_match_threshold=0.5,
        eye_confidence_threshold=0.3,
        head_tracking_gain=3.0,
        head_tracking_deadzone=0.03,
        stable_track_frames=1,
        vision_service=vision,
    )
    context = AlgorithmContext(head_sensitivity=-0.002)
    state = InputState(
        manual=ManualInputState(tracking_enabled=True, head_dx=10.0, head_dy=-5.0, w=True),
        telemetry=TelemetryFrame(),
        dt=0.1,
    )
    cmd = profile.tick(context, state)
    profile.post_tick(state)
    ui = profile.get_ui_state()

    assert cmd.mode == ControlMode.STARING
    assert cmd.speed > 0.0
    assert ui["staring_state"] == "manual"
    assert ui["staring_target"] is None
    assert state.manual.head_dx == 0.0
    assert state.manual.head_dy == 0.0


def test_staring_profile_tracks_first_face_when_no_known_match(tmp_path):
    vision = MockVisionService()
    profile = StaringProfile(
        forward_throttle=0.3,
        backward_throttle=-0.15,
        forward_fast_throttle=0.5,
        friend_embeddings_db=str(tmp_path / "friends.db"),
        face_match_threshold=0.99,
        eye_confidence_threshold=0.3,
        head_tracking_gain=3.0,
        head_tracking_deadzone=0.0,
        stable_track_frames=1,
        vision_service=vision,
    )
    vision.set_persons(
        [
            _person(11, [40, 50, 160, 260], [0.1] * 128),
            _person(12, [180, 80, 380, 320], [0.2] * 128),
        ],
        frame_id=22,
    )
    cmd = profile.tick(
        AlgorithmContext(head_sensitivity=-0.002),
        InputState(
            manual=ManualInputState(tracking_enabled=True),
            telemetry=TelemetryFrame(),
            dt=0.2,
        ),
    )
    ui = profile.get_ui_state()

    assert cmd.mode == ControlMode.STARING
    assert ui["staring_state"] == "staring_first_face_fallback"
    assert ui["staring_target"] is None
    assert ui["staring_track_id"] == 11


def test_staring_profile_wasd_works_when_tracking_disabled(tmp_path):
    vision = MockVisionService()
    profile = StaringProfile(
        forward_throttle=0.3,
        backward_throttle=-0.15,
        forward_fast_throttle=0.5,
        friend_embeddings_db=str(tmp_path / "friends.db"),
        face_match_threshold=0.5,
        eye_confidence_threshold=0.3,
        head_tracking_gain=3.0,
        head_tracking_deadzone=0.03,
        stable_track_frames=1,
        vision_service=vision,
    )
    cmd = profile.tick(
        AlgorithmContext(head_sensitivity=-0.002),
        InputState(
            manual=ManualInputState(tracking_enabled=False, w=True, d=True),
            telemetry=TelemetryFrame(),
            dt=0.1,
        ),
    )
    ui = profile.get_ui_state()

    assert cmd.speed > 0.0
    assert cmd.steering > 0.0
    assert ui["staring_state"] == "manual"
    assert ui["staring_track_id"] is None


def test_staring_profile_tracks_best_matching_face(tmp_path):
    vision = MockVisionService()
    profile = StaringProfile(
        forward_throttle=0.3,
        backward_throttle=-0.15,
        forward_fast_throttle=0.5,
        friend_embeddings_db=str(tmp_path / "friends.db"),
        face_match_threshold=0.5,
        eye_confidence_threshold=0.3,
        head_tracking_gain=3.0,
        head_tracking_deadzone=0.0,
        stable_track_frames=1,
        vision_service=vision,
    )
    vision.set_persons([_person(7, [10, 10, 200, 220], [1.0] * 128)], frame_id=10)
    profile.on_action({"type": "add_face", "name": "Kir"})
    vision.set_persons(
        [
            _person(7, [10, 10, 200, 220], [1.0] * 128),
            _person(8, [120, 60, 520, 470], [1.0] * 128),
        ],
        frame_id=11,
    )

    cmd = profile.tick(
        AlgorithmContext(head_sensitivity=-0.002),
        InputState(
            manual=ManualInputState(tracking_enabled=True),
            telemetry=TelemetryFrame(),
            dt=0.2,
        ),
    )
    ui = profile.get_ui_state()

    assert cmd.mode == ControlMode.STARING
    assert ui["staring_state"] == "staring"
    assert ui["staring_target"] == "Kir"
    assert ui["staring_track_id"] == 8
    assert ui["persons_count"] == 2
    assert ui["staring_debug"]["target_track_id"] == 8
    assert len(ui["staring_overlay"]) == 2
    assert "track_id" in ui["staring_overlay"][0]
    assert "head_yaw_deg" in ui["staring_overlay"][0]


def test_staring_profile_face_actions_persist_and_remove(tmp_path):
    vision = MockVisionService()
    profile = StaringProfile(
        forward_throttle=0.3,
        backward_throttle=-0.15,
        forward_fast_throttle=0.5,
        friend_embeddings_db=str(tmp_path / "friends.db"),
        face_match_threshold=0.5,
        eye_confidence_threshold=0.3,
        head_tracking_gain=3.0,
        head_tracking_deadzone=0.03,
        stable_track_frames=1,
        vision_service=vision,
    )
    vision.set_persons([_person(1, [0, 0, 100, 100], [0.7] * 128)])
    profile.on_action({"type": "add_face"})
    names_after_add = profile.get_ui_state()["known_faces"]
    assert names_after_add == ["face_1"]

    profile.on_action({"type": "remove_face", "name": "face_1"})
    assert profile.get_ui_state()["known_faces"] == []


def test_staring_profile_filters_unstable_tracks_except_current_target(tmp_path):
    vision = MockVisionService()
    profile = StaringProfile(
        forward_throttle=0.3,
        backward_throttle=-0.15,
        forward_fast_throttle=0.5,
        friend_embeddings_db=str(tmp_path / "friends.db"),
        face_match_threshold=0.5,
        eye_confidence_threshold=0.3,
        head_tracking_gain=3.0,
        head_tracking_deadzone=0.0,
        stable_track_frames=3,
        vision_service=vision,
    )
    vision.set_persons([_person(1, [40, 50, 160, 260], [1.0] * 128)], frame_id=0)
    profile.on_action({"type": "add_face", "name": "Kir"})

    for frame_id in (1, 2, 3):
        vision.set_persons([_person(1, [40, 50, 160, 260], [1.0] * 128)], frame_id=frame_id)
        profile.tick(
            AlgorithmContext(head_sensitivity=-0.002),
            InputState(manual=ManualInputState(tracking_enabled=True), telemetry=TelemetryFrame(), dt=0.1),
        )
    assert profile.get_ui_state()["staring_track_id"] == 1

    vision.set_persons(
        [
            _person(1, [40, 50, 160, 260], [1.0] * 128),
            _person(2, [80, 40, 520, 420], [1.0] * 128),
        ],
        frame_id=4,
    )
    profile.tick(
        AlgorithmContext(head_sensitivity=-0.002),
        InputState(manual=ManualInputState(tracking_enabled=True), telemetry=TelemetryFrame(), dt=0.1),
    )
    assert profile.get_ui_state()["staring_track_id"] == 1
