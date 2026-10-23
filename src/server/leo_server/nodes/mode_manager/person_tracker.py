"""Pick a stable person and the image point the head should look at."""

from __future__ import annotations

from dataclasses import dataclass

from leo_server.nodes.mode_manager.friends import FriendDB


@dataclass
class Target:
    track_id: int
    name: str | None
    cx: float
    cy: float
    state: str


class PersonTracker:
    def __init__(self, cfg: dict, friends: FriendDB) -> None:
        self._cfg = cfg
        self._friends = friends
        self._history: list[set[int]] = []
        self._last_frame = None
        self._locked_id: int | None = None
        self._last_cx: float | None = None
        self._last_cy: float | None = None
        self.state = "manual"
        self.target_name: str | None = None
        self.track_id: int | None = None
        self.debug: dict = {}
        self._persons: list[dict] = []

    def handle_action(self, action: dict | None, persons: list[dict]) -> None:
        if not action:
            return
        kind = str(action.get("type", ""))
        if kind == "add_face":
            self._add_face(action, persons)
        elif kind == "remove_face":
            self._friends.remove(str(action.get("name", "")))
            self.debug["last_action"] = "remove_face"

    def update(
        self,
        persons: list[dict],
        frame_id: int,
        frame_w: int,
        frame_h: int,
        dt: float,
        latency_ms: float,
        now: float,
        allow_fallback: bool,
    ) -> Target | None:
        self._persons = list(persons)
        self._remember(persons, frame_id)
        matched = self._matches(persons)
        chosen = self._choose(matched, persons, allow_fallback)
        if chosen is None:
            self._clear()
            return None
        person, name, state = chosen
        cx, cy = self._aim_point(person)
        cx, cy = self._extrapolate(cx, cy, frame_w, frame_h, dt, latency_ms, now)
        self.state = state
        self.target_name = name
        self.track_id = int(person["track_id"])
        self._locked_id = self.track_id
        self.debug = {"target_track_id": self.track_id, "state": state, "frame_id": frame_id}
        return Target(self.track_id, name, cx, cy, state)

    def ui(self) -> dict:
        return {
            "staring_state": self.state,
            "staring_target": self.target_name,
            "staring_track_id": self.track_id,
            "known_faces": self._friends.names(),
            "persons_count": len(self._persons),
            "staring_debug": dict(self.debug),
        }

    def _add_face(self, action: dict, persons: list[dict]) -> None:
        visible = [p for p in persons if p.get("face_visible") and p.get("face_embedding")]
        if not visible:
            self.debug["last_action"] = "add_face_skipped"
            return
        best = max(visible, key=lambda p: _area(p["bbox"]))
        name = str(action["name"]) if action.get("name") else self._next_name()
        self._friends.add(name, [float(v) for v in best["face_embedding"]])
        self.debug["last_action"] = f"add_face:{name}"

    def _next_name(self) -> str:
        taken = set(self._friends.names())
        index = 1
        while f"face_{index}" in taken:
            index += 1
        return f"face_{index}"

    def _remember(self, persons: list[dict], frame_id: int) -> None:
        if frame_id == self._last_frame:
            return
        present = {int(person["track_id"]) for person in persons}
        self._history.append(present)
        keep = int(self._cfg["stable_track_frames"])
        self._history = self._history[-keep:]
        self._last_frame = frame_id

    def _stable(self, track_id: int) -> bool:
        if self._locked_id is not None and track_id == self._locked_id:
            return True
        need = int(self._cfg["stable_track_frames"])
        if len(self._history) < need:
            return False
        return all(track_id in frame for frame in self._history[-need:])

    def _matches(self, persons: list[dict]) -> list[tuple[dict, str]]:
        found = []
        threshold = float(self._cfg["face_match_threshold"])
        for person in persons:
            track_id = int(person["track_id"])
            embedding = person.get("face_embedding") or []
            if not self._stable(track_id) or not embedding:
                continue
            name = self._friends.match([float(v) for v in embedding], threshold)
            if name is not None:
                found.append((person, name))
        return found

    def _choose(self, matched, persons, allow_fallback):
        if matched:
            person, name = max(matched, key=lambda item: _area(item[0]["bbox"]))
            return person, name, "staring"
        if not allow_fallback:
            return None
        stable = [p for p in persons if self._stable(int(p["track_id"]))]
        if not stable:
            return None
        return stable[0], None, "staring_first_face_fallback"

    def _aim_point(self, person: dict) -> tuple[float, float]:
        keypoints = person.get("keypoints") or []
        x1, y1, x2, y2 = [float(v) for v in person["bbox"][:4]]
        if len(keypoints) >= 3:
            left, right = keypoints[1], keypoints[2]
            need = float(self._cfg["eye_confidence_threshold"])
            if len(left) >= 3 and len(right) >= 3 and left[2] >= need and right[2] >= need:
                return (float(left[0]) + float(right[0])) * 0.5, (float(left[1]) + float(right[1])) * 0.5
        cx = x1 + float(self._cfg["bbox_target_x_frac"]) * (x2 - x1)
        cy = y1 + float(self._cfg["bbox_target_y_frac"]) * (y2 - y1)
        return cx, cy

    def _extrapolate(self, cx, cy, frame_w, frame_h, dt, latency_ms, now) -> tuple[float, float]:
        del now
        prev_x, prev_y = self._last_cx, self._last_cy
        self._last_cx, self._last_cy = cx, cy
        if prev_x is None or dt <= 0.0 or latency_ms <= 0.0:
            return cx, cy
        cap = float(self._cfg["latency_compensation_cap_frac"])
        dx = max(-cap * frame_w, min(cap * frame_w, (cx - prev_x) / dt * latency_ms / 1000.0))
        dy = max(-cap * frame_h, min(cap * frame_h, (cy - prev_y) / dt * latency_ms / 1000.0))
        return min(float(frame_w), max(0.0, cx + dx)), min(float(frame_h), max(0.0, cy + dy))

    def _clear(self) -> None:
        self.state = "manual"
        self.target_name = None
        self.track_id = None
        self._locked_id = None
        self._last_cx = None
        self._last_cy = None


def _area(bbox) -> float:
    x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)
