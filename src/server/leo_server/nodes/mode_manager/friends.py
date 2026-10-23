"""Known-face store shared by every mode that looks at people."""

from __future__ import annotations

import json
import math
from pathlib import Path


def _cosine(left: list[float], right: list[float]) -> float:
    if len(left) != len(right) or not left:
        return -1.0
    dot = sum(a * b for a, b in zip(left, right))
    left_norm = math.sqrt(sum(a * a for a in left))
    right_norm = math.sqrt(sum(b * b for b in right))
    if left_norm < 1e-9 or right_norm < 1e-9:
        return -1.0
    return dot / (left_norm * right_norm)


class FriendDB:
    def __init__(self, path: Path) -> None:
        self._path = Path(path)
        self._faces: dict[str, list[float]] = {}
        if self._path.is_file():
            loaded = json.loads(self._path.read_text(encoding="utf-8"))
            self._faces = {str(name): [float(v) for v in vector] for name, vector in loaded.items()}

    def names(self) -> list[str]:
        return sorted(self._faces)

    def match(self, embedding: list[float], threshold: float) -> str | None:
        best_name = None
        best_score = threshold
        for name, known in self._faces.items():
            score = _cosine(embedding, known)
            if score >= best_score:
                best_name = name
                best_score = score
        return best_name

    def add(self, name: str, embedding: list[float]) -> None:
        self._faces[name] = [float(v) for v in embedding]
        self._save()

    def remove(self, name: str) -> None:
        self._faces.pop(name, None)
        self._save()

    def _save(self) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self._path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self._faces), encoding="utf-8")
        tmp.replace(self._path)
