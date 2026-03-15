from __future__ import annotations

import json
from pathlib import Path
from tempfile import NamedTemporaryFile


def _dot(a: list[float], b: list[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def _norm(a: list[float]) -> float:
    return _dot(a, a) ** 0.5


def _cosine_similarity(a: list[float], b: list[float]) -> float:
    na = _norm(a)
    nb = _norm(b)
    if na <= 0.0 or nb <= 0.0:
        return -1.0
    return _dot(a, b) / (na * nb)


class FriendDB:
    """Persistent storage for named face embeddings."""

    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)
        self._faces: dict[str, list[float]] = {}
        self.load(self._path)

    def load(self, path: str | Path) -> None:
        p = Path(path)
        if not p.exists():
            self._faces = {}
            return
        with p.open("r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            raise ValueError("friend db content must be JSON object")
        parsed: dict[str, list[float]] = {}
        for name, embedding in data.items():
            if not isinstance(name, str):
                raise ValueError("friend name must be string")
            if not isinstance(embedding, list):
                raise ValueError("embedding must be list")
            parsed[name] = [float(v) for v in embedding]
        self._faces = parsed

    def save(self, path: str | Path | None = None) -> None:
        target = Path(path) if path is not None else self._path
        target.parent.mkdir(parents=True, exist_ok=True)
        data = {name: list(vec) for name, vec in self._faces.items()}
        with NamedTemporaryFile("w", encoding="utf-8", dir=target.parent, delete=False) as tmp:
            json.dump(data, tmp, ensure_ascii=True)
            tmp.flush()
            tmp_path = Path(tmp.name)
        tmp_path.replace(target)

    def add(self, name: str, embedding: list[float]) -> None:
        self._faces[str(name)] = [float(v) for v in embedding]
        self.save()

    def remove(self, name: str) -> None:
        key = str(name)
        if key in self._faces:
            del self._faces[key]
        self.save()

    def list_names(self) -> list[str]:
        return sorted(self._faces.keys())

    def match(self, embedding: list[float], threshold: float) -> str | None:
        best_name = None
        best_score = float(threshold)
        query = [float(v) for v in embedding]
        for name, known in self._faces.items():
            score = _cosine_similarity(query, known)
            if score >= best_score:
                best_score = score
                best_name = name
        return best_name
