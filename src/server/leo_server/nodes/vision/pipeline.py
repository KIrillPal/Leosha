"""YOLO pose on a camera frame. Face embeddings stay optional and off unless configured."""

from __future__ import annotations


class FrameGate:
    """True only when the camera stamp changes, so duplicate republishes are skipped."""

    def __init__(self) -> None:
        self._stamp = None

    def accept(self, stamp: int) -> bool:
        if stamp == self._stamp:
            return False
        self._stamp = stamp
        return True


def head_yaw(keypoints: list, cfg: dict) -> float:
    if len(keypoints) < 3:
        return 0.0
    left, right = keypoints[1], keypoints[2]
    if left[2] <= 0.0 or right[2] <= 0.0:
        return 0.0
    eye_dx = right[0] - left[0]
    if abs(eye_dx) < float(cfg["yaw_eye_dx_epsilon"]):
        return 0.0
    yaw = (right[1] - left[1]) / eye_dx * float(cfg["head_yaw_scale_deg"])
    return max(float(cfg["head_yaw_min_deg"]), min(float(cfg["head_yaw_max_deg"]), yaw))


class PosePipeline:
    def __init__(self, cfg: dict) -> None:
        self._cfg = cfg
        self._model = None
        self._session = None
        self._input_name = None
        self._cache: dict[int, list[float]] = {}
        self._ready = False
        self.error = ""

    def load(self) -> None:
        from ultralytics import YOLO

        self._model = YOLO(self._cfg["yolo_model"])
        if self._cfg["enable_face_embedding"]:
            import onnxruntime as ort

            self._session = ort.InferenceSession(self._cfg["face_model"], providers=["CPUExecutionProvider"])
            self._input_name = self._session.get_inputs()[0].name
        self._ready = True

    def infer(self, jpeg: bytes, frame_id: int, width: int, height: int) -> dict:
        if not self._ready:
            return self._empty(frame_id, width, height, "model not loaded")
        import cv2
        import numpy as np

        frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return self._empty(frame_id, width, height, "decode failed")
        persons = self._detect(frame, frame_id)
        return {
            "frame_id": frame_id,
            "width": int(frame.shape[1]),
            "height": int(frame.shape[0]),
            "persons": persons,
        }

    def _detect(self, frame, frame_id: int) -> list[dict]:
        result = self._model.track(
            frame,
            conf=float(self._cfg["yolo_conf"]),
            imgsz=int(self._cfg["yolo_imgsz"]),
            device=self._cfg["yolo_device"],
            tracker=self._cfg["tracker"],
            classes=[0],
            persist=True,
            verbose=False,
        )[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return []
        keypoints = result.keypoints
        people = []
        run_embedding = self._cfg["enable_face_embedding"] and frame_id % int(self._cfg["embedding_interval"]) == 0
        for index in range(len(boxes)):
            people.append(self._person(frame, boxes, keypoints, index, frame_id, run_embedding))
        return people

    def _person(self, frame, boxes, keypoints, index: int, frame_id: int, run_embedding: bool) -> dict:
        track_id = int(boxes.id[index].item()) if boxes.id is not None else index
        points = []
        if keypoints is not None and keypoints.xy is not None:
            xy = keypoints.xy[index]
            conf = keypoints.conf[index] if keypoints.conf is not None else None
            for key_index in range(int(xy.shape[0])):
                score = float(conf[key_index].item()) if conf is not None else 0.0
                points.append([float(xy[key_index][0].item()), float(xy[key_index][1].item()), score])
        visible = bool(points) and points[0][2] >= float(self._cfg["face_min_confidence"])
        embedding = self._embedding(frame, boxes.xyxy[index].tolist(), track_id, frame_id, visible, run_embedding)
        return {
            "track_id": track_id,
            "bbox": [float(v) for v in boxes.xyxy[index].tolist()],
            "keypoints": points,
            "head_yaw_deg": head_yaw(points, self._cfg),
            "face_visible": visible,
            "face_embedding": embedding,
            "confidence": float(boxes.conf[index].item()),
        }

    def _embedding(self, frame, bbox, track_id: int, frame_id: int, visible: bool, run_embedding: bool) -> list[float]:
        if not visible or not self._cfg["enable_face_embedding"]:
            return []
        if not run_embedding:
            return list(self._cache.get(track_id, []))
        vector = self._embed_crop(frame, bbox)
        self._cache[track_id] = vector
        if len(self._cache) > 200:
            self._cache.pop(next(iter(self._cache)))
        return vector

    def _embed_crop(self, frame, bbox) -> list[float]:
        import cv2
        import numpy as np

        x1, y1, x2, y2 = [int(v) for v in bbox]
        height, width = frame.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(width, max(x1 + 1, x2)), min(height, max(y1 + 1, y2))
        face_h = max(1, int((y2 - y1) * float(self._cfg["face_crop_ratio"])))
        crop = frame[y1:y1 + face_h, x1:x2]
        if crop.size == 0:
            return []
        size = int(self._cfg["embedding_input_size"])
        resized = cv2.resize(crop, (size, size))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype("float32")
        rgb = (rgb - float(self._cfg["embedding_norm_mean"])) / float(self._cfg["embedding_norm_std"])
        tensor = np.transpose(rgb, (2, 0, 1))[None, ...]
        output = self._session.run(None, {self._input_name: tensor})[0][0]
        norm = float(np.linalg.norm(output))
        if norm > 1e-6:
            output = output / norm
        return [float(v) for v in output.tolist()]

    def _empty(self, frame_id: int, width: int, height: int, reason: str) -> dict:
        self.error = reason
        return {"frame_id": frame_id, "width": width, "height": height, "persons": []}

