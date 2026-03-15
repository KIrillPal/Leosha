from __future__ import annotations

import argparse
import json
import logging
from dataclasses import dataclass
from time import perf_counter_ns

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

LOGGER = logging.getLogger(__name__)


@dataclass
class _TrackCache:
    embedding: list[float]
    last_frame_id: int


class VisionNode(Node):
    """Separate ROS2 process for person detection + face embedding inference."""

    def __init__(self) -> None:
        super().__init__("vision_node")
        self.declare_parameter("yolo_model")
        self.declare_parameter("yolo_imgsz")
        self.declare_parameter("tracker")
        self.declare_parameter("face_model")
        self.declare_parameter("embedding_interval")
        self.declare_parameter("face_min_confidence")
        self.declare_parameter("head_yaw_min_deg")
        self.declare_parameter("head_yaw_max_deg")
        self.declare_parameter("head_yaw_scale_deg")
        self.declare_parameter("face_crop_ratio")
        self.declare_parameter("embedding_input_size")
        self.declare_parameter("embedding_norm_mean")
        self.declare_parameter("embedding_norm_std")
        self.declare_parameter("yaw_eye_dx_epsilon")

        self._frame_id = 0
        self._embedding_cache: dict[int, _TrackCache] = {}

        self._yolo_model = self._require_param("yolo_model", str)
        self._yolo_imgsz = self._require_param("yolo_imgsz", int)
        self._tracker = self._require_param("tracker", str)
        self._face_model = self._require_param("face_model", str)
        self._embedding_interval = self._require_param("embedding_interval", int)
        self._face_min_confidence = self._require_param("face_min_confidence", float)
        self._head_yaw_min_deg = self._require_param("head_yaw_min_deg", float)
        self._head_yaw_max_deg = self._require_param("head_yaw_max_deg", float)
        self._head_yaw_scale_deg = self._require_param("head_yaw_scale_deg", float)
        self._face_crop_ratio = self._require_param("face_crop_ratio", float)
        self._embedding_input_size = self._require_param("embedding_input_size", int)
        self._embedding_norm_mean = self._require_param("embedding_norm_mean", float)
        self._embedding_norm_std = self._require_param("embedding_norm_std", float)
        self._yaw_eye_dx_epsilon = self._require_param("yaw_eye_dx_epsilon", float)
        self._latest_frame = None

        self._np = None
        self._cv2 = None
        self._yolo = None
        self._ort = None
        self._ort_session = None
        self._ort_input_name = ""
        self._init_runtimes()

        self._pub = self.create_publisher(String, "/vision/persons", 10)
        self.create_subscription(CompressedImage, "/camera/image_raw", self._on_frame, 10)
        LOGGER.info(
            "Vision node started | yolo_model=%s imgsz=%d tracker=%s face_model=%s embedding_interval=%d",
            self._yolo_model,
            self._yolo_imgsz,
            self._tracker,
            self._face_model,
            self._embedding_interval,
        )

    def _require_param(self, name: str, cast):
        value = self.get_parameter(name).value
        if value is None:
            raise ValueError(f"Required ROS parameter '{name}' is not set")
        if cast is str and str(value).strip() == "":
            raise ValueError(f"Required ROS parameter '{name}' must be non-empty")
        return cast(value)

    def _init_runtimes(self) -> None:
        import numpy as np
        import cv2
        from ultralytics import YOLO
        import onnxruntime as ort

        self._np = np
        self._cv2 = cv2
        self._yolo = YOLO(self._yolo_model)
        self._ort = ort
        self._ort_session = ort.InferenceSession(
            self._face_model,
            providers=["CPUExecutionProvider"],
        )
        self._ort_input_name = self._ort_session.get_inputs()[0].name

    def _on_frame(self, msg: CompressedImage) -> None:
        self._frame_id += 1
        started_ns = perf_counter_ns()
        persons = self._run_pipeline(msg.data, self._frame_id)
        elapsed_ms = (perf_counter_ns() - started_ns) / 1_000_000.0
        payload = {
            "timestamp_ns": int(self.get_clock().now().nanoseconds),
            "frame_id": self._frame_id,
            "persons": persons,
            "inference_ms": float(elapsed_ms),
        }
        out = String()
        out.data = json.dumps(payload, ensure_ascii=True)
        self._pub.publish(out)

    def _run_pipeline(self, jpeg_bytes: bytes, frame_id: int) -> list[dict]:
        frame = self._decode_jpeg(jpeg_bytes)
        tracks: list[dict] = self._detect_persons_fast(frame)
        embedding_frame = (frame_id % self._embedding_interval) == 0

        persons: list[dict] = []
        for track in tracks:
            track_id = int(track["track_id"])
            face_visible = bool(track["face_visible"])
            face_embedding: list[float] = []

            if face_visible and track["confidence"] >= self._face_min_confidence:
                if embedding_frame:
                    emb = self._compute_face_embedding(frame, track)
                    self._embedding_cache[track_id] = _TrackCache(embedding=emb, last_frame_id=frame_id)
                    face_embedding = emb
                elif track_id in self._embedding_cache:
                    face_embedding = list(self._embedding_cache[track_id].embedding)

            persons.append(
                {
                    "track_id": track_id,
                    "bbox": track["bbox"],
                    "keypoints": track["keypoints"],
                    "head_yaw_deg": float(track["head_yaw_deg"]),
                    "face_visible": face_visible,
                    "face_embedding": face_embedding,
                    "confidence": float(track["confidence"]),
                }
            )
        return persons

    def _decode_jpeg(self, jpeg_bytes: bytes):
        np_arr = self._np.frombuffer(jpeg_bytes, dtype=self._np.uint8)
        frame = self._cv2.imdecode(np_arr, self._cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError("Failed to decode JPEG frame")
        self._latest_frame = frame
        return frame

    def _detect_persons_fast(self, frame) -> list[dict]:
        result = self._yolo.track(
            frame,
            imgsz=self._yolo_imgsz,
            tracker=self._tracker,
            classes=[0],
            persist=True,
            verbose=False,
        )[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return []

        keypoints = result.keypoints
        has_ids = boxes.id is not None
        tracks: list[dict] = []
        for idx in range(len(boxes)):
            box_xyxy = boxes.xyxy[idx].tolist()
            confidence = float(boxes.conf[idx].item())
            track_id = int(boxes.id[idx].item()) if has_ids else idx
            kp_data = []
            face_visible = False
            if keypoints is not None and keypoints.xy is not None:
                xy = keypoints.xy[idx]
                conf = keypoints.conf[idx] if keypoints.conf is not None else None
                for kp_idx in range(int(xy.shape[0])):
                    x = float(xy[kp_idx][0].item())
                    y = float(xy[kp_idx][1].item())
                    c = float(conf[kp_idx].item()) if conf is not None else 1.0
                    kp_data.append([x, y, c])
                if len(kp_data) > 0:
                    face_visible = kp_data[0][2] >= self._face_min_confidence
            tracks.append(
                {
                    "track_id": track_id,
                    "bbox": [float(v) for v in box_xyxy],
                    "keypoints": kp_data,
                    "head_yaw_deg": self._estimate_head_yaw(kp_data),
                    "face_visible": face_visible,
                    "confidence": confidence,
                }
            )
        return tracks

    def _estimate_head_yaw(self, keypoints: list[list[float]]) -> float:
        # COCO keypoints: 0=nose, 1=left_eye, 2=right_eye
        if len(keypoints) < 3:
            return 0.0
        left_eye = keypoints[1]
        right_eye = keypoints[2]
        if left_eye[2] <= 0.0 or right_eye[2] <= 0.0:
            return 0.0
        eye_dx = right_eye[0] - left_eye[0]
        if abs(eye_dx) < self._yaw_eye_dx_epsilon:
            return 0.0
        # Lightweight proxy: eye baseline tilt mapped into yaw-like debug metric.
        yaw = (right_eye[1] - left_eye[1]) / eye_dx * self._head_yaw_scale_deg
        return max(self._head_yaw_min_deg, min(self._head_yaw_max_deg, yaw))

    def _compute_face_embedding(self, frame, track: dict) -> list[float]:
        x1, y1, x2, y2 = [int(v) for v in track["bbox"]]
        h, w = frame.shape[:2]
        x1 = max(0, min(w - 1, x1))
        y1 = max(0, min(h - 1, y1))
        x2 = max(x1 + 1, min(w, x2))
        y2 = max(y1 + 1, min(h, y2))

        # Face area approximation: upper part of person bbox.
        face_h = max(1, int((y2 - y1) * self._face_crop_ratio))
        crop = frame[y1:y1 + face_h, x1:x2]
        if crop.size == 0:
            return []
        resized = self._cv2.resize(
            crop,
            (self._embedding_input_size, self._embedding_input_size),
            interpolation=self._cv2.INTER_LINEAR,
        )
        rgb = self._cv2.cvtColor(resized, self._cv2.COLOR_BGR2RGB).astype("float32")
        rgb = (rgb - self._embedding_norm_mean) / self._embedding_norm_std
        tensor = self._np.transpose(rgb, (2, 0, 1))[self._np.newaxis, ...]
        output = self._ort_session.run(None, {self._ort_input_name: tensor})[0]
        vector = output[0].astype("float32")
        norm = float(self._np.linalg.norm(vector))
        if norm > 1e-6:
            vector = vector / norm
        return [float(v) for v in vector.tolist()]


def main() -> None:
    parser = argparse.ArgumentParser(description="Vision ROS2 node")
    parser.add_argument("--log-level", default="INFO", help="Python log level")
    args, _ = parser.parse_known_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), logging.INFO))

    rclpy.init(args=None)
    node = VisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
