from __future__ import annotations

import argparse
import json
import logging
import os
from dataclasses import dataclass
from pathlib import Path
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
        # Declare with default so rclpy does not warn; launch overwrites from config.
        self.declare_parameter("yolo_model", "")
        self.declare_parameter("yolo_device", "")
        self.declare_parameter("yolo_conf", 0.25)
        self.declare_parameter("yolo_imgsz", 640)
        self.declare_parameter("tracker", "")
        self.declare_parameter("enable_face_embedding", False)
        self.declare_parameter("face_model", "")
        self.declare_parameter("embedding_interval", 25)
        self.declare_parameter("face_min_confidence", 0.5)
        self.declare_parameter("head_yaw_min_deg", -45.0)
        self.declare_parameter("head_yaw_max_deg", 45.0)
        self.declare_parameter("head_yaw_scale_deg", 57.2958)
        self.declare_parameter("face_crop_ratio", 0.45)
        self.declare_parameter("embedding_input_size", 112)
        self.declare_parameter("embedding_norm_mean", 127.5)
        self.declare_parameter("embedding_norm_std", 128.0)
        self.declare_parameter("yaw_eye_dx_epsilon", 1e-6)

        self._frame_id = 0
        self._embedding_cache: dict[int, _TrackCache] = {}

        self._yolo_model = self._require_param("yolo_model", str)
        self._yolo_device = self._require_param("yolo_device", str)
        self._yolo_conf = self._require_param("yolo_conf", float)
        self._yolo_imgsz = self._require_param("yolo_imgsz", int)
        self._tracker = self._require_param("tracker", str)
        self._enable_face_embedding = self._require_param("enable_face_embedding", bool)
        face_model_raw = self.get_parameter("face_model").value
        if self._enable_face_embedding:
            if face_model_raw is None or str(face_model_raw).strip() == "":
                raise ValueError("Required ROS parameter 'face_model' must be set when enable_face_embedding=true")
            self._face_model = str(face_model_raw)
        else:
            self._face_model = str(face_model_raw) if face_model_raw is not None else ""
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
            "Vision node started | yolo_model=%s device=%s conf=%.2f imgsz=%d tracker=%s face_model=%s embedding_interval=%d",
            self._yolo_model,
            self._yolo_device,
            self._yolo_conf,
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
        if cast is bool:
            if isinstance(value, bool):
                return value
            text = str(value).strip().lower()
            if text in {"1", "true", "yes", "on"}:
                return True
            if text in {"0", "false", "no", "off"}:
                return False
            raise ValueError(f"Required ROS parameter '{name}' must be boolean")
        return cast(value)

    def _resolve_yolo_model_path(self) -> str:
        """Resolve YOLO model: if .pt, use or create OpenVINO (fp16) in same dir; if .xml use parent dir; else use as-is."""
        raw = self._yolo_model.strip()
        path = Path(raw).expanduser().resolve()
        if path.is_dir():
            # Already a directory: treat as OpenVINO if it contains .xml
            if any(path.glob("*.xml")):
                LOGGER.info("Using existing OpenVINO model dir: %s", path)
                return str(path)
            return raw
        if path.is_file() and path.suffix.lower() == ".xml":
            # OpenVINO: Ultralytics expects the directory containing .xml, not the .xml path
            parent = path.parent
            if any(parent.glob("*.xml")):
                LOGGER.info("Using OpenVINO model dir (from .xml path): %s", parent)
                return str(parent)
            return raw
        if path.suffix.lower() == ".pt":
            if not path.exists():
                return raw
            parent = path.parent
            openvino_dir = parent / (path.stem + "_openvino_model")
            if openvino_dir.is_dir() and any(openvino_dir.glob("*.xml")):
                LOGGER.info("Using existing OpenVINO model: %s", openvino_dir)
                return str(openvino_dir)
            from ultralytics import YOLO

            LOGGER.info("Converting YOLO .pt to OpenVINO (fp16) at %s", openvino_dir)
            model = YOLO(str(path))
            cwd = os.getcwd()
            try:
                os.chdir(parent)
                model.export(
                    format="openvino",
                    half=True,
                    imgsz=self._yolo_imgsz,
                )
            finally:
                os.chdir(cwd)
            if not openvino_dir.is_dir():
                raise RuntimeError(f"OpenVINO export did not create {openvino_dir}")
            return str(openvino_dir)
        return raw

    def _init_runtimes(self) -> None:
        import numpy as np
        import cv2
        from ultralytics import YOLO

        self._np = np
        self._cv2 = cv2
        model_path = self._resolve_yolo_model_path()
        self._yolo = YOLO(model_path)
        if self._enable_face_embedding:
            import onnxruntime as ort

            self._ort = ort
            self._ort_session = ort.InferenceSession(
                self._face_model,
                providers=["CPUExecutionProvider"],
            )
            self._ort_input_name = self._ort_session.get_inputs()[0].name

    def _on_frame(self, msg: CompressedImage) -> None:
        self._frame_id += 1
        started_ns = perf_counter_ns()
        persons, arcface_count, timing = self._run_pipeline(msg.data, self._frame_id)
        elapsed_ms = (perf_counter_ns() - started_ns) / 1_000_000.0
        LOGGER.info(
            "Vision frame_id=%d persons=%d inference_ms=%.1f arcface_ran=%s decode_ms=%.1f detect_ms=%.1f post_ms=%.1f",
            self._frame_id,
            len(persons),
            elapsed_ms,
            arcface_count if self._enable_face_embedding else "disabled",
            timing["decode_ms"],
            timing["detect_ms"],
            timing["post_ms"],
        )
        LOGGER.debug(
            "Vision detailed frame_id=%d bytes=%d tracks=%d embedding_frame=%s decode_ok=%s cache_size=%d",
            self._frame_id,
            len(msg.data),
            timing["tracks_count"],
            timing["embedding_frame"],
            timing["decode_ok"],
            len(self._embedding_cache),
        )
        payload = {
            "timestamp_ns": int(self.get_clock().now().nanoseconds),
            "frame_id": self._frame_id,
            "persons": persons,
            "inference_ms": float(elapsed_ms),
        }
        out = String()
        out.data = json.dumps(payload, ensure_ascii=True)
        self._pub.publish(out)

    def _run_pipeline(self, jpeg_bytes: bytes, frame_id: int) -> tuple[list[dict], int, dict]:
        decode_started_ns = perf_counter_ns()
        frame = self._decode_jpeg(jpeg_bytes)
        decode_ms = (perf_counter_ns() - decode_started_ns) / 1_000_000.0
        if frame is None:
            LOGGER.warning("Skipping frame_id=%d: failed to decode JPEG (%d bytes)", frame_id, len(jpeg_bytes))
            return [], 0, {
                "decode_ms": float(decode_ms),
                "detect_ms": 0.0,
                "post_ms": 0.0,
                "tracks_count": 0,
                "embedding_frame": False,
                "decode_ok": False,
            }
        detect_start_ns = perf_counter_ns()
        tracks: list[dict] = self._detect_persons_fast(frame)
        detect_ms = (perf_counter_ns() - detect_start_ns) / 1_000_000.0
        LOGGER.debug(
            "Detector frame_id=%d detected=%d detect_ms=%.1f",
            frame_id,
            len(tracks),
            detect_ms,
        )
        embedding_frame = (frame_id % self._embedding_interval) == 0
        arcface_count = 0

        post_start_ns = perf_counter_ns()
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
                    if self._enable_face_embedding and emb:
                        arcface_count += 1
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
        post_ms = (perf_counter_ns() - post_start_ns) / 1_000_000.0
        return persons, arcface_count, {
            "decode_ms": float(decode_ms),
            "detect_ms": float(detect_ms),
            "post_ms": float(post_ms),
            "tracks_count": len(tracks),
            "embedding_frame": embedding_frame,
            "decode_ok": True,
        }

    def _decode_jpeg(self, jpeg_bytes: bytes):
        """Decode JPEG bytes to BGR frame. Returns None if decode fails (e.g. truncated/empty)."""
        np_arr = self._np.frombuffer(jpeg_bytes, dtype=self._np.uint8)
        frame = self._cv2.imdecode(np_arr, self._cv2.IMREAD_COLOR)
        if frame is None:
            return None
        self._latest_frame = frame
        return frame

    def _detect_persons_fast(self, frame) -> list[dict]:
        result = self._yolo.track(
            frame,
            conf=self._yolo_conf,
            imgsz=self._yolo_imgsz,
            device=self._yolo_device,
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
        if not self._enable_face_embedding:
            return []
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
