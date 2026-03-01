#!/usr/bin/env python3
"""
Capture one frame with CameraController, run YOLO inference with NCNN (Ultralytics),
measure inference time, draw boxes, and save the image.

Usage:
  # Export NCNN model first (once):
  #   from ultralytics import YOLO
  #   YOLO("yolo26n.pt").export(format="ncnn")  # creates 'yolo26n_ncnn_model'
  python capture_yolo_save.py [--model yolo26n_ncnn_model] [--output out.jpg]
"""

import argparse
import sys
import time
from pathlib import Path

import cv2

# Project root
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from ultralytics import YOLO

from modules.camera import CameraController


def load_camera_config():
    """Load camera config for CameraController."""
    try:
        from omegaconf import OmegaConf
        cfg = OmegaConf.load(ROOT / "config" / "camera" / "camera.yaml")
        return cfg
    except ImportError:
        import yaml
        from types import SimpleNamespace
        with open(ROOT / "config" / "camera" / "camera.yaml") as f:
            cfg = yaml.safe_load(f)
        return SimpleNamespace(**cfg)


def main():
    parser = argparse.ArgumentParser(description="Capture frame, YOLO NCNN, save image with boxes")
    parser.add_argument("--model", type=str, default="yolo26n_ncnn_model",
                        help="Path to exported NCNN model (folder or path)")
    parser.add_argument("--output", type=str, default=None,
                        help="Output image path (default: captured_images/yolo_boxes.jpg)")
    args = parser.parse_args()

    # Camera: capture one frame
    camera_cfg = load_camera_config()
    print(camera_cfg)
    camera = CameraController(camera_cfg, output_dir=str(ROOT / "captured_images"), save_images=False)
    camera.setup()
    time.sleep(1.0)

    # YOLO NCNN inference
    model = YOLO(args.model)
    try:
        while True:
            t0 = time.perf_counter()
            frame = camera.get_frame()
            results = model(frame, conf=0.5)
            t1 = time.perf_counter()
            inference_ms = (t1 - t0) * 1000
            print(f"Inference time: {inference_ms:.2f} ms")

            # Image with boxes (Ultralytics plot returns BGR numpy array)
            vis = results[0].plot()
            # Optionally add inference time text
            cv2.putText(vis, f"NCNN {inference_ms:.1f} ms", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            out_path = args.output or str(ROOT / "captured_images" / "yolo_boxes.jpg")
            Path(out_path).parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(out_path, vis)
            print(f"Saved to {out_path}")
    finally:
        camera.stop()


if __name__ == "__main__":
    main()
