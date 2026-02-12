#!/usr/bin/env python3

from pathlib import Path

from adafruit_servokit import ServoKit
from flask import Flask, render_template, request, jsonify, Response
import hydra
from omegaconf import DictConfig, OmegaConf
import cv2
import argparse
import numpy as np
import json
from time import sleep

from modules.head import HeadController
from modules.camera import CameraController
from modules.car import Car
from modules.detector import HumanDetector

_CONFIG_DIR = str(Path(__file__).resolve().parent / "config")

# Global variables for control
current_x = 0
current_y = 0
tracking_enabled = False

# Keyboard state tracking
key_states = {
    'w': False,
    'a': False, 
    's': False,
    'd': False,
    'shift': False,
    'ctrl': False
}

# Global controller instances
head = None
camera = None
car = None
detector = None
config = None

# Tracking params (from config, applied when tracking is on)
tracking_gain = 0.5
tracking_deadzone = 0.05

app = Flask(__name__)

class NumpyEncoder(json.JSONEncoder):
    """Custom JSON encoder for NumPy types"""
    def default(self, obj):
        if isinstance(obj, (np.integer, np.int64, np.int32, np.int16, np.int8)):
            return int(obj)
        elif isinstance(obj, (np.floating, np.float64, np.float32, np.float16)):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, np.bool_):
            return bool(obj)
        return super().default(obj)

app.json_encoder = NumpyEncoder

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/position', methods=['POST'])
def set_position():
    global current_x, current_y
    
    try:
        data = request.get_json()
        dx = data.get('dx', 0)
        dy = data.get('dy', 0)
        
        # Update positions
        current_x += dx
        current_y += dy
        
        # Clamp positions to reasonable limits and convert to Python types
        current_x = float(np.clip(current_x, -1000, 1000))
        current_y = float(np.clip(current_y, -1000, 1000))
        
        # Control servos based on movement
        head.move_neck(dx * float(head.config.neck.sensitivity))
        head.move_face(dy * float(head.config.face.sensitivity))
        
        return jsonify({
            'success': True,
            'x': current_x,
            'y': current_y
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})

@app.route('/api/reset', methods=['POST'])
def reset_position():
    global current_x, current_y
    current_x = 0
    current_y = 0
    head.set_neck_angle(0)
    head.set_face_angle(0)
    return jsonify({'success': True, 'x': 0, 'y': 0})

@app.route('/api/status')
def get_status():
    return jsonify({
        'x': float(current_x),
        'y': float(current_y),
        'tracking_enabled': tracking_enabled,
        'key_states': key_states
    })

@app.route('/api/tracking', methods=['POST'])
def set_tracking():
    global tracking_enabled
    
    try:
        data = request.get_json()
        tracking = data.get('tracking', False)
        tracking_enabled = bool(tracking)
        
        return jsonify({
            'success': True,
            'tracking': tracking_enabled
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})

@app.route('/api/keyboard', methods=['POST'])
def handle_keyboard():
    """Handle keyboard key press/release events"""
    try:
        data = request.get_json()
        key = data.get('key', '').lower()
        state = data.get('state', False)  # True for pressed, False for released
        action = data.get('action', '')  # 'press' or 'release'
        
        # Update key state
        if key in key_states:
            key_states[key] = state
            # Call appropriate handler based on action
            if action == 'press':
                handle_key_press(key)
            elif action == 'release':
                handle_key_release(key)
        
        return jsonify({
            'success': True,
            'key': key,
            'state': state,
            'action': action
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})

def update_wheels():
    if key_states['a'] == key_states['d']:
        car.set_wheel(config.car.wheel.zero_throttle)
    elif key_states['a']:
        car.set_wheel(config.car.wheel.min_throttle)
    elif key_states['d']:
        car.set_wheel(config.car.wheel.max_throttle)
        
def update_motor():
    speed_mode = config.car.motor.speed.normal
    if key_states['shift']:
        speed_mode = config.car.motor.speed.fast
    
    if key_states['w'] == key_states['s']:
        car.set_speed(config.car.motor.speed.zero)
    elif key_states['w']:
        car.set_speed(speed_mode.forward)
    elif key_states['s']:
        car.set_speed(speed_mode.backward)

def handle_key_press(key):
    """Handler for key press events"""
    # Empty handler functions - to be implemented
    if key == 'w':
        handle_w_press()
    elif key == 'a':
        handle_a_press()
    elif key == 's':
        handle_s_press()
    elif key == 'd':
        handle_d_press()
    elif key == 'shift':
        handle_shift_press()
    elif key == 'ctrl':
        handle_ctrl_press()

def handle_key_release(key):
    """Handler for key release events"""
    # Empty handler functions - to be implemented
    if key == 'w':
        handle_w_release()
    elif key == 'a':
        handle_a_release()
    elif key == 's':
        handle_s_release()
    elif key == 'd':
        handle_d_release()
    elif key == 'shift':
        handle_shift_release()
    elif key == 'ctrl':
        handle_ctrl_release()

# Empty handler functions for key presses
def handle_w_press():
    """Handle W key press"""
    print("W key pressed")
    update_motor()

def handle_w_release():
    """Handle W key release"""
    print("W key released")
    update_motor()

def handle_a_press():
    """Handle A key press"""
    print("A key pressed")
    update_wheels()

def handle_a_release():
    """Handle A key release"""
    print("A key released")
    update_wheels()

def handle_s_press():
    """Handle S key press"""
    print("S key pressed")
    update_motor()

def handle_s_release():
    """Handle S key release"""
    print("S key released")
    update_motor()

def handle_d_press():
    """Handle D key press"""
    print("D key pressed")
    update_wheels()

def handle_d_release():
    """Handle D key release"""
    print("D key released")
    update_wheels()

def handle_shift_press():
    """Handle Shift key press"""
    print("Shift key pressed")
    update_motor()

def handle_shift_release():
    """Handle Shift key release"""
    print("Shift key released")
    update_motor()

def handle_ctrl_press():
    """Handle Ctrl key press"""
    print("Ctrl key pressed")
    update_motor()

def handle_ctrl_release():
    """Handle Ctrl key release"""
    print("Ctrl key released")
    update_motor()
        

def _apply_tracking(frame, h, w):
    """Run detector, draw person boxes, and move head to first person. Returns updated frame."""
    global current_x, current_y
    if detector is None:
        return frame
    bboxes, first_center = detector.detect_persons(frame)
    image_cx = w / 2
    image_cy = h / 2
    # Draw all person boxes; first one (tracked) in green, rest in blue
    for i, (x1, y1, x2, y2) in enumerate(bboxes):
        color = (0, 255, 0) if i == 0 else (255, 165, 0)  # BGR: green = tracked, blue = other
        thickness = 3 if i == 0 else 2
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
        if i == 0:
            cv2.putText(frame, "tracked", (int(x1), int(y1) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    # Move head to first person
    if first_center is not None:
        person_cx, person_cy = first_center
        error_x = (person_cx - image_cx) / max(w, 1)
        error_y = (person_cy - image_cy) / max(h, 1)
        if abs(error_x) >= tracking_deadzone:
            error_x = error_x
        else:
            error_x = 0
        if abs(error_y) >= tracking_deadzone:
            error_y = error_y
        else:
            error_y = 0
        dx = error_x * tracking_gain * 1000
        dy = error_y * tracking_gain * 1000
        current_x = float(np.clip(current_x + dx, -1000, 1000))
        current_y = float(np.clip(current_y + dy, -1000, 1000))
        head.move_neck(dx * float(head.config.neck.sensitivity))
        head.move_face(dy * float(head.config.face.sensitivity))
    return frame


def generate_frames():
    """Frame generator for video stream"""
    import time
    while True:
        frame = camera.get_frame()
        if frame is not None:
            h, w = frame.shape[:2]
            # Draw crosshair at center
            cv2.line(frame, (w//2, 0), (w//2, h), (0, 255, 0), 2)
            cv2.line(frame, (0, h//2), (w, h//2), (0, 255, 0), 2)
            # When tracking on: detect persons, draw boxes, move head
            if tracking_enabled and detector is not None:
                frame = _apply_tracking(frame, h, w)
            # Encode frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            # Yield frame for streaming
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            time.sleep(0.1)

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/capture', methods=['POST'])
def capture_image():
    try:
        camera.capture_image()
        return jsonify({'success': True, 'message': 'Image captured successfully'})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})
    
def init_pca(config):
    return ServoKit(
        channels=int(config.pca.channels), 
        frequency=int(config.pca.frequency)
    )

def main(cfg: DictConfig):
    global head, camera, car, detector, config, tracking_gain, tracking_deadzone

    config = cfg
    tracking_gain = float(OmegaConf.select(cfg, "tracking.gain", default=0.5))
    tracking_deadzone = float(OmegaConf.select(cfg, "tracking.deadzone", default=0.05))

    pca = init_pca(cfg)
    head = HeadController(pca, cfg.head)
    head.setup()

    camera = CameraController(cfg.camera, cfg.output.directory, cfg.output.save_images)
    camera.setup()

    car = Car(pca, cfg.car)
    car.setup()

    model_name = str(OmegaConf.select(cfg, "tracking.model", default="yolov8n.pt"))
    conf_threshold = float(OmegaConf.select(cfg, "tracking.conf_threshold", default=0.5))
    detector = HumanDetector(model_name=model_name, conf_threshold=conf_threshold)
    
    print(f"Starting web server on {cfg.app.host}:{cfg.app.port}")
    print("Open http://<your-pi-ip>:5000 in your browser")
    
    try:
        app.run(host=cfg.app.host, port=cfg.app.port, debug=cfg.app.debug)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # Cleanup
        camera.stop()
        head.neck.stop()
        head.set_face_angle(0)

if __name__ == '__main__':
    hydra_main = hydra.main(config_path=_CONFIG_DIR, config_name="config", version_base=None)(main)
    hydra_main()