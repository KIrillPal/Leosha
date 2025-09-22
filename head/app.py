#!/usr/bin/env python3

from flask import Flask, render_template, request, jsonify, Response
import hydra
from omegaconf import DictConfig, OmegaConf
import cv2
import argparse
import numpy as np
import json

from modules.head import HeadController
from modules.camera import CameraController

# Global variables for control
current_x = 0
current_y = 0
tracking_enabled = False

# Global controller instances
head = None
camera = None

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
        'tracking_enabled': tracking_enabled
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

def generate_frames():
    """Frame generator for video stream"""
    while True:
        frame = camera.get_frame()
        if frame is not None:
            # Draw crosshair at center
            h, w = frame.shape[:2]
            cv2.line(frame, (w//2, 0), (w//2, h), (0, 255, 0), 2)
            cv2.line(frame, (0, h//2), (w, h//2), (0, 255, 0), 2)
            
            # Encode frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            
            # Yield frame for streaming
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            # Wait if no frame available
            import time
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

def main(cfg: DictConfig):
    global head, camera
    
    # Initialize controllers
    head = HeadController(cfg.head)
    head.setup()
    
    camera = CameraController(cfg.camera, cfg.output.directory, cfg.output.save_images)
    camera.setup()
    
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
    # Run with Hydra
    hydra_main = hydra.main(version_base=None)(main)
    hydra_main()