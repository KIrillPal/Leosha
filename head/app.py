#!/usr/bin/env python3

from flask import Flask, render_template, request, jsonify, Response
import hydra
from omegaconf import DictConfig, OmegaConf
import cv2
import threading
import numpy as np

from modules.head import HeadController
from modules.camera import CameraController

# Глобальные переменные для управления
current_x = 0
current_y = 0
is_tracking = False

# Глобальные экземпляры контроллеров
head = None
camera = None

app = Flask(__name__)

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
        
        current_x += dx
        current_y += dy
        
        # Ограничиваем значения для безопасности
        current_x = max(min(current_x, 1000), -1000)
        current_y = max(min(current_y, 1000), -1000)
        
        # Управляем сервоприводами
        head.set_face_angle(-current_y * 0.1)
        head.set_neck_speed(dx / -13)
        
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
    head.set_face_angle(0.0)
    head.set_neck_speed(0.0)
    return jsonify({'success': True, 'x': 0, 'y': 0})

@app.route('/api/status')
def get_status():
    return jsonify({
        'x': current_x,
        'y': current_y,
        'tracking': is_tracking
    })

@app.route('/api/tracking', methods=['POST'])
def toggle_tracking():
    global is_tracking
    is_tracking = not is_tracking
    return jsonify({'tracking': is_tracking})

@app.route('/api/capture', methods=['POST'])
def capture_image():
    try:
        camera.capture_image()
        return jsonify({'success': True, 'message': 'Image captured successfully'})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})

def generate_frames():
    """Генератор кадров для видео потока"""
    while True:
        frame = camera.get_frame()
        if frame is not None:
            # Кодируем кадр в JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            
            # Формируем HTTP ответ с кадром
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            # Если кадр не доступен, ждем немного
            threading.Event().wait(0.1)

@app.route('/video_feed')
def video_feed():
    """Маршрут для видео потока"""
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@hydra.main(config_path="config", config_name="config", version_base=None)
def main(cfg: DictConfig):
    global head, camera
    
    # Инициализация контроллеров
    head = HeadController(cfg.head)
    head.setup()
    
    camera = CameraController(cfg.camera)
    camera.setup()
    
    print(f"Запуск веб-сервера на {cfg.app.host}:{cfg.app.port}")
    print("Откройте http://<IP-адрес-вашего-Pi>:5000 в браузере")
    
    try:
        app.run(host=cfg.app.host, port=cfg.app.port, debug=cfg.app.debug)
    except KeyboardInterrupt:
        print("\nЗавершение работы...")
    finally:
        # Гарантированная остановка сервоприводов и камеры
        camera.stop()
        head.set_neck_speed(0.0)
        head.set_face_angle(0.0)

if __name__ == '__main__':
    main()