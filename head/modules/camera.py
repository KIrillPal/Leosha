from picamera2 import Picamera2
from pathlib import Path
import cv2
import threading
import time

class CameraController:
    """Класс для управления камерой и захвата видео"""
    def __init__(self, config):
        self.config = config
        self.picam2 = Picamera2(tuning=config.tuning_file if config.tuning_file else None)
        self.frame = None
        self.lock = threading.Lock()
        self.running = False
        
    def setup(self):
        """Настройка и запуск камеры"""
        # Создание конфигурации
        config = self.picam2.create_still_configuration(
            main={"size": tuple(self.config.resolution)}
        )
        self.picam2.configure(config)
        
        # Установка параметров
        self.picam2.set_controls({
            "ExposureTime": self.config.exposure_time,
            "AnalogueGain": self.config.analogue_gain,
            "AwbEnable": self.config.awb_enable,
            "AeEnable": self.config.ae_enable
        })
        
        # Запуск камеры
        self.picam2.start()
        self.running = True
        
        # Запуск потока для захвата кадров
        self.thread = threading.Thread(target=self._capture_loop)
        self.thread.start()
        
    def _capture_loop(self):
        """Цикл захвата кадров в отдельном потоке"""
        while self.running:
            frame = self.picam2.capture_array()
            # Конвертация из RGB в BGR для OpenCV
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            with self.lock:
                self.frame = frame
            time.sleep(1 / self.config.fps)
    
    def get_frame(self):
        """Получение текущего кадра"""
        with self.lock:
            return self.frame
            
    def capture_image(self, output_path=None):
        """Захват и сохранение отдельного изображения"""
        if output_path is None:
            output_path = self.config.output_path
            
        output_dir = Path(output_path)
        output_dir.mkdir(exist_ok=True, parents=True)
        
        frame = self.picam2.capture_array()
        timestamp = int(time.time())
        cv2.imwrite(str(output_dir / f"capture_{timestamp}.jpg"), frame)
        
    def stop(self):
        """Остановка камеры"""
        self.running = False
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join()
        self.picam2.stop()