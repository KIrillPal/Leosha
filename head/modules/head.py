from adafruit_servokit import ServoKit
import numpy as np

class ServoController:
    """Класс для управления сервоприводами"""
    def __init__(self, config):
        self.config = config
        self.kit = ServoKit(
            channels=config.pca_channels, 
            frequency=config.pca_frequency
        )
        
        # Вычисляем нулевое положение шеи
        self.neck_zero = (config.neck_min_zero + config.neck_max_zero) / 2
        
    def set_neck_speed(self, channel, speed):
        """Установка скорости вращения сервопривода шеи"""
        throttle = max(min(speed, 1.0), -1.0) * self.config.neck_range + self.neck_zero
        self.kit.continuous_servo[channel].throttle = throttle
        
    def set_face_angle(self, channel, angle):
        """Установка угла поворота сервопривода лица"""
        self.kit.servo[channel].angle = np.clip(angle + self.config.face_zero_angle, -180, 180)

class HeadController:
    """Класс для управления головой робота"""
    def __init__(self, config):
        self.config = config
        self.servo = ServoController(config)
        self.is_set = False
    
    def setup(self):
        """Инициализация сервоприводов"""
        self.set_neck_speed(0.0)
        self.set_face_angle(0.0)
        self.is_set = True
        
    def set_neck_speed(self, speed):
        """Установка скорости вращения шеи"""
        self.servo.set_neck_speed(self.config.neck_channel, speed)
        
    def set_face_angle(self, angle):
        """Установка угла поворота лица"""
        self.servo.set_face_angle(self.config.face_channel, angle)
        
    def __del__(self):
        """Гарантированная остановка сервоприводов при уничтожении объекта"""
        if self.is_set:
            self.set_neck_speed(0.0)
            self.set_face_angle(0.0)
            self.is_set = False