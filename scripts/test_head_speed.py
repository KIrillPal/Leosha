from adafruit_servokit import ServoKit
from time import sleep
import time
import sys
import numpy as np


ZERO_THROTTLE = 0.05
SPEED_TO_THROTTLE_RATIO = 0.85

PWM_MIN_PULSE = 1000  # 1ms
PWM_MAX_PULSE = 2000  # 2ms

DEFAULT_NECK_CHANNEL = 3
DEFAULT_FACE_CHANNEL = 2

NECK_MIN_ZERO = 0.19
NECK_MAX_ZERO = 0.33
NECK_ZERO = (NECK_MIN_ZERO + NECK_MAX_ZERO) / 2

NECK_RANGE = 0.5
NECK_LEFT_LIMIT = NECK_MIN_ZERO - NECK_RANGE
NECK_RIGHT_LIMIT = NECK_MAX_ZERO + NECK_RANGE
FACE_ZERO_ANGLE = 120

PCA_CHANNELS = 16
PCA_FREQUENCY = 250

class Head:
    def __init__(
        self, 
        neck_channel : int = DEFAULT_NECK_CHANNEL, 
        face_channel : int = DEFAULT_FACE_CHANNEL
    ):
        self.neck_channel = neck_channel
        self.face_channel = face_channel
        
        self.kit = ServoKit(
            channels=PCA_CHANNELS, 
            frequency=PCA_FREQUENCY
        )
        self.neck = self.kit.continuous_servo[neck_channel]
        self.face = self.kit.servo[face_channel]
        self.is_set = False
        self.neck.set_pulse_width_range(750, 2750)
    
    def setup(self):
        self.set_neck_speed(0.0)
        self.set_face_angle(0.0)
        self.is_set = True
        #self.neck.set_pulse_width_range(0, 4096)
        
    def set_neck_speed(self, speed : float):
        """Speed from -1 (total backward) to +1 (total forward)"""
        throttle = max(min(speed, 1.0), -1.0) * NECK_RANGE + NECK_ZERO
        self.neck.throttle = throttle
        
    def set_neck_throttle(self, throttle : float):
        """Set throttle"""
        self.neck.throttle = throttle
    
    def set_face_angle(self, angle : float):
        """Degree from -1 (total left) to +1 (total right)"""
        self.face.angle = np.clip(angle + FACE_ZERO_ANGLE, -180, 180)
        
    def __del__(self):
        if self.is_set:
            self.set_neck_speed(0.0)
            self.set_face_angle(0.0)
            self.is_set = False


def track_mouse_position(interval=0.1):
    """
    Отслеживает и выводит текущие координаты мыши на экране.
    :param interval: интервал обновления в секундах
    """
    try:
        print("Отслеживание координат мыши. Нажмите Ctrl+C для выхода.")
        while True:
            x, y = pyautogui.position()
            print(f"X: {x:4} | Y: {y:4}", end='\n')
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nЗавершение работы.")
        sys.exit()


if __name__ == "__main__":
    #sleep(10)
    
    head = Head()
    head.setup()
    #head.set_neck_throttle(-1.0)
    # for throttle in np.linspace(-1, 1, 360):
    #     print("Set throttle", throttle)
    #     head.set_neck_throttle(throttle)
    #     sleep(5)
    
    head.neck.set_pulse_width_range(750, 2250)
    