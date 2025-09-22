from adafruit_servokit import ServoKit
import numpy as np
import time
import threading
from time import sleep

class ContinuousServoController:
    """Controller for continuous rotation servo with virtual angle tracking"""
    def __init__(self, servo, config):
        self.config = config
        self.servo = servo
        self.min_angle = float(config.min_angle)
        self.max_angle = float(config.max_angle)
        self.zero_point = float(config.start.zero_point)
        self.max_speed = float(config.max_speed)
        self.throttle_scale = float(config.throttle_scale)
        
        self.servo.set_pulse_width_range(750, 2750)
        self.servo.throttle = 0.0
        
        self.current_angle = 0.0
        self.target_angle = 0.0
        self.last_update = time.time()
        
        # Start update thread
        self.running = True
        self.thread = threading.Thread(target=self._update_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def _update_loop(self):
        """Background thread to update servo position"""
        while self.running:
            #print("Target", self.target_angle, "Current", self.current_angle)
            current_time = time.time()
            dt = current_time - self.last_update
            self.last_update = current_time
            
            # Calculate angle difference and speed
            angle_diff = self.target_angle - self.current_angle
            
            # Use proportional control to determine speed
            speed = np.clip(angle_diff * 10, -self.max_speed, self.max_speed)
            if abs(angle_diff) < 0.5:
                speed = 0
            
            # if abs(angle_diff) < 1.0:
            #     speed = 0.0
            
            # Update current angle based on speed
            self.current_angle += speed * dt
            
            # Clamp angle to limits
            self.current_angle = float(np.clip(self.current_angle, self.min_angle, self.max_angle))
            
            # Convert angle to throttle
            throttle = speed * self.throttle_scale
            self.servo.throttle = float(np.clip(throttle, -0.9, 0.8))
                
            time.sleep(0.01)
    
    def set_target_angle(self, angle):
        """Set target angle for servo"""
        self.target_angle = float(np.clip(angle, self.min_angle, self.max_angle))
    
    def get_current_angle(self):
        """Get current virtual angle"""
        return float(self.current_angle)
    
    def set_servo_mode(self, mode_config):
        self.servo.set_pulse_width_range(
            mode_config.pwm.min_pulse,
            mode_config.pwm.max_pulse
        )
        self.servo.throttle = mode_config.zero_point
    
    def stop(self):
        """Stop the servo and cleanup"""
        self.running = False
        self.set_servo_mode(self.config.stop)
        if self.thread.is_alive():
            self.thread.join()


class HeadController:
    """Class for controlling robot head"""
    def __init__(self, config):
        self.config = config
        self.kit = ServoKit(
            channels=int(config.pca.channels), 
            frequency=int(config.pca.frequency)
        )
        
        # Initialize neck (continuous servo)
        self.neck = ContinuousServoController(
            self.kit.continuous_servo[int(config.neck.channel)],
            config.neck
        )
        
        # Initialize face (standard servo)
        self.face = self.kit.servo[int(config.face.channel)]
        self.face.actuation_range = 180
        
        self.is_set = False
    
    def setup(self):
        """Initialize servos to default position"""
        self.set_neck_angle(0)
        self.set_face_angle(0)
        self.is_set = True
        
    def set_neck_angle(self, angle):
        """Set neck angle (wraps to continuous servo)"""
        self.neck.set_target_angle(float(angle))
        
    def set_face_angle(self, angle):
        """Set face angle"""
        clamped_angle = np.clip(float(angle) + float(self.config.face.zero_angle), 
                               float(self.config.face.min_angle) + float(self.config.face.zero_angle),
                               float(self.config.face.max_angle) + float(self.config.face.zero_angle))
        self.face.angle = float(clamped_angle)
        
    def move_neck(self, delta_angle):
        """Move neck by a delta angle"""
        current_angle = self.neck.get_current_angle()
        self.set_neck_angle(current_angle - float(delta_angle))
        
    def move_face(self, delta_angle):
        """Move face by a delta angle"""
        current_face_angle = self.face.angle - float(self.config.face.zero_angle)
        self.set_face_angle(current_face_angle - float(delta_angle))
        
    def __del__(self):
        """Cleanup on destruction"""
        if self.is_set:
            self.neck.stop()
            self.set_face_angle(0)
            self.is_set = False