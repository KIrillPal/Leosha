from adafruit_servokit import ServoKit

class Car:
    def __init__(self, kit, config):
        self.config = config
        self.kit = kit
        
        # Initialize motor and wheel servos
        self.motor = self.kit.continuous_servo[int(config.motor.channel)]
        self.wheel = self.kit.continuous_servo[int(config.wheel.channel)]
        
        self.is_set = False
    
    def setup(self):
        """Initialize car components"""
        # Set PWM pulse width range
        self.motor.set_pulse_width_range(
            int(self.config.pwm.min_pulse), 
            int(self.config.pwm.max_pulse)
        )
        self.wheel.set_pulse_width_range(
            int(self.config.pwm.min_pulse), 
            int(self.config.pwm.max_pulse)
        )
        
        # Set initial state
        self.set_speed(self.config.motor.speed.zero)
        self.set_wheel(float(self.config.wheel.zero_throttle))
        self.is_set = True
        
    def set_speed(self, speed: float):
        """Set speed from -1 (total backward) to +1 (total forward)"""
        speed = float(speed)
        throttle = float(self.config.motor.zero_throttle) + speed * float(self.config.motor.speed_to_throttle_ratio)
        self.motor.throttle = throttle
        
    def set_wheel(self, degree: float):
        """Set wheel position from -1 (total left) to +1 (total right)"""
        degree = float(degree)
        # Invert the degree for proper direction
        self.wheel.throttle = -degree
        
    def stop(self):
        """Stop the car"""
        self.set_speed(self.config.motor.speed.zero)
        self.set_wheel(float(self.config.wheel.zero_throttle))
        
    def __del__(self):
        """Cleanup on destruction"""
        if self.is_set:
            self.stop()
            self.is_set = False