from numpy import clip

class ServoController:
    """Unified servo controller."""

    def __init__(self, kit, config):
        self.config = config
        self.kit = kit
        channel = int(config.channel)
        self.servo = self.kit.servo[channel]

        self.zero = float(config.angle.zero)
        self.min_angle = float(config.angle.min)
        self.max_angle = float(config.angle.max)

        self.servo.actuation_range = int(config.range)
        if "pwm" in config:
            self.servo.set_pulse_width_range(
                int(config.pwm.min_pulse),
                int(config.pwm.max_pulse),
            )

    def set_angle(self, angle):
        """Set servo angle."""
        angle = float(angle)
        angle = clip(angle, self.min_angle, self.max_angle) 
        self.servo.angle = float(self.zero + angle)

    def get_angle(self):
        """Get current angle in user space."""
        return float(self.servo.angle) - self.zero

    def move(self, delta_angle):
        """Move by delta angle (relative)."""
        self.set_angle(self.get_angle() + float(delta_angle))

class HeadController:
    """Class for controlling robot head using ServoController for neck and face."""

    def __init__(self, kit, config):
        self.config = config
        self.kit = kit
        self.neck = ServoController(kit, config.neck)
        self.face = ServoController(kit, config.face)
        self.is_set = False

    def setup(self):
        """Initialize servos to default position."""
        self.set_neck_angle(0)
        self.set_face_angle(0)
        self.is_set = True

    def set_neck_angle(self, angle):
        """Set neck angle."""
        self.neck.set_angle(float(angle))

    def set_face_angle(self, angle):
        """Set face angle."""
        self.face.set_angle(float(angle))

    def move_neck(self, delta_angle):
        """Move neck by a delta angle."""
        self.neck.move(float(delta_angle))

    def move_face(self, delta_angle):
        """Move face by a delta angle."""
        self.face.move(float(delta_angle))

    def __del__(self):
        """Cleanup on destruction."""
        if self.is_set:
            self.set_neck_angle(0)
            self.set_face_angle(0)
            self.is_set = False
