from picamera2 import Picamera2
from pathlib import Path
import cv2
import threading
import time

class CameraController:
    """Class for controlling camera and capturing video"""
    def __init__(self, config, output_dir="captured_images", save_images=True):
        self.config = config
        self.output_dir = Path(output_dir)
        self.save_images = save_images
        self.output_dir.mkdir(exist_ok=True, parents=True)
        
        self.picam2 = Picamera2(tuning=config.tuning_file if config.tuning_file else None)
        self.frame = None
        self.lock = threading.Lock()
        self.running = False
        
    def setup(self):
        """Setup and start camera"""
        # Create configuration
        config = self.picam2.create_still_configuration(
            main={"size": tuple(self.config.resolution)}
        )
        self.picam2.configure(config)
        
        # Set controls
        self.picam2.set_controls({
            "ExposureTime": self.config.exposure_time,
            "AnalogueGain": self.config.analogue_gain,
            "AwbEnable": self.config.awb_enable,
            "AeEnable": self.config.ae_enable
        })
        
        # Start camera
        self.picam2.start()
        self.running = True
        
        # Start frame capture thread
        self.thread = threading.Thread(target=self._capture_loop)
        self.thread.daemon = True
        self.thread.start()
        
    def _capture_loop(self):
        """Frame capture loop in background thread"""
        while self.running:
            frame = self.picam2.capture_array()
            # Convert from RGB to BGR for OpenCV
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            with self.lock:
                self.frame = frame
            time.sleep(1 / self.config.fps)
    
    def get_frame(self):
        """Get current frame"""
        with self.lock:
            return self.frame
            
    def capture_image(self, filename=None):
        """Capture and save an image"""
        if not self.save_images:
            return
            
        if filename is None:
            timestamp = int(time.time())
            filename = f"capture_{timestamp}.jpg"
            
        frame = self.picam2.capture_array()
        cv2.imwrite(str(self.output_dir / filename), frame)
        
    def stop(self):
        """Stop camera"""
        self.running = False
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join()
        self.picam2.stop()