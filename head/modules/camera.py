from picamera2 import Picamera2
from pathlib import Path
import cv2
import threading
import asyncio
import time
import numpy as np

class CameraController:
    """Class for controlling camera and capturing video."""
    def __init__(self, config, output_dir="captured_images", save_images=True):
        self.config = config
        self.output_dir = Path(output_dir)
        self.save_images = save_images
        self.output_dir.mkdir(exist_ok=True, parents=True)
        
        self.picam2 = Picamera2(tuning=config.tuning_file if config.tuning_file else None)
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
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
        
        self._thread = threading.Thread(target=self._run_async_capture_loop)
        self._thread.daemon = True
        self._thread.start()
        
    async def _capture_loop_async(self):
        """Async capture loop: use asyncio.sleep instead of blocking time.sleep."""
        loop = asyncio.get_running_loop()
        while self.running:
            # Run blocking capture in executor so the event loop is not blocked
            frame = self.picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            with self.lock:
                self.frame = frame
            await asyncio.sleep(max(0, 1 / self.config.fps - 0.001))
        
    def _run_async_capture_loop(self):
        """Run the async capture loop in this thread's event loop."""
        asyncio.run(self._capture_loop_async())
    
    def get_frame(self):
        """Get current frame (copy)."""
        with self.lock:
            return self.frame.copy()
            
    def capture_image(self, filename=None):
        """Capture and save an image (uses current buffered frame)."""
        if not self.save_images:
            return
        if filename is None:
            timestamp = int(time.time())
            filename = f"capture_{timestamp}.jpg"
        frame = self.get_frame()
        if frame is not None:
            cv2.imwrite(str(self.output_dir / filename), frame)
        
    def stop(self):
        """Stop camera and capture thread."""
        self.running = False
        self._thread.join(timeout=2.0)
        self.picam2.stop()