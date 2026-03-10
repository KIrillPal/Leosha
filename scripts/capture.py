#!/usr/bin/env python3

from picamera2 import Picamera2
import time
import os
from tqdm import tqdm
from pathlib import Path

def capture_image(output_path="captured_images", tuning_file=None):
    # Initialize with tuning
    picam2 = Picamera2(tuning=tuning_file)
    
    # Get available controls
    controls = picam2.camera_controls
    print("Available controls:", list(controls.keys()))
    
    # Create and apply config
    config = picam2.create_still_configuration(
        main={"size": (1024, 768)}
    )
    picam2.configure(config)
    
    FPS = 60
    picam2.set_controls({
        "ExposureTime": 10000,
        "AnalogueGain": 50.0,
        "AwbEnable": True,
        "AeEnable": False
    })
    
    # Start capture
    picam2.start()
    
    output_dir = Path(output_path)
    output_dir.mkdir(exist_ok=True, parents=True)
    
    # Capture image
    for i in tqdm(range(1000)):
        picam2.capture_file(str(output_dir / f"{i:02}.jpg"))
        time.sleep(1 / FPS)
    
    picam2.stop()
    
    # Show sensor info
    sensor_modes = picam2.sensor_modes
    print(f"Sensor mode: {sensor_modes[0]}")

if __name__ == "__main__":
    # Capture with specific tuning
    capture_image("latest_photos", tuning_file="/usr/share/libcamera/ipa/rpi/pisp/imx290.json")
    
    # Or auto-detect (remove parameter)
    # capture_image("auto_photo.jpg")