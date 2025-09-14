#!/usr/bin/env python3
from picamera2 import Picamera2
import time
import os
import io
from datetime import datetime

# Configuration
output_dir = "video_frames"
frame_rate = 60  # Target FPS
duration = 10    # Recording duration in seconds
resolution = (1280, 720)

# Create output directory
os.makedirs(output_dir, exist_ok=True)

# Initialize camera
picam2 = Picamera2(tuning="/usr/share/libcamera/ipa/rpi/pisp/imx290.json")
config = picam2.create_video_configuration(
    main={
        "size": resolution,
    },
    controls={"FrameRate": frame_rate},
    raw={
        "size": picam2.sensor_resolution,  # Full sensor readout
    }
)
picam2.configure(config)

picam2.set_controls({
    "ExposureTime": 20000,
    "AnalogueGain": 100.0,
    "AwbEnable": True,
    "AeEnable": False
})

# Start recording
picam2.start()
start_time = time.time()
images = []
frame_count = 0
try:
    while time.time() - start_time < duration:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"{output_dir}/frame_{timestamp}_{frame_count:06d}.jpg"
        data = io.BytesIO()
        img = picam2.capture_array()
        print(img.shape)
        frame_count += 1
        # Maintain frame rate
        time.sleep(max(0, (1/frame_rate) - (time.time() - start_time - (frame_count-1)/frame_rate)))
finally:
    picam2.stop()
    print(f"Saved {frame_count} frames to {output_dir}")