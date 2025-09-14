#!/usr/bin/env python3
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput
import time

def capture_high_speed(output_file, duration=5, fps=60):
    picam2 = Picamera2()
    
    print("Available sensor modes:")
    for i, mode in enumerate(picam2.sensor_modes):
        print(f"{i}: {mode['size']} @ {mode['fps']}fps (crop: {mode['crop_limits']})")
    
    # Configure for high speed
    config = picam2.create_video_configuration(
        main={"size": (1280, 1080)},  # Lower resolution for higher FPS
        raw={
            "size": picam2.sensor_resolution,  # Full sensor readout
        },
        controls={
            "FrameRate": fps,
            "ExposureTime": int(1e6/fps),  # Max exposure for target FPS
            "AnalogueGain": 8.0,  # Moderate gain
            "AeEnable": False,    # Disable auto-exposure
            "AwbEnable": True    # Disable auto-white balance
        },
        buffer_count=6            # More buffers for better performance
    )
    
    picam2.configure(config)
    encoder = H264Encoder()
    output = FileOutput(output_file)
    
    # Start recording
    picam2.start_recording(encoder, output)
    print(f"Recording at {fps} FPS for {duration} seconds...")
    time.sleep(duration)
    picam2.stop_recording()
    print(f"Saved to {output_file}")

if __name__ == "__main__":
    capture_high_speed("high_speed.mp4", fps=60, duration=10)