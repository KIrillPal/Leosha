from __future__ import annotations

from client.models import ImuReading, LaserScan, Pose2D, Twist2D, WheelOdometry
from client.sensor_hub import SensorHub


def test_sensor_hub_snapshot_drains_imu_and_keeps_latest_values():
    hub = SensorHub()
    hub.put_frame(b"jpeg")
    hub.put_scan(
        LaserScan(
            timestamp_ns=1,
            angle_min=-1.0,
            angle_max=1.0,
            angle_increment=0.1,
            range_min=0.1,
            range_max=12.0,
            ranges=[1.0],
            intensities=[100.0],
        )
    )
    hub.append_imu(ImuReading(1, 0.0, 0.0, 9.81, 0.0, 0.0, 0.1))
    hub.put_odometry(
        WheelOdometry(
            timestamp_ns=1,
            pose=Pose2D(1.0, 2.0, 0.3),
            velocity=Twist2D(0.4, 0.1),
            steering_angle=0.2,
        )
    )
    hub.put_ultrasonic(1.2)

    snap1 = hub.snapshot()
    assert snap1.frame_jpeg == b"jpeg"
    assert snap1.scan is not None
    assert len(snap1.imu_readings) == 1
    assert snap1.odometry is not None
    assert snap1.ultrasonic_range_m == 1.2

    snap2 = hub.snapshot()
    assert len(snap2.imu_readings) == 0
    assert snap2.frame_jpeg == b"jpeg"

