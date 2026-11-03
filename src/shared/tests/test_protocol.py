import msgpack
import numpy as np

from leo_protocol.protocol import (
    CAMERA,
    LIDAR,
    Command,
    Hello,
    Scan,
    Telemetry,
    module_failed,
    pack_command,
    pack_hello,
    pack_reply,
    pack_telemetry,
    unpack_command,
    unpack_hello,
    unpack_reply,
    unpack_telemetry,
)


def test_hello_and_command_round_trip():
    hello = Hello(client_time_ns=5, sim=True, world="room", local_ip="192.168.31.6")
    assert unpack_hello(pack_hello(hello)) == hello
    assert unpack_hello(pack_hello(Hello(5, False, ""))).local_ip == ""
    command = Command(1, 2, 0.3, -0.1, 10.0, -2.0)
    assert unpack_command(pack_command(command)) == command


def test_telemetry_keeps_nan_ranges_and_jpeg():
    ranges = np.array([1.0, np.nan, 2.0], dtype=np.float32)
    scan = Scan(9, 0.1, -3.14, 0.01, 0.02, 6.0, ranges)
    packet = Telemetry(
        seq=3,
        client_time_ns=11,
        scan=scan,
        jpeg=b"jpeg",
        frame_time_ns=8,
        frame_width=4,
        frame_height=2,
        head_pan_deg=1.0,
        head_tilt_deg=2.0,
        steering_rad=0.1,
        speed_mps=0.2,
        gyro_z=None,
        fps_loop=1,
        fps_camera=2,
        fps_lidar=3,
        fps_telemetry=4,
        ground_truth=(1.0, 2.0, 0.5),
        modules={LIDAR: True, "neck servo": False},
    )
    header, jpeg = pack_telemetry(packet)
    back = unpack_telemetry(header, jpeg)
    assert jpeg == b"jpeg"
    assert back.ground_truth == (1.0, 2.0, 0.5)
    assert back.modules == {LIDAR: True, "neck servo": False}
    assert np.isnan(back.scan.ranges[1])
    assert abs(float(back.scan.ranges[0]) - 1.0) < 1e-6
    data = msgpack.unpackb(header, raw=False)
    del data["modules"]
    bare = msgpack.packb(data, use_bin_type=True)
    assert unpack_telemetry(bare, jpeg).modules == {}


def test_a_missing_module_flag_does_not_count_as_failed():
    assert module_failed({}, LIDAR) is False
    assert module_failed(None, LIDAR) is False
    assert module_failed({LIDAR: True, CAMERA: False}, CAMERA) is True
    assert module_failed({LIDAR: True}, LIDAR) is False


def test_reply_carries_ports():
    payload = pack_reply(1, 2, {"command_hz": 1}, None, {"telemetry_port": 5550, "command_port": 5552})
    data = unpack_reply(payload)
    assert data["ports"]["command_port"] == 5552
    assert data["client"]["command_hz"] == 1
