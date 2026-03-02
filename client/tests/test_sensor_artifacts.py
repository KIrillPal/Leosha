from __future__ import annotations

import copy
import math
import struct
import threading
import time
import types
import zlib
from pathlib import Path

from client.sensor_hub import SensorHub
from client.sensors.camera_sensor import CameraSensorThread
from client.sensors.lidar_sensor import TMiniProPlusLidarThread
from client.sensors.status_registry import SensorStatusRegistry
from client.stats import StatsCollector


ARTIFACTS_DIR = Path(__file__).resolve().parent / "artifacts"


def _write_png_rgb(path: Path, width: int, height: int, pixels: bytes) -> None:
    """Минимальный PNG writer (RGB8) без внешних зависимостей."""
    assert len(pixels) == width * height * 3
    path.parent.mkdir(parents=True, exist_ok=True)
    raw = bytearray()
    stride = width * 3
    for y in range(height):
        raw.append(0)  # filter type 0
        start = y * stride
        raw.extend(pixels[start:start + stride])
    compressed = zlib.compress(bytes(raw), level=6)

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"IDAT", compressed) + chunk(b"IEND", b"")
    path.write_bytes(png)


def _render_lidar_map_png(path: Path, ranges: list[float], angle_min: float, angle_inc: float) -> None:
    width, height = 256, 256
    cx, cy = width // 2, height // 2
    scale = 30.0  # px per meter
    img = bytearray([245] * (width * height * 3))

    def set_px(x: int, y: int, r: int, g: int, b: int) -> None:
        if x < 0 or y < 0 or x >= width or y >= height:
            return
        i = (y * width + x) * 3
        img[i] = r
        img[i + 1] = g
        img[i + 2] = b

    # Центр робота
    for dy in range(-2, 3):
        for dx in range(-2, 3):
            set_px(cx + dx, cy + dy, 30, 30, 200)

    # Лучи лидара
    for i, dist in enumerate(ranges):
        if not math.isfinite(dist) or dist <= 0.0:
            continue
        angle = angle_min + i * angle_inc
        x = int(cx + math.cos(angle) * dist * scale)
        y = int(cy - math.sin(angle) * dist * scale)
        set_px(x, y, 10, 10, 10)

    _write_png_rgb(path, width, height, bytes(img))


def test_camera_sensor_thread_writes_frame_artifact(monkeypatch, client_config):
    cfg = copy.deepcopy(client_config)
    cfg.sensors.camera.enabled = True
    cfg.sensors.camera.fail_policy.max_consecutive_failures = 3
    cfg.sensors.camera.fail_policy.auto_disable_on_fail = True
    cfg.sensors.camera.fps = 20.0
    cfg.sensors.camera.resolution = (64, 48)

    # synthetic RGB frame
    width, height = cfg.sensors.camera.resolution
    frame = bytearray(width * height * 3)
    for y in range(height):
        for x in range(width):
            i = (y * width + x) * 3
            frame[i] = int(255 * x / max(1, width - 1))
            frame[i + 1] = int(255 * y / max(1, height - 1))
            frame[i + 2] = 120
    frame_bytes = bytes(frame)

    class _FakeEncoded:
        def __init__(self, payload: bytes) -> None:
            self._payload = payload

        def tobytes(self) -> bytes:
            return self._payload

    class _FakePicamera2:
        def __init__(self, tuning=None):
            self.tuning = tuning

        def create_video_configuration(self, **kwargs):
            return kwargs

        def configure(self, _):
            return None

        def set_controls(self, _):
            return None

        def start(self):
            return None

        def stop(self):
            return None

        def capture_array(self):
            # Возвращаем псевдо-RGB матрицу как bytes-like proxy
            class _Arr:
                def __init__(self, raw: bytes):
                    self.raw = raw

            return _Arr(frame_bytes)

    fake_picamera2_mod = types.SimpleNamespace(Picamera2=_FakePicamera2)

    def _cvt_color(arr, _code):
        return arr.raw

    def _imencode(_ext, img, _params):
        return True, _FakeEncoded(b"\xff\xd8FAKEJPEG\xff\xd9" + img[:64])

    fake_cv2_mod = types.SimpleNamespace(
        COLOR_RGB2BGR=1,
        IMWRITE_JPEG_QUALITY=1,
        cvtColor=_cvt_color,
        imencode=_imencode,
    )

    monkeypatch.setitem(__import__("sys").modules, "picamera2", fake_picamera2_mod)
    monkeypatch.setitem(__import__("sys").modules, "cv2", fake_cv2_mod)

    hub = SensorHub()
    stats = StatsCollector()
    statuses = SensorStatusRegistry()
    statuses.register("camera", True)
    stats.register("camera", cfg.sensors.camera.fps)
    stop_event = threading.Event()

    thread = CameraSensorThread(cfg.sensors.camera, hub, stats, statuses, stop_event)
    thread.start()
    time.sleep(0.12)
    stop_event.set()
    thread.join(timeout=1.0)

    snap = hub.snapshot()
    assert snap.frame_jpeg.startswith(b"\xff\xd8")
    status = statuses.snapshot()["camera"]
    assert status.healthy is True

    artifact_path = ARTIFACTS_DIR / "camera_frame.png"
    _write_png_rgb(artifact_path, width, height, frame_bytes)
    assert artifact_path.exists()
    assert artifact_path.stat().st_size > 100


def test_lidar_sensor_thread_writes_map_artifact(monkeypatch, client_config):
    cfg = copy.deepcopy(client_config)
    cfg.sensors.lidar.enabled = True
    cfg.sensors.lidar.scan_hz = 8.0
    cfg.sensors.lidar.fail_policy.max_consecutive_failures = 3
    cfg.sensors.lidar.fail_policy.auto_disable_on_fail = True
    # Blind zone covering fake points at ~±11.5° (rad -0.2..0.2) with r=0.2 so they become inf
    cfg.robot_geometry.lidar.blind_zone.angle_start_deg = -15.0
    cfg.robot_geometry.lidar.blind_zone.angle_end_deg = 15.0
    cfg.robot_geometry.lidar.blind_zone.max_distance_m = 0.5

    class _Point:
        def __init__(self, angle, dist, intensity):
            self.angle = angle
            self.range = dist
            self.intensity = intensity

    class _LaserScan:
        def __init__(self):
            self.points = []

    class _FakeLidar:
        def setlidaropt(self, *args):
            return None

        def initialize(self):
            return True

        def turnOn(self):
            return True

        def doProcessSimple(self, scan_obj):
            pts = []
            for i in range(120):
                a = -math.pi / 2 + i * (math.pi / 120)
                # Сделаем дугу + точки в слепой зоне
                r = 1.5 + 0.15 * math.sin(i * 0.15)
                if -0.2 <= a <= 0.2:
                    r = 0.2  # должно быть отфильтровано blind-zone
                pts.append(_Point(a, r, 120.0))
            scan_obj.points = pts
            return True

        def turnOff(self):
            return None

        def disconnecting(self):
            return None

    def _lidar_port_list():
        return {}

    def _os_init():
        pass

    fake_yd = types.SimpleNamespace(
        CYdLidar=_FakeLidar,
        LaserScan=_LaserScan,
        os_init=_os_init,
        lidarPortList=_lidar_port_list,
        LidarPropSerialPort=1,
        LidarPropSerialBaudrate=2,
        LidarPropLidarType=3,
        LidarPropDeviceType=4,
        LidarPropScanFrequency=5,
        LidarPropSampleRate=6,
        LidarPropSingleChannel=7,
        LidarPropMaxAngle=12,
        LidarPropMinAngle=13,
        LidarPropMaxRange=8,
        LidarPropMinRange=9,
        LidarPropIntenstiy=14,
        TYPE_TRIANGLE=10,
        YDLIDAR_TYPE_SERIAL=11,
    )
    monkeypatch.setitem(__import__("sys").modules, "ydlidar", fake_yd)

    hub = SensorHub()
    stats = StatsCollector()
    statuses = SensorStatusRegistry()
    statuses.register("lidar", True)
    stats.register("lidar", cfg.sensors.lidar.scan_hz)
    stop_event = threading.Event()

    thread = TMiniProPlusLidarThread(cfg.sensors.lidar, cfg.robot_geometry, hub, stats, statuses, stop_event)
    thread.start()
    time.sleep(0.2)
    stop_event.set()
    thread.join(timeout=1.0)

    snap = hub.snapshot()
    assert snap.scan is not None
    status = statuses.snapshot()["lidar"]
    assert status.healthy is True
    # Проверяем, что часть лучей в blind-zone отфильтрована.
    assert any((not math.isfinite(v)) for v in snap.scan.ranges)

    artifact_path = ARTIFACTS_DIR / "lidar_map.png"
    _render_lidar_map_png(artifact_path, snap.scan.ranges, snap.scan.angle_min, snap.scan.angle_increment)
    assert artifact_path.exists()
    assert artifact_path.stat().st_size > 100

