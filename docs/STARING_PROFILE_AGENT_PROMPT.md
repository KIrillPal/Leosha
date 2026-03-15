## Задача

Реализовать `StaringProfile` — профиль управления роботом, в котором корпус управляется
оператором через WASD (как в `TeleoperationProfile`), а голова автоматически направляется
на лицо знакомого человека из базы. Если знакомых нет в кадре — голова управляется мышью
(fallback на ручной режим).

Для этого нужно создать всю vision-инфраструктуру: отдельную ROS2-ноду для inference,
сервис подписки на результаты, базу лиц, и сам профиль.

---

## Существующая архитектура

### Структура проекта

```
LEO/
├── configs/
│   └── server.yaml                    # Конфигурация сервера (профили, сеть, SLAM)
├── server/
│   └── server/
│       ├── main.py                    # Точка входа: build_runtime(), main()
│       ├── config.py                  # Dataclass-конфиг: ServerConfig, ProfilesSection, load_server_config()
│       ├── models.py                  # ControlMode, ControlCommand, ManualInputState, TelemetryFrame
│       ├── interfaces.py             # InputState, OperationProfile (ABC), AlgorithmContext
│       ├── ros_node.py               # Ros2ServerBridge: единственная ROS2-нода, PUB /scan, TF, SUB /map
│       ├── algorithms/
│       │   ├── __init__.py            # Экспорт всех профилей
│       │   └── profiles.py           # BaseControlProfile, PauseProfile, TeleoperationProfile,
│       │                              #   TeleopSlamProfile, AutonomyProfile1, FollowingProfile (скелет)
│       ├── services/
│       │   ├── controller_service.py  # ControllerService: оркестрация tick_once(), set_mode(), state persistence
│       │   ├── robot_client.py        # ZmqRobotClient / MockRobotClient: ZMQ-связь с роботом
│       │   ├── slam_service.py        # SlamService: SUB /map, TF listener, get_pose(), get_map_png()
│       │   └── ros_graph_service.py   # Реестр нод для UI
│       └── web/
│           ├── app_factory.py         # Flask API: /api/mode, /api/status, /api/keyboard, ...
│           └── templates/             # HTML-шаблоны
```

### Ключевые классы и контракты

#### `ControlMode` (server/server/models.py)

```python
class ControlMode(str, Enum):
    PAUSE = "pause"
    TELEOPERATION = "teleoperation"
    TELEOP_SLAM = "teleop_slam"
    AUTONOMY_PROFILE_1 = "autonomy_profile_1"
```

Нужно добавить `STARING = "staring"`.

#### `InputState` (server/server/interfaces.py)

Единый снимок данных, который получает каждый профиль в `tick()`:

```python
@dataclass
class InputState:
    manual: ManualInputState          # WASD, head_dx/dy, tracking_enabled
    telemetry: TelemetryFrame         # odom, speed, steering, imu, camera JPEG
    robot_config: dict | None         # геометрия робота (head min/max deg)
    lidar_scan: dict | None           # ranges, angle_min/max, ...
    camera_frame: bytes               # JPEG кадр с камеры робота
    slam_pose: tuple | None           # (x, y, theta) от slam_toolbox
    slam_status: str                  # "unknown", "mapping", ...
    pending_actions: list[dict]       # одноразовые действия от UI
    dt: float                         # время с прошлого tick
    timestamp: float
```

#### `OperationProfile` (server/server/interfaces.py)

Абстрактный контракт профиля:

```python
class OperationProfile(ABC):
    @property
    def mode(self) -> ControlMode: ...           # enum режима
    @property
    def title(self) -> str: ...                  # отображаемое название
    @property
    def requires_slam(self) -> bool: return False

    def on_activate(self, context): ...          # вызывается при переключении НА этот профиль
    def on_deactivate(self, context): ...        # вызывается при переключении С этого профиля
    def on_action(self, action: dict): ...       # одноразовое действие (add_face, и т.п.)
    def reset_runtime_state(self, manual): ...   # сброс head state
    def post_tick(self, input_state): ...        # очистка после tick (обнуление mouse delta)

    @abstractmethod
    def tick(self, context, input_state) -> ControlCommand: ...

    def get_ui_state(self) -> dict: ...          # данные для UI (x, y, + профиль-специфичное)
    def save_state(self) -> dict: ...            # сериализация для server_state.yaml
    def restore_state(self, state: dict): ...    # восстановление
```

#### `TeleoperationProfile` (server/server/algorithms/profiles.py)

Ключевой референс. StaringProfile наследует его логику WASD, но заменяет вычисление головы.

- `__init__`: принимает `forward_throttle`, `backward_throttle`, `forward_fast_throttle`, `title`, `control_mode`
- `tick()`: WASD → speed/steering, head_dx/dy → head_pan/tilt с clamp [-1, 1]
- `post_tick()`: обнуляет `manual.head_dx`, `manual.head_dy`
- `save_state/restore_state`: сохраняет `head_pan`, `head_tilt`

#### `ControllerService` (server/server/services/controller_service.py)

Оркестратор. В `tick_once()`:
1. Собирает `InputState` из всех источников
2. Вызывает `profile.on_action()` для каждого `pending_action`
3. Вызывает `profile.tick(context, input_state)` → `ControlCommand`
4. Отправляет `ControlCommand` в `robot_client`

Профили создаются в `_build_profiles()` из `profiles_config` (dict из YAML).
Каждый профиль привязан к `ControlMode`.

Состояние сохраняется в `server_state.yaml`:
```yaml
active_mode: "teleoperation"
profile_states:
  pause: {}
  teleoperation: {head_pan: 0.1, head_tilt: -0.3}
  ...
```

#### `Ros2ServerBridge` (server/server/ros_node.py)

Единственная ROS2-нода в проекте (`Node("server_bridge")`).
- PUB: `/scan` (LaserScan), TF odom→base_link — только когда `requires_slam=True`
- SUB: через `SlamService.try_subscribe_ros(node)` — подписывается на `/map`, TF listener

#### `SlamService` (server/server/services/slam_service.py)

Паттерн для VisionService:
- `try_subscribe_ros(node)`: создаёт подписки через переданный ROS2-node
- Хранит данные под `threading.Lock`
- Предоставляет getter-методы: `get_pose()`, `get_map_png()`, `get_stats()`

#### `configs/server.yaml`

```yaml
app:
  host: "0.0.0.0"
  port: 8080
  command_hz: 100.0

robot:
  ip: "192.168.31.232"
  backend: "zmq"

control:
  head_sensitivity: -0.002

profiles:
  pause: {}
  teleoperation:
    forward_throttle: 0.3
    backward_throttle: -0.15
    forward_fast_throttle: 0.5
  teleop_slam:
    forward_throttle: 0.3
    backward_throttle: -0.15
    forward_fast_throttle: 0.5
  autonomy_profile_1: {}
  following:
    friend_embeddings_db: "data/friends.db"
    approach_distance_m: 1.0
    max_head_tilt_deg: 60.0
    average_human_height_m: 1.7
    yolo_model: "yolov8n.pt"

network:
  grafana_url: ""
  bind_address: "0.0.0.0"
  telemetry_port: 5550
  command_port: 5552
  report_port: 5553
  recv_timeout_ms: 100
  send_high_water_mark: 2
  recv_high_water_mark: 1

slam:
  odom_confidence_threshold: 0.5
```

#### `app_factory.py` — Flask API

Ключевые endpoints:
- `POST /api/mode` → `controller.set_mode(mode_raw)` — переключение режима
- `POST /api/keyboard` → `controller.apply_keyboard(key, state)` — WASD-ввод
- `POST /api/position` → `controller.apply_mouse_delta(dx, dy)` — мышь
- `GET /api/status` → `controller.get_ui_status()` — полное состояние для UI

Нет `POST /api/action` — нужно добавить для передачи одноразовых действий
(add_face, remove_face и т.п.) в профиль через `controller.add_action(action)`.

#### Принципы кода

- **Fail-fast:** нет `try/except Exception`, нет `.get(key, default)` для обязательных полей.
  Если данные обязательны — прямой доступ `data["key"]`, при отсутствии — crash.
- **Runtime-fallbacks допустимы:** для данных, которые могут временно отсутствовать
  (robot_config до первого подключения, TF frame до запуска slam_toolbox),
  допустимы проверки `if X is not None` с fallback-значением.

---

## Что нужно создать

### 1. `vision_node` — отдельный ROS2-процесс

**Файл:** `LEO/server/server/vision_node.py` (или `LEO/vision/vision_node.py` — отдельный пакет).

**Функции:**
- SUB `/camera/image_raw` (`sensor_msgs/CompressedImage` — JPEG кадры)
- Двухконтурный pipeline:
  - Быстрый (каждый кадр): YOLO11n-pose NCNN + ByteTrack → bbox, keypoints, track_id
  - Медленный (раз в `embedding_interval` кадров): MobileFaceNet ArcFace ONNX → 128-dim embedding
- PUB `/vision/persons` (`std_msgs/String` — JSON)

**Формат `/vision/persons`:**

```json
{
  "timestamp_ns": 1710000000000,
  "frame_id": 42,
  "persons": [
    {
      "track_id": 3,
      "bbox": [120, 80, 340, 450],
      "keypoints": [[185, 95, 0.92], ...],
      "head_yaw_deg": 12.5,
      "face_visible": true,
      "face_embedding": [0.021, -0.134, ...],
      "confidence": 0.87
    }
  ],
  "inference_ms": 35.2
}
```

`face_embedding` = `[]` когда `face_visible=false` или не embedding-кадр (и нет кеша).

**Конфигурация** (параметры ROS2 node или YAML):
- `yolo_model`: путь к NCNN-модели
- `yolo_imgsz`: 640 или 320
- `tracker`: "bytetrack.yaml"
- `face_model`: путь к ONNX-модели MobileFaceNet
- `embedding_interval`: 25 (при 25 FPS = раз в секунду)
- `face_min_confidence`: 0.5

### 2. server_bridge: публикация камеры

В `Ros2ServerBridge` добавить:
- PUB `/camera/image_raw` (`sensor_msgs/CompressedImage`)
- Timer на ~30 Hz: берёт `robot_client.get_latest_frame()` (JPEG bytes) и публикует

Публикация камеры должна работать **всегда** (не зависеть от `requires_slam`),
потому что vision_node нужен для любого vision-профиля.

### 3. `VisionService`

**Файл:** `LEO/server/server/services/vision_service.py`

По паттерну `SlamService`:
- `try_subscribe_ros(node)`: подписывается на `/vision/persons`
- `get_persons() -> list[dict]`: последние детекции (thread-safe)
- `get_latest_frame_id() -> int`: для отслеживания свежести данных

### 4. `FriendDB`

**Файл:** `LEO/server/server/services/friend_db.py`

Хранилище face embeddings:
- `load(path)`: загрузка из файла (pickle или JSON)
- `save(path)`: сохранение
- `add(name, embedding)`: добавить/обновить лицо
- `remove(name)`: удалить
- `list_names() -> list[str]`: список имён
- `match(embedding, threshold) -> str | None`: cosine similarity поиск

Формат хранения: `{name: [float, ...]}` — dict с 128-dim векторами.
Файл: `data/friends.db` (JSON или pickle).

### 5. `StaringProfile`

**Файл:** `LEO/server/server/algorithms/profiles.py` (добавить в существующий файл)

Наследует `BaseControlProfile`.

**__init__:**
- `forward_throttle`, `backward_throttle`, `forward_fast_throttle` — как TeleoperationProfile
- `friend_embeddings_db`: путь к файлу FriendDB
- `face_match_threshold`: порог cosine similarity
- `head_tracking_gain`: скорость поворота головы к лицу
- `head_tracking_deadzone`: зона нечувствительности
- `vision_service: VisionService` — внедрение зависимости

**tick(context, input_state) → ControlCommand:**

1. `speed`, `steering` — идентично `TeleoperationProfile` (WASD + imu_yaw_rate damping)
2. `persons = self._vision.get_persons()`
3. Для каждого person с непустым `face_embedding`:
   - `match = friend_db.match(embedding, threshold)` → `name | None`
4. Если есть хотя бы один match:
   - `state = "staring"`
   - Выбрать ближайшего (по площади bbox — больше = ближе)
   - `head_pan, head_tilt = compute_head_from_bbox(bbox, frame_w, frame_h, current_pan, current_tilt, gain, deadzone)`
5. Если нет matchов:
   - `state = "manual"`
   - `head_pan`, `head_tilt` — из mouse input (как TeleoperationProfile)
6. Return `ControlCommand(speed, steering, head_pan, head_tilt, mode=self.mode)`

**on_action(action):**
- `{"type": "add_face"}` → взять persons из VisionService, найти ближайшее видимое лицо, сохранить embedding в FriendDB с автоименем `face_N`
- `{"type": "add_face", "name": "Kir"}` → то же, с явным именем
- `{"type": "remove_face", "name": "Kir"}` → удалить из FriendDB
- `{"type": "list_faces"}` → ничего не делает, результат в `get_ui_state()`

**get_ui_state():**
```python
{
    "x": head_pan,
    "y": head_tilt,
    "staring_state": "staring" | "manual",
    "staring_target": "Kir" | None,
    "known_faces": ["Kir", "Masha", ...],
    "persons_count": 3,
    "staring_debug": {...}
}
```

**save_state / restore_state:** `head_pan`, `head_tilt`

### 6. Интеграция

#### `ControlMode` (models.py)
Добавить `STARING = "staring"`.

#### `ProfilesSection` (config.py)
Добавить поле `staring: dict[str, Any]`.

#### `configs/server.yaml`
Добавить секцию:
```yaml
profiles:
  staring:
    forward_throttle: 0.3
    backward_throttle: -0.15
    forward_fast_throttle: 0.5
    friend_embeddings_db: "data/friends.db"
    face_match_threshold: 0.5
    head_tracking_gain: 3.0
    head_tracking_deadzone: 0.03
```

#### `ControllerService._build_profiles()`
Добавить создание `StaringProfile` и привязку к `ControlMode.STARING`.
`VisionService` передаётся в профиль через DI.

#### `main.py: build_runtime()`
- Создать `VisionService`
- Передать `VisionService` в `ControllerService` (для передачи в StaringProfile)
- В `Ros2ServerBridge`: добавить публикацию камеры и подписку VisionService

#### `app_factory.py`
Добавить endpoint:
```python
@app.post("/api/action")
def send_action():
    data = _require_json()
    controller.add_action(data)
    return jsonify({"success": True})
```

#### `algorithms/__init__.py`
Добавить `StaringProfile` в `__all__` и импорт.

---

## Важные ограничения

1. **Не менять** существующие профили (PauseProfile, TeleoperationProfile, TeleopSlamProfile,
   AutonomyProfile1) — только добавлять новый.

2. **Fail-fast стиль:** все обязательные поля конфига — прямой доступ `config["key"]`.
   Runtime-данные (VisionService может ещё не иметь данных) — допустимы fallback.

3. **vision_node — отдельный процесс.** Не импортировать YOLO/ultralytics в процесс server.
   Inference тяжёлый и не должен блокировать main loop сервера.

4. **FriendDB** сохраняется на диск при каждом add/remove (атомарная запись через tmp + rename).

5. **Камера публикуется всегда** (не только в STARING mode), т.к. vision_node работает
   независимо и может потребоваться для других профилей в будущем.

6. **Тесты:** в первой итерации — как минимум unit-тесты StaringProfile с мок-VisionService.

---

## Полный граф ROS2 после реализации

```
Robot (ZMQ) ──► RobotClient ──► server_bridge (Node)
                                    │
                  PUB /camera/image_raw ──────────► vision_node
                  PUB /scan ────────────────────► slam_toolbox
                                    │                    │
                  SUB /vision/persons ◄──── PUB /vision/persons
                  SUB /map ◄──────────────── PUB /map
                                    │
                             VisionService   SlamService
                                    │            │
                             ControllerService
                                    │
                     ┌──────────────┼──────────────┐
                     │              │              │
              StaringProfile  TeleopProfile  SlamProfile ...
```
