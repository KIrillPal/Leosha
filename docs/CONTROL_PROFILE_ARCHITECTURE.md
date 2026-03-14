# Архитектура профилей управления

## Цели

- `ControllerService` не содержит поведенческой логики отдельных режимов.
- Любой профиль получает единый `InputState` (даже с полями, которые ему не нужны).
- Профиль хранит и сериализует своё внутреннее состояние.
- Настройки каждого hardcoded-профиля задаются в YAML-конфиге.

## Ключевые сущности

### `InputState` (`server/server/interfaces.py`)

Единый снимок данных на тик:

- `manual: ManualInputState`
- `telemetry: TelemetryFrame`
- `robot_config: dict | None`
- `lidar_scan: dict | None`
- `camera_frame: bytes`
- `slam_pose: tuple[float, float, float] | None`
- `slam_status: str`
- `pending_actions: list[dict]`
- `dt: float`, `timestamp: float`

### `OperationProfile` + `BaseControlProfile`

Профиль реализует:

- `tick(context, input_state) -> ControlCommand`
- lifecycle hooks: `on_activate`, `on_deactivate`
- опциональные `on_action`, `post_tick`, `reset_runtime_state`
- UI/state hooks: `get_ui_state`, `save_state`, `restore_state`
- capability flag `requires_slam`

Для совместимости оставлен legacy-путь: `profile.algorithm.compute_command(...)`.

### `ControllerService`

Ответственность:

1. Собрать `InputState`.
2. Передать `pending_actions` в активный профиль.
3. Вызвать `profile.tick(...)`.
4. Отправить `ControlCommand` в `RobotClient`.
5. Сохранить/восстановить `active_mode` и `profile_states` в `server_state.yaml`.

## Конфиг профилей

`configs/server.yaml`:

```yaml
profiles:
  pause: {}
  teleoperation: {}
  teleop_slam: {}
  autonomy_profile_1: {}
  following:
    friend_embeddings_db: "data/friends.db"
    approach_distance_m: 1.0
    max_head_tilt_deg: 60.0
    average_human_height_m: 1.7
    yolo_model: "yolov8n.pt"
```

Профили остаются hardcoded в коде, но их параметры настраиваются из файла.

## FollowingProfile (скелет)

`FollowingProfile` добавлен как архитектурный шаблон и пока не реализует финальное поведение.

Внутри профиля зафиксированы TODO по ожидаемому пайплайну:

- детекция человека и позы (YOLO);
- эмбеддинг лица и сравнение с базой друзей;
- оценка положения цели в 2D (камера + лидар + средний рост);
- управление движением к цели;
- условие остановки: расстояние ≤ 1м или head tilt ≥ 60°.

Профиль хранит базовое runtime-состояние:

- `state`
- `target_id`
- `last_target_pose`
- `debug`-метрики

и умеет сериализовать его через `save_state/restore_state`.

## SLAM активация

`Ros2ServerBridge` теперь может работать не только через проверку конкретного mode.
Если передан callback `is_slam_active`, мост публикует `/scan` и odom TF на основании capability активного профиля (`requires_slam`).

Fallback на старую логику по `ControlMode.TELEOP_SLAM` сохранён для совместимости.
