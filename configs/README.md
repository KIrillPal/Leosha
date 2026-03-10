# Конфигурации Leosha

Все конфиги клиента (робот) и сервера хранятся в этой папке: `Leosha/code/configs/`. Шаблоны — в `templates/`.

## Расположение файлов

| Файл | Назначение |
|------|------------|
| `client.yaml` | Конфиг клиента по умолчанию (разработка, mock) |
| `client.hardware.yaml` | Конфиг клиента для реального железа (камера, лидар, PCA9685) |
| `server.yaml` | Конфиг сервера |
| `templates/client.template.yaml` | Шаблон конфига клиента |
| `templates/server.template.yaml` | Шаблон конфига сервера |

Запуск с указанием конфига:

```bash
# Клиент (из корня репозитория или code/client)
python -m client --config /path/to/Leosha/code/configs/client.yaml

# Сервер
python -m server --config /path/to/Leosha/code/configs/server.yaml
```

---

## Клиент (`client.yaml`, `client.template.yaml`)

### app

| Аргумент | Тип | Описание |
|----------|-----|----------|
| `telemetry_hz` | float | Частота отправки телеметрии на сервер (Гц) |
| `command_hz` | float | Частота опроса команд от сервера (Гц) |

### watchdog

| Аргумент | Тип | Описание |
|----------|-----|----------|
| `warn_ms` | float | Порог задержки пакета для предупреждения (мс) |
| `timeout_ms` | float | Таймаут: после него робот замедляется (мс) |
| `critical_ms` | float | Критический таймаут: аварийная остановка (мс) |

### diagnostics

| Аргумент | Тип | Описание |
|----------|-----|----------|
| `battery_voltage` | float | Напряжение АКБ для отображения (В) |
| `cpu_temp_c` | float | Температура CPU для отображения (°C) |
| `wifi_rssi_dbm` | float | Уровень сигнала Wi‑Fi для отображения (дБм) |

### network

| Аргумент | Тип | Описание |
|----------|-----|----------|
| `server_host` | str | IP или hostname сервера |
| `telemetry_port` | int | Порт публикации телеметрии (ZMQ PUB) |
| `command_port` | int | Порт приёма команд (ZMQ SUB) |
| `report_port` | int | Порт отчётов о пропущенных пакетах (ZMQ PUB) |
| `recv_timeout_ms` | int | Таймаут приёма (мс); 0 — не блокировать |
| `send_high_water_mark` | int | High water mark для исходящих ZMQ-сокетов |
| `recv_high_water_mark` | int | High water mark для входящих ZMQ-сокетов |

### actuators

| Аргумент | Тип | Описание |
|----------|-----|----------|
| `backend` | str | `"mock"` или `"pca9685"` |
| `pca.channels` | int | Количество каналов PCA9685 |
| `pca.frequency` | int | Частота ШИМ PCA9685 (Гц) |
| `motor.channel` | int | Номер канала мотора |
| `motor.zero_throttle` | float | Нулевой throttle для остановки |
| `motor.speed_to_throttle_ratio` | float | Коэффициент скорость → throttle |
| `motor.pwm_min_pulse` | int | Минимальный импульс ШИМ (мкс) |
| `motor.pwm_max_pulse` | int | Максимальный импульс ШИМ (мкс) |
| `wheel.channel` | int | Номер канала рулевого сервопривода |
| `wheel.min_throttle` | float | Минимальный throttle руля |
| `wheel.zero_throttle` | float | Нулевой throttle (прямо) |
| `wheel.max_throttle` | float | Максимальный throttle руля |
| `wheel.invert` | bool | Инвертировать направление руля |
| `wheel.pwm_min_pulse` | int | Минимальный импульс ШИМ (мкс) |
| `wheel.pwm_max_pulse` | int | Максимальный импульс ШИМ (мкс) |
| `neck.channel` | int | Канал сервопривода шеи (yaw) |
| `neck.angle_min` | float | Минимальный угол шеи (°) |
| `neck.angle_max` | float | Максимальный угол шеи (°) |
| `neck.angle_zero` | float | Угол «ноль» шеи (°) |
| `neck.actuation_range` | int | Диапазон серво (°) |
| `neck.pwm_min_pulse` | int | Минимальный импульс ШИМ (мкс) |
| `neck.pwm_max_pulse` | int | Максимальный импульс ШИМ (мкс) |
| `face.channel` | int | Канал сервопривода головы (pitch) |
| `face.angle_min` | float | Минимальный угол головы (°) |
| `face.angle_max` | float | Максимальный угол головы (°) |
| `face.angle_zero` | float | Угол «ноль» головы (°) |
| `face.actuation_range` | int | Диапазон серво (°) |
| `face.pwm_min_pulse` | int | Минимальный импульс ШИМ (мкс) |
| `face.pwm_max_pulse` | int | Максимальный импульс ШИМ (мкс) |

### robot_geometry

| Аргумент | Тип | Описание |
|----------|-----|----------|
| `body_size.length` | float | Длина корпуса (м) |
| `body_size.width` | float | Ширина корпуса (м) |
| `body_size.height` | float | Высота корпуса (м) |
| `pose_on_body.x/y/z` | float | Положение базовой позы на корпусе (м) |
| `wheel.radius` | float | Радиус колеса (м) |
| `wheel.width` | float | Ширина колеса (м) |
| `lidar.position.x/y/z` | float | Положение лидара относительно позы (м) |
| `lidar.blind_zone.angle_start_deg` | float | Начало слепой зоны (град) |
| `lidar.blind_zone.angle_end_deg` | float | Конец слепой зоны (град) |
| `lidar.blind_zone.max_distance_m` | float | Макс. дистанция точек слепой зоны (м) |
| `head.position.x/y/z` | float | Положение головы относительно позы (м) |
| `head.neck_min_deg` | float | Минимальный угол шеи (°) |
| `head.neck_max_deg` | float | Максимальный угол шеи (°) |
| `head.face_min_deg` | float | Минимальный угол головы (°) |
| `head.face_max_deg` | float | Максимальный угол головы (°) |

### sensors (общие для каждого датчика)

| Аргумент | Тип | Описание |
|----------|-----|----------|
| `enabled` | bool | Включён ли датчик |
| `required_for_motion` | bool | Нужен ли для движения (иначе экстренная остановка) |
| `fail_policy.max_consecutive_failures` | int | После скольких подряд сбоев считать отказ |
| `fail_policy.auto_disable_on_fail` | bool | Отключать ли датчик при отказе |
| `fail_policy.retry_interval_sec` | float | Интервал повторных попыток (с) |

### sensors.camera

| Аргумент | Тип | Описание |
|----------|-----|----------|
| `fps` | float | Целевой FPS камеры |
| `resolution` | [int, int] | Разрешение [ширина, высота] |
| `tuning_file` | str | Путь к JSON tuning для libcamera (imx290 и т.д.) |
| `exposure_time` | int | Время экспозиции (мкс) |
| `analogue_gain` | float | Аналоговое усиление |
| `awb_enable` | bool | Автобаланс белого |
| `ae_enable` | bool | Автоэкспозиция |
| `jpeg_quality` | int | Качество JPEG (1–100) |
| `healthcheck_timeout_sec` | float | Таймаут healthcheck (с) |
| `healthcheck_test_timeout_sec` | float | Макс. время ожидания теста (с), включая блокирующий init |
| `healthcheck_min_frames` | int | Минимум кадров для успешного healthcheck |

### sensors.lidar

| Аргумент | Тип | Описание |
|----------|-----|----------|
| `port` | str | Последовательный порт (например `/dev/ydlidar`) |
| `auto_discover_port` | bool | Искать порт через SDK (lidarPortList) |
| `baudrate` | int | Скорость порта |
| `scan_hz` | float | Частота сканирования (Гц) |
| `min_angle_deg` | float | Минимальный угол скана (°) |
| `max_angle_deg` | float | Максимальный угол скана (°) |
| `range_min_m` | float | Минимальная дальность (м) |
| `range_max_m` | float | Максимальная дальность (м) |
| `intensity_enabled` | bool | Включить интенсивность |
| `invert_angle` | bool | Инвертировать углы (T-mini Pro Plus) |
| `healthcheck_timeout_sec` | float | Таймаут healthcheck (с) |
| `healthcheck_test_timeout_sec` | float | Макс. время ожидания теста (с), включая блокирующий init |
| `healthcheck_min_points` | int | Минимум точек для успешного healthcheck |

### sensors.imu / sensors.encoder

| Аргумент | Тип | Описание |
|----------|-----|----------|
| `hz` | float | Частота опроса (Гц) |

### sensors.ultrasonic

| Аргумент | Тип | Описание |
|----------|-----|----------|
| `hz` | float | Частота опроса (Гц) |
| `emergency_stop_distance_m` | float | Порог дистанции для экстренной остановки (м) |

---

## Сервер (`server.yaml`, `server.template.yaml`)

### app

| Аргумент | Тип | Описание |
|----------|-----|----------|
| `host` | str | Хост веб‑интерфейса и API |
| `port` | int | Порт веб‑сервера |
| `command_hz` | float | Частота отправки команд роботу (Гц) |

### robot

| Аргумент | Тип | Описание |
|----------|-----|----------|
| `ip` | str | IP робота (для отображения и кнопки Ping) |

### control

| Аргумент | Тип | Описание |
|----------|-----|----------|
| `head_sensitivity` | float | Чувствительность управления головой (мышь). Руление задаётся в норм. ±1, фактический диапазон — из конфига клиента `actuators.wheel` (min_throttle, max_throttle, zero_throttle). |

### network

| Аргумент | Тип | Описание |
|----------|-----|----------|
| `grafana_url` | str | URL дашборда Grafana (вкладка «Сеть») |
