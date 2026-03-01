# Client: потоковый клиент робота

Пакет `client` реализует клиентскую часть архитектуры робота:

- параллельный сбор сенсоров (`threading`);
- атомарный `SensorHub` со `snapshot()`;
- цикл команд 100 Гц;
- цикл телеметрии 30 Гц;
- локальное состояние робота (`LocalState`);
- `Watchdog` с деградацией и аварийной остановкой;
- расширяемые исполнители режимов (`Pause`, `Teleoperation`, `Autonomy`).
- полноценный YAML-конфиг геометрии робота и сенсоров.

## Архитектура

Основные слои:

- `client.models` — доменные структуры данных и сетевые пакеты;
- `client.sensor_hub` — потокобезопасное хранилище данных сенсоров;
- `client.sensors` — реальные потоки камеры IMX290 и лидара YDLidar;
- `client.stats` — статистика таймингов сенсоров для Grafana на сервере;
- `client.local_state` — объединение локальных и серверных обновлений;
- `client.executors.*` — исполнители режимов;
- `client.network.*` — mock-сервер и in-memory bridge;
- `client.network.*` — mock-сервер и реальный `ZeroMQ` bridge;
- `client.runtime` — оркестрация потоков и циклов 30/100 Гц.

## Быстрый запуск

Из `code/client`:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e ".[test]"
python -m client.main --transport mock --duration-sec 3
```

Подключение к реальному серверу через ZeroMQ:

```bash
python -m client.main --transport zmq
```

Параметры `server_host` и портов настраиваются в `config/client.yaml` (`network.*`).

## Тесты

```bash
pytest -q
```

## Ограничения реализации

- документация на русском;
- файлы ограничены размером до 500 строк;
- применены принципы SOLID и ООП, интерфейсы и реализации разделены.

## Конфигурирование робота

`config/client.yaml` включает:

- геометрию корпуса, положение Pose, размеры колеса;
- положение лидара (XYZ), сектор слепой зоны и радиус отсечения;
- положение головы (XYZ), диапазоны шеи и головы;
- параметры камеры IMX290 (tuning, exposure, gain, JPEG quality);
- параметры YDLidar T-mini Pro Plus (порт, baudrate, scan_hz, диапазоны);
- fail-policy для каждого сенсора (автоотключение, ретраи).

Каждый сенсор можно отключать через `enabled: false`. Если сенсор не отвечает,
он переводится в неактивный/неисправный статус без падения клиента. Статусы
и тайминги сенсоров отправляются серверу в каждом пакете телеметрии.

## Управление приводами (перенесено из code/head)

В `config/client.yaml` и `config/client.hardware.yaml` добавлена секция `actuators`,
где повторены параметры из `code/head/config/*`:

- PWM частота PCA9685 (`pca.frequency: 250`);
- каналы мотора/руля/шеи/головы;
- калибровка мотора (`zero_throttle`, `speed_to_throttle_ratio`);
- калибровка руля (`min/zero/max throttle`, `invert`);
- ограничения и нули углов сервоприводов шеи/головы;
- pulse width для мотора/руля и шеи.

Реальный драйвер: `Pca9685ActuatorDriver` (`client.actuators`).
Выбор backend: `actuators.backend = "pca9685"` или `"mock"`.

## Hardware healthcheck

В `config/client.hardware.yaml` добавлен профиль для реального стенда.
Для реальных сенсоров реализованы `healthcheck()` и отдельные тесты:

- камера IMX290: `tests/test_camera_health_real.py`
- лидар T-mini Pro Plus: `tests/test_lidar_health_real.py`

Запуск hardware-тестов:

```bash
export LEOSHA_RUN_HARDWARE_TESTS=1
export LEOSHA_HARDWARE_CONFIG=/home/KIR/Leosha/code/client/config/client.hardware.yaml
pytest -m hardware -q
```

