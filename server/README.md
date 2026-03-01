# Server: ROS2 пакет управления роботом

Этот пакет реализует серверную часть для удаленного управления роботом под именем ROS2-пакета `server`:

- единый интерфейс профилей и алгоритмов управления;
- web GUI с тремя вкладками;
- mock-клиент робота для локальной разработки и тестов;
- запуск как ROS2-пакет (`ros2 run server server`);
- единый YAML-конфиг для параметров приложения.

## Что реализовано

1. **Вкладка "Настройки"**
   - IP робота;
   - Ping до робота;
   - статус соединения;
   - выбор режима:
     - `pause`,
     - `teleoperation`,
     - `autonomy_profile_1`.

2. **Вкладка "Телеуправление"**
   - поток последних кадров (`/video_feed`);
   - управление WASD и мышью;
   - интерфейс максимально близок к исходному фронтенду из `code/head/templates/index.html`.

3. **Вкладка "Сеть"**
   - текущие метрики: latency, RSSI, байт/с и пакеты/с;
   - поле для Grafana URL и встраивание iframe при наличии ссылки.

## Архитектура

Основные слои:

- `server.interfaces` — абстракции:
  - `ControlAlgorithm`;
  - `AutonomyAlgorithm` (интерфейс для будущей самоуправляемой логики);
  - `OperationProfile`.
- `server.algorithms.profiles` — реализации профилей:
  - `PauseProfile`;
  - `TeleoperationProfile` (SLAM + телеуправление);
  - `AutonomyProfile1` (заглушка).
- `server.services.controller_service` — оркестратор режимов, ручного ввода и цикла команд.
- `server.services.robot_client` — интерфейс клиента робота и `MockRobotClient`.
- `server.web.app_factory` — Flask web GUI и HTTP API.
- `config/server.yaml` — централизованный конфиг.

## Быстрый запуск

Из каталога `code/server`:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e ".[test]"
python -m server.main
```

По умолчанию сервер поднимается на `http://127.0.0.1:8080`.

## Запуск как ROS2 пакет

```bash
colcon build --packages-select server
source install/setup.bash
ros2 run server server
```

Через launch:

```bash
ros2 launch server server.launch.py
```

## YAML-конфиг

Все runtime-параметры вынесены в `config/server.yaml`:

- сеть приложения (`app.host`, `app.port`);
- частота командного цикла (`app.command_hz`);
- IP робота (`robot.ip`);
- лимиты управления (`control.*`);
- URL Grafana (`network.grafana_url`).

## Запуск тестов

```bash
pytest -q
```

## ROS2-совместимость

Пакет оформлен как ROS2 Python package (`package.xml`, `setup.py`) с именем `server`.
Если `rclpy` отсутствует, приложение работает в standalone-режиме (web + mock).
На Ubuntu с ROS2 можно запускать в штатной среде colcon.
