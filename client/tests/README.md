# Тесты клиента

Набор тестов покрывает:

- `SensorHub` и атомарный `snapshot`;
- сериализацию/десериализацию серверных пакетов;
- работу `Watchdog`;
- исполнители режимов;
- интеграционный цикл `ClientRuntime` с `MockServer`.

Цель: подтвердить, что клиент устойчиво работает на частотах 30/100 Гц, корректно
обрабатывает потерю команд сервера и отправляет полезную телеметрию.

Дополнительно тест `test_sensor_artifacts.py` сохраняет артефакты:

- `tests/artifacts/camera_frame.png`
- `tests/artifacts/lidar_map.png`

## Healthcheck тесты реального железа

Для каждого реального сенсора есть отдельный hardware-тест:

- `test_camera_health_real.py` — запускает `CameraSensorThread.healthcheck()`;
- `test_lidar_health_real.py` — запускает `TMiniProPlusLidarThread.healthcheck()`.

Артефакты hardware-тестов:

- `tests/artifacts/camera_frame_real.png`
- `tests/artifacts/lidar_map_real.png`

Запуск:

```bash
export LEOSHA_RUN_HARDWARE_TESTS=1
export LEOSHA_HARDWARE_CONFIG=/home/KIR/Leosha/code/client/config/client.hardware.yaml
pytest -m hardware -q
```

Если `LEOSHA_RUN_HARDWARE_TESTS` не выставлен, hardware-тесты пропускаются.

