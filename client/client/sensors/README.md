# Сенсорные потоки

Подпакет `client.sensors` содержит:

- `CameraSensorThread` — реальная камера IMX290 через `picamera2`;
- `TMiniProPlusLidarThread` — реальный YDLidar T-mini Pro Plus через `ydlidar`;
- `HealthcheckResult` — единый результат healthcheck;
- `SensorStatusRegistry` — потокобезопасные статусы сенсоров.

Каждый реальный сенсор реализует `healthcheck(timeout_sec)`:

- камера: проверка старта, захват и JPEG-кодирование нескольких кадров;
- лидар: проверка старта, получение скана с минимальным числом точек.

Для YDLidar T-mini Pro Plus дополнительно поддержаны параметры из рабочего скрипта
`play/plot_tminiplus_test.py`:

- автоопределение порта через `lidarPortList`;
- `min/max angle` (обычно `-180..180`);
- включение `intensity`;
- инверсия угла (`angle = -point.angle`) для согласованной ориентации.

Поведение при сбоях:

- каждый сенсор имеет `fail_policy` в YAML;
- при ошибке увеличивается `failure_count` и обновляется `last_error`;
- при превышении порога сенсор может автоотключаться, не роняя клиент;
- статус сенсора передается в `TelemetryPacket.sensor_status`.

