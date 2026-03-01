# Сетевой слой клиента

В текущей реализации для тестов используется in-memory транспорт:

- `MockServer` — имитация сервера с режимами `teleoperation` и `autonomy_profile_1`;
- `InMemoryBridge` — мост, передающий пакеты клиенту без сокетов;
- `ZmqBridge` — реальный транспорт ZeroMQ (`PUB/SUB`);
- `serialization.py` — преобразование структур в бинарный вид (`msgpack`).

Такая схема позволяет стабильно тестировать бизнес-логику клиента без зависимости
от реальной сети и оборудования.

Сетевые endpoint'ы `ZmqBridge` задаются в `config/client.yaml`:

- `network.server_host`
- `network.telemetry_port` (Robot -> Server)
- `network.command_port` (Server -> Robot)
- `network.report_port` (Robot -> Server, reports)

