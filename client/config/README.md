# Конфиги клиента

Конфигурации хранятся в общей папке конфигов.

**Используйте:** `code/configs/` (относительно корня репозитория).

- `code/configs/client.yaml` — конфиг по умолчанию (разработка, mock)
- `code/configs/client.hardware.yaml` — конфиг для реального железа
- `code/configs/templates/client.template.yaml` — шаблон с комментариями
- `code/configs/README.md` — таблица всех аргументов

Запуск с конфигом: `python -m client --config /path/to/Leosha/code/configs/client.yaml`
