# Алгоритмы и профили

Интерфейсы:

- `ControlAlgorithm` — единый контракт алгоритмов управления;
- `AutonomyAlgorithm` — контракт будущих автопилотов;
- `OperationProfile` — унифицированный интерфейс профилей.

Реализации:

- `PauseProfile`;
- `TeleoperationProfile` (SLAM + телеуправление);
- `AutonomyProfile1` (интерфейсная заглушка).
