# flight_viz

Офлайн-визуализация полётной сессии (`logs/session_<timestamp>/`),
которую пишет `bringup/launch/flight.launch.py`.

## Что умеет

Конфиг `config/visualize.yaml` описывает список графиков. Поддерживаются:

| `type`              | Что рисует |
|---|---|
| `trajectory_xy`     | Горизонтальный путь (East/North в метрах от анкера, либо локальные X/Y). |
| `altitude_time`     | Высота во времени для всех источников. |
| `xyz_separately`    | Три subplots X(t), Y(t), Z(t) — оси раздельно. |
| `marker_visibility` | Путь по GPS, цвет точки = виден ли AprilTag в этот момент. |

## Цветовая схема

* `gps` (truth) — синий;
* предсказанная поза в пределах `tolerance_m` от GPS — зелёная точка;
* за пределами — красная;
* для `marker_visibility`: зелёная точка = тег виден в окне `visibility_gap_sec`,
  красная = не виден.

## Запуск

```bash
# Общий ROS 2 environment + рабочее пространство
source /opt/ros/humble/setup.bash
source ~/vtol_ws/install/setup.bash

# Зависимости (один раз)
pip install -r ~/vtol_ws/tools/flight_viz/requirements.txt

# Самая свежая сессия из ~/vtol_ws/logs/session_*/
python3 -m flight_viz

# Конкретная сессия + альтернативный конфиг
python3 -m flight_viz --session ~/vtol_ws/logs/session_20260505-180000 \
                     --config ~/vtol_ws/config/visualize.yaml \
                     --format html
```

Результат — `~/vtol_ws/logs/viz/session_*__<ts>/index.html` с
интерактивными plotly-графиками. Открывается в любом браузере (или
напрямую как HTML в IDE).

## Что должно лежать в сессии

Минимально необходимый набор для базовой визуализации:

* `bag/` — rosbag2 (mcap или sqlite3) с топиками из `flight.yaml.logging.rosbag.profile`.
* `manifest.yaml` — пишется автоматически launch'ем и содержит метаданные.

Опционально:

* `flight_data_*.csv` — CSV от `telemetry_logger_node`.
* `video/*.mp4` — записи камеры (пока не используются в plots, оставлены для overlay'я).

## Как добавить новый тип графика

1. Добавить функцию `make_<type>(session, plot_cfg)` в `plots.py`.
2. Зарегистрировать в `PLOT_BUILDERS`.
3. Описать новый блок в `config/visualize.yaml`.

## Зависимости

* `plotly` (интерактивный рендер)
* `numpy`
* `PyYAML`
* `rosbag2_py` + `rclpy` + `rosidl_runtime_py` (из стандартной ROS-установки)
* `kaleido` (опционально, для `--format png`)
