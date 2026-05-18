# Пакет `flight_manager` — модули узлов

Кратко: что делает каждый файл в этой директории.

| Модуль | Назначение |
|--------|------------|
| `__init__.py` | Маркер Python-пакета (содержимое пустое). |
| `mission_manager_node.py` | Высокоуровневая машина состояний миссии на внешней навигации: ждёт готовность по `/nav/status`, переключение EKF, VTOL-взлёт, зависание в QLOITER, опциональный крейс по точкам из `~/coords.txt` (только при `enable_waypoints=True`, т.е. для navigation∈{dpvo,hybrid}, у нас квадраплан) через GUIDED + `/ap/cmd/goto_global` в самолётном forward-flight режиме, back-transition в QLOITER, посадка в QLAND. Tags-режим этот крейс пропускает и остаётся чистым квадракоптерным циклом takeoff→hover→land. Учитывает отказы армирования, таймауты на точку и потерю визуального лока в любом из «летящих» состояний (ASCENDING/HOLD/REQUEST_WAYPOINT/CRUISE_TO_WAYPOINT). |
| `vision_fusion_node.py` | Гибрид тегового локализатора и VO: оценивает смещение SE(3) между картой тегов и кадром VO, публикует единую позу `/vision_pose_enu` и здоровье `/nav/status` (режимы tag / VO / плавный возврат). |
| `single_source_router_node.py` | В режиме одного источника (теги, DPVO, ORB-SLAM3 и т.п.) пробрасывает нативные топики позы и статуса на единый контракт `/vision_pose_enu` и `/nav/status`, чтобы мост, TF и миссия не зависели от выбранного бэкенда. |
| `tf_broadcaster_node.py` | TF: динамически `map` → `base_link` из `/vision_pose_enu`, статически `base_link` → камера по параметрам калибровки. |
| `safety_log_supervisor_node.py` | Гейтит запись по фронту `/ap/safety_released`: на `False → True` (safety released, моторы готовы к работе) открывает новую `logs/session_<timestamp>` и стартует rosbag + опц. видео + опц. CSV; на `True → False` (safety engaged) шлёт SIGINT всем рекордерам с пер-процессорным дедлайном (sqlite успеет flush, MP4 успеет финализировать moov-atom), пишет `manifest.yaml` и копирует снимок `discovered_map.yaml` + `anchor_*.yaml` из runtime-каталога локалайзера в папку цикла. Активен только при `flight.yaml -> logging.safety_gated_sessions.enabled: true`. |
| `gps_takeoff_land_node.py` | Регрессионный сценарий только GPS: подключение, GUIDED, арм, взлёт на заданную высоту, зависание, посадка через сервис контроллера (QLAND для VTOL). |
| `gps_waypoints_node.py` | GPS-миссия в AUTO: после готовности строит простую цепочку waypoints от «дома» на заданном расстоянии/высоте и заливает её через сервисы ArduPilot. |
| `gps_range_test_node.py` | Аналогично GPS waypoints, но заточен под дальностный тест: миссия из 16 waypoints плюс возврат домой (см. лог инициализации узла). |

Точки входа и параметры задаются в `setup.py` и launch-файлах пакета `flight_manager`.
