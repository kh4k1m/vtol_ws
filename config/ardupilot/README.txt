ArduPilot параметры для vtol_ws
=================================

1) ARMING_CHECK
-----------------------------------------------------------------------
Эталонные значения в `config/flight.yaml`:
  `flight_config.ardupilot.arming_check`
  — поля `real`, `gazebo`, `from_log` (целое число — битмаска ArduPilot).

Файлы для ручной загрузки в QGC / Mission Planner:
  - `arming_check_real.parm`
  - `arming_check_gazebo.parm`

Применить с Jetson по текущему `environment` из flight.yaml:
  cd ~/vtol_ws
  source install/setup.bash
  python3 tools/apply_arming_check.py

Требуется свободный MAVLink-порт (закройте другие GCS на этом порту).

Точное значение битов ARMING_CHECK смотрите в документации вашей
прошивки; для боевого реала обычно не используют 0.


2) SITL — честный полёт по визуалке (без GPS / без truth-cheat)
-----------------------------------------------------------------------
ArduPilot SITL по дефолту:
  AHRS_EKF_TYPE = 10     -> AHRS читает позу прямо из физсимуляции
                             Gazebo (truth-pose cheat). EKF3 крутится
                             на фоне, но в управлении не участвует.
  SIM_GPS_DISABLE = 0    -> SITL генерит идеальные GPS-фиксы.

В таком режиме всё «летает» даже если наша визуалка сломана — это и
есть «cheat»: контроллер читает truth-pose, не EKF. Чтобы поведение
SITL соответствовало реальному GPS-denied борту, используем
`visual_only_sitl.parm`. Он переключает:
  AHRS_EKF_TYPE      = 3   (AHRS использует EKF3)
  EK2_ENABLE         = 0   (нет фоллбэка на EKF2)
  EK3_SRC1_POSXY/Z   = 6   (позиция из ExternalNav)
  EK3_SRC1_VELXY/Z   = 6   (скорость из ExternalNav)
  EK3_SRC1_YAW       = 6   (yaw из ExternalNav)
  VISO_TYPE          = 1, VISO_QUAL_MIN = 0
  SIM_GPS_DISABLE    = 1   (SITL-GPS выключен -> EKF не «съезжает» назад на GPS)
  ARMING_CHECK       = 0

Способы применить:
  a) Через sim_vehicle.py при старте симуляции (надёжнее всего, т.к.
     параметры применяются до старта EKF, без reboot):
        sim_vehicle.py -v ArduPlane -f gazebo-zephyr \
            --add-param-file=$(pwd)/config/ardupilot/visual_only_sitl.parm

  b) К уже работающему SITL (с авто-ребутом автопилота):
        python3 tools/apply_sitl_visual_only.py

     По умолчанию подключается к tcp:127.0.0.1:5763 (свободный порт
     SITL, чтобы не мешать работающему bridge на 5762). Скрипт делает
     PARAM_SET по каждой строке файла, проверяет PARAM_VALUE-echo и
     отправляет MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN. После ребута запустите
     заново `ros2 launch bringup flight.launch.py`.

После этой настройки:
  - mission_manager какое-то время сидит в WAIT_FOR_NAV/WAIT_FOR_SETTLE,
    пока EKF3 не сойдётся на ExternalNav (5-10 сек — это нормально).
  - Если визуалка пропадёт надолго, AP реально потеряет контроль и
    выкинет failsafe — ровно как на реальном GPS-denied борту.
