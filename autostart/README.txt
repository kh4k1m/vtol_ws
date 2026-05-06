Автозапуск на Jetson
====================

ВАРИАНТ 1: только VTOL стек, без автономного взлёта (безопасный для отладки)
---------------------------------------------------------------------------
1) Собрать воркспейс:
   cd ~/vtol_ws && colcon build

2) Один раз поставить сервис:
   sudo bash ~/vtol_ws/autostart/install_systemd.sh

3) Что произойдёт после ребута:
   * через 45 секунд после загрузки: запустится `ros2 launch bringup flight.launch.py`
   * автономного взлёта НЕ будет
   * пилот сам решает, запускать ли takeoff командой

4) Проверка:
   systemctl list-timers --all | grep vtol      # увидеть, когда сработает
   systemctl status vtol-flight.service          # статус flight стека
   journalctl -u vtol-flight.service -f          # логи в реальном времени


ВАРИАНТ 2: ПОЛНАЯ автономия (boot -> ROS -> взлёт -> зависание -> посадка)
--------------------------------------------------------------------------
ВНИМАНИЕ: после --with-takeoff каждое включение Jetson приведёт к АВТОНОМНОМУ
взлёту дрона. Перед включением убедитесь, что:
   * дрон в открытом безопасном месте
   * пропеллеры свободны
   * RC-передатчик включён, mode-switch в QHOVER на случай ручного перехвата

   sudo bash ~/vtol_ws/autostart/install_systemd.sh --with-takeoff

Параметры takeoff (можно переопределить через env):
   TARGET_ALT_M=15.0           # высота зависания, метров
   HOVER_SEC=10.0              # время зависания, секунд
   FLIGHT_DELAY_SEC=45         # задержка до запуска flight.launch.py (сек)
   TAKEOFF_DELAY_SEC=60        # задержка до запуска gps_takeoff_land_node
                               # (отсчёт от boot, не от flight)

Например, изменить высоту до 20м:
   sudo TARGET_ALT_M=20.0 bash ~/vtol_ws/autostart/install_systemd.sh --with-takeoff

KILL-SWITCH (быстро отключить ТОЛЬКО takeoff, не трогая flight стек):
   touch ~/.disable_autotakeoff       # на Jetson, ДО boot
   rm    ~/.disable_autotakeoff       # снова включить

Полностью удалить весь автозапуск:
   sudo bash ~/vtol_ws/autostart/install_systemd.sh --uninstall


Полезные команды
----------------
   systemctl list-timers --all | grep vtol      # таймеры, время до срабатывания
   systemctl status vtol-flight.service          # статус flight стека
   systemctl status vtol-takeoff.service         # статус takeoff (если установлен)
   journalctl -u vtol-flight.service -f          # логи flight
   journalctl -u vtol-takeoff.service -f         # логи takeoff
   sudo systemctl stop vtol-flight.service vtol-takeoff.service  # остановить сейчас
   sudo systemctl start vtol-flight.service                       # запустить сейчас, без ребута

Как это работает
----------------
* `vtol-flight.timer`  => запускает `vtol-flight.service` через FLIGHT_DELAY_SEC после boot.
* `vtol-takeoff.timer` => запускает `vtol-takeoff.service` через TAKEOFF_DELAY_SEC после boot.
* `vtol-takeoff.service` имеет `Requires=vtol-flight.service`: если flight упал,
  takeoff не запустится.
* Логи всех сервисов идут в journald (см. journalctl -u ...).
