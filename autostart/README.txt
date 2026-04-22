Автозапуск на Jetson
====================

Рекомендуемый способ — через systemctl (как обычные службы Linux)
----------------------------------------------------------------
1) Собрать воркспейс:
   cd ~/vtol_ws && colcon build

2) Один раз установить и включить сервисы (подставится твой пользователь, например nvidia):
   sudo bash ~/vtol_ws/autostart/install_systemd.sh

3) Проверка:
   sudo systemctl status vtol-flight.service
   sudo journalctl -u vtol-flight.service -f

Полезно:
   sudo systemctl stop vtol-flight.service vtol-logger.service   — остановить
   sudo systemctl disable vtol-flight.service vtol-logger.service — убрать автозапуск при загрузке

Что это: systemctl управляет «службами» (отдельными программами в фоне). enable = «стартовать при
включении компьютера». Логи ROS и нод смотри в journalctl (см. выше) и в ~/.ros/log/.


Альтернатива — crontab (если не хочешь systemd)
-----------------------------------------------
crontab — это «планировщик задач»: список команд и время, когда их запускать.
Запись @reboot означает «один раз при загрузке системы».

   crontab -e
   Добавить строку (путь свой):
   @reboot /home/nvidia/vtol_ws/autostart/run_on_boot.sh

Перед этим: chmod +x .../run_on_boot.sh .../launch_flight.sh .../launch_logger.sh

Логи при таком варианте: /tmp/vtol_flight.log и /tmp/vtol_logger.log
