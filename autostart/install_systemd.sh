#!/usr/bin/env bash
# Ставит systemd-сервисы для автозапуска VTOL стека на Jetson.
#
# По умолчанию ставится только vtol-flight (без автономного взлёта):
#     sudo bash autostart/install_systemd.sh
#
# Чтобы включить ПОЛНУЮ цепочку (boot -> 2min -> flight.launch.py -> 1min ->
# автономный takeoff/hover/land):
#     sudo bash autostart/install_systemd.sh --with-takeoff
#
# !! WARNING про --with-takeoff: после --with-takeoff каждое включение
# Jetson приведёт к автономному взлёту дрона через ~3 минуты. Используйте
# kill-switch для отладки:  touch ~/.disable_autotakeoff
#
# Удалить весь автозапуск:
#     sudo bash autostart/install_systemd.sh --uninstall

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Нужны права root. Запусти так:"
  echo "  sudo bash $(readlink -f "${BASH_SOURCE[0]}") [--with-takeoff|--uninstall]"
  exit 1
fi

# --- параметры -------------------------------------------------------------
WITH_TAKEOFF=0
DO_UNINSTALL=0
TARGET_ALT_M="${TARGET_ALT_M:-15.0}"
HOVER_SEC="${HOVER_SEC:-10.0}"
FLIGHT_DELAY_SEC="${FLIGHT_DELAY_SEC:-120}"   # 2 мин после boot
TAKEOFF_DELAY_SEC="${TAKEOFF_DELAY_SEC:-180}" # 3 мин после boot (= ~1 мин после flight)

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-takeoff)   WITH_TAKEOFF=1 ;;
    --uninstall)      DO_UNINSTALL=1 ;;
    *) echo "Неизвестный аргумент: $1" >&2; exit 1 ;;
  esac
  shift
done

RUN_USER="${SUDO_USER:-${USER:-root}}"
RUN_GROUP="$(id -gn "${RUN_USER}" 2>/dev/null || echo "${RUN_USER}")"
USER_HOME="$(getent passwd "${RUN_USER}" | cut -d: -f6)"
[[ -n "${USER_HOME}" ]] || { echo "Нет пользователя ${RUN_USER}" >&2; exit 1; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${SCRIPT_DIR}/.." && pwd)"
FLIGHT_SH="${WS}/autostart/launch_flight.sh"
TAKEOFF_SH="${WS}/autostart/launch_takeoff.sh"

# --- uninstall -------------------------------------------------------------
if [[ "${DO_UNINSTALL}" -eq 1 ]]; then
  echo "Удаляю vtol-* сервисы и таймеры..."
  for unit in vtol-takeoff.timer vtol-takeoff.service \
              vtol-flight.timer  vtol-flight.service \
              vtol-logger.service; do
    systemctl disable --now "${unit}" 2>/dev/null || true
    rm -f "/etc/systemd/system/${unit}"
  done
  systemctl daemon-reload
  systemctl reset-failed 2>/dev/null || true
  echo "Готово. Все автозапуски VTOL отключены."
  exit 0
fi

# --- prereq checks --------------------------------------------------------
chmod 755 "${FLIGHT_SH}"
chown "${RUN_USER}:${RUN_GROUP}" "${FLIGHT_SH}" 2>/dev/null || true
if [[ "${WITH_TAKEOFF}" -eq 1 ]]; then
  [[ -f "${TAKEOFF_SH}" ]] || { echo "Нет ${TAKEOFF_SH}" >&2; exit 1; }
  chmod 755 "${TAKEOFF_SH}"
  chown "${RUN_USER}:${RUN_GROUP}" "${TAKEOFF_SH}" 2>/dev/null || true
fi

# Чистим устаревший vtol-logger от прежних установок.
if systemctl list-unit-files | grep -q '^vtol-logger\.service'; then
  systemctl disable --now vtol-logger.service 2>/dev/null || true
  rm -f /etc/systemd/system/vtol-logger.service
fi

write_unit() {
  local file="$1" body="$2"
  printf '%s\n' "${body}" > "${file}"
  chmod 644 "${file}"
}

# --- vtol-flight.service ---------------------------------------------------
# Сервис запускается ТАЙМЕРОМ (vtol-flight.timer) через 2 мин после boot.
# Сам по себе он НЕ enabled на boot; стартует только когда таймер сработает.
write_unit "/etc/systemd/system/vtol-flight.service" "[Unit]
Description=VTOL ros2 launch bringup flight.launch.py
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${RUN_USER}
Group=${RUN_GROUP}
WorkingDirectory=${WS}
Environment=HOME=${USER_HOME}
Environment=ROS_LOG_DIR=${USER_HOME}/.ros/log
ExecStart=${FLIGHT_SH}
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal
"

write_unit "/etc/systemd/system/vtol-flight.timer" "[Unit]
Description=Trigger VTOL flight stack ${FLIGHT_DELAY_SEC}s after boot

[Timer]
OnBootSec=${FLIGHT_DELAY_SEC}s
AccuracySec=1s
Unit=vtol-flight.service

[Install]
WantedBy=timers.target
"

systemctl daemon-reload
# enable таймер; сам сервис НЕ enable, он запускается через таймер.
systemctl enable vtol-flight.timer
# сразу не запускаем, т.к. install ставится обычно при работающем терминале;
# таймер стартует на следующий boot.

echo "Установлен vtol-flight.timer: запустит flight.launch.py через ${FLIGHT_DELAY_SEC}s после boot."

# --- vtol-takeoff.service (опционально) ------------------------------------
if [[ "${WITH_TAKEOFF}" -eq 1 ]]; then
  write_unit "/etc/systemd/system/vtol-takeoff.service" "[Unit]
Description=VTOL autonomous GPS takeoff/hover/land
After=vtol-flight.service
Requires=vtol-flight.service

[Service]
Type=simple
User=${RUN_USER}
Group=${RUN_GROUP}
WorkingDirectory=${WS}
Environment=HOME=${USER_HOME}
Environment=ROS_LOG_DIR=${USER_HOME}/.ros/log
Environment=TARGET_ALT_M=${TARGET_ALT_M}
Environment=HOVER_SEC=${HOVER_SEC}
ExecStart=${TAKEOFF_SH}
Restart=no
StandardOutput=journal
StandardError=journal
"

  write_unit "/etc/systemd/system/vtol-takeoff.timer" "[Unit]
Description=Trigger autonomous takeoff ${TAKEOFF_DELAY_SEC}s after boot

[Timer]
OnBootSec=${TAKEOFF_DELAY_SEC}s
AccuracySec=1s
Unit=vtol-takeoff.service

[Install]
WantedBy=timers.target
"

  systemctl daemon-reload
  systemctl enable vtol-takeoff.timer

  cat <<EOF

==============================================================================
!! AUTONOMOUS TAKEOFF ENABLED                                              !!
==============================================================================
Каждое включение Jetson => через ${FLIGHT_DELAY_SEC}s стартует flight.launch.py,
ещё через ~$(( TAKEOFF_DELAY_SEC - FLIGHT_DELAY_SEC ))s gps_takeoff_land_node поднимет дрон на ${TARGET_ALT_M}m,
зависнет ${HOVER_SEC}s и сядет (QLAND).

Перед каждым включением убедитесь:
  * Дрон стоит в открытом месте, пропеллеры свободны.
  * GPS получит fix (открытое небо).
  * RC-передатчик включён, mode-switch в QHOVER/QLOITER на случай перехвата.

KILL-SWITCH (отключить взлёт без удаления сервиса):
  touch ${USER_HOME}/.disable_autotakeoff   # на Jetson; перед boot
  rm    ${USER_HOME}/.disable_autotakeoff   # снова включить

ПОСМОТРЕТЬ КОГДА СРАБОТАЕТ:
  systemctl list-timers --all | grep vtol

ПОЛНОСТЬЮ УДАЛИТЬ АВТОЗАПУСК (включая takeoff):
  sudo bash ${SCRIPT_DIR}/install_systemd.sh --uninstall
==============================================================================
EOF
fi

cat <<EOF

Установлено. Подсказки:
  systemctl list-timers --all | grep vtol      # увидеть, когда сработают
  systemctl status vtol-flight.service          # статус flight стека
  journalctl -u vtol-flight.service -f          # логи flight в реальном времени
  journalctl -u vtol-takeoff.service -f         # логи takeoff (если установлен)

Чтобы вступило в силу - перезагрузите Jetson (sudo reboot).
Для немедленного запуска без ребута:
  sudo systemctl start vtol-flight.service
EOF
