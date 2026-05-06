#!/usr/bin/env bash
# Ставит systemd-сервис vtol-flight (logging встроено в flight.launch.py).
# Запуск:  cd ~/vtol_ws && sudo bash autostart/install_systemd.sh
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Нужны права root. Запусти так:"
  echo "  sudo bash $(readlink -f "${BASH_SOURCE[0]}")"
  exit 1
fi

RUN_USER="${SUDO_USER:-${USER:-root}}"
RUN_GROUP="$(id -gn "${RUN_USER}" 2>/dev/null || echo "${RUN_USER}")"
USER_HOME="$(getent passwd "${RUN_USER}" | cut -d: -f6)"
[[ -n "${USER_HOME}" ]] || { echo "Нет пользователя ${RUN_USER}" >&2; exit 1; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${SCRIPT_DIR}/.." && pwd)"
FLIGHT="${WS}/autostart/launch_flight.sh"
chmod 755 "${FLIGHT}"
chown "${RUN_USER}:${RUN_GROUP}" "${FLIGHT}" 2>/dev/null || true

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
ExecStart=${FLIGHT}
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
"

systemctl daemon-reload
systemctl enable --now vtol-flight.service

echo ""
echo "Готово. Сервис: vtol-flight (запуск от пользователя ${RUN_USER})"
echo "Логирование контролируется блоком logging в config/flight.yaml."
echo "Статус:   sudo systemctl status vtol-flight.service"
echo "Логи:     sudo journalctl -u vtol-flight.service -f"
echo "Отключить автозагрузку: sudo systemctl disable vtol-flight.service"
