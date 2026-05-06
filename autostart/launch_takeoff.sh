#!/usr/bin/env bash
# Автономный VTOL takeoff/hover/land после старта flight.launch.py.
#
# !! ВНИМАНИЕ: при boot Jetson дрон взлетит САМ через задержку из таймера
#    (по умолчанию ~3 минуты после включения). Прежде чем включать питание -
#    убедитесь что дрон в безопасной позиции (открытое поле, пропеллеры свободны).
#
# Kill-switch: создайте файл ~/.disable_autotakeoff (touch) -- этот скрипт
# тогда выйдет с кодом 0 без запуска takeoff. Удобно для отладки на стенде:
#     touch ~/.disable_autotakeoff   # отключить
#     rm   ~/.disable_autotakeoff    # снова включить
#
# Параметры через окружение (можно задать в systemd unit, см. install_systemd.sh):
#   TARGET_ALT_M  - высота зависания, метров (default: 15.0)
#   HOVER_SEC     - время зависания, секунд (default: 10.0)

set -eo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_TAG="[$(date -Iseconds) launch_takeoff]"

# === SAFETY: kill-switch ====================================================
if [[ -f "${HOME}/.disable_autotakeoff" ]]; then
  echo "${LOG_TAG} kill-switch ${HOME}/.disable_autotakeoff exists -> NOT taking off."
  exit 0
fi

# === ROS env ================================================================
ROS_DISTRO="${ROS_DISTRO:-humble}"
[[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || {
  echo "${LOG_TAG} нет /opt/ros/${ROS_DISTRO}/setup.bash" >&2; exit 1; }
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[[ -f "${WS}/install/setup.bash" ]] || {
  echo "${LOG_TAG} нет ${WS}/install/setup.bash (выполните colcon build)" >&2; exit 1; }
# shellcheck source=/dev/null
source "${WS}/install/setup.bash"
cd "${WS}"

# === Параметры takeoff =====================================================
TARGET_ALT="${TARGET_ALT_M:-15.0}"
HOVER_SEC="${HOVER_SEC:-10.0}"

echo "${LOG_TAG} starting gps_takeoff_land_node target=${TARGET_ALT}m hover=${HOVER_SEC}s"
exec ros2 run flight_manager gps_takeoff_land_node \
  --ros-args \
  -p target_altitude_m:="${TARGET_ALT}" \
  -p hover_duration_sec:="${HOVER_SEC}"
