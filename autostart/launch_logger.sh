#!/usr/bin/env bash
# Вызывается из systemd или вручную. Поднимает только logger.
set -eo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Сборка vtol_ws под Humble; другой дистрибутив: export ROS_DISTRO=...
ROS_DISTRO="${ROS_DISTRO:-humble}"
[[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || { echo "Нет /opt/ros/${ROS_DISTRO}/setup.bash" >&2; exit 1; }
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck source=/dev/null
source "${WS}/install/setup.bash"
cd "${WS}"
exec ros2 launch bringup logger.launch.py
