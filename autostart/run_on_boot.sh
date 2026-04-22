#!/usr/bin/env bash
# Вариант без systemd: один раз добавить в crontab строку @reboot (см. README.txt)
set -eo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
nohup "${DIR}/launch_flight.sh" >>/tmp/vtol_flight.log 2>&1 &
nohup "${DIR}/launch_logger.sh" >>/tmp/vtol_logger.log 2>&1 &
