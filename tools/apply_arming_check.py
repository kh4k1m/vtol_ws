#!/usr/bin/env python3
"""Set ArduPilot ARMING_CHECK from config/flight.yaml for current environment."""
from __future__ import annotations

import argparse
import os
import sys
import time

import yaml
from pymavlink import mavutil


def _load_flight_config(workspace: str) -> dict:
    path = os.path.join(workspace, 'config', 'flight.yaml')
    with open(path, 'r', encoding='utf-8') as f:
        root = yaml.safe_load(f)
    return root.get('flight_config', {})


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        '--workspace',
        default=os.environ.get('VTOL_WS_ROOT', os.path.expanduser('~/vtol_ws')),
        help='Workspace root (contains config/flight.yaml)',
    )
    ap.add_argument(
        '--baud',
        type=int,
        default=115200,
        help='Serial baud if connection_string is /dev/tty...',
    )
    args = ap.parse_args()

    fc = _load_flight_config(args.workspace)
    env = str(fc.get('environment', 'real')).strip().lower()
    conn_map = {
        'real': fc.get('real_connection', '/dev/ttyACM0'),
        'gazebo': fc.get('gazebo_connection', 'tcp:127.0.0.1:5762'),
        'from_log': fc.get('log_connection', 'tcp:127.0.0.1:5762'),
    }
    conn = str(conn_map.get(env, conn_map['real']))

    ard = fc.get('ardupilot', {}) or {}
    ac = ard.get('arming_check', {}) or {}
    value = ac.get(env)
    if value is None:
        print(f'No arming_check.{env} in flight.yaml', file=sys.stderr)
        return 1

    print(f'Connecting to {conn} (env={env}) ...')
    master = mavutil.mavlink_connection(conn, baud=args.baud)
    master.wait_heartbeat(timeout=15)
    print(
        f'Heartbeat from sys={master.target_system} comp={master.target_component}'
    )

    param_id = 'ARMING_CHECK'.encode('ascii')
    param_id = param_id + bytes(16 - len(param_id))

    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_id,
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )

    deadline = time.time() + 5.0
    confirmed = None
    while time.time() < deadline:
        msg = master.recv_match(type='PARAM_VALUE', blocking=False)
        if msg is None:
            time.sleep(0.05)
            continue
        name = msg.param_id
        if isinstance(name, bytes):
            name = name.split(b'\x00')[0].decode('ascii', errors='ignore')
        if name != 'ARMING_CHECK':
            continue
        confirmed = msg.param_value
        break

    if confirmed is None:
        print('PARAM_VALUE for ARMING_CHECK not received (may still have applied).')
    else:
        print(f'ARMING_CHECK is now {confirmed}')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
