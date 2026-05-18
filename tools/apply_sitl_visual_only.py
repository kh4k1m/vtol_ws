#!/usr/bin/env python3
"""Configure an already-running ArduPilot SITL for honest visual-only flight.

Loads ``config/ardupilot/visual_only_sitl.parm``, PARAM_SETs every line
on the autopilot, verifies the new values via PARAM_VALUE echo, then
reboots the autopilot so EKF reinitialises with ExternalNav as the
only nav source (no GPS, no SITL truth-pose cheat).

Why this matters: SITL ships with ``AHRS_EKF_TYPE=10`` (SITL cheat
backend) and a simulated GPS, which makes ArduPilot fly perfectly even
when our visual odometry is junk -- because the controller reads
truth pose directly from the physics simulation. That hides bugs that
*will* surface on a real GPS-denied airframe. This script puts the
simulator into the same regime the real vehicle uses.

Usage:
    python3 tools/apply_sitl_visual_only.py
    python3 tools/apply_sitl_visual_only.py --no-reboot
    python3 tools/apply_sitl_visual_only.py --connection tcp:127.0.0.1:5763

Connect to a *free* MAVLink port (the bridge usually uses 5762; SITL
exposes 5760/5762/5763 by default). The default 5763 leaves the
running bridge on 5762 untouched, so you can apply this without
stopping the rest of the stack.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Dict, List, Tuple

from pymavlink import mavutil


DEFAULT_PARM = 'config/ardupilot/visual_only_sitl.parm'
DEFAULT_CONNECTION = 'tcp:127.0.0.1:5763'
PARAM_ACK_TIMEOUT_SEC = 3.0


def parse_parm(path: str) -> List[Tuple[str, float]]:
    items: List[Tuple[str, float]] = []
    with open(path, 'r', encoding='utf-8') as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            # Common formats: "NAME,VALUE", "NAME VALUE", "NAME=VALUE"
            for sep in (',', '=', None):
                parts = line.split(sep, 1) if sep else line.split(None, 1)
                if len(parts) == 2:
                    break
            else:
                continue
            name = parts[0].strip().upper()
            try:
                value = float(parts[1].strip().split()[0])
            except ValueError:
                print(f'  skip non-numeric line: {raw!r}', file=sys.stderr)
                continue
            items.append((name, value))
    return items


def encode_param_id(name: str) -> bytes:
    encoded = name.encode('ascii')
    if len(encoded) > 16:
        raise ValueError(f'PARAM name too long: {name}')
    return encoded + bytes(16 - len(encoded))


def apply_params(master, params: List[Tuple[str, float]]) -> Dict[str, float]:
    """PARAM_SET every entry then read back each one. Returns confirmed values."""
    confirmed: Dict[str, float] = {}
    for name, value in params:
        master.mav.param_set_send(
            master.target_system, master.target_component,
            encode_param_id(name), float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
    # Drain echoes
    deadline = time.time() + PARAM_ACK_TIMEOUT_SEC
    names_wanted = {n for n, _ in params}
    while time.time() < deadline and len(confirmed) < len(names_wanted):
        msg = master.recv_match(type='PARAM_VALUE', blocking=False)
        if msg is None:
            time.sleep(0.02)
            continue
        nm_raw = msg.param_id
        if isinstance(nm_raw, bytes):
            nm = nm_raw.split(b'\x00')[0].decode('ascii', errors='ignore')
        else:
            nm = str(nm_raw).split('\x00')[0]
        if nm in names_wanted:
            confirmed[nm] = float(msg.param_value)
    return confirmed


def reboot_autopilot(master) -> None:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1,  # 1 = reboot autopilot
        0, 0, 0, 0, 0, 0,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        '--workspace',
        default=os.environ.get('VTOL_WS_ROOT', os.path.expanduser('~/vtol_ws')),
        help='Workspace root (used to resolve relative --parm-file path)',
    )
    ap.add_argument(
        '--parm-file', default=DEFAULT_PARM,
        help=f'Param file to load (default: {DEFAULT_PARM})',
    )
    ap.add_argument(
        '--connection', default=DEFAULT_CONNECTION,
        help=f'MAVLink connection string (default: {DEFAULT_CONNECTION})',
    )
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument(
        '--no-reboot', action='store_true',
        help='Skip MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN after applying params',
    )
    args = ap.parse_args()

    parm_path = args.parm_file
    if not os.path.isabs(parm_path):
        parm_path = os.path.join(args.workspace, parm_path)
    if not os.path.isfile(parm_path):
        print(f'No such param file: {parm_path}', file=sys.stderr)
        return 1

    params = parse_parm(parm_path)
    if not params:
        print(f'Param file is empty: {parm_path}', file=sys.stderr)
        return 1
    print(f'Loaded {len(params)} params from {parm_path}')

    print(f'Connecting to {args.connection} ...')
    master = mavutil.mavlink_connection(
        args.connection, baud=args.baud, source_system=200
    )
    master.wait_heartbeat(timeout=15)
    print(f'Heartbeat from sys={master.target_system}'
          f' comp={master.target_component}')

    print('Applying parameters:')
    confirmed = apply_params(master, params)

    failures: List[Tuple[str, float, float]] = []
    for name, wanted in params:
        actual = confirmed.get(name)
        status = 'OK' if actual is not None and abs(actual - wanted) < 1e-3 else 'MISMATCH'
        actual_str = f'{actual:.4g}' if actual is not None else 'NO ECHO'
        print(f'  {name:18s} wanted={wanted:>8.3f}  got={actual_str:>10s}  [{status}]')
        if status != 'OK':
            failures.append((name, wanted, actual if actual is not None else float('nan')))

    if failures:
        print(
            f'\nWARNING: {len(failures)} param(s) did not confirm. '
            'AP may not actually have applied them. Re-run after a reboot, '
            'or check that the connection string points at the live AP.',
            file=sys.stderr,
        )

    if not args.no_reboot:
        print('\nRebooting autopilot so EKF reinitialises with new sources ...')
        reboot_autopilot(master)
        # After reboot the connection will be dropped by AP. Give it
        # a moment so the operator sees the SITL log line.
        time.sleep(1.0)
        print('Reboot command sent. Restart your bridge/launch once SITL is back up.')
    else:
        print('\n--no-reboot passed; AHRS_EKF_TYPE change will not take effect '
              'until you reboot AP yourself.')

    return 0 if not failures else 2


if __name__ == '__main__':
    raise SystemExit(main())
