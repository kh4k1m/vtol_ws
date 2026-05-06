"""CLI: replay one parameter set, or grid-search across many.

Examples
--------
Single run with default params (matches the live node defaults)::

    python3 -m tools.localizer_replay logs/session_20260506-112623

Override parameters from the CLI (any field of ``ReplayParams``)::

    python3 -m tools.localizer_replay logs/session_20260506-112623 \\
        --param cost_threshold_per_tag_m=5.0 \\
        --param detection_gap_reset_sec=2.0 \\
        --param stable_xy_stddev_m=1.0

Grid-search a curated preset and print a ranked table::

    python3 -m tools.localizer_replay logs/session_20260506-112623 --grid

Drop the stability gate (tagDB-only upper bound on coverage)::

    python3 -m tools.localizer_replay <session> --no-stability-gate
"""

from __future__ import annotations

import argparse
import dataclasses
import itertools
import os
import sys
import time
from typing import Dict, List

from .replay import ReplayParams, ReplayResult, replay_session


def _autodetect_session(workspace_dir: str):
    log_root = os.path.join(workspace_dir, 'logs')
    if not os.path.isdir(log_root):
        return None
    candidates = [
        os.path.join(log_root, d)
        for d in os.listdir(log_root)
        if d.startswith('session_') and os.path.isdir(os.path.join(log_root, d))
    ]
    if not candidates:
        return None
    return max(candidates, key=os.path.getmtime)


def _coerce(value: str):
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        pass
    if value.lower() in ('true', 'false'):
        return value.lower() == 'true'
    return value


def _params_from_overrides(overrides: List[str]) -> ReplayParams:
    p = ReplayParams()
    for kv in overrides:
        if '=' not in kv:
            raise SystemExit(f'--param expects key=value, got {kv!r}')
        k, v = kv.split('=', 1)
        if not hasattr(p, k):
            raise SystemExit(
                f'unknown param {k!r}. Known: {[f.name for f in dataclasses.fields(ReplayParams)]}'
            )
        setattr(p, k, _coerce(v))
    return p


# A small but high-signal preset. We sweep:
#   * cost_threshold_per_tag_m: low-alt default vs high-alt loosened values
#   * detection_gap_reset_sec : tighter vs looser when tag set switches mid-flight
#   * stable_xy_stddev_m       : default cm-tight vs decimetre/metre tolerant
#   * sliding_window           : 3 vs 5 (smaller reacts faster, larger smooths more)
# Total combinations stay under 80 so a session replays in a few seconds.
GRID = {
    'cost_threshold_per_tag_m': [0.5, 2.0, 5.0, 10.0],
    'detection_gap_reset_sec': [0.6, 2.0, 5.0],
    'stable_xy_stddev_m': [0.08, 0.5, 1.0],
    'sliding_window': [3, 5],
}


def _run_grid(session_dir: str, base: ReplayParams) -> List[ReplayResult]:
    keys = list(GRID.keys())
    values = [GRID[k] for k in keys]
    results: List[ReplayResult] = []
    combos = list(itertools.product(*values))
    print(f'Grid: {len(combos)} combinations on {os.path.basename(session_dir)}')
    t0 = time.time()
    for i, combo in enumerate(combos, 1):
        params = dataclasses.replace(base)
        for k, v in zip(keys, combo):
            setattr(params, k, v)
        # Mirror stable_xy with stable_z so we don't separately sweep it.
        params.stable_z_stddev_m = max(params.stable_z_stddev_m, params.stable_xy_stddev_m * 1.5)
        try:
            r = replay_session(session_dir, params, quiet=True)
        except Exception as exc:
            print(f'  [{i}/{len(combos)}] FAIL {combo}: {exc}')
            continue
        results.append(r)
        print(f'  [{i}/{len(combos)}] '
              f'cost={params.cost_threshold_per_tag_m:>5}  '
              f'gap={params.detection_gap_reset_sec:>4}  '
              f'xy_std={params.stable_xy_stddev_m:>4}  '
              f'sw={params.sliding_window}  '
              f'-> cov={r.coverage:.0%}  '
              f'cov>2m={r.coverage_above(2.0):.0%}  '
              f'cov>5m={r.coverage_above(5.0):.0%}  '
              f'z_rmse>2m={r.z_rmse(2.0):.2f}m  '
              f'max_alt={r.max_alt_with_pose():.1f}m')
    print(f'Grid done in {time.time()-t0:.1f}s, {len(results)} ok')
    return results


def _print_top(results: List[ReplayResult], n: int = 8):
    """Rank by high-altitude coverage, break ties by Z-RMSE."""
    def score(r: ReplayResult):
        # Prefer combinations that produce poses high up (cov>5m), then
        # quality (low Z-RMSE), then overall coverage.
        return (-r.coverage_above(5.0),
                r.z_rmse(2.0) if r.z_rmse(2.0) == r.z_rmse(2.0) else 1e9,
                -r.coverage_above(2.0))

    ranked = sorted(results, key=score)
    print('\nTop combinations (rank by cov>5m, then z_rmse>2m):')
    print(f"  {'cost':>5}  {'gap':>4}  {'xy_std':>6}  {'sw':>2}  "
          f"{'cov':>5}  {'cov>2m':>6}  {'cov>5m':>6}  "
          f"{'z_rmse>2m':>10}  {'max_alt':>8}")
    for r in ranked[:n]:
        p = r.params
        z = r.z_rmse(2.0)
        print(f"  {p.cost_threshold_per_tag_m:>5}  "
              f"{p.detection_gap_reset_sec:>4}  "
              f"{p.stable_xy_stddev_m:>6}  "
              f"{p.sliding_window:>2}  "
              f"{r.coverage:>5.0%}  "
              f"{r.coverage_above(2.0):>6.0%}  "
              f"{r.coverage_above(5.0):>6.0%}  "
              f"{(f'{z:.2f}m' if z == z else '-'):>10}  "
              f"{r.max_alt_with_pose():>7.1f}m")


def _workspace_dir() -> str:
    return os.environ.get('VTOL_WS_ROOT', os.path.expanduser('~/vtol_ws'))


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(prog='localizer_replay')
    parser.add_argument('session_dir', nargs='?', default=None,
                        help='Path to session_*/ (default: most recent in logs/)')
    parser.add_argument('--param', action='append', default=[],
                        help='Override key=value (repeatable). Keys: any ReplayParams field.')
    parser.add_argument('--grid', action='store_true',
                        help='Run the curated parameter grid and rank results.')
    parser.add_argument('--no-stability-gate', action='store_true',
                        help='Skip localizer_node._tracking_is_stable; report tagDB-only coverage.')
    parser.add_argument('--csv', default=None,
                        help='Dump per-frame samples (t,state,n_visible,x,y,z,gps_alt) to this CSV.')
    parser.add_argument('--seed-map', default=None,
                        help='Optional persisted_map yaml to start from (mirrors live persist_map_path).')
    parser.add_argument('--camera', default=None,
                        help='Override camera yaml (default: config/camera.yaml).')
    args = parser.parse_args(argv)

    session_dir = args.session_dir or _autodetect_session(_workspace_dir())
    if not session_dir:
        print('ERROR: no session_dir given and none found in logs/', file=sys.stderr)
        return 2

    base_params = _params_from_overrides(args.param)
    if args.no_stability_gate:
        base_params.apply_stability_gate = False

    if args.grid:
        results = _run_grid(session_dir, base_params)
        if results:
            _print_top(results)
        return 0

    print(f'replay: {os.path.basename(session_dir)}  params={base_params}')
    result = replay_session(session_dir, base_params,
                            seed_map_yaml=args.seed_map,
                            camera_yaml=args.camera)

    if args.csv:
        import csv
        with open(args.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_ns', 'state', 'n_visible', 'pose_valid',
                        'x', 'y', 'z', 'gps_alt'])
            for s in result.samples:
                truth = result._truth_at(s.t_ns)
                w.writerow([s.t_ns, s.state, s.n_visible, int(s.pose_valid),
                            f'{s.x:.4f}' if s.pose_valid else '',
                            f'{s.y:.4f}' if s.pose_valid else '',
                            f'{s.z:.4f}' if s.pose_valid else '',
                            f'{truth:.4f}' if truth is not None else ''])
        print(f'csv: wrote {len(result.samples)} samples to {args.csv}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
