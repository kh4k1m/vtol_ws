"""Command-line entry point: ``python -m flight_viz``."""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import List

import yaml

from .loader import autodetect_session, load_session
from .plots import PLOT_BUILDERS


def _default_workspace() -> str:
    return os.environ.get('VTOL_WS_ROOT', os.path.expanduser('~/vtol_ws'))


def _default_config_path() -> str:
    return os.path.join(_default_workspace(), 'config', 'visualize.yaml')


def _load_yaml(path: str):
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def _parse_args(argv: List[str]):
    parser = argparse.ArgumentParser(
        prog='flight_viz',
        description='Render plots from a vtol_ws flight session.',
    )
    parser.add_argument(
        '--config',
        default=_default_config_path(),
        help='Path to visualize.yaml (default: $VTOL_WS_ROOT/config/visualize.yaml)',
    )
    parser.add_argument(
        '--session',
        default=None,
        help='Override session directory (default: visualize.yaml session_dir, '
             'or autodetect last session under <log_root>).',
    )
    parser.add_argument(
        '--output-dir',
        default=None,
        help='Override output directory.',
    )
    parser.add_argument(
        '--format',
        choices=['html', 'png', 'both'],
        default=None,
        help='Override output_format from the config.',
    )
    return parser.parse_args(argv)


def _resolve_session_dir(viz_cfg, override) -> str:
    if override:
        return os.path.abspath(override)
    cfg_dir = (viz_cfg.get('session_dir') or '').strip()
    if cfg_dir:
        if not os.path.isabs(cfg_dir):
            cfg_dir = os.path.join(_default_workspace(), cfg_dir)
        return cfg_dir
    log_root = os.path.join(_default_workspace(), 'logs')
    detected = autodetect_session(log_root)
    if not detected:
        raise SystemExit(
            f'flight_viz: no session_dir provided and no session_*/ folder found '
            f'under {log_root}. Run a flight first or pass --session.'
        )
    return detected


def _resolve_output_dir(viz_cfg, override, session_dir) -> str:
    target = override or viz_cfg.get('output_dir', 'logs/viz')
    if not os.path.isabs(target):
        target = os.path.join(_default_workspace(), target)
    timestamp = time.strftime('%Y%m%d-%H%M%S')
    session_name = os.path.basename(os.path.normpath(session_dir))
    return os.path.join(target, f'{session_name}__{timestamp}')


def _write_html(figs, names, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    for fig, name in zip(figs, names):
        path = os.path.join(out_dir, f'{name}.html')
        fig.write_html(path, include_plotlyjs='cdn', full_html=True)
    # Also a combined index page for convenience.
    index_path = os.path.join(out_dir, 'index.html')
    sections = []
    for name in names:
        sections.append(
            f'<li><a href="./{name}.html">{name}</a></li>'
        )
    with open(index_path, 'w') as f:
        f.write(
            '<!doctype html><html><head><meta charset="utf-8">'
            '<title>flight_viz</title></head><body>'
            f'<h1>flight_viz output</h1><ul>{"".join(sections)}</ul></body></html>'
        )
    return index_path


def _write_png(figs, names, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    for fig, name in zip(figs, names):
        path = os.path.join(out_dir, f'{name}.png')
        try:
            fig.write_image(path)
        except Exception as exc:
            print(f'flight_viz: PNG export failed for {name}: {exc} '
                  '(install kaleido to enable)', file=sys.stderr)


def main(argv=None) -> int:
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    viz_cfg = _load_yaml(args.config).get('visualize', {})

    session_dir = _resolve_session_dir(viz_cfg, args.session)
    if not os.path.isdir(session_dir):
        raise SystemExit(f'flight_viz: session dir not found: {session_dir}')

    print(f'flight_viz: loading session {session_dir}')
    session = load_session(session_dir, viz_cfg)
    if session.bag_dir is None:
        print('flight_viz: warning - no bag found, plots will be empty.',
              file=sys.stderr)

    print(f'flight_viz: sources loaded ({len(session.sources)}):')
    for name in sorted(session.sources):
        src = session.sources[name]
        spec_topic = ''
        for plot_cfg in viz_cfg.get('plots', []):
            for entry in (plot_cfg.get('sources') or []) + (plot_cfg.get('predicted') or []):
                if isinstance(entry, dict) and entry.get('name') == name:
                    spec_topic = entry.get('topic', '')
                    break
            if spec_topic:
                break
        n = len(src.times_ns)
        marker = 'OK ' if n else '!! '
        print(f'  {marker}{name:14s} kind={src.kind:5s} samples={n:6d} topic={spec_topic}')
    if session.tag_detections is not None:
        print(f'  -- /tag_detections      samples={len(session.tag_detections[0])}')

    figs = []
    names = []
    for i, plot_cfg in enumerate(viz_cfg.get('plots', [])):
        ptype = plot_cfg.get('type')
        builder = PLOT_BUILDERS.get(ptype)
        if builder is None:
            print(f'flight_viz: unknown plot type "{ptype}", skipping.',
                  file=sys.stderr)
            continue
        try:
            fig = builder(session, plot_cfg)
        except Exception as exc:
            print(f'flight_viz: plot {ptype} failed: {exc}', file=sys.stderr)
            continue
        figs.append(fig)
        names.append(f'{i:02d}_{ptype}')

    if not figs:
        print('flight_viz: no plots were rendered.', file=sys.stderr)
        return 1

    fmt = args.format or viz_cfg.get('output_format', 'html')
    out_dir = _resolve_output_dir(viz_cfg, args.output_dir, session_dir)

    if fmt in ('html', 'both'):
        index = _write_html(figs, names, out_dir)
        print(f'flight_viz: wrote {len(figs)} HTML plots -> {index}')
    if fmt in ('png', 'both'):
        _write_png(figs, names, out_dir)
        print(f'flight_viz: wrote PNG plots -> {out_dir}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
