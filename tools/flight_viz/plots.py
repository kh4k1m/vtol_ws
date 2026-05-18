"""Plotly figure builders.

Each ``make_*`` function takes a normalized session dict (output of
``loader.load_session``) plus a single ``plot_cfg`` block from
visualize.yaml and returns a ``plotly.graph_objects.Figure``.
"""

from __future__ import annotations

from typing import Dict, List

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from .classify import (
    COLOR_BAD,
    COLOR_OK,
    COLOR_REFERENCE,
    colorize_by_tolerance,
    nearest_match_indices,
    visibility_colors,
)


_DEFAULT_TOLERANCE_NS = int(0.5 * 1e9)   # 500 ms time-match window


def _times_seconds(times_ns: List[int], t0_ns: int) -> List[float]:
    return [(t - t0_ns) / 1e9 for t in times_ns]


def _resolve_source(session, name):
    src = session.sources.get(name)
    if src is None:
        raise KeyError(f"unknown source name '{name}' in plot config")
    return src


def _alignment_offset(src_times_ns: List[int],
                      src_vals: List[float],
                      ref_times_ns: List[int],
                      ref_vals: List[float]) -> float:
    """Single-axis translation so that ``src[0]`` coincides with ``ref``
    at the time of the first src sample.

    Returns the delta to ADD to every src value. This is "honest"
    alignment - it's a rigid translation, not min-max scaling.
    Drift / shape mismatch after t0 stays visible.
    """
    if not src_times_ns or not ref_times_ns:
        return 0.0
    t0 = src_times_ns[0]
    arr = np.asarray(ref_times_ns, dtype=np.int64)
    idx = int(np.argmin(np.abs(arr - t0)))
    return float(ref_vals[idx]) - float(src_vals[0])


def _interp_at(src_times_ns: List[int],
               ref_times_ns: List[int],
               ref_vals: List[float]) -> List[float]:
    """Linearly interpolate ``ref_vals`` at every ``src_times_ns``.
    Outside ref's range, np.interp clamps - good enough for residual plots.
    """
    if not src_times_ns or not ref_times_ns:
        return []
    src_arr = np.asarray(src_times_ns, dtype=np.float64)
    ref_arr = np.asarray(ref_times_ns, dtype=np.float64)
    val_arr = np.asarray(ref_vals, dtype=np.float64)
    return np.interp(src_arr, ref_arr, val_arr).tolist()


def _compute_ate_xy(
    pred_times_ns: List[int],
    pred_xs: List[float],
    pred_ys: List[float],
    ref_times_ns: List[int],
    ref_xs: List[float],
    ref_ys: List[float],
) -> Dict[str, float]:
    """Absolute Trajectory Error in XY after time-alignment.

    ATE here is the RMSE of the horizontal residual between the
    predicted trajectory and the reference, where the reference is
    linearly interpolated at every predicted timestamp. Only samples
    whose timestamp falls inside the reference's time span (i.e. not
    clamped by np.interp) are counted - this prevents flat tails before
    the first GPS fix or after the last one from polluting the metric.

    Returns ``{'ate_rmse_m', 'ate_mean_m', 'ate_max_m', 'count'}``.
    Empty fields are ``nan`` and ``count == 0`` when there are no
    overlapping samples.
    """
    nan = float('nan')
    empty = {
        'ate_rmse_m': nan, 'ate_mean_m': nan,
        'ate_max_m': nan, 'count': 0,
    }
    if not pred_times_ns or not ref_times_ns:
        return empty

    pred_t = np.asarray(pred_times_ns, dtype=np.float64)
    ref_t = np.asarray(ref_times_ns, dtype=np.float64)

    mask = (pred_t >= ref_t[0]) & (pred_t <= ref_t[-1])
    if not np.any(mask):
        return empty

    pred_x = np.asarray(pred_xs, dtype=np.float64)[mask]
    pred_y = np.asarray(pred_ys, dtype=np.float64)[mask]
    pred_t_m = pred_t[mask]

    ref_x_arr = np.asarray(ref_xs, dtype=np.float64)
    ref_y_arr = np.asarray(ref_ys, dtype=np.float64)
    ix = np.interp(pred_t_m, ref_t, ref_x_arr)
    iy = np.interp(pred_t_m, ref_t, ref_y_arr)

    dx = pred_x - ix
    dy = pred_y - iy
    d = np.hypot(dx, dy)
    return {
        'ate_rmse_m': float(np.sqrt(np.mean(d * d))),
        'ate_mean_m': float(np.mean(d)),
        'ate_max_m': float(np.max(d)),
        'count': int(d.size),
    }


def _normalize_predicted_specs(plot_cfg) -> List[Dict]:
    """Return the predicted-source list of dicts. Accepts either a list of
    strings (names) or a list of dicts (full specs)."""
    out: List[Dict] = []
    raw = plot_cfg.get('predicted', [])
    if isinstance(raw, str):
        raw = [raw]
    for entry in raw:
        if isinstance(entry, str):
            out.append({'name': entry})
        elif isinstance(entry, dict):
            out.append(dict(entry))
    return out


def make_trajectory_xy(session, plot_cfg) -> go.Figure:
    coords = str(plot_cfg.get('coords', 'gps')).lower()
    title = plot_cfg.get('title', 'XY trajectory')

    fig = go.Figure()
    fig.update_layout(
        title=title,
        xaxis_title='East [m]' if coords == 'gps' else 'X [m]',
        yaxis_title='North [m]' if coords == 'gps' else 'Y [m]',
        yaxis=dict(scaleanchor='x', scaleratio=1),
        legend=dict(orientation='h'),
    )

    sources = plot_cfg.get('sources', [])
    for src_cfg in sources:
        name = src_cfg['name']
        src = _resolve_source(session, name)
        if src.is_empty():
            continue
        times_ns, xs, ys, _zs = src.as_xyz(coords=coords, session=session)
        if not xs:
            continue

        ref_name = src_cfg.get('reference')
        if ref_name and ref_name != name:
            ref = _resolve_source(session, ref_name)
            ref_times_ns, rxs, rys, rzs = ref.as_xyz(coords=coords, session=session)
            tolerance_m = float(src_cfg.get('tolerance_m', 1.0))
            nearest = nearest_match_indices(times_ns, ref_times_ns, _DEFAULT_TOLERANCE_NS)
            colors, _err = colorize_by_tolerance(
                list(zip(xs, ys, [0.0] * len(xs))),
                list(zip(rxs, rys, rzs)),
                nearest,
                tolerance_m,
                horizontal_only=True,
            )
            fig.add_trace(go.Scatter(
                x=xs, y=ys, mode='markers',
                marker=dict(color=colors, size=4),
                name=f'{name} (vs {ref_name}, tol {tolerance_m} m)',
            ))
        else:
            color = src_cfg.get('color', COLOR_REFERENCE if name == 'gps' else None)
            fig.add_trace(go.Scatter(
                x=xs, y=ys, mode='lines+markers',
                line=dict(color=color, width=2),
                marker=dict(size=3),
                name=src_cfg.get('label', name),
            ))
    return fig


def make_altitude_time(session, plot_cfg) -> go.Figure:
    title = plot_cfg.get('title', 'Altitude over time')
    fig = go.Figure()
    fig.update_layout(
        title=title,
        xaxis_title='time [s]',
        yaxis_title='Z [m]',
        legend=dict(orientation='h'),
    )

    t0_ns = session.t0_ns
    sources = plot_cfg.get('sources', [])
    for src_name in sources:
        if isinstance(src_name, dict):
            cfg = src_name
            name = cfg['name']
        else:
            cfg = {'name': src_name}
            name = src_name
        src = _resolve_source(session, name)
        if src.is_empty():
            continue
        times_ns, _xs, _ys, zs = src.as_xyz(coords='gps', session=session)
        if not zs:
            continue
        ts = _times_seconds(times_ns, t0_ns)
        if name == 'gps':
            color = COLOR_REFERENCE
        else:
            color = cfg.get('color', '#ff7f0e')
        fig.add_trace(go.Scatter(
            x=ts, y=zs, mode='lines',
            line=dict(color=color, width=2),
            name=cfg.get('label', name),
        ))
    return fig


def make_xyz_separately(session, plot_cfg) -> go.Figure:
    title = plot_cfg.get('title', 'X(t), Y(t), Z(t)')
    fig = make_subplots(rows=3, cols=1, shared_xaxes=True,
                        subplot_titles=('X (East) [m]', 'Y (North) [m]', 'Z (Up) [m]'))
    fig.update_layout(title=title, legend=dict(orientation='h'))
    fig.update_xaxes(title_text='time [s]', row=3, col=1)

    sources = plot_cfg.get('sources', [])
    t0_ns = session.t0_ns
    for src_name in sources:
        if isinstance(src_name, dict):
            cfg = src_name
            name = cfg['name']
        else:
            cfg = {'name': src_name}
            name = src_name
        src = _resolve_source(session, name)
        if src.is_empty():
            continue
        times_ns, xs, ys, zs = src.as_xyz(coords='gps', session=session)
        ts = _times_seconds(times_ns, t0_ns)
        color = COLOR_REFERENCE if name == 'gps' else cfg.get('color', '#ff7f0e')
        for row, vals in enumerate((xs, ys, zs), start=1):
            fig.add_trace(
                go.Scatter(
                    x=ts, y=vals, mode='lines',
                    line=dict(color=color, width=1.5),
                    name=cfg.get('label', name),
                    showlegend=(row == 1),
                    legendgroup=name,
                ),
                row=row, col=1,
            )
    return fig


def make_altitude_visibility(session, plot_cfg) -> go.Figure:
    """Altitude(t) coloured by AprilTag visibility (same scheme as
    ``marker_visibility``: green = tag seen within ``visibility_gap_sec``,
    red = not seen)."""
    title = plot_cfg.get('title', 'Altitude over time (coloured by tag visibility)')
    fig = go.Figure()
    fig.update_layout(
        title=title,
        xaxis_title='time [s]',
        yaxis_title='Z [m]',
        legend=dict(orientation='h'),
    )

    visibility_gap_sec = float(plot_cfg.get('visibility_gap_sec', 0.5))
    visibility_gap_ns = int(visibility_gap_sec * 1e9)
    show_reference_line = bool(plot_cfg.get('show_reference_line', True))

    detections = session.tag_detections
    t0_ns = session.t0_ns
    sources = plot_cfg.get('sources', ['gps'])

    for src_name in sources:
        if isinstance(src_name, dict):
            cfg = src_name
            name = cfg['name']
        else:
            cfg = {'name': src_name}
            name = src_name
        src = _resolve_source(session, name)
        if src.is_empty():
            continue
        times_ns, _xs, _ys, zs = src.as_xyz(coords='gps', session=session)
        if not zs:
            continue
        ts = _times_seconds(times_ns, t0_ns)

        if show_reference_line:
            fig.add_trace(go.Scatter(
                x=ts, y=zs, mode='lines',
                line=dict(color='lightgray', width=1),
                name=f'{name} (reference)',
                hoverinfo='skip',
                showlegend=False,
            ))

        if detections is not None and detections[0]:
            colors = visibility_colors(
                times_ns, detections[0], detections[1], visibility_gap_ns,
            )
        else:
            colors = [COLOR_BAD] * len(zs)

        fig.add_trace(go.Scatter(
            x=ts, y=zs, mode='markers',
            marker=dict(color=colors, size=4),
            name=f'{name} (green = tag visible, red = lost)',
        ))

    fig.add_trace(go.Scatter(x=[None], y=[None], mode='markers',
                             marker=dict(color=COLOR_OK, size=10),
                             name='tag visible'))
    fig.add_trace(go.Scatter(x=[None], y=[None], mode='markers',
                             marker=dict(color=COLOR_BAD, size=10),
                             name='tag lost'))
    return fig


def make_trajectory_xy_aligned(session, plot_cfg) -> go.Figure:
    """XY trajectory with predicted sources rigidly translated in XY so
    each predicted track coincides with the truth at its first sample.

    The translation is XY-only (no rotation, no scaling, no Z), so:
    * GPS gives the spatial truth in real metres.
    * Predicted shows real-metre divergence vs GPS over the flight.
    * Initial-position offset between map-frame and GPS-ENU is removed,
      so XY shape comparison is meaningful regardless of where the
      marker map's origin sits in the world.

    Per-source ``align_xy: false`` suppresses the translation (raw
    overlay). The applied delta is shown in the legend.

    For every predicted source we also compute the **Absolute
    Trajectory Error** (ATE) - horizontal RMSE between the post-shift
    predicted track and the reference, time-aligned by linear
    interpolation. ATE values are surfaced both in the per-source
    legend label and in a single subtitle line above the plot, so a
    glance at the figure tells the operator how much each visual
    backend drifted relative to GPS truth during the flight.

    ``relative_origin: false`` re-centers both trajectories so the
    reference's first sample sits at (0, 0). Defaults to ``true``
    because GPS-ENU is already anchored at the first GPS fix, but
    flipping it off matches the ``coords: local`` case.
    """
    ref_name = str(plot_cfg.get('reference', 'gps'))
    coords = str(plot_cfg.get('coords', 'gps')).lower()
    title = plot_cfg.get('title', f'XY trajectory aligned to {ref_name}')
    relative_origin = bool(plot_cfg.get('relative_origin', True))

    ref = _resolve_source(session, ref_name)
    fig = go.Figure()
    fig.update_layout(
        title=title,
        xaxis_title='East [m]' if coords == 'gps' else 'X [m]',
        yaxis_title='North [m]' if coords == 'gps' else 'Y [m]',
        yaxis=dict(scaleanchor='x', scaleratio=1),
        legend=dict(orientation='h'),
    )

    if ref.is_empty():
        fig.add_annotation(
            text=f'reference source "{ref_name}" is empty in this session',
            showarrow=False, font=dict(color=COLOR_BAD, size=14),
        )
        return fig

    ref_t, rxs, rys, _rzs = ref.as_xyz(coords=coords, session=session)

    # Optionally drop both trajectories to a (0, 0) starting point so
    # the plot reads as "horizontal divergence from the takeoff point"
    # rather than "from the GPS anchor". GPS-ENU is already anchored at
    # the first fix, so for the default visualize.yaml this is a no-op,
    # but it makes the local-coords variant comparable.
    if relative_origin and rxs and rys:
        ox, oy = rxs[0], rys[0]
        rxs = [x - ox for x in rxs]
        rys = [y - oy for y in rys]

    fig.add_trace(go.Scatter(
        x=rxs, y=rys, mode='lines',
        line=dict(color=COLOR_REFERENCE, width=2),
        name=f'{ref_name} (truth)',
    ))

    ate_lines: List[str] = []

    for spec in _normalize_predicted_specs(plot_cfg):
        name = spec.get('name')
        if not name or name == ref_name:
            continue
        try:
            src = _resolve_source(session, name)
        except KeyError:
            continue
        if src.is_empty():
            continue
        st, sxs, sys, _szs = src.as_xyz(coords=coords, session=session)
        if not sxs:
            continue
        if bool(spec.get('align_xy', True)):
            # Use the un-shifted reference for the alignment offset
            # because that is what makes the predicted's first sample
            # land on the reference at the same wall time. The 0,0
            # shift below moves both traces together.
            _ref_t_for_align, _rxs_raw, _rys_raw, _ = \
                ref.as_xyz(coords=coords, session=session)
            dx = _alignment_offset(st, sxs, _ref_t_for_align, _rxs_raw)
            dy = _alignment_offset(st, sys, _ref_t_for_align, _rys_raw)
            sxs = [x + dx for x in sxs]
            sys = [y + dy for y in sys]
            align_note = f"XY anchored at t0: Δ=({dx:+.2f}, {dy:+.2f}) m"
        else:
            align_note = "raw"

        # ATE is computed against the un-shifted reference so the
        # number reflects real-world divergence, not the cosmetic
        # 0,0 re-centering applied to the plotted lines.
        _ref_t_for_ate, _rxs_for_ate, _rys_for_ate, _ = \
            ref.as_xyz(coords=coords, session=session)
        ate = _compute_ate_xy(
            st, sxs, sys, _ref_t_for_ate, _rxs_for_ate, _rys_for_ate,
        )

        if relative_origin and rxs and rys:
            # ``rxs[0]`` is now 0 because we shifted; recover the
            # original origin to apply the same shift to ``sxs/sys``.
            _ref_t_for_origin, _rxs_origin, _rys_origin, _ = \
                ref.as_xyz(coords=coords, session=session)
            sxs = [x - _rxs_origin[0] for x in sxs]
            sys = [y - _rys_origin[0] for y in sys]

        if ate['count'] > 0:
            ate_str = f"ATE={ate['ate_rmse_m']:.2f} m"
            ate_lines.append(
                f"{spec.get('label', name)}: ATE(RMSE)={ate['ate_rmse_m']:.2f} m "
                f"| mean={ate['ate_mean_m']:.2f} | max={ate['ate_max_m']:.2f} "
                f"(n={ate['count']})"
            )
        else:
            ate_str = "ATE=n/a"
            ate_lines.append(
                f"{spec.get('label', name)}: ATE=n/a (no time overlap with {ref_name})"
            )

        label = f"{spec.get('label', name)} ({align_note}, {ate_str})"

        fig.add_trace(go.Scatter(
            x=sxs, y=sys, mode='lines+markers',
            line=dict(color=spec.get('color', '#ff7f0e'), width=1.5),
            marker=dict(size=3),
            name=label,
        ))

    if ate_lines:
        # Subtitle annotation: one line per predicted source. Sits just
        # below the main title so the operator sees ATE numbers without
        # having to mouse over legend entries.
        subtitle = '<br>'.join(ate_lines)
        fig.update_layout(
            title=dict(
                text=f"{title}<br>"
                     f"<span style='font-size:12px;color:#666'>{subtitle}</span>",
                x=0.5,
                xanchor='center',
            ),
        )

    return fig


def make_altitude_compare(session, plot_cfg) -> go.Figure:
    """Z(t) for truth vs predicted on a SHARED Z axis in real metres.

    By default applies NO Z translation (so a constant bias like
    "GPS=35 m, tags=15 m" is honestly visible). Set per-source
    ``align_z: true`` to translate that source's Z so it matches the
    truth at t0 (useful when the map-frame z origin really is offset).

    Optional secondary y-axis with the residual ``predicted - truth``
    is rendered when ``show_error: true`` (default).
    """
    ref_name = str(plot_cfg.get('reference', 'gps'))
    title = plot_cfg.get('title', f'Z(t): {ref_name} vs predicted')
    show_error = bool(plot_cfg.get('show_error', True))

    ref = _resolve_source(session, ref_name)
    if show_error:
        fig = make_subplots(specs=[[{'secondary_y': True}]])
    else:
        fig = go.Figure()
    fig.update_layout(
        title=title,
        xaxis_title='time [s]',
        legend=dict(orientation='h'),
    )

    if ref.is_empty():
        fig.add_annotation(
            text=f'reference source "{ref_name}" is empty in this session',
            showarrow=False, font=dict(color=COLOR_BAD, size=14),
        )
        return fig

    t0_ns = session.t0_ns
    ref_t, _rxs, _rys, rzs = ref.as_xyz(coords='gps', session=session)
    ref_ts = _times_seconds(ref_t, t0_ns)

    if show_error:
        fig.add_trace(
            go.Scatter(x=ref_ts, y=rzs, mode='lines',
                       line=dict(color=COLOR_REFERENCE, width=2),
                       name=f'{ref_name} Z (truth)'),
            secondary_y=False,
        )
        fig.update_yaxes(title_text='Z [m]', secondary_y=False)
        fig.update_yaxes(title_text='Z error (predicted - truth) [m]',
                         secondary_y=True)
    else:
        fig.add_trace(go.Scatter(
            x=ref_ts, y=rzs, mode='lines',
            line=dict(color=COLOR_REFERENCE, width=2),
            name=f'{ref_name} Z (truth)',
        ))
        fig.update_yaxes(title_text='Z [m]')

    for spec in _normalize_predicted_specs(plot_cfg):
        name = spec.get('name')
        if not name or name == ref_name:
            continue
        try:
            src = _resolve_source(session, name)
        except KeyError:
            continue
        if src.is_empty():
            continue
        st, _xs, _ys, szs = src.as_xyz(coords='gps', session=session)
        if not szs:
            continue
        sts = _times_seconds(st, t0_ns)

        label_extra = ''
        if bool(spec.get('align_z', False)):
            dz = _alignment_offset(st, szs, ref_t, rzs)
            szs = [z + dz for z in szs]
            label_extra = f' (Z aligned, Δ={dz:+.2f} m)'

        color = spec.get('color', '#ff7f0e')
        if show_error:
            fig.add_trace(
                go.Scatter(x=sts, y=szs, mode='lines',
                           line=dict(color=color, width=2),
                           name=f"{spec.get('label', name)}{label_extra}"),
                secondary_y=False,
            )
            ref_z_at_src = _interp_at(st, ref_t, rzs)
            if ref_z_at_src:
                err = (np.asarray(szs, dtype=float)
                       - np.asarray(ref_z_at_src, dtype=float)).tolist()
                fig.add_trace(
                    go.Scatter(x=sts, y=err, mode='lines',
                               line=dict(color=color, width=1, dash='dot'),
                               name=f'{name} error (Δ to {ref_name})',
                               legendgroup=name, showlegend=True),
                    secondary_y=True,
                )
        else:
            fig.add_trace(go.Scatter(
                x=sts, y=szs, mode='lines',
                line=dict(color=color, width=2),
                name=f"{spec.get('label', name)}{label_extra}",
            ))

    if show_error:
        fig.add_hline(y=0, line=dict(color='lightgray', width=1, dash='dash'),
                      secondary_y=True)
    return fig


def make_marker_visibility(session, plot_cfg) -> go.Figure:
    title = plot_cfg.get('title', 'Marker visibility')
    fig = go.Figure()
    fig.update_layout(
        title=title,
        xaxis_title='East [m]',
        yaxis_title='North [m]',
        yaxis=dict(scaleanchor='x', scaleratio=1),
        legend=dict(orientation='h'),
    )

    visibility_gap_sec = float(plot_cfg.get('visibility_gap_sec', 0.5))
    visibility_gap_ns = int(visibility_gap_sec * 1e9)

    sources = plot_cfg.get('sources', ['gps'])
    detections = session.tag_detections
    for src_name in sources:
        if isinstance(src_name, dict):
            name = src_name['name']
        else:
            name = src_name
        src = _resolve_source(session, name)
        if src.is_empty():
            continue
        times_ns, xs, ys, _zs = src.as_xyz(coords='gps', session=session)
        if detections is not None and detections[0]:
            colors = visibility_colors(times_ns, detections[0], detections[1], visibility_gap_ns)
        else:
            colors = [COLOR_BAD] * len(xs)
        fig.add_trace(go.Scatter(
            x=xs, y=ys, mode='markers',
            marker=dict(color=colors, size=4),
            name=f'{name} (green = tag visible, red = lost)',
        ))
    # Hint legend with sentinel traces.
    fig.add_trace(go.Scatter(x=[None], y=[None], mode='markers',
                             marker=dict(color=COLOR_OK, size=10),
                             name='tag visible'))
    fig.add_trace(go.Scatter(x=[None], y=[None], mode='markers',
                             marker=dict(color=COLOR_BAD, size=10),
                             name='tag lost'))
    return fig


PLOT_BUILDERS: Dict[str, callable] = {
    'trajectory_xy': make_trajectory_xy,
    'trajectory_xy_aligned': make_trajectory_xy_aligned,
    'altitude_time': make_altitude_time,
    'altitude_visibility': make_altitude_visibility,
    'altitude_compare': make_altitude_compare,
    'xyz_separately': make_xyz_separately,
    'marker_visibility': make_marker_visibility,
}
