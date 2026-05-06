"""Glue between the bag readers and the plot builders.

A loaded ``Session`` exposes named ``Source`` objects (gps, tags,
dpvo, ...). Each source can produce ``(times_ns, xs, ys, zs)`` lists
in either GPS-derived ENU coordinates or local-frame coordinates,
which is exactly what plot builders consume.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import readers
from .geo import lla_array_to_enu


@dataclass
class Source:
    """One labeled trajectory source ready for plotting."""

    name: str
    kind: str            # gps | pose
    times_ns: List[int]
    # For GPS sources we keep the raw lat/lon/alt so we can also convert
    # them to ENU once the origin is known.
    lats: List[float] = field(default_factory=list)
    lons: List[float] = field(default_factory=list)
    alts: List[float] = field(default_factory=list)
    # For pose sources xs/ys/zs are in the publisher's local frame.
    xs: List[float] = field(default_factory=list)
    ys: List[float] = field(default_factory=list)
    zs: List[float] = field(default_factory=list)

    def is_empty(self) -> bool:
        return not self.times_ns

    def as_xyz(self, coords: str, session: 'Session'):
        """Return (times_ns, xs, ys, zs) in the requested frame."""
        if self.kind == 'gps':
            origin = session.origin
            if origin is None:
                # No origin -> pretend lat/lon are already metres (degenerate).
                return list(self.times_ns), list(self.lons), list(self.lats), list(self.alts)
            east, north, up = lla_array_to_enu(
                self.lats, self.lons, self.alts,
                origin[0], origin[1], origin[2],
            )
            return (
                list(self.times_ns),
                list(np.asarray(east).tolist()),
                list(np.asarray(north).tolist()),
                list(np.asarray(up).tolist()),
            )
        return list(self.times_ns), list(self.xs), list(self.ys), list(self.zs)


@dataclass
class Session:
    session_dir: str
    bag_dir: Optional[str]
    manifest: Dict
    origin: Optional[Tuple[float, float, float]]
    t0_ns: int
    sources: Dict[str, Source]
    tag_detections: Optional[Tuple[List[int], List[int]]]


def _origin_from_first_gps(gps: Source) -> Optional[Tuple[float, float, float]]:
    if gps.is_empty():
        return None
    return (gps.lats[0], gps.lons[0], gps.alts[0])


def _topics_for_sources(source_specs):
    topics = set()
    for spec in source_specs:
        kind = str(spec.get('kind', '')).lower()
        if kind == 'gps':
            topics.add(spec.get('topic', '/ap/gps/fix'))
        elif kind == 'pose':
            topics.add(spec.get('topic', '/vision_pose_enu'))
    return sorted(topics)


def load_session(session_dir: str, viz_cfg: Dict) -> Session:
    """Load a session directory and prepare named sources for plotting."""
    bag_dir = readers.find_bag_dir(session_dir)
    manifest = readers.load_manifest(session_dir)

    source_specs: List[Dict] = []
    for plot_cfg in viz_cfg.get('plots', []):
        for src in plot_cfg.get('sources', []) or []:
            if isinstance(src, dict):
                source_specs.append(src)
        # New plot types use `predicted: [...]` (and a separate
        # `reference: <name>` string). Hoist their dict entries into the
        # global source registry so loader can subscribe to their topics.
        predicted = plot_cfg.get('predicted', [])
        if isinstance(predicted, str):
            predicted = [predicted]
        for src in predicted or []:
            if isinstance(src, dict):
                source_specs.append(src)

    source_specs_by_name: Dict[str, Dict] = {}
    for spec in source_specs:
        name = spec.get('name')
        if name and name not in source_specs_by_name:
            source_specs_by_name[name] = spec

    topics = _topics_for_sources(source_specs_by_name.values())
    if any(plot.get('type') == 'marker_visibility' for plot in viz_cfg.get('plots', [])):
        topics.append('/tag_detections')
    topics = sorted(set(topics))

    bag_data: Dict[str, readers.TimeSeries] = {}
    if bag_dir and topics:
        bag_data = readers.read_bag(bag_dir, topics)

    sources: Dict[str, Source] = {}
    for name, spec in source_specs_by_name.items():
        kind = str(spec.get('kind', 'pose')).lower()
        topic = spec.get('topic', '/vision_pose_enu')
        series = bag_data.get(topic)
        if series is None:
            sources[name] = Source(name=name, kind=kind, times_ns=[])
            continue
        if kind == 'gps':
            t, lat, lon, alt = readers.extract_navsatfix(series)
            sources[name] = Source(name=name, kind=kind,
                                   times_ns=t, lats=lat, lons=lon, alts=alt)
        else:
            t, x, y, z = readers.extract_pose_xyz(series)
            sources[name] = Source(name=name, kind=kind,
                                   times_ns=t, xs=x, ys=y, zs=z)

    origin = _resolve_origin(viz_cfg, sources)
    t0_ns = _resolve_t0(sources)

    detections = None
    det_series = bag_data.get('/tag_detections')
    if det_series is not None:
        detections = readers.extract_marker_visibility(det_series)

    return Session(
        session_dir=session_dir,
        bag_dir=bag_dir,
        manifest=manifest,
        origin=origin,
        t0_ns=t0_ns,
        sources=sources,
        tag_detections=detections,
    )


def _resolve_origin(viz_cfg: Dict, sources: Dict[str, Source]):
    cfg_origin = viz_cfg.get('origin', {}) or {}
    lat = cfg_origin.get('lat')
    lon = cfg_origin.get('lon')
    alt = cfg_origin.get('alt')
    if lat is not None and lon is not None:
        return (float(lat), float(lon), float(alt) if alt is not None else 0.0)
    gps = sources.get('gps')
    if gps and not gps.is_empty():
        return _origin_from_first_gps(gps)
    return None


def _resolve_t0(sources: Dict[str, Source]) -> int:
    candidates = [s.times_ns[0] for s in sources.values() if s.times_ns]
    return min(candidates) if candidates else 0


def autodetect_session(log_root: str) -> Optional[str]:
    """Pick the most recent session_<ts> directory under log_root."""
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
