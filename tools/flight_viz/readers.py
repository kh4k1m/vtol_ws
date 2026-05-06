"""Read rosbag2 / CSV / yaml session artifacts produced by flight.launch.py.

Bags can be in mcap or sqlite3 format. Two backends are supported:

* ``rosbags`` (pure Python, no ROS environment needed). Preferred because
  it works inside conda envs without sourcing ROS - the system
  ``rosbag2_py`` typically picks up conda's libsqlite3 and dies with
  "disk I/O error" during ``StorageOptions.open()``. If ``rosbags`` is
  installed (``pip install rosbags``) we use it first.
* ``rosbag2_py`` (system ROS 2 plugin). Used as a fallback when
  ``rosbags`` is unavailable.

Custom message types from this workspace (``marker_interfaces/...``)
are registered into the rosbags typestore at runtime by parsing the
``.msg`` files under ``src/``.
"""

from __future__ import annotations

import glob
import logging
import os
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

import yaml

_LOG = logging.getLogger('flight_viz.readers')


def find_bag_dir(session_dir: str) -> Optional[str]:
    """Return the directory inside session_dir holding bag files, if any."""
    candidate = os.path.join(session_dir, 'bag')
    if os.path.isdir(candidate):
        return candidate
    # Some legacy sessions stored bag at the session root.
    if any(name.endswith(('.mcap', '.db3')) for name in os.listdir(session_dir)):
        return session_dir
    return None


def find_telemetry_csv(session_dir: str) -> Optional[str]:
    matches = sorted(glob.glob(os.path.join(session_dir, 'flight_data_*.csv')))
    return matches[-1] if matches else None


def find_video_files(session_dir: str) -> List[str]:
    video_dir = os.path.join(session_dir, 'video')
    base_dir = video_dir if os.path.isdir(video_dir) else session_dir
    return sorted(glob.glob(os.path.join(base_dir, '*.mp4')))


def load_manifest(session_dir: str) -> Dict:
    path = os.path.join(session_dir, 'manifest.yaml')
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


@dataclass
class TimeSeries:
    """Generic time series. Times are in nanoseconds (ROS time)."""
    name: str
    times_ns: List[int]
    values: List[Dict]

    def is_empty(self) -> bool:
        return not self.times_ns


def _detect_storage_id(bag_dir: str) -> str:
    """Look at metadata.yaml or file extensions to pick rosbag2 plugin."""
    meta_path = os.path.join(bag_dir, 'metadata.yaml')
    if os.path.exists(meta_path):
        with open(meta_path, 'r') as f:
            meta = yaml.safe_load(f) or {}
        sid = meta.get('rosbag2_bagfile_information', {}).get('storage_identifier')
        if sid:
            return str(sid)
    if glob.glob(os.path.join(bag_dir, '*.mcap')):
        return 'mcap'
    if glob.glob(os.path.join(bag_dir, '*.db3')):
        return 'sqlite3'
    return 'mcap'


def _workspace_dir() -> str:
    return os.environ.get('VTOL_WS_ROOT', os.path.expanduser('~/vtol_ws'))


def _custom_msg_packages() -> Dict[str, str]:
    """Return ``{package_name: msg_dir}`` for every workspace package
    that ships .msg files (used to register custom types into rosbags)."""
    src_root = os.path.join(_workspace_dir(), 'src')
    pkgs: Dict[str, str] = {}
    if not os.path.isdir(src_root):
        return pkgs
    for entry in sorted(os.listdir(src_root)):
        msg_dir = os.path.join(src_root, entry, 'msg')
        if os.path.isdir(msg_dir) and glob.glob(os.path.join(msg_dir, '*.msg')):
            pkgs[entry] = msg_dir
    return pkgs


def _read_bag_with_rosbags(
    bag_dir: str, topics: List[str]
) -> Dict[str, TimeSeries]:
    """Read using the pure-Python ``rosbags`` library (no ROS env required)."""
    try:
        from pathlib import Path
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError(
            "rosbags library is not installed. "
            "Run: pip install rosbags"
        ) from exc

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    for pkg_name, msg_dir in _custom_msg_packages().items():
        for msg_file in sorted(glob.glob(os.path.join(msg_dir, '*.msg'))):
            type_name = os.path.splitext(os.path.basename(msg_file))[0]
            full_name = f'{pkg_name}/msg/{type_name}'
            try:
                with open(msg_file, 'r') as f:
                    msg_text = f.read()
                type_defs = typestore.parse_msg(msg_text, full_name)
                typestore.register(type_defs)
            except Exception as exc:
                _LOG.debug('skip register %s: %s', full_name, exc)

    wanted: Optional[set] = set(topics) if topics else None
    out: Dict[str, TimeSeries] = {}

    with AnyReader([Path(bag_dir)], default_typestore=typestore) as reader:
        connections = [
            c for c in reader.connections
            if (wanted is None or c.topic in wanted)
        ]
        if wanted:
            missing = wanted - {c.topic for c in connections}
            if missing:
                _LOG.info(
                    'rosbags: %d requested topic(s) not in bag (e.g. %s)',
                    len(missing), next(iter(missing)),
                )
        for connection, t_ns, raw in reader.messages(connections=connections):
            try:
                msg = reader.deserialize(raw, connection.msgtype)
            except Exception as exc:
                _LOG.debug(
                    'deserialize fail on %s (%s): %s',
                    connection.topic, connection.msgtype, exc,
                )
                continue
            series = out.setdefault(
                connection.topic,
                TimeSeries(connection.topic, [], []),
            )
            series.times_ns.append(int(t_ns))
            series.values.append(msg)

    return out


def _read_bag_with_rosbag2py(
    bag_dir: str, topics: List[str]
) -> Dict[str, TimeSeries]:
    """Read selected topics from a rosbag2 directory using rosbag2_py."""
    try:
        from rosbag2_py import (
            ConverterOptions,
            SequentialReader,
            StorageFilter,
            StorageOptions,
        )
    except ImportError as exc:
        raise RuntimeError(
            'rosbag2_py is not available in this Python environment. '
            'Source your ROS 2 setup.bash before running flight_viz.'
        ) from exc

    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            'rclpy / rosidl_runtime_py are required for message deserialization.'
        ) from exc

    storage_id = _detect_storage_id(bag_dir)
    storage_opts = StorageOptions(uri=bag_dir, storage_id=storage_id)
    converter_opts = ConverterOptions('', '')

    reader = SequentialReader()
    reader.open(storage_opts, converter_opts)

    if topics:
        reader.set_filter(StorageFilter(topics=topics))

    type_map: Dict[str, str] = {
        meta.name: meta.type for meta in reader.get_all_topics_and_types()
    }
    msg_class_cache: Dict[str, type] = {}
    out: Dict[str, TimeSeries] = {}

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        type_name = type_map.get(topic)
        if not type_name:
            continue
        msg_cls = msg_class_cache.get(type_name)
        if msg_cls is None:
            try:
                msg_cls = get_message(type_name)
            except Exception:
                continue
            msg_class_cache[type_name] = msg_cls
        try:
            msg = deserialize_message(raw, msg_cls)
        except Exception:
            continue

        series = out.setdefault(topic, TimeSeries(topic, [], []))
        series.times_ns.append(int(t_ns))
        series.values.append(msg)

    return out


def _is_rosbags_available() -> bool:
    try:
        import rosbags  # noqa: F401
        return True
    except ImportError:
        return False


def _is_rosbag2py_available() -> bool:
    try:
        import rosbag2_py  # noqa: F401
        return True
    except ImportError:
        return False


def read_bag(bag_dir: str, topics: List[str]) -> Dict[str, TimeSeries]:
    """Read selected topics, returning per-topic TimeSeries.

    Tries rosbags first (pure Python, conda-friendly), then rosbag2_py.
    Whichever opens and yields data wins. If both fail, raises with a
    combined diagnostic.
    """
    backends: List[Tuple[str, Callable]] = []
    if _is_rosbags_available():
        backends.append(('rosbags', _read_bag_with_rosbags))
    if _is_rosbag2py_available():
        backends.append(('rosbag2_py', _read_bag_with_rosbag2py))

    if not backends:
        raise RuntimeError(
            "No rosbag2 reader available. Install one of:\n"
            "  pip install rosbags         # pure-Python, recommended\n"
            "  source /opt/ros/humble/setup.bash  # uses system rosbag2_py"
        )

    errors: List[str] = []
    for name, fn in backends:
        try:
            data = fn(bag_dir, topics)
            if data:
                _LOG.info(
                    'flight_viz: read %d topic(s) from %s using %s',
                    len(data), os.path.basename(bag_dir), name,
                )
                return data
            errors.append(f'{name}: opened bag but produced 0 messages')
        except Exception as exc:
            errors.append(f'{name}: {exc}')

    raise RuntimeError(
        'All rosbag backends failed:\n  - ' + '\n  - '.join(errors)
    )


def extract_navsatfix(series: TimeSeries) -> Tuple[List[int], List[float], List[float], List[float]]:
    """Pull (times_ns, lat_deg, lon_deg, alt_m) out of a NavSatFix series."""
    if series.is_empty():
        return [], [], [], []
    lats, lons, alts = [], [], []
    times = []
    for t_ns, msg in zip(series.times_ns, series.values):
        # NavSatFix without a fix sets lat/lon to 0; drop those.
        try:
            lat = float(getattr(msg, 'latitude'))
            lon = float(getattr(msg, 'longitude'))
            alt = float(getattr(msg, 'altitude'))
        except Exception:
            continue
        if lat == 0.0 and lon == 0.0:
            continue
        times.append(int(t_ns))
        lats.append(lat)
        lons.append(lon)
        alts.append(alt)
    return times, lats, lons, alts


def extract_pose_xyz(series: TimeSeries) -> Tuple[List[int], List[float], List[float], List[float]]:
    """Pull (times_ns, x, y, z) out of a PoseStamped series."""
    if series.is_empty():
        return [], [], [], []
    times, xs, ys, zs = [], [], [], []
    for t_ns, msg in zip(series.times_ns, series.values):
        try:
            pose = msg.pose
            xs.append(float(pose.position.x))
            ys.append(float(pose.position.y))
            zs.append(float(pose.position.z))
            times.append(int(t_ns))
        except Exception:
            continue
    return times, xs, ys, zs


def extract_marker_visibility(series: TimeSeries) -> Tuple[List[int], List[int]]:
    """Pull (times_ns, visible_count) out of a MarkerDetectionArray series."""
    if series.is_empty():
        return [], []
    times, counts = [], []
    for t_ns, msg in zip(series.times_ns, series.values):
        try:
            n = len(msg.detections)
        except Exception:
            n = 0
        times.append(int(t_ns))
        counts.append(int(n))
    return times, counts
