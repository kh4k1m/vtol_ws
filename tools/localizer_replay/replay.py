"""Offline replay of tag_localizer logic against a recorded session.

The replay deliberately mirrors what `tag_localizer/localizer_node.py`
does inside `det_cb`, but without rclpy or any ROS process - so a single
parameter combination runs in well under a second on a typical session.
That makes grid-search over `tagDB` tunables practical.

Inputs (read from the bag):
  * /tag_detections     - per-frame detections (used as input)
  * /ap/relative_alt    - GPS-derived height ASL relative to home (truth Z)
  * /tf_static / cam_trans/rot - taken from config/camera.yaml instead of
                                 the bag for simplicity (camera is rigid)

Outputs (returned by `replay_session`):
  * per-frame predicted (x,y,z) in map frame
  * per-frame state code (TRACKING / SETTLING / INITIALIZING / REJECTED)
  * coverage and Z-RMSE versus ground-truth altitude

Coordinate convention: tag_localizer publishes pose in ENU map frame, so
predicted Z is altitude above the map origin; we compare against
``/ap/relative_alt`` (height above takeoff), which is approximately the
same shifted by the constant camera->ground offset captured at t=0.
"""

from __future__ import annotations

import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R


def _workspace_dir() -> str:
    return os.environ.get('VTOL_WS_ROOT', os.path.expanduser('~/vtol_ws'))


def _ensure_tag_localizer_on_path() -> None:
    """Make ``tag_localizer.geo`` importable without sourcing ROS install/.

    The python source lives under src/, but it is not pip-installed in
    the conda env we run from. Inject the package directory into
    ``sys.path`` lazily.
    """
    pkg_root = os.path.join(_workspace_dir(), 'src', 'tag_localizer')
    if pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)


_ensure_tag_localizer_on_path()
from tag_localizer.geo import tagDB  # noqa: E402


# ---------------------------------------------------------------------------
# config loading
# ---------------------------------------------------------------------------

def _load_yaml(path: str) -> Dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def _load_camera_offset(camera_yaml: str) -> Tuple[List[float], List[float]]:
    cfg = _load_yaml(camera_yaml)
    co = cfg.get('camera_offset', {}) or {}
    trans = co.get('translation', [0.0, 0.0, -0.18])
    rot = co.get('rotation', [180.0, 0.0, -90.0])
    return list(map(float, trans)), list(map(float, rot))


def _load_persisted_map(map_yaml: Optional[str]) -> Dict[int, np.ndarray]:
    """Return ``{tag_id: T_map_tag (4x4)}`` from a discovered_map.yaml-style file.

    Returns an empty dict when ``map_yaml`` is None, missing or empty.
    The replay then mirrors live behavior: anchor is bootstrapped from
    the first detection inside ``replay_session``.
    """
    if not map_yaml or not os.path.exists(map_yaml):
        return {}
    cfg = _load_yaml(map_yaml)
    raw = cfg.get('markers', [])
    out: Dict[int, np.ndarray] = {}
    for entry in raw:
        pose = list(map(float, entry.get('pose', [])))
        if len(pose) != 6:
            continue
        T = np.eye(4)
        T[0:3, 3] = pose[:3]
        T[0:3, 0:3] = R.from_euler('xyz', pose[3:], degrees=False).as_matrix()
        out[int(entry['id'])] = T
    return out


def _build_T_base_camera(trans: List[float], rot_deg: List[float]) -> np.ndarray:
    T = np.eye(4)
    T[0:3, 3] = trans
    T[0:3, 0:3] = R.from_euler('xyz', rot_deg, degrees=True).as_matrix()
    return T


# ---------------------------------------------------------------------------
# bag reading (delegates to flight_viz.readers, which already handles
# custom message registration and rosbags-vs-rosbag2_py fallback)
# ---------------------------------------------------------------------------

def _load_session_streams(session_dir: str):
    """Return ``(detections, alts, attitudes, gps)`` time series.

    Each list entry is ``(t_ns, value)``; ``value`` is the raw bag
    message or a primitive depending on the topic. Topics absent in
    the bag yield empty lists - the caller decides the consequence
    (e.g. fall back to first_seen anchor).
    """
    sys.path.insert(0, os.path.join(_workspace_dir(), 'tools'))
    from flight_viz import readers  # noqa: E402

    bag_dir = readers.find_bag_dir(session_dir)
    if not bag_dir:
        raise FileNotFoundError(f'No bag found under {session_dir}')

    topics = [
        '/tag_detections', '/ap/relative_alt', '/ap/gps/fix',
        '/ap/attitude', '/ap/imu/data',
    ]
    bag_data = readers.read_bag(bag_dir, topics)

    def _series(name):
        s = bag_data.get(name)
        if s is None:
            return []
        return list(zip([int(t) for t in s.times_ns], s.values))

    detections = _series('/tag_detections')
    alts_raw = _series('/ap/relative_alt')
    gps = _series('/ap/gps/fix')
    attitudes = _series('/ap/attitude')

    alts: List[Tuple[int, float]] = []
    for t_ns, msg in alts_raw:
        try:
            alts.append((t_ns, float(msg.data)))
        except Exception:
            continue

    return detections, alts, attitudes, gps


# ---------------------------------------------------------------------------
# Core replay loop
# ---------------------------------------------------------------------------

@dataclass
class ReplayParams:
    """Subset of localizer params relevant to grid search."""
    pose_solver: str = 'absolute'        # 'differential' | 'absolute'
    absolute_max_per_tag_spread_m: float = 3.0
    sliding_window: int = 15
    cost_threshold_per_tag_m: float = 0.5
    z_score_threshold: float = 3.0
    kalman_R_pos_m: float = 0.06
    kalman_Q_pos: float = 0.01
    kalman_Q_vel: float = 0.1
    kalman_Q_accel: float = 0.05
    detection_gap_reset_sec: float = 0.6
    # Stability gate (mirrors localizer_node._tracking_is_stable).
    stable_window_size: int = 10
    stable_xy_stddev_m: float = 1.0
    stable_z_stddev_m: float = 2.0
    # If False the stability gate is skipped (we accept the tagDB pose
    # as soon as it converges). Useful for upper-bound coverage.
    apply_stability_gate: bool = True
    # Anchor bootstrap (matches live tag_localizer behaviour).
    anchor_strategy: str = 'first_seen_with_gps'   # | 'first_seen'
    anchor_marker_id: int = -1
    learn_unknown_tags: bool = True
    # If first_seen_with_gps and the bag has no /ap/attitude in the
    # first N detection frames, silently fall back to first_seen so old
    # bags (recorded before the bridge published ATTITUDE) still replay.
    # 0 = never fall back (replay reports WAITING_FOR_ANCHOR forever).
    anchor_attitude_wait_frames: int = 30


@dataclass
class ReplaySample:
    t_ns: int
    state: str
    n_visible: int
    pose_valid: bool
    x: float = math.nan
    y: float = math.nan
    z: float = math.nan
    cost: float = math.nan
    drop_reason: str = ''


@dataclass
class ReplayResult:
    samples: List[ReplaySample] = field(default_factory=list)
    alt_truth: List[Tuple[int, float]] = field(default_factory=list)
    params: ReplayParams = field(default_factory=ReplayParams)

    # ---- aggregate metrics -------------------------------------------------
    @property
    def n_frames_with_detections(self) -> int:
        return sum(1 for s in self.samples if s.n_visible > 0)

    @property
    def n_poses_published(self) -> int:
        return sum(1 for s in self.samples if s.pose_valid)

    @property
    def coverage(self) -> float:
        n = self.n_frames_with_detections
        return self.n_poses_published / n if n else 0.0

    def coverage_above(self, alt_min_m: float) -> float:
        """Coverage restricted to frames where GPS truth alt > alt_min_m."""
        if not self.alt_truth or not self.samples:
            return 0.0
        ok, total = 0, 0
        for s in self.samples:
            if s.n_visible == 0:
                continue
            truth = self._truth_at(s.t_ns)
            if truth is None or truth <= alt_min_m:
                continue
            total += 1
            if s.pose_valid:
                ok += 1
        return ok / total if total else 0.0

    def z_rmse(self, alt_min_m: float = 0.0) -> float:
        if not self.alt_truth:
            return math.nan
        sq, n = 0.0, 0
        for s in self.samples:
            if not s.pose_valid:
                continue
            truth = self._truth_at(s.t_ns)
            if truth is None or truth < alt_min_m:
                continue
            sq += (s.z - truth) ** 2
            n += 1
        return math.sqrt(sq / n) if n else math.nan

    def max_alt_with_pose(self) -> float:
        best = 0.0
        for s in self.samples:
            if not s.pose_valid:
                continue
            truth = self._truth_at(s.t_ns)
            if truth is None:
                continue
            if truth > best:
                best = truth
        return best

    def _truth_at(self, t_ns: int) -> Optional[float]:
        """Linear interp of /ap/relative_alt at t_ns. Returns None if out of range."""
        if not self.alt_truth:
            return None
        ts = self.alt_truth
        # binary search
        lo, hi = 0, len(ts) - 1
        if t_ns <= ts[0][0]:
            return ts[0][1]
        if t_ns >= ts[-1][0]:
            return ts[-1][1]
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if ts[mid][0] <= t_ns:
                lo = mid
            else:
                hi = mid
        t1, v1 = ts[lo]
        t2, v2 = ts[hi]
        f = (t_ns - t1) / max(1, (t2 - t1))
        return v1 + f * (v2 - v1)


class _DetTagAdapter:
    """Adapter to mirror the DetectedTag the live node passes to tag_db."""
    __slots__ = ('tag_id', 'pose_R', 'pose_t')

    def __init__(self, tag_id: int, pose_R: np.ndarray, pose_t: np.ndarray):
        self.tag_id = tag_id
        self.pose_R = pose_R
        self.pose_t = pose_t


def _pose_to_matrix(pose) -> Optional[np.ndarray]:
    qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-6:
        return None
    T = np.eye(4)
    T[0, 3] = pose.position.x
    T[1, 3] = pose.position.y
    T[2, 3] = pose.position.z
    T[0:3, 0:3] = R.from_quat([qx, qy, qz, qw]).as_matrix()
    return T


def _bootstrap_anchor(
    db: tagDB,
    msg,
    T_base_camera: np.ndarray,
    *,
    strategy: str,
    anchor_marker_id: int,
    latest_attitude_yaw_ned: Optional[float],
) -> bool:
    """Mirror localizer_node._maybe_bootstrap_anchor for offline replay.

    Returns True iff the anchor was set on this call.
    """
    candidate = None
    for det in msg.detections:
        if math.isnan(det.pose_camera_marker.position.x):
            continue
        if anchor_marker_id >= 0 and int(det.id) != anchor_marker_id:
            continue
        T_cm = _pose_to_matrix(det.pose_camera_marker)
        if T_cm is None:
            continue
        candidate = (int(det.id), T_cm)
        break
    if candidate is None:
        return False

    tag_id, T_cam_mark = candidate
    T_base_mark = T_base_camera @ T_cam_mark

    if strategy == 'first_seen_with_gps' and latest_attitude_yaw_ned is None:
        # Replay caller decides fallback policy; here just report failure.
        return False

    if strategy == 'first_seen_with_gps':
        yaw_enu = (math.pi * 0.5) - float(latest_attitude_yaw_ned)
        R_world_base = R.from_euler('z', yaw_enu).as_matrix()
        p_world_base = -R_world_base @ T_base_mark[0:3, 3]
        T_world_base = np.eye(4)
        T_world_base[0:3, 0:3] = R_world_base
        T_world_base[0:3, 3] = p_world_base
        T_world_mark = T_world_base @ T_base_mark
    else:  # 'first_seen'
        T_world_mark = np.eye(4)
        T_world_base = np.linalg.inv(T_base_mark)

    db.tagPlacement[tag_id] = T_world_mark
    db.T_VehToWorld.clear()
    db.T_VehToWorld.append(T_world_base)
    return True


def replay_session(
    session_dir: str,
    params: ReplayParams,
    *,
    seed_map_yaml: Optional[str] = None,
    camera_yaml: Optional[str] = None,
    quiet: bool = False,
) -> ReplayResult:
    """Run the localizer on a recorded session with the given params.

    Anchor is bootstrapped from the bag the same way as the live node:
    first detection (optionally combined with /ap/attitude for the
    ENU yaw). Pass ``seed_map_yaml`` to start from a non-empty map
    (mirrors discovery.persist_map_path in production).
    """
    if camera_yaml is None:
        camera_yaml = os.path.join(_workspace_dir(), 'config', 'camera.yaml')

    cam_trans, cam_rot = _load_camera_offset(camera_yaml)
    T_base_camera = _build_T_base_camera(cam_trans, cam_rot)
    seed_map = _load_persisted_map(seed_map_yaml) if seed_map_yaml else {}

    detections, alts, attitudes, _gps = _load_session_streams(session_dir)
    if not detections:
        raise RuntimeError('No /tag_detections in bag')

    # Pre-sort attitude by time (rosbag2 normally is, but be defensive).
    attitudes = sorted(attitudes, key=lambda x: x[0])
    att_idx = 0

    def _attitude_yaw_at(t_ns: int) -> Optional[float]:
        nonlocal att_idx
        if not attitudes:
            return None
        # Advance index to last attitude <= t_ns. Yaw is in attitudes[i][1].vector.z (rad, NED).
        while att_idx + 1 < len(attitudes) and attitudes[att_idx + 1][0] <= t_ns:
            att_idx += 1
        return float(attitudes[att_idx][1].vector.z)

    db = tagDB(
        debug=False,
        slidingWindow=int(params.sliding_window),
        R=float(params.kalman_R_pos_m),
        Ppos=float(params.kalman_Q_pos),
        PVel=float(params.kalman_Q_vel),
        PAccel=float(params.kalman_Q_accel),
        learn_unknown_tags=bool(params.learn_unknown_tags),
        z_score_threshold=float(params.z_score_threshold),
        cost_threshold_per_tag_m=float(params.cost_threshold_per_tag_m),
        pose_solver=str(params.pose_solver),
        absolute_max_per_tag_spread_m=float(params.absolute_max_per_tag_spread_m),
    )
    db.tagPlacement = dict(seed_map)
    anchor_set = bool(seed_map)
    anchor_attempts_without_attitude = 0
    effective_strategy = str(params.anchor_strategy)

    from collections import deque
    stability_window = deque(maxlen=int(params.stable_window_size))

    def _tracking_is_stable() -> bool:
        if len(stability_window) < int(params.stable_window_size):
            return False
        samples = np.asarray(stability_window, dtype=float)
        xy_std = np.std(samples[:, :2], axis=0)
        z_std = float(np.std(samples[:, 2]))
        return (
            max(float(xy_std[0]), float(xy_std[1])) <= params.stable_xy_stddev_m
            and z_std <= params.stable_z_stddev_m
        )

    out = ReplayResult(alt_truth=alts, params=params)

    last_t_sec = -1.0
    for t_ns, msg in detections:
        t_sec = t_ns / 1e9

        # Mirror localizer_node.det_cb gap-reset logic.
        if last_t_sec > 0 and (t_sec - last_t_sec) > params.detection_gap_reset_sec:
            db.reset_filter_state()
            stability_window.clear()
        last_t_sec = t_sec

        n_vis = len(msg.detections)
        sample = ReplaySample(t_ns=t_ns, state='NO_DATA', n_visible=n_vis,
                              pose_valid=False)

        if n_vis == 0:
            out.samples.append(sample)
            continue

        if not anchor_set:
            yaw_ned = _attitude_yaw_at(t_ns)
            # Auto-fallback for legacy bags without /ap/attitude.
            if (
                effective_strategy == 'first_seen_with_gps'
                and yaw_ned is None
                and params.anchor_attitude_wait_frames > 0
            ):
                anchor_attempts_without_attitude += 1
                if anchor_attempts_without_attitude > params.anchor_attitude_wait_frames:
                    if not quiet:
                        print(
                            f'replay: no /ap/attitude after '
                            f'{anchor_attempts_without_attitude} det frames; '
                            'falling back to anchor_strategy=first_seen'
                        )
                    effective_strategy = 'first_seen'

            ok = _bootstrap_anchor(
                db, msg, T_base_camera,
                strategy=effective_strategy,
                anchor_marker_id=int(params.anchor_marker_id),
                latest_attitude_yaw_ned=yaw_ned,
            )
            if not ok:
                sample.state = 'WAITING_FOR_ANCHOR'
                sample.drop_reason = (
                    'no_attitude' if (effective_strategy == 'first_seen_with_gps'
                                      and yaw_ned is None)
                    else 'no_anchor_marker'
                )
                out.samples.append(sample)
                continue
            anchor_set = True

        db.newFrame()
        any_added = False
        for det in msg.detections:
            if math.isnan(det.pose_camera_marker.position.x):
                continue
            T_cam_mark = _pose_to_matrix(det.pose_camera_marker)
            if T_cam_mark is None:
                continue
            db.addTag(
                _DetTagAdapter(int(det.id), T_cam_mark[0:3, 0:3], T_cam_mark[0:3, 3]),
                T_base_camera,
            )
            any_added = True

        if not any_added:
            sample.state = 'INVALID_DETECTIONS'
            sample.drop_reason = 'all_detections_nan'
            out.samples.append(sample)
            continue

        # Use the message's stamp seconds to keep tag_db's internal dt
        # semantics consistent with the live system (it expects sim time).
        try:
            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception:
            stamp_sec = t_sec

        before_advance_n = len(db.timestamps)
        db.getBestTransform(stamp_sec)
        after_advance_n = len(db.timestamps)

        if after_advance_n == before_advance_n:
            # tag_db dropped the frame (cost gate or other internal reject).
            # We tag it REJECTED_POSE so the report matches localizer_node states.
            sample.state = (
                'REJECTED_POSE' if before_advance_n > 0 else 'INITIALIZING'
            )
            sample.drop_reason = 'tagdb_cost_gate'
            out.samples.append(sample)
            continue

        if before_advance_n == 0:
            sample.state = 'INITIALIZING'
            stability_window.clear()
            out.samples.append(sample)
            continue

        pos = np.asarray(db.reportedPos, dtype=float).reshape(-1)
        if pos.shape[0] != 3 or not np.all(np.isfinite(pos)):
            sample.state = 'INVALID_POSE'
            sample.drop_reason = 'reportedPos_not_finite'
            out.samples.append(sample)
            continue

        stability_window.append(pos)
        sample.x, sample.y, sample.z = float(pos[0]), float(pos[1]), float(pos[2])
        if params.apply_stability_gate and not _tracking_is_stable():
            sample.state = 'TRACKING_SETTLING'
            sample.drop_reason = 'stability_gate'
            out.samples.append(sample)
            continue

        sample.state = 'TRACKING'
        sample.pose_valid = True
        out.samples.append(sample)

    if not quiet:
        n = out.n_frames_with_detections
        print(f'replay: detections_with_tags={n}  poses_published={out.n_poses_published}'
              f'  coverage={out.coverage:.2%}'
              f'  cov_above_2m={out.coverage_above(2.0):.2%}'
              f'  cov_above_5m={out.coverage_above(5.0):.2%}'
              f'  z_rmse_all={out.z_rmse():.2f}m'
              f'  z_rmse_above_2m={out.z_rmse(2.0):.2f}m'
              f'  max_alt_with_pose={out.max_alt_with_pose():.1f}m')
    return out
