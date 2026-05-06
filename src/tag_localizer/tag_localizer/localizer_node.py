import csv
import math
import os
import tempfile
import time
from collections import deque
from typing import Optional

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Vector3, Vector3Stamped
from marker_interfaces.msg import LocalizerStatus, MarkerDetectionArray, NavStatus
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from .geo import tagDB

# WGS-84 mean Earth radius (m). Used for the small-angle equirectangular
# GPS->ENU conversion in the anchor bootstrap. Replace with a proper
# geodesic projection (pyproj / GeographicLib) if you need accuracy at
# tens of kilometres.
_EARTH_RADIUS_M = 6371000.0


class DetectedTag:
    """Adapter from ROS MarkerDetection to the geo.py tag interface."""

    __slots__ = ('tag_id', 'pose_R', 'pose_t')

    def __init__(self, tag_id, pose_R, pose_t):
        self.tag_id = tag_id
        self.pose_R = pose_R
        self.pose_t = pose_t


# State codes shared with NavStatus.
STATE_NO_DATA = NavStatus.STATE_NO_DATA
STATE_INITIALIZING = NavStatus.STATE_INITIALIZING
STATE_SETTLING = NavStatus.STATE_SETTLING
STATE_TRACKING_DEGRADED = NavStatus.STATE_TRACKING_DEGRADED
STATE_TRACKING = NavStatus.STATE_TRACKING
STATE_REJECTED = NavStatus.STATE_REJECTED


class TagLocalizerNode(Node):
    def __init__(self):
        super().__init__('tag_localizer')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('cam_trans', [0.0, 0.0, -0.18])
        self.declare_parameter('cam_rot', [180.0, 0.0, -90.0])
        self.declare_parameter(
            'height_log_dir', os.path.expanduser('~/vtol_ws/logs')
        )
        self.declare_parameter('stable_window_size', 5)
        self.declare_parameter('stable_xy_stddev_m', 0.08)
        self.declare_parameter('stable_z_stddev_m', 0.12)

        # Online map discovery: every marker the localizer sees that's
        # not in the map is observed for tagDB.tagCandidates frames and
        # then added to tagPlacement. Recommended ON: with no static map
        # this is the only way new markers ever enter the map.
        self.declare_parameter('learn_unknown_tags', True)
        # If detections stop for longer than this, drop the Kalman state
        # so we don't predict forward from stale velocities at re-acquisition.
        self.declare_parameter('detection_gap_reset_sec', 0.6)

        # ---- Anchor bootstrap (cold-start map origin) -------------------
        # See config/tags_detector.yaml for the prose explanation of each
        # strategy. In short: with no static map, the very first detection
        # has to fix the map origin somehow - either from the marker
        # alone ("first_seen"), or from GPS+compass at that moment
        # ("first_seen_with_gps").
        self.declare_parameter('anchor_strategy', 'first_seen_with_gps')
        self.declare_parameter('anchor_marker_id', -1)
        self.declare_parameter('gps_topic', '/ap/gps/fix')
        self.declare_parameter('attitude_topic', '/ap/attitude')
        self.declare_parameter('gps_wait_timeout_sec', 5.0)
        # If non-empty, persist discovered markers to this YAML on every
        # change. On next launch the same path is read back as the seed
        # map. Path is resolved relative to $VTOL_WS_ROOT.
        self.declare_parameter('persist_map_path', '')
        # Where to write anchor_<session>.yaml (lat, lon, alt, yaw, id).
        self.declare_parameter('anchor_log_dir', '')

        # Default standard deviations published in NavStatus.
        self.declare_parameter('position_std_m', 0.10)
        self.declare_parameter('rotation_std_rad', 0.05)
        self.declare_parameter('velocity_std_m_s', 0.30)
        # When >1 marker is in view, divide std by sqrt(N) to reflect the
        # better geometric solve, capped at this many markers.
        self.declare_parameter('quality_marker_cap', 4)

        # Internal tagDB tunables. See geo.tagDB.__init__ for full docs.
        # Defaults match the historical hard-coded values so existing
        # low-altitude behavior is preserved when these are not set.
        # Override per-scene from flight.yaml -> launch -> here.
        self.declare_parameter('sliding_window', 5)
        self.declare_parameter('kalman_R_pos_m', 0.06)
        self.declare_parameter('kalman_Q_pos', 0.01)
        self.declare_parameter('kalman_Q_vel', 0.1)
        self.declare_parameter('kalman_Q_accel', 0.05)
        self.declare_parameter('z_score_threshold', 3.0)
        self.declare_parameter('cost_threshold_per_tag_m', 0.5)
        # 'differential' (legacy) or 'absolute'. See geo.tagDB for details.
        self.declare_parameter('pose_solver', 'differential')
        self.declare_parameter('absolute_max_per_tag_spread_m', 3.0)

        self.stable_window_size = max(2, int(self.get_parameter('stable_window_size').value))
        self.stable_xy_stddev_m = float(self.get_parameter('stable_xy_stddev_m').value)
        self.stable_z_stddev_m = float(self.get_parameter('stable_z_stddev_m').value)
        self.learn_unknown_tags = bool(self.get_parameter('learn_unknown_tags').value)
        self.detection_gap_reset_sec = float(self.get_parameter('detection_gap_reset_sec').value)
        self.position_std_m = float(self.get_parameter('position_std_m').value)
        self.rotation_std_rad = float(self.get_parameter('rotation_std_rad').value)
        self.velocity_std_m_s = float(self.get_parameter('velocity_std_m_s').value)
        self.quality_marker_cap = max(1, int(self.get_parameter('quality_marker_cap').value))

        sliding_window = max(2, int(self.get_parameter('sliding_window').value))
        kalman_R = float(self.get_parameter('kalman_R_pos_m').value)
        kalman_Qp = float(self.get_parameter('kalman_Q_pos').value)
        kalman_Qv = float(self.get_parameter('kalman_Q_vel').value)
        kalman_Qa = float(self.get_parameter('kalman_Q_accel').value)
        z_score_th = float(self.get_parameter('z_score_threshold').value)
        cost_th = float(self.get_parameter('cost_threshold_per_tag_m').value)
        pose_solver = str(self.get_parameter('pose_solver').value)
        abs_spread = float(self.get_parameter('absolute_max_per_tag_spread_m').value)
        self.tag_db = tagDB(
            debug=False,
            slidingWindow=sliding_window,
            R=kalman_R,
            Ppos=kalman_Qp,
            PVel=kalman_Qv,
            PAccel=kalman_Qa,
            z_score_threshold=z_score_th,
            cost_threshold_per_tag_m=cost_th,
            pose_solver=pose_solver,
            absolute_max_per_tag_spread_m=abs_spread,
            learn_unknown_tags=self.learn_unknown_tags,
        )
        self.get_logger().info(
            f'tagDB: pose_solver={pose_solver} sliding_window={sliding_window} '
            f'R={kalman_R} z_score_th={z_score_th} '
            f'cost_th_per_tag_m={cost_th}; '
            f'detection_gap_reset_sec={self.detection_gap_reset_sec} '
            f'stable_xy_stddev_m={self.stable_xy_stddev_m}'
        )

        self.height_log_file = None
        self.height_log_writer = None
        self.latest_reference_height = math.nan
        self.latest_sim_time = math.nan
        self.pending_estimated_height = 0.0
        self.has_pending_estimate = False
        self.recent_pose_samples = deque(maxlen=self.stable_window_size)
        self.last_detection_monotonic = 0.0

        # ---- Anchor / discovery state ----------------------------------
        self.anchor_strategy = str(self.get_parameter('anchor_strategy').value)
        self.anchor_marker_id = int(self.get_parameter('anchor_marker_id').value)
        self.gps_wait_timeout_sec = float(self.get_parameter('gps_wait_timeout_sec').value)
        self.persist_map_path = self._resolve_workspace_path(
            self.get_parameter('persist_map_path').value
        )
        anchor_log_dir = self.get_parameter('anchor_log_dir').value or os.path.join(
            os.path.expanduser('~/vtol_ws'), 'logs', 'maps'
        )
        self.anchor_log_dir = anchor_log_dir
        # Bootstrap state.
        self.anchor_set: bool = False
        self.first_unseen_anchor_attempt_monotonic: float = 0.0
        self.latest_gps: Optional[NavSatFix] = None
        self.latest_attitude_rpy: Optional[list] = None  # [roll, pitch, yaw] rad (NED)
        # Track how many tags are in tagPlacement so we can detect when
        # tagDB.learn_unknown_tags adds a new one and persist on change.
        self._known_tag_count = 0

        trans = self.get_parameter('cam_trans').value
        rot_deg = self.get_parameter('cam_rot').value
        self.T_base_camera = np.eye(4)
        self.T_base_camera[0:3, 3] = trans
        self.T_base_camera[0:3, 0:3] = R.from_euler('xyz', rot_deg, degrees=True).as_matrix()
        self.get_logger().info(
            f'Camera offset: trans={trans}, rot={rot_deg} deg; '
            f'learn_unknown_tags={self.learn_unknown_tags}'
        )
        self._load_persisted_map()
        self._init_height_logger()

        from rclpy.qos import qos_profile_sensor_data

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, qos_profile_sensor_data
        )
        self.sub = self.create_subscription(
            MarkerDetectionArray, '/tag_detections', self.det_cb, qos_profile_sensor_data
        )
        self.reference_height_sub = self.create_subscription(
            Float64, '/ap/relative_alt', self.reference_height_cb, qos_profile_sensor_data
        )
        # Anchor inputs. If they never arrive we fall back to plain
        # first_seen after gps_wait_timeout_sec.
        self.gps_sub = self.create_subscription(
            NavSatFix,
            str(self.get_parameter('gps_topic').value),
            self._gps_cb,
            qos_profile_sensor_data,
        )
        self.attitude_sub = self.create_subscription(
            Vector3Stamped,
            str(self.get_parameter('attitude_topic').value),
            self._attitude_cb,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Anchor: strategy={self.anchor_strategy} '
            f'marker_id={self.anchor_marker_id} '
            f'persist_map_path={self.persist_map_path or "(in-memory)"} '
            f'gps_wait_timeout_sec={self.gps_wait_timeout_sec}'
        )
        # Publish to source-named topics. Routing them to the unified
        # /vision_pose_enu and /nav/status (consumed by mavlink_bridge,
        # tf_broadcaster, mission_manager) is the responsibility of the
        # higher-level launch:
        #   - single-source modes (tags / dpvo / orb_slam3) start
        #     single_source_router_node which forwards 1:1.
        #   - hybrid mode lets vision_fusion_node multiplex tag + VO
        #     and emit the unified topics itself.
        # This keeps the per-source channel always populated (useful for
        # logging, viz and replay) and decouples *which source is active*
        # from *what each source thinks*.
        self.pose_pub = self.create_publisher(
            PoseStamped, '/tag_localizer/pose', 10
        )
        self.status_pub = self.create_publisher(LocalizerStatus, '/localizer_status', 10)
        self.nav_status_pub = self.create_publisher(
            NavStatus, '/tag_localizer/nav_status', 10
        )

    def det_cb(self, msg: MarkerDetectionArray):
        now_monotonic = self.get_clock().now().nanoseconds / 1e9
        if (
            self.last_detection_monotonic > 0.0
            and (now_monotonic - self.last_detection_monotonic)
            > self.detection_gap_reset_sec
        ):
            self.tag_db.reset_filter_state()
            self._reset_stability_window()
            self.get_logger().debug(
                'Detection gap exceeded reset threshold; flushed Kalman state'
            )

        status = LocalizerStatus()
        status.header.stamp = msg.header.stamp
        status.pose_valid = False
        status.active_marker_source = ''
        status.visible_ids = []
        status.quality_score = 0.0
        status.state_code = STATE_NO_DATA
        status.state = 'NO_DATA'

        nav_status = NavStatus()
        nav_status.header.stamp = msg.header.stamp
        nav_status.source = 'tags'
        nav_status.state = NavStatus.STATE_NO_DATA
        nav_status.position_std_m = float(self.position_std_m)
        nav_status.rotation_std_rad = float(self.rotation_std_rad)
        nav_status.velocity_std_m_s = float(self.velocity_std_m_s)

        if not msg.detections:
            self._publish_status(status, nav_status)
            return

        self.last_detection_monotonic = now_monotonic

        # Bootstrap the map anchor on the first usable detection. Until
        # we have an anchor the map frame is undefined and there is no
        # point running the solver - any pose published would be in a
        # frame nobody else understands.
        if not self.anchor_set:
            anchor_outcome = self._maybe_bootstrap_anchor(msg, now_monotonic)
            if not self.anchor_set:
                status.state_code = STATE_INITIALIZING
                status.state = f'WAITING_FOR_ANCHOR:{anchor_outcome}'
                nav_status.state = NavStatus.STATE_INITIALIZING
                nav_status.detail = anchor_outcome
                self._publish_status(status, nav_status)
                return

        self.tag_db.newFrame()

        for det in msg.detections:
            if math.isnan(det.pose_camera_marker.position.x):
                continue
            T_cam_mark = self.pose_to_matrix(det.pose_camera_marker)
            if T_cam_mark is None:
                continue
            self.tag_db.addTag(
                DetectedTag(det.id, T_cam_mark[0:3, 0:3], T_cam_mark[0:3, 3]),
                self.T_base_camera,
            )
            status.visible_ids.append(det.id)

        nav_status.visible_ids = list(status.visible_ids)
        if not status.visible_ids:
            status.state_code = STATE_REJECTED
            status.state = 'INVALID_DETECTIONS'
            nav_status.state = NavStatus.STATE_REJECTED
            nav_status.detail = 'invalid_detections'
            self._reset_stability_window()
            self._publish_status(status, nav_status)
            return

        n_visible = len(status.visible_ids)
        status.active_marker_source = f'markers_{status.visible_ids}'
        nav_status.detail = status.active_marker_source

        # Quality: scale by number of visible markers, capped.
        score01 = min(1.0, n_visible / float(self.quality_marker_cap))
        status.quality_score = float(n_visible)
        nav_status.quality_score = float(score01)

        # Tighter covariance with more markers.
        scale = 1.0 / max(1.0, math.sqrt(min(n_visible, self.quality_marker_cap)))
        nav_status.position_std_m = float(self.position_std_m * scale)
        nav_status.rotation_std_rad = float(self.rotation_std_rad * scale)
        nav_status.velocity_std_m_s = float(self.velocity_std_m_s * scale)

        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.tag_db.getBestTransform(current_time)

        # If tag_db.learn_unknown_tags promoted a new marker into the map
        # this frame, persist the updated map.
        new_count = len(self.tag_db.tagPlacement)
        if new_count != self._known_tag_count:
            added = new_count - self._known_tag_count
            self.get_logger().info(
                f'Map size changed: {self._known_tag_count} -> {new_count} '
                f'({added:+d}). Persisting.'
            )
            self._known_tag_count = new_count
            self._persist_map_atomic()

        if len(self.tag_db.timestamps) == 0:
            status.state_code = STATE_INITIALIZING
            status.state = 'INITIALIZING'
            nav_status.state = NavStatus.STATE_INITIALIZING
            self._reset_stability_window()
            self._publish_status(status, nav_status)
            return

        latest_timestamp = float(self.tag_db.timestamps[-1])
        if abs(latest_timestamp - current_time) > 1e-3:
            status.state_code = STATE_REJECTED
            status.state = 'REJECTED_POSE'
            nav_status.state = NavStatus.STATE_REJECTED
            nav_status.detail = 'rejected_pose'
            self._reset_stability_window()
            self._publish_status(status, nav_status)
            return

        reported_pos = self._coerce_vector(self.tag_db.reportedPos, 3)
        reported_rot = self._coerce_vector(self.tag_db.reportedRot, 3)
        if reported_pos is None or reported_rot is None:
            status.state_code = STATE_REJECTED
            status.state = 'INVALID_POSE'
            nav_status.state = NavStatus.STATE_REJECTED
            nav_status.detail = 'invalid_pose'
            self._reset_stability_window()
            self._publish_status(status, nav_status)
            return

        self.recent_pose_samples.append(np.array(reported_pos, dtype=float))
        tracking_mode = 'SINGLE_TAG' if n_visible == 1 else 'MULTI_TAG'
        if not self._tracking_is_stable():
            status.state_code = STATE_SETTLING
            status.state = f'TRACKING_{tracking_mode}_SETTLING'
            nav_status.state = NavStatus.STATE_SETTLING
            self._publish_status(status, nav_status)
            return

        status.pose_valid = True
        status.state_code = STATE_TRACKING if n_visible > 1 else STATE_TRACKING_DEGRADED
        status.state = f'TRACKING_{tracking_mode}'
        nav_status.state = (
            NavStatus.STATE_TRACKING if n_visible > 1 else NavStatus.STATE_TRACKING_DEGRADED
        )

        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = self.get_parameter('map_frame').value
        pose_msg.pose.position.x = float(reported_pos[0])
        pose_msg.pose.position.y = float(reported_pos[1])
        pose_msg.pose.position.z = float(reported_pos[2])

        rot = R.from_euler('xyz', reported_rot, degrees=False)
        quat = rot.as_quat()
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])

        velocity = self._coerce_vector(getattr(self.tag_db, 'reportedVelocity', None), 3)
        if velocity is not None and np.all(np.isfinite(velocity)):
            nav_status.has_velocity = True
            nav_status.velocity_enu = Vector3(
                x=float(velocity[0]), y=float(velocity[1]), z=float(velocity[2])
            )

        self.pending_estimated_height = float(reported_pos[2])
        self.has_pending_estimate = True
        self.pose_pub.publish(pose_msg)
        self._publish_status(status, nav_status)

    def _publish_status(self, status: LocalizerStatus, nav_status: NavStatus):
        self.status_pub.publish(status)
        self.nav_status_pub.publish(nav_status)

    def pose_to_matrix(self, pose):
        T = np.eye(4)
        T[0, 3] = pose.position.x
        T[1, 3] = pose.position.y
        T[2, 3] = pose.position.z
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-6:
            return None
        rot = R.from_quat([qx, qy, qz, qw])
        T[0:3, 0:3] = rot.as_matrix()
        return T

    def clock_cb(self, msg: Clock):
        self.latest_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def reference_height_cb(self, msg: Float64):
        self.latest_reference_height = float(msg.data)
        timestamp_sec = self.latest_sim_time if math.isfinite(self.latest_sim_time) else time.time()
        estimated_height_m = self.pending_estimated_height if self.has_pending_estimate else 0.0
        self._log_height_sample(timestamp_sec, estimated_height_m, self.latest_reference_height)
        self.has_pending_estimate = False

    # ------------------------------------------------------------------
    # Anchor bootstrap & online map persistence
    # ------------------------------------------------------------------

    @staticmethod
    def _resolve_workspace_path(value) -> str:
        """Resolve `value` against $VTOL_WS_ROOT (or ~/vtol_ws) when relative."""
        if not value:
            return ''
        value = str(value)
        if os.path.isabs(value):
            return value
        ws = os.environ.get('VTOL_WS_ROOT', os.path.expanduser('~/vtol_ws'))
        return os.path.join(ws, value)

    def _load_persisted_map(self) -> None:
        """Load `persist_map_path` (if any) into tagDB.tagPlacement."""
        path = self.persist_map_path
        if not path or not os.path.exists(path):
            self.get_logger().info(
                'Starting with EMPTY marker map. Anchor will be set from '
                'first detection; subsequent markers added online.'
            )
            return
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            for entry in data.get('markers', []):
                pose = entry.get('pose')
                if not pose or len(pose) != 6:
                    continue
                tid = int(entry['id'])
                T = np.eye(4)
                T[0:3, 3] = [float(v) for v in pose[:3]]
                T[0:3, 0:3] = R.from_euler('xyz', [float(v) for v in pose[3:]]).as_matrix()
                self.tag_db.tagPlacement[tid] = T
            self._known_tag_count = len(self.tag_db.tagPlacement)
            self.get_logger().info(
                f'Loaded {self._known_tag_count} markers from persisted map {path}'
            )
            # If we already have a map, the anchor is implicitly defined
            # (the persisted markers ARE the anchor) - skip waiting for
            # GPS+attitude in det_cb.
            self.anchor_set = self._known_tag_count > 0
        except Exception as exc:
            self.get_logger().warning(f'Failed to load persisted map {path}: {exc}')

    def _maybe_bootstrap_anchor(self, msg: MarkerDetectionArray, now_monotonic: float) -> str:
        """Try to set the map anchor from the current detection frame.

        Returns a short reason string for diagnostics. ``self.anchor_set``
        is updated on success.

        Strategies (selected by ``self.anchor_strategy``):
          * ``"first_seen_with_gps"``: requires a fresh /ap/gps/fix and
            /ap/attitude. Map ENU origin is set such that the marker
            sits at (0,0,0) and map yaw matches compass yaw.
            Falls through to ``"first_seen"`` after gps_wait_timeout_sec.
          * ``"first_seen"``: marker pose in its own frame becomes the
            map (i.e. the marker placement is identity). No
            georeferencing.
        """
        if self.first_unseen_anchor_attempt_monotonic == 0.0:
            self.first_unseen_anchor_attempt_monotonic = now_monotonic

        # Pick the first detection that's a valid anchor candidate.
        candidate = None
        for det in msg.detections:
            if math.isnan(det.pose_camera_marker.position.x):
                continue
            if self.anchor_marker_id >= 0 and int(det.id) != self.anchor_marker_id:
                continue
            T_cam_mark = self.pose_to_matrix(det.pose_camera_marker)
            if T_cam_mark is None:
                continue
            candidate = (int(det.id), T_cam_mark)
            break

        if candidate is None:
            return (
                f'no_anchor_marker_in_view'
                if self.anchor_marker_id >= 0
                else 'no_valid_detection'
            )

        tag_id, T_cam_mark = candidate
        T_base_mark = self.T_base_camera @ T_cam_mark

        strategy = self.anchor_strategy
        elapsed = now_monotonic - self.first_unseen_anchor_attempt_monotonic
        if strategy == 'first_seen_with_gps':
            need_gps = self.latest_gps is not None
            need_att = self.latest_attitude_rpy is not None
            if not (need_gps and need_att):
                if self.gps_wait_timeout_sec > 0 and elapsed > self.gps_wait_timeout_sec:
                    self.get_logger().warning(
                        f'GPS/attitude unavailable after {elapsed:.1f}s '
                        f'(gps={need_gps}, att={need_att}); '
                        'falling back to anchor_strategy=first_seen'
                    )
                    strategy = 'first_seen'
                else:
                    return (
                        f'waiting_for_gps_and_attitude'
                        f'(gps={need_gps},att={need_att},'
                        f'elapsed={elapsed:.1f}s)'
                    )

        if strategy == 'first_seen_with_gps':
            yaw_ned = float(self.latest_attitude_rpy[2])
            # ArduPilot ATTITUDE.yaw is a body-to-NED Euler. The map
            # frame here is ENU. yaw_enu = pi/2 - yaw_ned.
            yaw_enu = (math.pi * 0.5) - yaw_ned
            R_world_base = R.from_euler('z', yaw_enu).as_matrix()
            # Marker must be at (0,0,0). Solve for drone position in world.
            p_world_base = -R_world_base @ T_base_mark[0:3, 3]
            T_world_base = np.eye(4)
            T_world_base[0:3, 0:3] = R_world_base
            T_world_base[0:3, 3] = p_world_base
            T_world_mark = T_world_base @ T_base_mark
            # Sanity: numerical drift from the construction must be sub-mm.
            assert np.allclose(T_world_mark[0:3, 3], 0, atol=1e-6), (
                f'anchor construction broken: {T_world_mark[0:3, 3]}'
            )

            self.tag_db.tagPlacement[tag_id] = T_world_mark
            self.tag_db.T_VehToWorld.clear()
            self.tag_db.T_VehToWorld.append(T_world_base)
            self.anchor_set = True
            self._known_tag_count = len(self.tag_db.tagPlacement)

            anchor_info = {
                'strategy': 'first_seen_with_gps',
                'anchor_marker_id': tag_id,
                'anchor_lat_deg': float(self.latest_gps.latitude),
                'anchor_lon_deg': float(self.latest_gps.longitude),
                'anchor_alt_m_msl': float(self.latest_gps.altitude),
                'anchor_yaw_ned_rad': yaw_ned,
                'anchor_yaw_enu_rad': yaw_enu,
                'anchor_yaw_enu_deg': math.degrees(yaw_enu),
                'drone_pos_world_xyz_m': p_world_base.tolist(),
                'created_unix_sec': time.time(),
            }
            self.get_logger().info(
                f'ANCHOR set from marker {tag_id} (first_seen_with_gps): '
                f'lat={anchor_info["anchor_lat_deg"]:.7f} '
                f'lon={anchor_info["anchor_lon_deg"]:.7f} '
                f'alt={anchor_info["anchor_alt_m_msl"]:.2f}m '
                f'yaw_enu={anchor_info["anchor_yaw_enu_deg"]:.1f}deg'
            )
            self._save_anchor_log(anchor_info)
            self._persist_map_atomic()
            return 'anchor_set'

        if strategy == 'first_seen':
            # Marker IS the map: placement = identity; drone position is
            # whatever T_base_marker says (with rotation arbitrary).
            T_world_mark = np.eye(4)
            T_world_base = np.linalg.inv(T_base_mark)
            self.tag_db.tagPlacement[tag_id] = T_world_mark
            self.tag_db.T_VehToWorld.clear()
            self.tag_db.T_VehToWorld.append(T_world_base)
            self.anchor_set = True
            self._known_tag_count = len(self.tag_db.tagPlacement)
            self.get_logger().info(
                f'ANCHOR set from marker {tag_id} (first_seen): '
                'no georeference recorded'
            )
            self._save_anchor_log({
                'strategy': 'first_seen',
                'anchor_marker_id': tag_id,
                'created_unix_sec': time.time(),
            })
            self._persist_map_atomic()
            return 'anchor_set'

        return f'unknown_strategy:{strategy}'

    def _save_anchor_log(self, info: dict) -> None:
        try:
            os.makedirs(self.anchor_log_dir, exist_ok=True)
            ts = time.strftime('%Y%m%d-%H%M%S')
            path = os.path.join(self.anchor_log_dir, f'anchor_{ts}.yaml')
            with open(path, 'w') as f:
                yaml.safe_dump(info, f, sort_keys=False)
            self.get_logger().info(f'Wrote anchor log: {path}')
        except Exception as exc:
            self.get_logger().warning(f'Failed to write anchor log: {exc}')

    def _persist_map_atomic(self) -> None:
        path = self.persist_map_path
        if not path:
            return
        markers = []
        for tid in sorted(self.tag_db.tagPlacement.keys()):
            T = self.tag_db.tagPlacement[tid]
            pos = T[0:3, 3]
            rpy = R.from_matrix(T[0:3, 0:3]).as_euler('xyz')
            markers.append({
                'id': int(tid),
                'pose': [float(pos[0]), float(pos[1]), float(pos[2]),
                         float(rpy[0]), float(rpy[1]), float(rpy[2])],
            })
        data = {'markers': markers, 'updated_unix_sec': time.time()}
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with tempfile.NamedTemporaryFile(
                mode='w',
                delete=False,
                dir=os.path.dirname(path),
                prefix=os.path.basename(path) + '.',
                suffix='.tmp',
            ) as tmp:
                yaml.safe_dump(data, tmp, sort_keys=False)
                tmp.flush()
                os.fsync(tmp.fileno())
                tmp_path = tmp.name
            os.replace(tmp_path, path)
        except Exception as exc:
            self.get_logger().warning(f'Failed to persist map to {path}: {exc}')

    # ------------------------------------------------------------------
    # GPS / attitude callbacks (anchor inputs)
    # ------------------------------------------------------------------

    def _gps_cb(self, msg: NavSatFix) -> None:
        # NavSatFix without a lock sends lat=0,lon=0; ignore those.
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return
        self.latest_gps = msg

    def _attitude_cb(self, msg: Vector3Stamped) -> None:
        # /ap/attitude reports body-to-NED Euler (roll, pitch, yaw) in
        # radians as ArduPilot's ATTITUDE message provides them.
        self.latest_attitude_rpy = [
            float(msg.vector.x), float(msg.vector.y), float(msg.vector.z)
        ]

    def _init_height_logger(self):
        log_dir = self.get_parameter('height_log_dir').value
        try:
            os.makedirs(log_dir, exist_ok=True)
            timestamp = time.strftime('%Y%m%d-%H%M%S')
            log_path = os.path.join(log_dir, f'localizer_height_{timestamp}.csv')
            self.height_log_file = open(log_path, 'w', newline='')
            self.height_log_writer = csv.writer(self.height_log_file)
            self.height_log_writer.writerow(['timestamp', 'estimated_height_m', 'reference_height_m'])
            self.height_log_file.flush()
            self.get_logger().info(f'Logging computed height to {log_path}')
        except Exception as exc:
            self.height_log_file = None
            self.height_log_writer = None
            self.get_logger().warning(f'Failed to initialize height logger: {exc}')

    def _log_height_sample(self, timestamp_sec, estimated_height_m, reference_height_m):
        if self.height_log_writer is None or self.height_log_file is None:
            return
        self.height_log_writer.writerow(
            [
                float(timestamp_sec),
                float(estimated_height_m),
                float(reference_height_m) if math.isfinite(reference_height_m) else float('nan'),
            ]
        )
        self.height_log_file.flush()

    def _coerce_vector(self, value, expected_size):
        if value is None:
            return None
        array = np.asarray(value, dtype=float)
        if array.size != expected_size or not np.all(np.isfinite(array)):
            return None
        return array.reshape(expected_size)

    def _reset_stability_window(self):
        self.recent_pose_samples.clear()

    def _tracking_is_stable(self):
        if len(self.recent_pose_samples) < self.stable_window_size:
            return False
        samples = np.asarray(self.recent_pose_samples, dtype=float)
        xy_std = np.std(samples[:, :2], axis=0)
        z_std = float(np.std(samples[:, 2]))
        return (
            max(float(xy_std[0]), float(xy_std[1])) <= self.stable_xy_stddev_m
            and z_std <= self.stable_z_stddev_m
        )

    def destroy_node(self):
        if self.height_log_file is not None:
            self.height_log_file.close()
            self.height_log_file = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TagLocalizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
