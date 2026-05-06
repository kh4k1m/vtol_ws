"""Forward fused vision pose to ArduPilot as MAVLink ODOMETRY.

Subscribes to ``/vision_pose_enu`` (PoseStamped, ENU/FLU) and the
optional ``/nav/status`` (NavStatus) topic. The latter carries the
covariance bounds and an optional velocity sample emitted by the
upstream estimator (tag localizer's Kalman, fusion node's offset
correction, etc.).

If NavStatus is absent we fall back to constant covariance defaults and
a finite-difference velocity, which is what the previous bridge was
doing - but with a warning.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import numpy as np
from geometry_msgs.msg import PoseStamped
from marker_interfaces.msg import NavStatus
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from . import frame_transforms as ft


class VisionForwarder:
    """Convert vision pose to MAVLink ODOMETRY and resend at a fixed rate."""

    def __init__(self, node, connection):
        self._node = node
        self._logger = node.get_logger()
        self._conn = connection

        self._declare_params()
        self._read_params()

        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pose_sub = node.create_subscription(
            PoseStamped, '/vision_pose_enu', self._pose_cb, sub_qos
        )
        self._nav_status_sub = node.create_subscription(
            NavStatus, '/nav/status', self._nav_status_cb, sub_qos
        )

        self._last_nav_status: Optional[NavStatus] = None
        self._last_nav_status_monotonic = 0.0

        self._last_x = None
        self._last_y = None
        self._last_z = None
        self._last_vision_sample_sec: Optional[float] = None
        self._last_vision_stamp_sec: Optional[float] = None
        self._last_odometry_send_monotonic = 0.0
        self._last_odometry_time_usec = 0
        self._cached_payload = None
        self._cached_proposed_usec = 0

        self._last_vision_timing_warning_monotonic = 0.0
        self._last_debug_log_monotonic = 0.0

        if self._odometry_resend_period_sec is not None:
            self._resend_timer = node.create_timer(
                self._odometry_resend_period_sec, self._resend_latest_odometry
            )
        else:
            self._resend_timer = None

        if not self._forward_enabled:
            self._logger.warning(
                'VisionForwarder: vision_forward_enabled=false. '
                '/vision_pose_enu will be observed but NOT sent to ArduPilot. '
                '(Set flight.yaml mode != log_only to enable forwarding.)'
            )

    # -- ROS parameters ---------------------------------------------------

    def _declare_params(self):
        self._node.declare_parameter('odometry_resend_rate_hz', 20.0)
        self._node.declare_parameter('odometry_resend_hold_timeout_sec', 1.0)
        self._node.declare_parameter('vision_debug_log_period_sec', 2.0)
        self._node.declare_parameter('default_position_std_m', 0.10)
        self._node.declare_parameter('default_rotation_std_rad', 0.05)
        self._node.declare_parameter('default_velocity_std_m_s', 0.30)
        self._node.declare_parameter('nav_status_max_age_sec', 0.5)
        self._node.declare_parameter('vision_quality_min', 1.0)
        # Master switch: when False, the forwarder still subscribes (so the
        # pose/nav_status traffic flows through ROS for logging and viz),
        # but never sends MAVLink ODOMETRY to ArduPilot. Used by mode=log_only
        # in flight.yaml to fly on GPS while collecting vision predictions.
        self._node.declare_parameter('vision_forward_enabled', True)

    def _read_params(self):
        get = self._node.get_parameter
        self._odometry_resend_rate_hz = max(0.0, float(get('odometry_resend_rate_hz').value))
        self._odometry_resend_hold_timeout_sec = max(
            0.0, float(get('odometry_resend_hold_timeout_sec').value)
        )
        self._odometry_resend_period_sec = (
            1.0 / self._odometry_resend_rate_hz
            if self._odometry_resend_rate_hz > 0.0
            else None
        )
        self._vision_debug_log_period_sec = max(
            0.0, float(get('vision_debug_log_period_sec').value)
        )
        self._default_position_std_m = float(get('default_position_std_m').value)
        self._default_rotation_std_rad = float(get('default_rotation_std_rad').value)
        self._default_velocity_std_m_s = float(get('default_velocity_std_m_s').value)
        self._nav_status_max_age_sec = float(get('nav_status_max_age_sec').value)
        self._vision_quality_min = float(get('vision_quality_min').value)
        self._forward_enabled = bool(get('vision_forward_enabled').value)

    # -- callbacks --------------------------------------------------------

    def _nav_status_cb(self, msg: NavStatus):
        self._last_nav_status = msg
        self._last_nav_status_monotonic = self._node.get_clock().now().nanoseconds / 1e9

    def _pose_cb(self, msg: PoseStamped):
        if self._conn.master is None and self._forward_enabled:
            return

        pos_enu = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=float,
        )
        q_xyzw = np.array(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ],
            dtype=float,
        )

        sample_sec = self._stamp_to_sec(msg.header.stamp)
        if self._last_vision_stamp_sec is not None:
            stamp_dt = sample_sec - self._last_vision_stamp_sec
            if stamp_dt <= 0.0:
                self._warn_vision_timing(
                    f'Non-monotonic /vision_pose_enu stamp: dt={stamp_dt:.6f}s '
                    f'current={sample_sec:.6f}s previous={self._last_vision_stamp_sec:.6f}s'
                )
        self._last_vision_stamp_sec = sample_sec

        if self._last_vision_sample_sec is not None:
            dt = sample_sec - self._last_vision_sample_sec
        else:
            dt = 0.0

        velocity_enu, velocity_from_status = self._resolve_velocity_enu(
            msg, pos_enu, dt
        )

        odometry = ft.encode_odometry_ned(pos_enu, q_xyzw, velocity_enu)
        pose_cov, vel_cov = self._resolve_covariances()
        quality = self._resolve_quality()

        usec = self._stamp_to_usec(msg.header.stamp)
        proposed_usec = self._next_odometry_time_usec(usec)

        payload = {
            'x': float(odometry.position_ned[0]),
            'y': float(odometry.position_ned[1]),
            'z': float(odometry.position_ned[2]),
            'q': list(odometry.quaternion_wxyz),
            'vx': float(odometry.velocity_body_frd[0]),
            'vy': float(odometry.velocity_body_frd[1]),
            'vz': float(odometry.velocity_body_frd[2]),
            'pose_covariance': pose_cov,
            'velocity_covariance': vel_cov,
            'quality': int(quality),
        }

        if self._forward_enabled:
            try:
                self._send_odometry(payload, proposed_usec)
            except Exception as exc:
                self._logger.error(f'Failed to send MAVLink ODOMETRY: {exc}')
                return

        self._last_x, self._last_y, self._last_z = pos_enu[0], pos_enu[1], pos_enu[2]
        self._last_vision_sample_sec = sample_sec
        self._cached_payload = payload
        self._cached_proposed_usec = proposed_usec
        self._maybe_log_debug(odometry, pose_cov, vel_cov, quality, velocity_from_status)

    # -- helpers ----------------------------------------------------------

    def _resolve_velocity_enu(self, msg, pos_enu, dt) -> tuple:
        status = self._fresh_nav_status()
        if status is not None and status.has_velocity:
            return (
                np.array(
                    [
                        status.velocity_enu.x,
                        status.velocity_enu.y,
                        status.velocity_enu.z,
                    ],
                    dtype=float,
                ),
                True,
            )
        if 1e-6 < dt <= 0.5 and self._last_x is not None:
            return (
                np.array(
                    [
                        (pos_enu[0] - self._last_x) / dt,
                        (pos_enu[1] - self._last_y) / dt,
                        (pos_enu[2] - self._last_z) / dt,
                    ],
                    dtype=float,
                ),
                False,
            )
        return np.zeros(3, dtype=float), False

    def _resolve_covariances(self) -> tuple:
        status = self._fresh_nav_status()
        if status is None:
            pos_std = self._default_position_std_m
            rot_std = self._default_rotation_std_rad
            vel_std = self._default_velocity_std_m_s
        else:
            pos_std = float(status.position_std_m) if status.position_std_m > 0 else self._default_position_std_m
            rot_std = float(status.rotation_std_rad) if status.rotation_std_rad > 0 else self._default_rotation_std_rad
            vel_std = float(status.velocity_std_m_s) if status.velocity_std_m_s > 0 else self._default_velocity_std_m_s
        pose_cov = list(ft.diagonal_pose_covariance(pos_std, rot_std))
        vel_cov = list(ft.diagonal_velocity_covariance(vel_std, rot_std))
        return pose_cov, vel_cov

    def _resolve_quality(self) -> int:
        status = self._fresh_nav_status()
        if status is None:
            return 50
        score01 = max(0.0, min(1.0, float(status.quality_score)))
        return int(round(score01 * 100.0))

    def _fresh_nav_status(self) -> Optional[NavStatus]:
        if self._last_nav_status is None:
            return None
        age = (self._node.get_clock().now().nanoseconds / 1e9) - self._last_nav_status_monotonic
        if age > self._nav_status_max_age_sec:
            return None
        return self._last_nav_status

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + (float(stamp.nanosec) * 1e-9)

    @staticmethod
    def _stamp_to_usec(stamp) -> int:
        return int(int(stamp.sec) * 1_000_000 + int(stamp.nanosec) // 1000)

    def _warn_vision_timing(self, message: str):
        now = self._node.get_clock().now().nanoseconds / 1e9
        if (now - self._last_vision_timing_warning_monotonic) < 2.0:
            return
        self._logger.warning(message)
        self._last_vision_timing_warning_monotonic = now

    def _next_odometry_time_usec(self, proposed_usec: int) -> int:
        next_usec = int(proposed_usec)
        if next_usec <= self._last_odometry_time_usec:
            next_usec = self._last_odometry_time_usec + 1
        self._last_odometry_time_usec = next_usec
        return next_usec

    def _send_odometry(self, payload, proposed_usec):
        with self._conn.lock():
            master = self._conn.master
            if master is None:
                return
            master.mav.send(
                mavlink2.MAVLink_odometry_message(
                    int(proposed_usec),
                    mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
                    mavutil.mavlink.MAV_FRAME_BODY_FRD,
                    payload['x'], payload['y'], payload['z'],
                    payload['q'],
                    payload['vx'], payload['vy'], payload['vz'],
                    0.0, 0.0, 0.0,
                    payload['pose_covariance'],
                    payload['velocity_covariance'],
                    0,
                    mavlink2.MAV_ESTIMATOR_TYPE_VIO,
                    payload['quality'],
                )
            )
        self._last_odometry_send_monotonic = self._node.get_clock().now().nanoseconds / 1e9

    def _resend_latest_odometry(self):
        if not self._forward_enabled:
            return
        if self._cached_payload is None or self._odometry_resend_period_sec is None:
            return
        if self._conn.master is None:
            return
        now = self._node.get_clock().now().nanoseconds / 1e9
        sample_age_sec = now - self._last_odometry_send_monotonic
        if sample_age_sec > self._odometry_resend_hold_timeout_sec:
            return
        if (now - self._last_odometry_send_monotonic) < (self._odometry_resend_period_sec * 0.9):
            return
        now_usec = int(self._node.get_clock().now().nanoseconds / 1000)
        proposed_usec = self._next_odometry_time_usec(now_usec)
        try:
            self._send_odometry(self._cached_payload, proposed_usec)
        except Exception as exc:
            self._logger.warning(f'Resend failed: {exc}')

    def _maybe_log_debug(self, odometry, pose_cov, vel_cov, quality, velocity_from_status):
        if self._vision_debug_log_period_sec <= 0.0:
            return
        now = self._node.get_clock().now().nanoseconds / 1e9
        if (now - self._last_debug_log_monotonic) < self._vision_debug_log_period_sec:
            return
        self._last_debug_log_monotonic = now
        prefix = 'ODOMETRY -> ' if self._forward_enabled else 'ODOMETRY (DRY) -> '
        self._logger.info(
            prefix
            + f'pos_ned=({odometry.position_ned[0]:.3f}, {odometry.position_ned[1]:.3f}, {odometry.position_ned[2]:.3f}) '
            + f'q_wxyz=({odometry.quaternion_wxyz[0]:.3f}, {odometry.quaternion_wxyz[1]:.3f}, '
            + f'{odometry.quaternion_wxyz[2]:.3f}, {odometry.quaternion_wxyz[3]:.3f}) '
            + f'vel_body_frd=({odometry.velocity_body_frd[0]:.3f}, '
            + f'{odometry.velocity_body_frd[1]:.3f}, {odometry.velocity_body_frd[2]:.3f}) '
            + f'pos_var={pose_cov[0]:.5f} rot_var={pose_cov[15]:.5f} '
            + f'vel_var={vel_cov[0]:.5f} quality={quality} '
            + f'src={"nav_status" if velocity_from_status else "diff"}'
        )
