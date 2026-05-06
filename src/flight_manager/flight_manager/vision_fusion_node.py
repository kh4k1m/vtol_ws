"""Multiplex pose between AprilTag localizer and a visual-odometry source.

The VO source is intentionally generic - the node does not know whether the
poses come from DPVO, ORB-SLAM3 or any future backend. The launch file wires
the chosen backend's topics into ``vo_pose_topic`` / ``vo_nav_status_topic``
and sets ``vo_source_label`` for diagnostics. Both backends publish poses in
their own non-gravity-aligned local frame; we estimate a full SE(3) offset
that aligns them to the tag map whenever the tag is in view, then apply that
offset while the tag is lost.

Subscribes to:
  * /tag_localizer/pose       (PoseStamped, ENU)
  * <vo_pose_topic>           (PoseStamped, VO local frame)
  * <vo_nav_status_topic>     (NavStatus,  optional VO health from backend)
  * /tag_localizer/nav_status (NavStatus, optional tag health)

Publishes:
  * /vision_pose_enu  (PoseStamped) - the active pose to be sent to ArduPilot
  * /nav/status       (NavStatus)   - the active source health/uncertainty

State machine:
  WAITING_FOR_TAG -> TAG_MODE -> VO_MODE -> SMOOTH_RETURN -> TAG_MODE

In TAG_MODE we re-estimate ``T_map_vo`` so that the VO pose, after applying
the offset, matches the tag pose. When the tag is lost we switch to VO_MODE
and apply the *frozen* offset, which is correct because the offset itself is,
by construction, what aligns the two frames.

The offset is a full SE(3) (rotation + translation) - VO frames are rarely
exactly gravity-aligned, so a 6-DoF alignment is the only way to get usable
roll/pitch/z out of VO during tag-lost intervals.
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from marker_interfaces.msg import NavStatus
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R


class VisionFusionNode(Node):
    def __init__(self):
        super().__init__('vision_fusion_node')

        self.declare_parameter('fusion_mode', 'auto')  # auto | tags_only | vo_only
        self.declare_parameter('vo_timeout_sec', 0.5)
        self.declare_parameter('tag_timeout_sec', 0.5)
        self.declare_parameter('smooth_return_rate_m_s', 0.5)
        self.declare_parameter('smooth_return_rate_rad_s', 0.3)
        self.declare_parameter('smooth_return_max_jump_m', 1.5)

        self.declare_parameter('vo_pose_topic', '/vo/pose')
        self.declare_parameter('vo_nav_status_topic', '/vo/nav_status')
        self.declare_parameter('vo_source_label', 'vo')

        self.declare_parameter('default_position_std_m', 0.30)
        self.declare_parameter('default_rotation_std_rad', 0.10)
        self.declare_parameter('default_velocity_std_m_s', 0.50)

        raw_mode = str(self.get_parameter('fusion_mode').value)
        # backward compat: old launches used 'dpvo_only'
        self.fusion_mode = 'vo_only' if raw_mode == 'dpvo_only' else raw_mode
        self.vo_timeout_sec = float(self.get_parameter('vo_timeout_sec').value)
        self.tag_timeout_sec = float(self.get_parameter('tag_timeout_sec').value)
        self.smooth_return_rate_m_s = float(self.get_parameter('smooth_return_rate_m_s').value)
        self.smooth_return_rate_rad_s = float(self.get_parameter('smooth_return_rate_rad_s').value)
        self.smooth_return_max_jump_m = float(self.get_parameter('smooth_return_max_jump_m').value)
        self.vo_pose_topic = str(self.get_parameter('vo_pose_topic').value)
        self.vo_nav_status_topic = str(self.get_parameter('vo_nav_status_topic').value)
        self.vo_source_label = str(self.get_parameter('vo_source_label').value)
        self.default_position_std_m = float(self.get_parameter('default_position_std_m').value)
        self.default_rotation_std_rad = float(self.get_parameter('default_rotation_std_rad').value)
        self.default_velocity_std_m_s = float(self.get_parameter('default_velocity_std_m_s').value)

        self.tag_sub = self.create_subscription(
            PoseStamped, '/tag_localizer/pose', self._tag_cb, 10
        )
        self.vo_sub = self.create_subscription(
            PoseStamped, self.vo_pose_topic, self._vo_cb, 10
        )
        self.tag_status_sub = self.create_subscription(
            NavStatus, '/tag_localizer/nav_status', self._tag_status_cb, 10
        )
        self.vo_status_sub = self.create_subscription(
            NavStatus, self.vo_nav_status_topic, self._vo_status_cb, 10
        )

        self.pose_pub = self.create_publisher(PoseStamped, '/vision_pose_enu', 10)
        self.nav_status_pub = self.create_publisher(NavStatus, '/nav/status', 10)

        self.timer = self.create_timer(0.05, self._tick)

        self.state = 'WAITING_FOR_TAG'

        self.last_tag_pose: Optional[PoseStamped] = None
        self.last_tag_time = 0.0
        self.last_tag_nav_status: Optional[NavStatus] = None

        self.last_vo_pose: Optional[PoseStamped] = None
        self.last_vo_time = 0.0
        self.last_vo_nav_status: Optional[NavStatus] = None

        # Offset T_map_vo such that pose_map = T_map_vo @ pose_vo.
        self.T_map_vo_R = np.eye(3)
        self.T_map_vo_t = np.zeros(3)
        self.target_R = np.eye(3)
        self.target_t = np.zeros(3)

        self.last_update_time = time.monotonic()

        self.get_logger().info(
            f"Vision fusion ({self.fusion_mode}) started in state {self.state}; "
            f"vo backend='{self.vo_source_label}' subscribing to "
            f"pose='{self.vo_pose_topic}' status='{self.vo_nav_status_topic}'"
        )

    def _tag_cb(self, msg: PoseStamped):
        self.last_tag_pose = msg
        self.last_tag_time = time.monotonic()

    def _tag_status_cb(self, msg: NavStatus):
        self.last_tag_nav_status = msg

    def _vo_cb(self, msg: PoseStamped):
        self.last_vo_pose = msg
        self.last_vo_time = time.monotonic()

    def _vo_status_cb(self, msg: NavStatus):
        self.last_vo_nav_status = msg

    def _tick(self):
        now = time.monotonic()
        dt = max(0.0, now - self.last_update_time)
        self.last_update_time = now

        tag_active = (
            self.last_tag_pose is not None
            and (now - self.last_tag_time) < self.tag_timeout_sec
        )
        vo_active = (
            self.last_vo_pose is not None
            and (now - self.last_vo_time) < self.vo_timeout_sec
        )

        if self.fusion_mode == 'tags_only':
            self._publish_tag_only(tag_active)
            return
        if self.fusion_mode == 'vo_only':
            self._publish_vo_only(vo_active)
            return

        if self.state == 'WAITING_FOR_TAG':
            if tag_active:
                self.state = 'TAG_MODE'
                self.get_logger().info('Vision fusion: -> TAG_MODE')
            return

        if self.state == 'TAG_MODE':
            if not tag_active:
                if vo_active:
                    self.state = 'VO_MODE'
                    self.get_logger().info(
                        f"Vision fusion: tag lost -> VO_MODE ({self.vo_source_label})"
                    )
                else:
                    self.get_logger().warning(
                        'Vision fusion: tag and VO both stale; holding last pose'
                    )
                return
            if vo_active:
                R_t, t_t = self._estimate_offset()
                self.T_map_vo_R = R_t
                self.T_map_vo_t = t_t
                self.target_R = R_t.copy()
                self.target_t = t_t.copy()
            self._publish_pose(self.last_tag_pose, source='tags', from_tag=True)
            return

        if self.state == 'VO_MODE':
            if tag_active and vo_active:
                R_t, t_t = self._estimate_offset()
                self.target_R = R_t
                self.target_t = t_t
                jump_m = float(np.linalg.norm(self.target_t - self.T_map_vo_t))
                if jump_m > self.smooth_return_max_jump_m:
                    self.T_map_vo_R = self.target_R.copy()
                    self.T_map_vo_t = self.target_t.copy()
                    self.state = 'TAG_MODE'
                    self.get_logger().info(
                        f'Vision fusion: tag jump {jump_m:.2f} m exceeds threshold; '
                        'snapping to tag and -> TAG_MODE'
                    )
                else:
                    self.state = 'SMOOTH_RETURN'
                    self.get_logger().info(
                        f'Vision fusion: tag regained (jump {jump_m:.2f} m) -> SMOOTH_RETURN'
                    )
            elif vo_active:
                pose_map = self._project_vo_to_map(self.last_vo_pose)
                self.pose_pub.publish(pose_map)
                self._publish_vo_status_during_lost_tag()
            else:
                self.get_logger().warning('Vision fusion: lost VO in VO_MODE')
            return

        if self.state == 'SMOOTH_RETURN':
            if not tag_active:
                self.state = 'VO_MODE'
                self.get_logger().info(
                    'Vision fusion: lost tag during smooth return -> VO_MODE'
                )
                return
            if not vo_active:
                self.T_map_vo_R = self.target_R.copy()
                self.T_map_vo_t = self.target_t.copy()
                self.state = 'TAG_MODE'
                self.get_logger().info(
                    'Vision fusion: lost VO during smooth return -> TAG_MODE'
                )
                return

            target_R, target_t = self._estimate_offset()
            self.target_R = target_R
            self.target_t = target_t

            translation_done = self._step_translation(dt)
            rotation_done = self._step_rotation(dt)

            if translation_done and rotation_done:
                self.state = 'TAG_MODE'
                self.get_logger().info('Vision fusion: smooth return done -> TAG_MODE')

            pose_map = self._project_vo_to_map(self.last_vo_pose)
            self.pose_pub.publish(pose_map)
            self._publish_nav_status(
                'fusion', NavStatus.STATE_SETTLING, detail='smooth_return'
            )
            return

    def _publish_tag_only(self, tag_active):
        if tag_active:
            self.pose_pub.publish(self.last_tag_pose)
            self._publish_pose(self.last_tag_pose, source='tags', from_tag=True)
        else:
            self._publish_nav_status('tags', NavStatus.STATE_LOST)

    def _publish_vo_only(self, vo_active):
        if vo_active:
            self.pose_pub.publish(self.last_vo_pose)
            self._publish_vo_status_during_lost_tag()
        else:
            self._publish_nav_status(self.vo_source_label, NavStatus.STATE_LOST)

    def _publish_pose(self, pose: PoseStamped, source: str, from_tag: bool):
        self.pose_pub.publish(pose)
        if from_tag and self.last_tag_nav_status is not None:
            status = NavStatus()
            status.header.stamp = pose.header.stamp
            status.source = source
            status.state = self.last_tag_nav_status.state
            status.detail = self.last_tag_nav_status.detail
            status.quality_score = self.last_tag_nav_status.quality_score
            status.position_std_m = self.last_tag_nav_status.position_std_m
            status.rotation_std_rad = self.last_tag_nav_status.rotation_std_rad
            status.velocity_std_m_s = self.last_tag_nav_status.velocity_std_m_s
            status.has_velocity = self.last_tag_nav_status.has_velocity
            status.velocity_enu = self.last_tag_nav_status.velocity_enu
            status.visible_ids = list(self.last_tag_nav_status.visible_ids)
            self.nav_status_pub.publish(status)
        else:
            self._publish_nav_status(source, NavStatus.STATE_TRACKING)

    def _publish_vo_status_during_lost_tag(self):
        """Forward the VO backend's NavStatus (if any) into /nav/status.

        We label it with the active backend (e.g. 'orb_slam3') so the mission
        manager / ArduPilot bridge can apply the right covariance. If the VO
        backend never sent a NavStatus we fall back to a generic degraded
        state with the configured default std-devs.
        """
        status = NavStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        if self.last_vo_nav_status is not None:
            src = self.last_vo_nav_status
            status.source = (
                src.source if src.source else self.vo_source_label)
            status.state = (
                src.state if src.state != NavStatus.STATE_UNKNOWN
                else NavStatus.STATE_TRACKING_DEGRADED)
            status.detail = src.detail
            status.quality_score = src.quality_score
            status.position_std_m = src.position_std_m
            status.rotation_std_rad = src.rotation_std_rad
            status.velocity_std_m_s = src.velocity_std_m_s
            status.has_velocity = src.has_velocity
            status.velocity_enu = src.velocity_enu
            status.visible_ids = list(src.visible_ids)
        else:
            status.source = self.vo_source_label
            status.state = NavStatus.STATE_TRACKING_DEGRADED
            status.position_std_m = self.default_position_std_m
            status.rotation_std_rad = self.default_rotation_std_rad
            status.velocity_std_m_s = self.default_velocity_std_m_s
        self.nav_status_pub.publish(status)

    def _publish_nav_status(self, source: str, state: int, detail: str = ''):
        status = NavStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.source = source
        status.state = int(state)
        status.detail = detail
        status.position_std_m = self.default_position_std_m
        status.rotation_std_rad = self.default_rotation_std_rad
        status.velocity_std_m_s = self.default_velocity_std_m_s
        self.nav_status_pub.publish(status)

    # -- alignment helpers ---------------------------------------------------

    def _estimate_offset(self):
        """Return rotation+translation that maps VO pose into map frame.

        T_map_body = T_map_vo * T_vo_body
            => T_map_vo = T_map_body * T_vo_body^-1
        """
        T_map_body = self._pose_to_T(self.last_tag_pose)
        T_vo_body = self._pose_to_T(self.last_vo_pose)
        T_map_vo = T_map_body @ np.linalg.inv(T_vo_body)
        return T_map_vo[:3, :3], T_map_vo[:3, 3]

    def _project_vo_to_map(self, vo_pose: PoseStamped) -> PoseStamped:
        T_vo_body = self._pose_to_T(vo_pose)
        T_map_vo = np.eye(4)
        T_map_vo[:3, :3] = self.T_map_vo_R
        T_map_vo[:3, 3] = self.T_map_vo_t
        T_map_body = T_map_vo @ T_vo_body
        return self._T_to_pose(T_map_body, vo_pose.header.stamp)

    def _step_translation(self, dt) -> bool:
        diff = self.target_t - self.T_map_vo_t
        dist = float(np.linalg.norm(diff))
        max_step = self.smooth_return_rate_m_s * dt
        if dist <= max_step or dist <= 1e-6:
            self.T_map_vo_t = self.target_t.copy()
            return True
        self.T_map_vo_t = self.T_map_vo_t + (diff / dist) * max_step
        return False

    def _step_rotation(self, dt) -> bool:
        R_delta = self.target_R @ self.T_map_vo_R.T
        rotvec = R.from_matrix(R_delta).as_rotvec()
        angle = float(np.linalg.norm(rotvec))
        max_step = self.smooth_return_rate_rad_s * dt
        if angle <= max_step or angle <= 1e-6:
            self.T_map_vo_R = self.target_R.copy()
            return True
        axis = rotvec / angle
        step_R = R.from_rotvec(axis * max_step).as_matrix()
        self.T_map_vo_R = step_R @ self.T_map_vo_R
        return False

    @staticmethod
    def _pose_to_T(msg: PoseStamped) -> np.ndarray:
        T = np.eye(4)
        T[:3, 3] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        T[:3, :3] = R.from_quat(q).as_matrix()
        return T

    @staticmethod
    def _T_to_pose(T: np.ndarray, stamp) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(T[0, 3])
        msg.pose.position.y = float(T[1, 3])
        msg.pose.position.z = float(T[2, 3])
        q = R.from_matrix(T[:3, :3]).as_quat()
        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = VisionFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
