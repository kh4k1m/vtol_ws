"""Publish ROS TF tree based on the active vision pose source.

* ``map -> base_link``         dynamic broadcast from /vision_pose_enu
* ``base_link -> camera_optical_frame`` static broadcast configured via
  ROS parameters (must match the calibration used by tag_localizer)

This node is intentionally separate from the localizer so that any
navigation source (tag_localizer, dpvo, vision_fusion) can drive TF
without each having to depend on tf2.
"""

from __future__ import annotations

import math
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class TfBroadcasterNode(Node):
    def __init__(self):
        super().__init__('vision_tf_broadcaster')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('cam_trans', [0.0, 0.0, -0.18])
        self.declare_parameter('cam_rot', [180.0, 0.0, -90.0])

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_link_frame = str(self.get_parameter('base_link_frame').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        cam_trans = list(self.get_parameter('cam_trans').value)
        cam_rot = list(self.get_parameter('cam_rot').value)

        self.dynamic_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self._publish_static_camera_transform(cam_trans, cam_rot)

        self.create_subscription(PoseStamped, '/vision_pose_enu', self._pose_cb, 10)
        self.get_logger().info(
            f'TF broadcaster active: {self.map_frame} -> {self.base_link_frame} '
            f'(dynamic), {self.base_link_frame} -> {self.camera_frame} (static)'
        )

    def _publish_static_camera_transform(self, cam_trans, cam_rot):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_link_frame
        msg.child_frame_id = self.camera_frame
        msg.transform.translation.x = float(cam_trans[0])
        msg.transform.translation.y = float(cam_trans[1])
        msg.transform.translation.z = float(cam_trans[2])
        q = R.from_euler('xyz', [math.radians(a) for a in cam_rot]).as_quat()
        msg.transform.rotation.x = float(q[0])
        msg.transform.rotation.y = float(q[1])
        msg.transform.rotation.z = float(q[2])
        msg.transform.rotation.w = float(q[3])
        self.static_broadcaster.sendTransform(msg)

    def _pose_cb(self, pose: PoseStamped):
        msg = TransformStamped()
        msg.header.stamp = pose.header.stamp
        msg.header.frame_id = pose.header.frame_id or self.map_frame
        msg.child_frame_id = self.base_link_frame
        msg.transform.translation.x = float(pose.pose.position.x)
        msg.transform.translation.y = float(pose.pose.position.y)
        msg.transform.translation.z = float(pose.pose.position.z)
        msg.transform.rotation = pose.pose.orientation
        self.dynamic_broadcaster.sendTransform(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfBroadcasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
