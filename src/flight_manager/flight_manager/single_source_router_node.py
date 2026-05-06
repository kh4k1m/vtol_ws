"""Forward a single navigation source to the unified vision topics.

In single-source modes (``navigation: tags | dpvo | orb_slam3``) the
chosen source publishes to its own native topic (e.g. ``/tag_localizer/pose``)
so that loggers, viz and replay always see the source-named channel.

Downstream consumers - the MAVLink bridge, ``tf_broadcaster_node`` and
``mission_manager_node`` - are written against the unified
``/vision_pose_enu`` + ``/nav/status`` pair (the same contract that
``vision_fusion_node`` honors in hybrid mode). This node simply forwards
the active source's pose / nav status 1:1 onto those unified topics so
the rest of the stack does not need to know which navigation backend is
running.

Whether the unified pose actually reaches ArduPilot (and whether
``mission_manager`` is even started) is decided one level up by
``flight.yaml``'s ``mode``; this router publishes unconditionally.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from marker_interfaces.msg import NavStatus
from rclpy.node import Node


class SingleSourceRouterNode(Node):
    def __init__(self):
        super().__init__('single_source_router_node')

        self.declare_parameter('source_pose_topic', '/tag_localizer/pose')
        self.declare_parameter(
            'source_nav_status_topic', '/tag_localizer/nav_status'
        )
        self.declare_parameter('out_pose_topic', '/vision_pose_enu')
        self.declare_parameter('out_nav_status_topic', '/nav/status')
        self.declare_parameter('source_label', 'unknown')

        src_pose = self.get_parameter('source_pose_topic').value
        src_nav = self.get_parameter('source_nav_status_topic').value
        out_pose = self.get_parameter('out_pose_topic').value
        out_nav = self.get_parameter('out_nav_status_topic').value
        label = self.get_parameter('source_label').value

        self.pose_pub = self.create_publisher(PoseStamped, out_pose, 10)
        self.nav_status_pub = self.create_publisher(NavStatus, out_nav, 10)

        self.create_subscription(
            PoseStamped, src_pose, self._pose_cb, 10
        )
        self.create_subscription(
            NavStatus, src_nav, self._nav_status_cb, 10
        )

        self.get_logger().info(
            f"single_source_router up: source='{label}' "
            f"{src_pose} -> {out_pose}, "
            f"{src_nav} -> {out_nav}"
        )

    def _pose_cb(self, msg: PoseStamped):
        self.pose_pub.publish(msg)

    def _nav_status_cb(self, msg: NavStatus):
        self.nav_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SingleSourceRouterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
