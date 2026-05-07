"""ROS 2 node that bridges between ArduPilot (over MAVLink) and the rest
of the visual navigation stack.

The node itself is intentionally thin: it owns the executor wiring and a
shared :class:`MavlinkConnection`. All real work happens in:

* :mod:`mavlink_io` - thread-safe pymavlink ownership and dispatch.
* :mod:`telemetry_publisher` - rebroadcast IMU/GPS/state to ROS topics.
* :mod:`vision_forwarder` - convert /vision_pose_enu + /nav/status to
  MAVLink ODOMETRY (with proper covariances).
* :mod:`command_services` - ROS service backends for arm/takeoff/land
  /goto/upload_mission/set_mode/switch_ekf_source. They run on a
  reentrant callback group so they no longer block the read loop.

A multi-threaded executor is required so that one service can wait on a
COMMAND_ACK while telemetry keeps streaming.
"""

from __future__ import annotations

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from .command_services import CommandServices
from .mavlink_io import MavlinkConnection
from .telemetry_publisher import TelemetryPublisher
from .vision_forwarder import VisionForwarder


class MavlinkBridgeNode(Node):
    def __init__(self):
        super().__init__('mavlink_bridge_node')

        self.declare_parameter('connection_string', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        conn_str = self.get_parameter('connection_string').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.get_logger().info(f'Connecting to ArduPilot on {conn_str} at {baud} baud...')
        self.connection = MavlinkConnection(conn_str, baud, self.get_logger())

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.telemetry = TelemetryPublisher(self, sensor_qos)
        self.telemetry.install(self.connection)

        self.vision = VisionForwarder(self, self.connection)
        self.commands = CommandServices(self, self.connection, self.telemetry)

        io_group = MutuallyExclusiveCallbackGroup()
        status_group = MutuallyExclusiveCallbackGroup()

        self._connection_timer = self.create_timer(
            1.0, self._connection_tick, callback_group=io_group
        )
        self._read_timer = self.create_timer(
            0.01, self._read_tick, callback_group=io_group
        )
        self._status_timer = self.create_timer(
            0.5, self._status_tick, callback_group=status_group
        )
        # Outgoing GCS heartbeat at 1 Hz. Without this ArduPilot triggers
        # FS_GCS (Failsafe -> RTL/QLAND) ~5s after it stops hearing a GCS
        # heartbeat, which aborted every AUTO takeoff routed through this
        # bridge. See MavlinkConnection.send_gcs_heartbeat for details.
        self._heartbeat_timer = self.create_timer(
            1.0, self._heartbeat_tick, callback_group=io_group
        )

        forward_state = (
            'enabled' if self.vision._forward_enabled else 'DISABLED (log_only mode)'
        )
        self.get_logger().info(
            'MAVLink bridge node started. Listening to /vision_pose_enu and /nav/status. '
            f'Vision -> ODOMETRY forwarding: {forward_state}.'
        )

    def _connection_tick(self):
        self.connection.open_if_needed()

    def _read_tick(self):
        self.connection.poll()

    def _status_tick(self):
        self.telemetry.publish_status_snapshot()

    def _heartbeat_tick(self):
        self.connection.send_gcs_heartbeat()


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
