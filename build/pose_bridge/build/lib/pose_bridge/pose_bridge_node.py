import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class PoseBridgeNode(Node):
    def __init__(self):
        super().__init__('pose_bridge_node')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            PoseStamped,
            '/tag_pose_camera',
            self.pose_callback,
            qos_profile
        )

        self.vision_pose_pub = self.create_publisher(
            PoseStamped,
            '/vision_pose',
            qos_profile
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info('Pose bridge node started (Python + TF2).')

    def pose_callback(self, msg: PoseStamped):
        # 1. Broadcast TF from map -> camera_link (Mock transformation)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'camera_link'
        
        # In a real scenario, you'd calculate the drone's position in the map
        # based on the tag's position in the camera frame.
        # Here we just mock it for the pipeline.
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

        # 2. Publish vision_pose for MAVLink
        vision_msg = PoseStamped()
        vision_msg.header.stamp = msg.header.stamp
        vision_msg.header.frame_id = 'map'
        
        # Applying NED (North-East-Down) or ENU (East-North-Up) conversion if needed
        # For now, just pass it through
        vision_msg.pose = msg.pose
        
        self.vision_pose_pub.publish(vision_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
