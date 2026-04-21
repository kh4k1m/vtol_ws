import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageListener(Node):
    def __init__(self):
        super().__init__('image_listener')
        self.subscription = self.create_subscription(
            Image,
            '/world/runway/model/alti_transition_quad/link/down_camera_link/sensor/down_camera_sensor/image',
            self.listener_callback,
            10)
        self.count = 0

    def listener_callback(self, msg):
        self.get_logger().info(f'Received image: {msg.width}x{msg.height}, encoding={msg.encoding}, step={msg.step}, data_len={len(msg.data)}')
        self.count += 1
        if self.count > 3:
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = ImageListener()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
