import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class VideoPlayerNode(Node):
    def __init__(self):
        super().__init__('video_player_node')
        self.declare_parameter('video_path', 'data/C0026/C0026.MP4')
        self.declare_parameter('fps', 30.0)
        
        video_path = self.get_parameter('video_path').value
        fps = self.get_parameter('fps').value
        
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video: {video_path}")
            return
            
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"Playing video {video_path} at {fps} fps")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("End of video. Restarting...")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()
            if not ret:
                return
                
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_optical_frame"
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VideoPlayerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
