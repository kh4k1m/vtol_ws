import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os
import glob

class VideoPlayerNode(Node):
    def __init__(self):
        super().__init__('video_player_node')
        self.declare_parameter('video_dir', 'logs')
        self.declare_parameter('fps', 30.0)
        
        video_dir = self.get_parameter('video_dir').value
        fps = self.get_parameter('fps').value
        
        # Find all mp4 files in the directory and sort them
        search_pattern = os.path.join(video_dir, '*.mp4')
        self.video_files = sorted(glob.glob(search_pattern))
        
        if not self.video_files:
            self.get_logger().error(f"No video files found in {video_dir}")
            return
            
        self.current_video_idx = 0
        self.cap = cv2.VideoCapture(self.video_files[self.current_video_idx])
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video: {self.video_files[self.current_video_idx]}")
            return
            
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"Playing video sequence from {video_dir} at {fps} fps")
        self.get_logger().info(f"Current video: {self.video_files[self.current_video_idx]}")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info(f"End of video {self.video_files[self.current_video_idx]}.")
            self.current_video_idx += 1
            if self.current_video_idx >= len(self.video_files):
                self.get_logger().info("End of all videos. Restarting sequence...")
                self.current_video_idx = 0
                
            self.cap.release()
            self.cap = cv2.VideoCapture(self.video_files[self.current_video_idx])
            self.get_logger().info(f"Playing video: {self.video_files[self.current_video_idx]}")
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
