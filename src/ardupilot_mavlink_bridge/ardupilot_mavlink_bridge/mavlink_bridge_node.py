import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

class MavlinkBridgeNode(Node):
    def __init__(self):
        super().__init__('mavlink_bridge_node')

        self.declare_parameter('connection_string', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        conn_str = self.get_parameter('connection_string').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.get_logger().info(f'Connecting to ArduPilot on {conn_str} at {baud} baud...')
        
        try:
            self.master = mavutil.mavlink_connection(conn_str, baud=baud)
            self.master.wait_heartbeat(timeout=5)
            self.get_logger().info(f'Successfully connected to ArduPilot (System ID: {self.master.target_system})')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ArduPilot: {e}')
            self.master = None

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            PoseStamped,
            '/vision_pose_enu',
            self.vision_pose_callback,
            qos_profile
        )

        self.get_logger().info('MAVLink bridge node started. Listening to /vision_pose_enu')
        
        self.timer = self.create_timer(1.0, self.send_ekf_origin)
        self.origin_sent_count = 0
        
        self.last_time = time.time()
        self.last_x = None
        self.last_y = None
        self.last_z = None

    def send_ekf_origin(self):
        if self.master is None:
            return
            
        self.origin_sent_count += 1
        if self.origin_sent_count > 5:
            self.timer.cancel()
            return
            
        self.get_logger().info(f'Sending EKF Global Origin (attempt {self.origin_sent_count}/5)...')
        
        target_system = self.master.target_system
        
        try:
            self.master.mav.set_gps_global_origin_send(
                target_system,
                0,      
                0,      
                0,      
                int(time.time() * 1e6) 
            )
        except Exception as e:
            self.get_logger().error(f'Failed to send EKF Origin: {e}')

    def vision_pose_callback(self, msg: PoseStamped):
        if self.master is None:
            return

        x_ned = msg.pose.position.y
        y_ned = msg.pose.position.x
        z_ned = -msg.pose.position.z

        q_enu = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        
        rot = R.from_quat(q_enu)
        roll_enu, pitch_enu, yaw_enu = rot.as_euler('xyz')
        
        yaw_ned = math.pi/2.0 - yaw_enu
        roll_ned = roll_enu
        pitch_ned = -pitch_enu

        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt > 0 and self.last_x is not None:
            vx = (x_ned - self.last_x) / dt
            vy = (y_ned - self.last_y) / dt
            vz = (z_ned - self.last_z) / dt
        else:
            vx, vy, vz = 0.0, 0.0, 0.0
            
        self.last_x, self.last_y, self.last_z = x_ned, y_ned, z_ned
        self.last_time = current_time

        usec = int(current_time * 1e6)

        try:
            self.master.mav.vision_position_estimate_send(
                usec,   
                x_ned,      
                y_ned,      
                z_ned,      
                roll_ned,   
                pitch_ned,  
                yaw_ned     
            )
            
            self.master.mav.vision_speed_estimate_send(
                usec,   
                vx,     
                vy,     
                vz      
            )
            
        except Exception as e:
            self.get_logger().error(f'Failed to send MAVLink message: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
