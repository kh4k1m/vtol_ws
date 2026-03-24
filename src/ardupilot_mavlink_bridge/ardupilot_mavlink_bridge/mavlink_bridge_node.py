import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import math
import time

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

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
            '/vision_pose',
            self.vision_pose_callback,
            qos_profile
        )

        self.get_logger().info('MAVLink bridge node started. Listening to /vision_pose')
        
        # Send EKF Origin to initialize the filter without GPS
        # We need to send it a few times to ensure it's received and processed
        self.timer = self.create_timer(1.0, self.send_ekf_origin)
        self.origin_sent_count = 0
        
        # Variables for velocity calculation
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
        
        # Target system and component
        target_system = self.master.target_system
        
        # Send SET_GPS_GLOBAL_ORIGIN message
        # https://mavlink.io/en/messages/common.html#SET_GPS_GLOBAL_ORIGIN
        # We set it to Lat: 0, Lon: 0, Alt: 0
        try:
            self.master.mav.set_gps_global_origin_send(
                target_system,
                0,      # latitude (int32, 1e7)
                0,      # longitude (int32, 1e7)
                0,      # altitude (int32, mm)
                int(time.time() * 1e6) # time_usec
            )
        except Exception as e:
            self.get_logger().error(f'Failed to send EKF Origin: {e}')

    def vision_pose_callback(self, msg: PoseStamped):
        if self.master is None:
            return

        # Extract position
        # Convert to NED (North-East-Down) coordinate system expected by ArduPilot
        # OpenCV Camera Frame: X right, Y down, Z forward
        # ArduPilot NED Frame: X North(forward), Y East(right), Z Down
        
        # If the camera is pointing STRAIGHT FORWARD (like a dashcam):
        # OpenCV Z (depth/forward) -> NED X (North/forward)
        # OpenCV X (right)         -> NED Y (East/right)
        # OpenCV Y (down)          -> NED Z (Down)
        
        # Since solvePnP gives the position of the TAG relative to the CAMERA,
        # we need to invert it to get the position of the CAMERA relative to the TAG.
        # Camera_pos = -Tag_pos
        
        x = msg.pose.position.z   # Forward/Backward (depth)
        y = msg.pose.position.x   # Left/Right
        z = msg.pose.position.y   # Up/Down (Y in OpenCV is down, which matches NED Z)

        # Extract orientation and convert to Euler angles
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        roll, pitch, yaw = euler_from_quaternion(qx, qy, qz, qw)
        
        # Adjust angles for downward-facing camera
        # If the camera is pitched down 90 degrees relative to the drone body
        # we need to transform the orientation to match the drone's body frame
        # For a simple test, let's just send 0 for roll and pitch, and only send yaw
        # This tells EKF3: "Trust my position, but don't use my roll/pitch for attitude correction"
        roll = 0.0
        pitch = 0.0

        # Calculate velocity (simple finite difference)
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt > 0 and self.last_x is not None:
            vx = (x - self.last_x) / dt
            vy = (y - self.last_y) / dt
            vz = (z - self.last_z) / dt
        else:
            vx, vy, vz = 0.0, 0.0, 0.0
            
        self.last_x, self.last_y, self.last_z = x, y, z
        self.last_time = current_time

        # Get current time in microseconds
        usec = int(current_time * 1e6)

        # Send VISION_POSITION_ESTIMATE
        # https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
        try:
            self.master.mav.vision_position_estimate_send(
                usec,   # us Timestamp (UNIX time or time since system boot)
                x,      # Global X position
                y,      # Global Y position
                z,      # Global Z position
                roll,   # Roll angle
                pitch,  # Pitch angle
                yaw     # Yaw angle
            )
            
            # Send VISION_SPEED_ESTIMATE to help EKF3 fuse the data
            # https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE
            self.master.mav.vision_speed_estimate_send(
                usec,   # us Timestamp
                vx,     # Global X speed
                vy,     # Global Y speed
                vz      # Global Z speed
            )
            
            self.get_logger().info(f'Sent to ArduPilot -> x: {x:.2f}, y: {y:.2f}, z: {z:.2f}, yaw: {math.degrees(yaw):.1f}°')
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
