import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import time
import csv
import os

class TelemetryLoggerNode(Node):
    def __init__(self):
        super().__init__('telemetry_logger_node')

        self.declare_parameter('connection_string', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('log_dir', '/home/nvidia/vtol_ws/')

        conn_str = self.get_parameter('connection_string').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        log_dir = self.get_parameter('log_dir').get_parameter_value().string_value

        self.get_logger().info(f'Connecting to ArduPilot on {conn_str} at {baud} baud (READ ONLY)...')
        
        try:
            self.master = mavutil.mavlink_connection(conn_str, baud=baud)
            self.master.wait_heartbeat(timeout=5)
            self.get_logger().info(f'Successfully connected to ArduPilot (System ID: {self.master.target_system})')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ArduPilot: {e}')
            self.master = None
            return

        # Create CSV file
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        self.csv_filename = os.path.join(log_dir, f'flight_data_{timestamp}.csv')
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write header
        self.csv_writer.writerow([
            'timestamp', 'ros_time',
            'roll', 'pitch', 'yaw', 
            'lat', 'lon', 'alt', 'relative_alt',
            'vx', 'vy', 'vz',
            'heading', 'airspeed', 'groundspeed',
            'voltage', 'current', 'battery_remaining'
        ])
        
        self.get_logger().info(f'Logging telemetry to {self.csv_filename}')

        # State variables to keep latest data
        self.attitude = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.global_pos = {'lat': 0, 'lon': 0, 'alt': 0, 'relative_alt': 0, 'vx': 0, 'vy': 0, 'vz': 0, 'hdg': 0}
        self.vfr_hud = {'airspeed': 0.0, 'groundspeed': 0.0, 'heading': 0}
        self.battery = {'voltage': 0.0, 'current': 0.0, 'remaining': 0}

        # Request data streams
        self.request_data_stream(mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10) # ATTITUDE
        self.request_data_stream(mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10) # GLOBAL_POSITION_INT
        self.request_data_stream(mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10) # VFR_HUD
        self.request_data_stream(mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2) # SYS_STATUS

        # Start reading loop in a timer (fast enough to catch messages)
        self.timer = self.create_timer(0.01, self.read_mavlink)
        self.log_timer = self.create_timer(0.1, self.log_data) # Log at 10Hz

    def request_data_stream(self, stream_id, rate):
        if self.master is None: return
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            stream_id,
            rate,
            1 # Start
        )

    def read_mavlink(self):
        if self.master is None: return
        
        while True:
            msg = self.master.recv_match(blocking=False)
            if not msg:
                break
                
            msg_type = msg.get_type()
            
            if msg_type == 'ATTITUDE':
                self.attitude['roll'] = msg.roll
                self.attitude['pitch'] = msg.pitch
                self.attitude['yaw'] = msg.yaw
            elif msg_type == 'GLOBAL_POSITION_INT':
                self.global_pos['lat'] = msg.lat / 1e7
                self.global_pos['lon'] = msg.lon / 1e7
                self.global_pos['alt'] = msg.alt / 1000.0
                self.global_pos['relative_alt'] = msg.relative_alt / 1000.0
                self.global_pos['vx'] = msg.vx / 100.0
                self.global_pos['vy'] = msg.vy / 100.0
                self.global_pos['vz'] = msg.vz / 100.0
                self.global_pos['hdg'] = msg.hdg / 100.0
            elif msg_type == 'VFR_HUD':
                self.vfr_hud['airspeed'] = msg.airspeed
                self.vfr_hud['groundspeed'] = msg.groundspeed
                self.vfr_hud['heading'] = msg.heading
            elif msg_type == 'SYS_STATUS':
                self.battery['voltage'] = msg.voltage_battery / 1000.0
                self.battery['current'] = msg.current_battery / 100.0
                self.battery['remaining'] = msg.battery_remaining

    def log_data(self):
        if self.master is None: return
        
        now = time.time()
        ros_time = self.get_clock().now().nanoseconds / 1e9
        
        self.csv_writer.writerow([
            now, ros_time,
            self.attitude['roll'], self.attitude['pitch'], self.attitude['yaw'],
            self.global_pos['lat'], self.global_pos['lon'], self.global_pos['alt'], self.global_pos['relative_alt'],
            self.global_pos['vx'], self.global_pos['vy'], self.global_pos['vz'],
            self.vfr_hud['heading'], self.vfr_hud['airspeed'], self.vfr_hud['groundspeed'],
            self.battery['voltage'], self.battery['current'], self.battery['remaining']
        ])
        # Flush to ensure data is saved even if crashed
        self.csv_file.flush()

    def destroy_node(self):
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
