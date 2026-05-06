"""GPS-only takeoff and land regression test.

Sequence:
    WAIT_CONN -> WAIT_GPS -> GUIDED -> ARM -> TAKEOFF -> WAIT_ALT
                -> HOVER -> LAND -> DONE

Landing is handled natively by the flight controller (QLAND mode for VTOLs).
For a soft landing, ensure ArduPilot parameters Q_LAND_SPEED and 
Q_LAND_FINAL_ALT are configured correctly in the flight controller.
"""

import time

import rclpy
from flight_interfaces.srv import SetMode, Takeoff
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float64
from std_srvs.srv import SetBool, Trigger


class GpsTakeoffLandNode(Node):
    def __init__(self):
        super().__init__('gps_takeoff_land_node')
        self.declare_parameter('target_altitude_m', 7.0)
        self.declare_parameter('hover_duration_sec', 10.0)

        self.target_alt = float(self.get_parameter('target_altitude_m').value)
        self.hover_duration = float(self.get_parameter('hover_duration_sec').value)

        self.current_alt = 0.0
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.is_connected = False

        self.create_subscription(
            Float64, '/ap/relative_alt', self.alt_cb, qos_profile_sensor_data
        )
        self.create_subscription(Bool, '/ap/connected', self.conn_cb, 10)
        self.create_subscription(
            NavSatFix, '/ap/gps/fix', self.gps_cb, qos_profile_sensor_data
        )

        self.set_mode_client = self.create_client(SetMode, '/ap/cmd/set_mode')
        self.arm_client = self.create_client(SetBool, '/ap/cmd/arm')
        self.takeoff_client = self.create_client(Takeoff, '/ap/cmd/takeoff')
        self.land_client = self.create_client(Trigger, '/ap/cmd/land')

        self.state = 'WAIT_CONN'
        self.timer = self.create_timer(0.5, self.loop)
        self.state_start_time = time.time()

        self.get_logger().info('GPS Takeoff & Land Node initialized.')

    def alt_cb(self, msg):
        self.current_alt = msg.data

    def conn_cb(self, msg):
        self.is_connected = msg.data

    def gps_cb(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def call_service(self, client, req, name):
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'Service {name} not available')
            return None
        return client.call_async(req)

    def set_state(self, new_state):
        self.state = new_state
        self.state_start_time = time.time()
        self.get_logger().info(f'>>> STATE: {new_state} <<<')

    def loop(self):
        elapsed = time.time() - self.state_start_time

        if self.state == 'WAIT_CONN':
            if self.is_connected:
                self.get_logger().info('Connected to ArduPilot!')
                self.set_state('WAIT_GPS')

        elif self.state == 'WAIT_GPS':
            if self.current_lat != 0.0 and self.current_lon != 0.0:
                self.get_logger().info(
                    f'GPS fix acquired: lat={self.current_lat:.7f}, '
                    f'lon={self.current_lon:.7f}'
                )
                self.set_state('GUIDED')
            elif elapsed > 30.0:
                self.get_logger().error('Timeout waiting for GPS fix; aborting.')
                self.set_state('DONE')

        elif self.state == 'GUIDED':
            req = SetMode.Request()
            req.mode = 'GUIDED'
            self.call_service(self.set_mode_client, req, 'set_mode')
            self.set_state('ARM')

        elif self.state == 'ARM':
            if elapsed > 2.0:
                req = SetBool.Request()
                req.data = True
                self.call_service(self.arm_client, req, 'arm')
                self.set_state('TAKEOFF')

        elif self.state == 'TAKEOFF':
            if elapsed > 3.0:
                req = Takeoff.Request()
                req.altitude_m = self.target_alt
                self.call_service(self.takeoff_client, req, 'takeoff')
                self.get_logger().info(f'Taking off to {self.target_alt}m...')
                self.set_state('WAIT_ALT')

        elif self.state == 'WAIT_ALT':
            if self.current_alt >= self.target_alt - 0.5:
                self.get_logger().info(
                    f'Reached target altitude: {self.current_alt:.1f}m. '
                    f'Hovering for {self.hover_duration}s.'
                )
                self.set_state('HOVER')
            elif elapsed > 180.0: # Увеличил таймаут для больших высот
                self.get_logger().error('Takeoff timeout! Forcing land.')
                self.set_state('LAND')

        elif self.state == 'HOVER':
            if elapsed > self.hover_duration:
                self.set_state('LAND')

        elif self.state == 'LAND':
            # Вместо вызова land_client, переключаем полетный режим на QLAND
            req = SetMode.Request()
            req.mode = 'QLAND'
            self.call_service(self.set_mode_client, req, 'set_mode')
            self.get_logger().info('Landing commanded (QLAND mode).')
            self.set_state('DONE')

        elif self.state == 'DONE':
            pass


def main(args=None):
    rclpy.init(args=args)
    node = GpsTakeoffLandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
