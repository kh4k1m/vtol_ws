import time
import math
import rclpy
from rclpy.node import Node
from flight_interfaces.srv import SetMode, UploadMission
from std_srvs.srv import SetBool
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

def add_meters_to_latlon(lat, lon, dx_meters, dy_meters):
    R = 6371000.0
    new_lat = lat + math.degrees(dy_meters / R)
    new_lon = lon + math.degrees(dx_meters / (R * math.cos(math.radians(lat))))
    return new_lat, new_lon


def latlon_from_home_meters(home_lat, home_lon, dx, dy):
    return add_meters_to_latlon(home_lat, home_lon, dx, dy)


def point_on_radius(home_lat, home_lon, radius_m, bearing_rad):
    """bearing_rad: 0 = North, increases clockwise (same as sin/cos pairing below)."""
    dx = radius_m * math.sin(bearing_rad)
    dy = radius_m * math.cos(bearing_rad)
    return latlon_from_home_meters(home_lat, home_lon, dx, dy)


# Default bearings (deg, clockwise from North). Order is mission order.
# Leading 0° is moved to end when defer_first_zero_bearing is true (fly that leg last).
_DEFAULT_BEARING_DEG = (
    0.0,
    30.0,
    210.0,
    240.0,
    60.0,
    90.0,
    270.0,
    300.0,
    120.0,
    150.0,
    330.0,
)


def mission_bearings_deg(bearings, defer_first_zero):
    """Return list of float bearings in flight order."""
    out = [float(x) for x in bearings]
    if defer_first_zero and out and out[0] == 0.0:
        return out[1:] + [0.0]
    return out


def ring_waypoints_from_bearings_deg(home_lat, home_lon, radius_m, bearings_deg):
    lats, lons = [], []
    for deg in bearings_deg:
        la, lo = point_on_radius(home_lat, home_lon, radius_m, math.radians(deg))
        lats.append(la)
        lons.append(lo)
    return lats, lons

class GpsRangeTestNode(Node):
    def __init__(self):
        super().__init__('gps_range_test_node')
        self.declare_parameter('target_altitude_m', 40.0)
        self.declare_parameter('waypoint_distance_m', 300.0)
        self.declare_parameter('bearing_deg', list(_DEFAULT_BEARING_DEG))
        self.declare_parameter('defer_first_zero_bearing', True)

        self.target_alt = self.get_parameter('target_altitude_m').value
        self.wp_dist = self.get_parameter('waypoint_distance_m').value

        self.current_alt = 0.0
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.is_connected = False
        self.is_armed = False

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Float64, '/ap/relative_alt', self.alt_cb, qos_profile)
        self.create_subscription(Bool, '/ap/connected', self.conn_cb, 10)
        self.create_subscription(Bool, '/ap/armed', self.armed_cb, 10)
        self.create_subscription(NavSatFix, '/ap/gps/fix', self.gps_cb, qos_profile)

        self.set_mode_client = self.create_client(SetMode, '/ap/cmd/set_mode')
        self.arm_client = self.create_client(SetBool, '/ap/cmd/arm')
        self.upload_mission_client = self.create_client(UploadMission, '/ap/cmd/upload_mission')

        self.state = 'WAIT_CONN'
        self.timer = self.create_timer(0.5, self.loop)
        self.state_start_time = time.time()
        
        self.home_lat = 0.0
        self.home_lon = 0.0

        self.get_logger().info(
            'GPS Range Test Node (fixed bearing list on ring, no home legs) initialized.'
        )

    def alt_cb(self, msg):
        self.current_alt = msg.data

    def conn_cb(self, msg):
        self.is_connected = msg.data

    def armed_cb(self, msg):
        self.is_armed = msg.data

    def gps_cb(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def call_service(self, client, req, name):
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'Service {name} not available')
            return None
        future = client.call_async(req)
        return future

    def set_state(self, new_state):
        self.state = new_state
        self.state_start_time = time.time()
        self.get_logger().info(f'>>> STATE: {new_state} <<<')

    def loop(self):
        now = time.time()
        elapsed = now - self.state_start_time

        if self.state == 'WAIT_CONN':
            if self.is_connected and self.current_lat != 0.0:
                self.get_logger().info('Connected to ArduPilot and GPS valid!')
                self.set_state('UPLOAD_MISSION')

        elif self.state == 'UPLOAD_MISSION':
            self.home_lat = self.current_lat
            self.home_lon = self.current_lon

            raw_b = list(self.get_parameter('bearing_deg').value)
            defer = bool(self.get_parameter('defer_first_zero_bearing').value)
            bearings = mission_bearings_deg(raw_b, defer)

            lats, lons = ring_waypoints_from_bearings_deg(
                self.home_lat, self.home_lon, self.wp_dist, bearings
            )
            n = len(bearings)
            alts = [self.target_alt] * n

            req = UploadMission.Request()
            req.lats = lats
            req.lons = lons
            req.alts = alts

            self.call_service(self.upload_mission_client, req, 'upload_mission')
            if defer and raw_b and float(raw_b[0]) == 0.0:
                self.get_logger().info(
                    'bearing_deg[0]==0: flying 0° on ring as last waypoint (defer_first_zero_bearing).'
                )
            self.get_logger().info(
                f'Mission uploaded: {n} waypoints at radius {self.wp_dist:.0f} m, '
                f'bearings deg: {bearings}.'
            )
            self.set_state('GUIDED')

        elif self.state == 'GUIDED':
            if elapsed > 2.0:
                req = SetMode.Request()
                req.mode = 'GUIDED'
                self.call_service(self.set_mode_client, req, 'set_mode')
                self.set_state('ARM')

        elif self.state == 'ARM':
            if elapsed > 2.0:
                req = SetBool.Request()
                req.data = True
                self.call_service(self.arm_client, req, 'arm')
                self.set_state('AUTO')

        elif self.state == 'AUTO':
            if elapsed > 2.0:
                req = SetMode.Request()
                req.mode = 'AUTO'
                self.call_service(self.set_mode_client, req, 'set_mode')
                self.get_logger().info('Switched to AUTO mode. Mission started!')
                self.set_state('WAIT_LAND')

        elif self.state == 'WAIT_LAND':
            # In AUTO mode, it will fly the mission, RTL, and land automatically.
            # We just wait until it disarms.
            if elapsed > 30.0 and not self.is_armed:
                self.get_logger().info('Disarmed! Mission complete.')
                self.set_state('DONE')

        elif self.state == 'DONE':
            pass

def main(args=None):
    rclpy.init(args=args)
    node = GpsRangeTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
