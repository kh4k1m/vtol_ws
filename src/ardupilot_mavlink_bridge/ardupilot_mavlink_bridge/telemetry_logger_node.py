"""CSV logger for ArduPilot telemetry.

Previously this node opened its own pymavlink connection in parallel
with ``mavlink_bridge_node`` - which would race for the same serial
port. Now it consumes the ROS topics already published by the bridge:

* /ap/imu/raw          (sensor_msgs/Imu)
* /ap/gps/fix          (sensor_msgs/NavSatFix)
* /ap/relative_alt     (std_msgs/Float64)
* /ap/flight_mode      (std_msgs/String)
* /ap/armed            (std_msgs/Bool)

This means it can run alongside the regular bridge with no extra setup,
and adding new fields requires only a ROS subscription instead of
keeping a separate MAVLink stream-rate request alive.
"""

from __future__ import annotations

import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Float64, String


class TelemetryLoggerNode(Node):
    def __init__(self):
        super().__init__('telemetry_logger_node')

        default_log_dir = os.environ.get(
            'VTOL_WS_ROOT', os.path.expanduser('~/vtol_ws')
        )
        self.declare_parameter('log_dir', default_log_dir)
        self.declare_parameter('log_rate_hz', 10.0)

        log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
        log_rate_hz = max(0.1, float(self.get_parameter('log_rate_hz').value))

        os.makedirs(log_dir, exist_ok=True)
        timestamp = time.strftime('%Y%m%d-%H%M%S')
        self.csv_filename = os.path.join(log_dir, f'flight_data_{timestamp}.csv')
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                'monotonic',
                'ros_time',
                'lin_acc_x',
                'lin_acc_y',
                'lin_acc_z',
                'gyro_x',
                'gyro_y',
                'gyro_z',
                'lat',
                'lon',
                'alt_msl',
                'relative_alt',
                'flight_mode',
                'armed',
            ]
        )
        self.csv_file.flush()
        self.get_logger().info(f'Logging telemetry to {self.csv_filename}')

        self.latest_imu: Imu = None
        self.latest_gps: NavSatFix = None
        self.relative_alt = math.nan
        self.flight_mode = ''
        self.armed = False

        self.create_subscription(Imu, '/ap/imu/raw', self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/ap/gps/fix', self._gps_cb, qos_profile_sensor_data)
        self.create_subscription(
            Float64, '/ap/relative_alt', self._rel_alt_cb, qos_profile_sensor_data
        )
        self.create_subscription(String, '/ap/flight_mode', self._mode_cb, 10)
        self.create_subscription(Bool, '/ap/armed', self._armed_cb, 10)

        self.timer = self.create_timer(1.0 / log_rate_hz, self._tick)

    def _imu_cb(self, msg: Imu):
        self.latest_imu = msg

    def _gps_cb(self, msg: NavSatFix):
        self.latest_gps = msg

    def _rel_alt_cb(self, msg: Float64):
        self.relative_alt = float(msg.data)

    def _mode_cb(self, msg: String):
        self.flight_mode = str(msg.data)

    def _armed_cb(self, msg: Bool):
        self.armed = bool(msg.data)

    def _tick(self):
        ros_time = self.get_clock().now().nanoseconds / 1e9
        imu = self.latest_imu
        gps = self.latest_gps
        self.csv_writer.writerow(
            [
                time.monotonic(),
                ros_time,
                getattr(getattr(imu, 'linear_acceleration', None), 'x', float('nan')) if imu else float('nan'),
                getattr(getattr(imu, 'linear_acceleration', None), 'y', float('nan')) if imu else float('nan'),
                getattr(getattr(imu, 'linear_acceleration', None), 'z', float('nan')) if imu else float('nan'),
                getattr(getattr(imu, 'angular_velocity', None), 'x', float('nan')) if imu else float('nan'),
                getattr(getattr(imu, 'angular_velocity', None), 'y', float('nan')) if imu else float('nan'),
                getattr(getattr(imu, 'angular_velocity', None), 'z', float('nan')) if imu else float('nan'),
                gps.latitude if gps else float('nan'),
                gps.longitude if gps else float('nan'),
                gps.altitude if gps else float('nan'),
                self.relative_alt,
                self.flight_mode,
                int(bool(self.armed)),
            ]
        )
        self.csv_file.flush()

    def destroy_node(self):
        if hasattr(self, 'csv_file') and self.csv_file:
            try:
                self.csv_file.close()
            except Exception:
                pass
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
