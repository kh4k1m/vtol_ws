"""Republish ArduPilot telemetry from MAVLink to ROS topics.

Topics:

* ``/ap/relative_alt``         - std_msgs/Float64, relative altitude (m, AGL)
* ``/ap/connected``            - std_msgs/Bool, heartbeat-derived liveness
* ``/ap/armed``                - std_msgs/Bool
* ``/ap/external_nav_ready``   - std_msgs/Bool, EKF3 external nav ready
* ``/ap/flight_mode``          - std_msgs/String
* ``/ap/imu/raw``              - sensor_msgs/Imu, RAW_IMU translated to SI
                                 (no orientation - REP-145 compliant)
* ``/ap/imu/data``             - sensor_msgs/Imu, RAW_IMU + ATTITUDE-derived
                                 orientation (EKF-fused). This is what
                                 anything that needs vehicle pose (compass
                                 yaw, anchor bootstrap, ORB-SLAM IMU mode)
                                 should subscribe to.
* ``/ap/attitude``             - geometry_msgs/Vector3Stamped, ATTITUDE
                                 message converted to (roll, pitch, yaw)
                                 in radians, ENU/FRD convention as
                                 ArduPilot reports it. Convenience topic
                                 for nodes that only need the yaw/heading.
* ``/ap/gps/fix``              - sensor_msgs/NavSatFix, GLOBAL_POSITION_INT

The bridge previously published ``/mavros/imu/data_raw`` and
``/mavros/global_position/raw/fix``. MAVROS is not part of this
deployment; the topic names were misleading and have been renamed.
"""

from __future__ import annotations

import math
import time
from typing import List, Optional

from geometry_msgs.msg import Vector3Stamped
from pymavlink import mavutil
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float64, String


class TelemetryPublisher:
    """Convert MAVLink telemetry into ROS topics."""

    G_M_S2 = 9.80665

    # Default IMU noise floors. RAW_IMU has no covariance metadata, so we
    # publish reasonable diagonal sigmas so downstream consumers (ORB-SLAM3
    # IMU-mono, EKF replay tools) see a non-UNKNOWN covariance.
    # These match a typical low-cost MEMS IMU; override via ROS params if you
    # have a calibrated noise model for your specific autopilot.
    DEFAULT_LIN_ACC_STD_M_S2 = 0.30
    DEFAULT_ANG_VEL_STD_RAD_S = 0.02
    # NavSatFix GPS horizontal/vertical 1-sigma defaults (m).
    DEFAULT_GPS_H_STD_M = 2.5
    DEFAULT_GPS_V_STD_M = 5.0

    def __init__(self, node, sensor_qos):
        self._node = node
        self._logger = node.get_logger()

        node.declare_parameter('imu_lin_acc_std_m_s2', self.DEFAULT_LIN_ACC_STD_M_S2)
        node.declare_parameter('imu_ang_vel_std_rad_s', self.DEFAULT_ANG_VEL_STD_RAD_S)
        node.declare_parameter('gps_horizontal_std_m', self.DEFAULT_GPS_H_STD_M)
        node.declare_parameter('gps_vertical_std_m', self.DEFAULT_GPS_V_STD_M)

        self._imu_lin_var = float(node.get_parameter('imu_lin_acc_std_m_s2').value) ** 2
        self._imu_ang_var = float(node.get_parameter('imu_ang_vel_std_rad_s').value) ** 2
        gps_h = float(node.get_parameter('gps_horizontal_std_m').value)
        gps_v = float(node.get_parameter('gps_vertical_std_m').value)
        self._gps_position_covariance = [
            gps_h * gps_h, 0.0, 0.0,
            0.0, gps_h * gps_h, 0.0,
            0.0, 0.0, gps_v * gps_v,
        ]

        self.relative_alt_pub = node.create_publisher(Float64, '/ap/relative_alt', sensor_qos)
        self.connected_pub = node.create_publisher(Bool, '/ap/connected', 10)
        self.armed_pub = node.create_publisher(Bool, '/ap/armed', 10)
        self.external_nav_ready_pub = node.create_publisher(Bool, '/ap/external_nav_ready', 10)
        self.flight_mode_pub = node.create_publisher(String, '/ap/flight_mode', 10)

        self.imu_raw_pub = node.create_publisher(Imu, '/ap/imu/raw', sensor_qos)
        self.imu_data_pub = node.create_publisher(Imu, '/ap/imu/data', sensor_qos)
        self.attitude_pub = node.create_publisher(Vector3Stamped, '/ap/attitude', sensor_qos)
        self.gps_raw_pub = node.create_publisher(NavSatFix, '/ap/gps/fix', sensor_qos)

        self.latest_mode = 'UNKNOWN'
        self.latest_armed = False
        self.latest_external_nav_ready = False
        self.latest_relative_alt = math.nan
        self.latest_global_lat_int: Optional[int] = None
        self.latest_global_lon_int: Optional[int] = None
        self.latest_global_alt_msl_m = math.nan
        # EKF-fused attitude from MAVLink ATTITUDE. Used to (a) fill
        # /ap/imu/data orientation and (b) tag-localizer compass anchor.
        # roll, pitch, yaw in radians, body->NED rotation as ArduPilot
        # reports them. We translate to ENU at the consumer.
        self.latest_attitude_rpy: Optional[List[float]] = None
        # Default 1-sigma for orientation when ATTITUDE is fresh; ATTITUDE
        # is post-EKF so cm-rad accuracy is realistic in steady flight.
        self._att_var = 0.01 ** 2  # ~0.6 deg 1-sigma

        self.external_nav_ekf_ready = {'EKF2': False, 'EKF3': False}

        self.prearm_failures: List[str] = []
        self._last_prearm_log_monotonic = 0.0

        self._connection = None  # set by ``install``

    def install(self, connection) -> None:
        """Register MAVLink message handlers on the connection."""
        self._connection = connection
        connection.add_handler('HEARTBEAT', self._on_heartbeat)
        connection.add_handler('GLOBAL_POSITION_INT', self._on_global_position)
        connection.add_handler('RAW_IMU', self._on_raw_imu)
        connection.add_handler('ATTITUDE', self._on_attitude)
        connection.add_handler('STATUSTEXT', self._on_statustext)

    def publish_status_snapshot(self) -> None:
        """Publish the latest cached state. Call from a periodic timer."""
        connected = self._connection.is_connected() if self._connection else False
        self.connected_pub.publish(Bool(data=bool(connected)))
        self.armed_pub.publish(Bool(data=bool(self.latest_armed)))
        self.external_nav_ready_pub.publish(
            Bool(data=bool(self.latest_external_nav_ready and connected))
        )
        self.flight_mode_pub.publish(String(data=str(self.latest_mode)))

    def _on_heartbeat(self, msg) -> None:
        if self._connection is None or not self._connection.is_primary_autopilot_message(msg):
            return
        self.latest_mode = mavutil.mode_string_v10(msg)
        self.latest_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def _on_global_position(self, msg) -> None:
        self.latest_global_lat_int = int(msg.lat)
        self.latest_global_lon_int = int(msg.lon)
        self.latest_global_alt_msl_m = msg.alt / 1000.0
        self.latest_relative_alt = msg.relative_alt / 1000.0
        self.relative_alt_pub.publish(Float64(data=float(self.latest_relative_alt)))

        gps_msg = NavSatFix()
        gps_msg.header.stamp = self._node.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'base_link'
        gps_msg.status.status = NavSatStatus.STATUS_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS
        gps_msg.latitude = msg.lat / 1e7
        gps_msg.longitude = msg.lon / 1e7
        gps_msg.altitude = msg.alt / 1000.0
        gps_msg.position_covariance = list(self._gps_position_covariance)
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gps_raw_pub.publish(gps_msg)

    def _on_raw_imu(self, msg) -> None:
        imu_msg = Imu()
        imu_msg.header.stamp = self._node.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'
        imu_msg.linear_acceleration.x = msg.xacc * self.G_M_S2 / 1000.0
        imu_msg.linear_acceleration.y = msg.yacc * self.G_M_S2 / 1000.0
        imu_msg.linear_acceleration.z = msg.zacc * self.G_M_S2 / 1000.0
        imu_msg.angular_velocity.x = msg.xgyro / 1000.0
        imu_msg.angular_velocity.y = msg.ygyro / 1000.0
        imu_msg.angular_velocity.z = msg.zgyro / 1000.0
        # Orientation is unknown in RAW_IMU. Per REP-145, signal that with
        # a -1 in the first element of orientation_covariance.
        imu_msg.orientation_covariance = [
            -1.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
        ]
        imu_msg.linear_acceleration_covariance = [
            self._imu_lin_var, 0.0, 0.0,
            0.0, self._imu_lin_var, 0.0,
            0.0, 0.0, self._imu_lin_var,
        ]
        imu_msg.angular_velocity_covariance = [
            self._imu_ang_var, 0.0, 0.0,
            0.0, self._imu_ang_var, 0.0,
            0.0, 0.0, self._imu_ang_var,
        ]
        self.imu_raw_pub.publish(imu_msg)

        # If we already have a fresh ATTITUDE, also publish /ap/imu/data
        # with orientation filled in. Consumers that need vehicle attitude
        # (compass anchor, ORB-SLAM IMU mode) prefer this topic; consumers
        # that strictly need raw measurements stay on /ap/imu/raw.
        if self.latest_attitude_rpy is not None:
            data_msg = Imu()
            data_msg.header = imu_msg.header
            data_msg.linear_acceleration = imu_msg.linear_acceleration
            data_msg.angular_velocity = imu_msg.angular_velocity
            data_msg.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance
            data_msg.angular_velocity_covariance = imu_msg.angular_velocity_covariance
            qx, qy, qz, qw = self._rpy_to_quat_xyzw(*self.latest_attitude_rpy)
            data_msg.orientation.x = qx
            data_msg.orientation.y = qy
            data_msg.orientation.z = qz
            data_msg.orientation.w = qw
            data_msg.orientation_covariance = [
                self._att_var, 0.0, 0.0,
                0.0, self._att_var, 0.0,
                0.0, 0.0, self._att_var,
            ]
            self.imu_data_pub.publish(data_msg)

    def _on_attitude(self, msg) -> None:
        # ArduPilot ATTITUDE: roll, pitch, yaw in radians, body->NED.
        roll = float(msg.roll)
        pitch = float(msg.pitch)
        yaw = float(msg.yaw)
        self.latest_attitude_rpy = [roll, pitch, yaw]

        out = Vector3Stamped()
        out.header.stamp = self._node.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.vector.x = roll
        out.vector.y = pitch
        out.vector.z = yaw
        self.attitude_pub.publish(out)

    @staticmethod
    def _rpy_to_quat_xyzw(roll: float, pitch: float, yaw: float):
        """Convert ATTITUDE (roll, pitch, yaw) to quaternion (x,y,z,w).

        ArduPilot's ATTITUDE message reports Tait-Bryan angles in the
        standard aerospace ZYX intrinsic convention (yaw applied first
        about body Z, then pitch about the rotated Y, then roll about
        the rotated X). This matches ``scipy.spatial.transform.Rotation
        .from_euler('xyz', [r,p,y])`` (lowercase xyz = extrinsic xyz =
        ZYX intrinsic).

        Pure stdlib so the bridge keeps its zero-dep posture (no scipy).
        """
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def _on_statustext(self, msg) -> None:
        text = getattr(msg, 'text', '') or ''
        severity = getattr(msg, 'severity', None)
        self._logger.info(f'STATUSTEXT severity={severity}: {text}')
        self._update_external_nav_readiness(text)
        self._record_prearm_failure(text, severity)

    def _update_external_nav_readiness(self, text: str) -> None:
        if 'is using external nav data' not in text:
            return
        if text.startswith('EKF2'):
            self.external_nav_ekf_ready['EKF2'] = True
        elif text.startswith('EKF3'):
            self.external_nav_ekf_ready['EKF3'] = True
        else:
            return
        if self.external_nav_ekf_ready.get('EKF3', False):
            if not self.latest_external_nav_ready:
                self.latest_external_nav_ready = True
                self._logger.info(
                    'ArduPilot EKF3 now reports external navigation ready for arming'
                )

    def _record_prearm_failure(self, text: str, severity) -> None:
        # ArduPilot prefixes pre-arm rejections with "PreArm:" or "Arm:".
        lowered = text.lower()
        if not (lowered.startswith('prearm:') or lowered.startswith('arm:')):
            return
        self.prearm_failures.append(text)
        # Keep only the last few.
        if len(self.prearm_failures) > 8:
            self.prearm_failures = self.prearm_failures[-8:]
