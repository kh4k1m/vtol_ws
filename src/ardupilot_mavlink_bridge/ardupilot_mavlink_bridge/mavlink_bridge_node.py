import math
import time

import rclpy
from flight_interfaces.srv import SetMode, Takeoff
from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import Imu, NavSatFix


ALTITUDE_ONLY_LOCAL_OFFSET_MASK = 3579


class MavlinkBridgeNode(Node):
    def __init__(self):
        super().__init__('mavlink_bridge_node')

        self.declare_parameter('connection_string', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('origin_lat_deg', -35.363261)
        self.declare_parameter('origin_lon_deg', 149.16523)
        self.declare_parameter('origin_alt_m', 584.0)
        self.declare_parameter('vision_timestamp_mode', 'message_stamp')
        self.declare_parameter('vision_debug_log_period_sec', 2.0)
        self.declare_parameter('odometry_resend_rate_hz', 20.0)
        self.declare_parameter('odometry_resend_hold_timeout_sec', 1.0)

        conn_str = self.get_parameter('connection_string').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.origin_lat_deg = self.get_parameter('origin_lat_deg').value
        self.origin_lon_deg = self.get_parameter('origin_lon_deg').value
        self.origin_alt_m = self.get_parameter('origin_alt_m').value
        self.vision_timestamp_mode = str(
            self.get_parameter('vision_timestamp_mode').value
        ).strip().lower()
        self.vision_debug_log_period_sec = max(
            0.0,
            float(self.get_parameter('vision_debug_log_period_sec').value),
        )
        self.odometry_resend_rate_hz = max(
            0.0,
            float(self.get_parameter('odometry_resend_rate_hz').value),
        )
        self.odometry_resend_hold_timeout_sec = max(
            0.0,
            float(self.get_parameter('odometry_resend_hold_timeout_sec').value),
        )
        self.odometry_resend_period_sec = (
            1.0 / self.odometry_resend_rate_hz
            if self.odometry_resend_rate_hz > 0.0
            else None
        )
        if self.vision_timestamp_mode not in {'message_stamp', 'bridge_monotonic'}:
            self.get_logger().warning(
                f'Unsupported vision_timestamp_mode={self.vision_timestamp_mode!r}, '
                'falling back to message_stamp'
            )
            self.vision_timestamp_mode = 'message_stamp'

        self.get_logger().info(f'Connecting to ArduPilot on {conn_str} at {baud} baud...')

        self.conn_str = conn_str
        self.baud = baud
        self.master = None
        self.autopilot_system_id = None
        self.autopilot_component_id = None
        self.last_heartbeat_monotonic = 0.0
        self.is_fully_connected = False
        self.latest_mode = 'UNKNOWN'
        self.latest_armed = False
        self.latest_external_nav_ready = False
        self.latest_relative_alt = math.nan
        self.latest_global_lat_int = None
        self.latest_global_lon_int = None
        self.latest_global_alt_msl_m = math.nan
        self.pending_guided_altitude_m = None
        self.pending_guided_altitude_deadline = 0.0
        self.external_nav_ekf_ready = {
            'EKF2': False,
            'EKF3': False,
        }

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
        self.relative_alt_pub = self.create_publisher(Float64, '/ap/relative_alt', qos_profile)
        self.connected_pub = self.create_publisher(Bool, '/ap/connected', 10)
        self.armed_pub = self.create_publisher(Bool, '/ap/armed', 10)
        self.external_nav_ready_pub = self.create_publisher(Bool, '/ap/external_nav_ready', 10)
        self.flight_mode_pub = self.create_publisher(String, '/ap/flight_mode', 10)
        
        # Publishers for telemetry logging (replaces MAVROS)
        self.imu_raw_pub = self.create_publisher(Imu, '/mavros/imu/data_raw', qos_profile)
        self.gps_raw_pub = self.create_publisher(NavSatFix, '/mavros/global_position/raw/fix', qos_profile)

        self.create_service(SetMode, '/ap/cmd/set_mode', self.set_mode_callback)
        self.create_service(SetBool, '/ap/cmd/arm', self.arm_callback)
        self.create_service(Takeoff, '/ap/cmd/takeoff', self.takeoff_callback)
        self.create_service(Trigger, '/ap/cmd/land', self.land_callback)

        self.get_logger().info(
            'MAVLink bridge node started. Listening to /vision_pose_enu. '
            f'Takeoff strategy: AUTO mission upload. '
            f'External-nav MAVLink message: ODOMETRY. '
            f'Timestamp mode: {self.vision_timestamp_mode}. '
            f'ODOMETRY resend: {self.odometry_resend_rate_hz:.1f} Hz, '
            f'hold timeout: {self.odometry_resend_hold_timeout_sec:.2f}s'
        )

        self.connection_timer = self.create_timer(1.0, self.check_connection)
        self.timer = self.create_timer(1.0, self.send_ekf_origin)
        self.read_timer = self.create_timer(0.01, self.read_mavlink) # Увеличили частоту чтения для IMU
        self.status_timer = self.create_timer(0.5, self.publish_status)
        self.guided_target_timer = self.create_timer(0.2, self.send_guided_altitude_target)
        if self.odometry_resend_period_sec is not None:
            self.odometry_resend_timer = self.create_timer(
                self.odometry_resend_period_sec,
                self.resend_latest_odometry,
            )
        else:
            self.odometry_resend_timer = None
        self.origin_sent_count = 0

        self.last_vision_sample_sec = None
        self.last_x = None
        self.last_y = None
        self.last_z = None
        self.last_vision_stamp_sec = None
        self.last_vision_timing_warning_monotonic = 0.0
        self.last_vision_debug_log_monotonic = 0.0
        self.last_odometry_send_monotonic = 0.0
        self.last_odometry_time_usec = 0
        self.cached_odometry = None

    def check_connection(self):
        now = self._monotonic_now()
        # Если связь была, но heartbeat давно не приходил - считаем, что отключились
        if self.is_fully_connected and (now - self.last_heartbeat_monotonic) > 5.0:
            self.get_logger().warning("Lost connection to ArduPilot (heartbeat timeout). Reconnecting...")
            if self.master is not None:
                try:
                    self.master.close()
                except:
                    pass
            self.master = None
            self.is_fully_connected = False
            self.autopilot_system_id = None
            
        # Если нет объекта соединения, пытаемся открыть порт
        if self.master is None:
            try:
                self.master = mavutil.mavlink_connection(self.conn_str, baud=self.baud)
                self.get_logger().info(f"Opened connection to {self.conn_str}, waiting for heartbeat...")
                self.last_heartbeat_monotonic = now # Даем время на получение первого heartbeat
            except Exception as e:
                self.get_logger().error(f"Failed to open connection: {e}")
                self.master = None

    def request_data_streams(self):
        if self.master is not None:
            self.get_logger().info("Requesting MAVLink data streams...")
            self.request_data_stream(mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10)
            self.request_data_stream(mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 50) # Для IMU (RAW_IMU)
            self.request_data_stream(mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 10) # Для GPS
            self.request_data_stream(mavutil.mavlink.MAV_DATA_STREAM_ALL, 10)

    def request_data_stream(self, stream_id, rate):
        if self.master is None:
            return

        try:
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                stream_id,
                rate,
                1
            )
        except Exception as e:
            self.get_logger().warning(f'Failed to request MAVLink data stream {stream_id}: {e}')

    def _monotonic_now(self):
        return time.monotonic()

    def _stamp_to_sec(self, stamp):
        return float(stamp.sec) + (float(stamp.nanosec) * 1e-9)

    def _stamp_to_usec(self, stamp):
        return int(int(stamp.sec) * 1_000_000 + int(stamp.nanosec) // 1000)

    def _warn_vision_timing(self, message):
        now = self._monotonic_now()
        if (now - self.last_vision_timing_warning_monotonic) < 2.0:
            return
        self.get_logger().warning(message)
        self.last_vision_timing_warning_monotonic = now

    def _maybe_log_vision_payload(
        self,
        msg,
        sample_sec,
        usec,
        dt,
        x_ned,
        y_ned,
        z_ned,
        roll_ned,
        pitch_ned,
        yaw_ned,
        vx_ned,
        vy_ned,
        vz_ned,
        q_wxyz,
        vx_body_frd,
        vy_body_frd,
        vz_body_frd,
    ):
        if self.vision_debug_log_period_sec <= 0.0:
            return

        now = self._monotonic_now()
        if (now - self.last_vision_debug_log_monotonic) < self.vision_debug_log_period_sec:
            return

        stamp_sec = self._stamp_to_sec(msg.header.stamp)
        self.get_logger().info(
            'ODOMETRY payload -> '
            f'ts_mode={self.vision_timestamp_mode} '
            f'msg_stamp={stamp_sec:.6f}s tx_time={sample_sec:.6f}s usec={usec} dt={dt:.3f}s '
            f'pos_enu=({float(msg.pose.position.x):.3f}, {float(msg.pose.position.y):.3f}, {float(msg.pose.position.z):.3f}) '
            f'pos_local_frd=({x_ned:.3f}, {y_ned:.3f}, {z_ned:.3f}) '
            f'rpy_local_rad=({roll_ned:.3f}, {pitch_ned:.3f}, {yaw_ned:.3f}) '
            f'q_wxyz=({q_wxyz[0]:.3f}, {q_wxyz[1]:.3f}, {q_wxyz[2]:.3f}, {q_wxyz[3]:.3f}) '
            f'vel_ned=({vx_ned:.3f}, {vy_ned:.3f}, {vz_ned:.3f}) '
            f'vel_body_frd=({vx_body_frd:.3f}, {vy_body_frd:.3f}, {vz_body_frd:.3f})'
        )
        self.last_vision_debug_log_monotonic = now

    def _maybe_log_odometry_resend(self, payload, age_sec, tx_time_usec):
        if self.vision_debug_log_period_sec <= 0.0:
            return

        now = self._monotonic_now()
        if (now - self.last_vision_debug_log_monotonic) < self.vision_debug_log_period_sec:
            return

        self.get_logger().info(
            'ODOMETRY resend -> '
            f'age={age_sec:.3f}s tx_time={tx_time_usec / 1e6:.6f}s usec={tx_time_usec} '
            f'pos_local_frd=({payload["x"]:.3f}, {payload["y"]:.3f}, {payload["z"]:.3f}) '
            f'q_wxyz=({payload["q"][0]:.3f}, {payload["q"][1]:.3f}, {payload["q"][2]:.3f}, {payload["q"][3]:.3f}) '
            f'vel_body_frd=({payload["vx"]:.3f}, {payload["vy"]:.3f}, {payload["vz"]:.3f})'
        )
        self.last_vision_debug_log_monotonic = now

    def _next_odometry_time_usec(self, proposed_usec):
        next_usec = int(proposed_usec)
        if next_usec <= self.last_odometry_time_usec:
            next_usec = self.last_odometry_time_usec + 1
        self.last_odometry_time_usec = next_usec
        return next_usec

    def _send_odometry_message(self, payload, proposed_usec):
        if self.master is None:
            return None

        tx_time_usec = self._next_odometry_time_usec(proposed_usec)
        self.master.mav.send(
            mavlink2.MAVLink_odometry_message(
                tx_time_usec,
                mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
                mavutil.mavlink.MAV_FRAME_BODY_FRD,
                payload['x'],
                payload['y'],
                payload['z'],
                payload['q'],
                payload['vx'],
                payload['vy'],
                payload['vz'],
                0.0,
                0.0,
                0.0,
                payload['pose_covariance'],
                payload['velocity_covariance'],
                0,
                mavlink2.MAV_ESTIMATOR_TYPE_VIO,
                payload['quality'],
            )
        )
        self.last_odometry_send_monotonic = self._monotonic_now()
        return tx_time_usec

    def resend_latest_odometry(self):
        if (
            self.master is None
            or self.cached_odometry is None
            or self.odometry_resend_period_sec is None
        ):
            return

        now = self._monotonic_now()
        sample_age_sec = now - self.cached_odometry['updated_monotonic']
        if sample_age_sec > self.odometry_resend_hold_timeout_sec:
            return

        if (now - self.last_odometry_send_monotonic) < (self.odometry_resend_period_sec * 0.9):
            return

        proposed_usec = int(
            (self.cached_odometry['sample_sec'] + sample_age_sec) * 1e6
        )
        tx_time_usec = self._send_odometry_message(self.cached_odometry, proposed_usec)
        if tx_time_usec is None:
            return
        self._maybe_log_odometry_resend(
            self.cached_odometry,
            sample_age_sec,
            tx_time_usec,
        )

    def _vision_sample_time(self, msg):
        stamp_sec = self._stamp_to_sec(msg.header.stamp)
        monotonic_now = self._monotonic_now()

        if self.last_vision_stamp_sec is not None:
            stamp_dt = stamp_sec - self.last_vision_stamp_sec
            if stamp_dt <= 0.0:
                self._warn_vision_timing(
                    'Non-monotonic /vision_pose_enu stamp detected: '
                    f'dt={stamp_dt:.6f}s current={stamp_sec:.6f}s '
                    f'previous={self.last_vision_stamp_sec:.6f}s'
                )
        self.last_vision_stamp_sec = stamp_sec

        if self.vision_timestamp_mode == 'bridge_monotonic':
            sample_sec = monotonic_now
            if (
                self.last_vision_sample_sec is not None
                and sample_sec <= self.last_vision_sample_sec
            ):
                sample_sec = self.last_vision_sample_sec + 1e-6
            return sample_sec, int(sample_sec * 1e6)

        if stamp_sec > 0.0 or self.last_vision_sample_sec is None:
            return stamp_sec, self._stamp_to_usec(msg.header.stamp)

        self._warn_vision_timing(
            'Zero /vision_pose_enu stamp detected in message_stamp mode, '
            'falling back to bridge monotonic time for this sample'
        )
        if (
            self.last_vision_sample_sec is not None
            and monotonic_now <= self.last_vision_sample_sec
        ):
            monotonic_now = self.last_vision_sample_sec + 1e-6
        return monotonic_now, int(monotonic_now * 1e6)

    def is_connected(self):
        return self.is_fully_connected and (self._monotonic_now() - self.last_heartbeat_monotonic) < 3.0

    def publish_status(self):
        self.connected_pub.publish(Bool(data=self.is_connected()))
        self.armed_pub.publish(Bool(data=bool(self.latest_armed)))
        self.external_nav_ready_pub.publish(
            Bool(data=bool(self.latest_external_nav_ready and self.is_connected()))
        )
        self.flight_mode_pub.publish(String(data=str(self.latest_mode)))

    def read_mavlink(self):
        if self.master is None:
            return

        while True:
            try:
                msg = self.master.recv_match(blocking=False)
                if not msg:
                    break

                self._handle_mavlink_message(msg)
            except Exception as e:
                self.get_logger().debug(f"MAVLink read error (might be disconnected): {e}")
                break

    def _handle_mavlink_message(self, msg):
        msg_type = msg.get_type()

        if msg_type == 'HEARTBEAT':
            # Если мы еще не "полностью подключены", первый heartbeat от автопилота инициализирует нас
            if not self.is_fully_connected and msg.get_srcComponent() == 1:
                self.autopilot_system_id = int(msg.get_srcSystem())
                self.autopilot_component_id = int(msg.get_srcComponent())
                self.is_fully_connected = True
                self.origin_sent_count = 0 # Сбрасываем счетчик EKF Origin
                self.get_logger().info(f'Successfully connected to ArduPilot (System ID: {self.autopilot_system_id})')
                self.request_data_streams()

            if not self._is_primary_autopilot_message(msg):
                return
            self.last_heartbeat_monotonic = self._monotonic_now()
            self.latest_mode = mavutil.mode_string_v10(msg)
            self.latest_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            
        elif msg_type == 'GLOBAL_POSITION_INT':
            self.latest_global_lat_int = int(msg.lat)
            self.latest_global_lon_int = int(msg.lon)
            self.latest_global_alt_msl_m = msg.alt / 1000.0
            self.latest_relative_alt = msg.relative_alt / 1000.0
            self.relative_alt_pub.publish(Float64(data=float(self.latest_relative_alt)))
            
            # Публикуем GPS для логгера
            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = "base_link"
            gps_msg.latitude = msg.lat / 1e7
            gps_msg.longitude = msg.lon / 1e7
            gps_msg.altitude = msg.alt / 1000.0
            self.gps_raw_pub.publish(gps_msg)
            
        elif msg_type == 'RAW_IMU':
            # Публикуем сырые данные IMU для логгера
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "base_link"
            
            # Конвертация из сырых единиц MAVLink в СИ
            # Ускорения: mG -> m/s^2
            imu_msg.linear_acceleration.x = msg.xacc * 9.80665 / 1000.0
            imu_msg.linear_acceleration.y = msg.yacc * 9.80665 / 1000.0
            imu_msg.linear_acceleration.z = msg.zacc * 9.80665 / 1000.0
            
            # Угловые скорости: mrad/s -> rad/s
            imu_msg.angular_velocity.x = msg.xgyro / 1000.0
            imu_msg.angular_velocity.y = msg.ygyro / 1000.0
            imu_msg.angular_velocity.z = msg.zgyro / 1000.0
            
            self.imu_raw_pub.publish(imu_msg)
            
        elif msg_type == 'STATUSTEXT':
            text = getattr(msg, 'text', '')
            severity = getattr(msg, 'severity', None)
            self.get_logger().info(f'STATUSTEXT severity={severity}: {text}')
            self._update_external_nav_readiness(text)
            
        elif msg_type == 'COMMAND_ACK':
            self._log_command_ack(msg)

    def _update_external_nav_readiness(self, text):
        if 'is using external nav data' not in text:
            return

        if text.startswith('EKF2'):
            self.external_nav_ekf_ready['EKF2'] = True
        elif text.startswith('EKF3'):
            self.external_nav_ekf_ready['EKF3'] = True
        else:
            return

        ready_now = all(self.external_nav_ekf_ready.values())
        if ready_now and not self.latest_external_nav_ready:
            self.latest_external_nav_ready = True
            self.get_logger().info(
                'ArduPilot EKF now reports external navigation ready for arming'
            )

    def _is_primary_autopilot_message(self, msg):
        if self.master is None or self.autopilot_system_id is None:
            return False
        if int(msg.get_srcSystem()) != int(self.autopilot_system_id):
            return False

        if self.autopilot_component_id in (None, 0):
            self.autopilot_component_id = int(msg.get_srcComponent())
            return True

        return int(msg.get_srcComponent()) == int(self.autopilot_component_id)

    def _command_result_name(self, result):
        result_enum = mavutil.mavlink.enums.get('MAV_RESULT', {})
        return result_enum.get(result).name if result in result_enum else str(result)

    def _wait_for_command_ack(self, expected_command, timeout_sec=2.0):
        if self.master is None:
            return None

        deadline = self._monotonic_now() + timeout_sec
        while self._monotonic_now() < deadline:
            remaining = max(0.0, deadline - self._monotonic_now())
            msg = self.master.recv_match(blocking=True, timeout=remaining)
            if not msg:
                continue

            self._handle_mavlink_message(msg)
            if msg.get_type() == 'COMMAND_ACK' and msg.command == expected_command:
                return msg

        return None

    def _log_command_ack(self, msg):
        result_name = self._command_result_name(msg.result)
        self.get_logger().info(f'COMMAND_ACK command={msg.command} result={result_name}')

    def _mission_result_name(self, result):
        result_enum = mavutil.mavlink.enums.get('MAV_MISSION_RESULT', {})
        return result_enum.get(result).name if result in result_enum else str(result)

    def _wait_for_mode(self, expected_mode, timeout_sec=3.0):
        if self.master is None:
            return False

        normalized_mode = self._normalize_mode_name(expected_mode)
        deadline = self._monotonic_now() + timeout_sec
        while self._monotonic_now() < deadline:
            if self.latest_mode == normalized_mode:
                return True

            remaining = max(0.0, deadline - self._monotonic_now())
            msg = self.master.recv_match(blocking=True, timeout=remaining)
            if not msg:
                continue

            self._handle_mavlink_message(msg)
            if self.latest_mode == normalized_mode:
                return True

        return self.latest_mode == normalized_mode

    def _wait_for_armed_state(self, expected_armed, timeout_sec=3.0):
        if self.master is None:
            return False

        expected_armed = bool(expected_armed)
        deadline = self._monotonic_now() + timeout_sec
        while self._monotonic_now() < deadline:
            if bool(self.latest_armed) == expected_armed:
                return True

            remaining = max(0.0, deadline - self._monotonic_now())
            msg = self.master.recv_match(blocking=True, timeout=remaining)
            if not msg:
                continue

            self._handle_mavlink_message(msg)
            if bool(self.latest_armed) == expected_armed:
                return True

        return bool(self.latest_armed) == expected_armed

    def _clear_rc_overrides(self):
        if self.master is None:
            raise RuntimeError('ArduPilot connection is not available')

        # Release any stale MAVProxy/manual overrides before autonomous commands.
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def _clear_mission(self, timeout_sec=2.0):
        if self.master is None:
            raise RuntimeError('ArduPilot connection is not available')

        self.master.mav.mission_clear_all_send(
            self.master.target_system,
            self.master.target_component,
        )

        deadline = self._monotonic_now() + timeout_sec
        while self._monotonic_now() < deadline:
            remaining = max(0.0, deadline - self._monotonic_now())
            msg = self.master.recv_match(
                type=['MISSION_ACK', 'MISSION_REQUEST', 'MISSION_REQUEST_INT'],
                blocking=True,
                timeout=remaining,
            )
            if not msg:
                continue
            if msg.get_type() == 'MISSION_ACK':
                return
            self._handle_mavlink_message(msg)

    def _upload_mission_items(self, items, timeout_sec=10.0):
        if self.master is None:
            raise RuntimeError('ArduPilot connection is not available')

        self.master.mav.mission_count_send(
            self.master.target_system,
            self.master.target_component,
            len(items),
        )

        deadline = self._monotonic_now() + timeout_sec
        while self._monotonic_now() < deadline:
            remaining = max(0.0, deadline - self._monotonic_now())
            msg = self.master.recv_match(
                type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
                blocking=True,
                timeout=remaining,
            )
            if not msg:
                continue

            msg_type = msg.get_type()
            if msg_type == 'MISSION_ACK':
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    return
                mission_result = self._mission_result_name(msg.type)
                raise RuntimeError(f'Mission upload rejected with {mission_result}')

            self._handle_mavlink_message(msg)
            seq = int(msg.seq)
            if seq < 0 or seq >= len(items):
                raise RuntimeError(f'Unexpected mission item request for seq={seq}')

            item = items[seq]
            self.master.mav.mission_item_int_send(
                self.master.target_system,
                self.master.target_component,
                seq,
                item['frame'],
                item['command'],
                item['current'],
                item['autocontinue'],
                item['param1'],
                item['param2'],
                item['param3'],
                item['param4'],
                item['x'],
                item['y'],
                item['z'],
            )

        raise RuntimeError('Timed out waiting for mission upload handshake')

    def _build_vtol_takeoff_mission(self, altitude_m):
        if (
            self.latest_global_lat_int is None
            or self.latest_global_lon_int is None
            or not math.isfinite(self.latest_global_alt_msl_m)
        ):
            raise RuntimeError('Current global position is unavailable for mission upload')

        current_lat = int(self.latest_global_lat_int)
        current_lon = int(self.latest_global_lon_int)
        current_abs_alt_m = float(self.latest_global_alt_msl_m)
        target_rel_alt_m = float(altitude_m)

        return [
            {
                'frame': mavutil.mavlink.MAV_FRAME_GLOBAL,
                'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                'current': 0,
                'autocontinue': 1,
                'param1': 0.0,
                'param2': 0.0,
                'param3': 0.0,
                'param4': 0.0,
                'x': current_lat,
                'y': current_lon,
                'z': current_abs_alt_m,
            },
            {
                'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                'command': mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF,
                'current': 0,
                'autocontinue': 1,
                'param1': 0.0,
                'param2': 0.0,
                'param3': 0.0,
                'param4': 0.0,
                'x': current_lat,
                'y': current_lon,
                'z': target_rel_alt_m,
            },
            {
                'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                'command': mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                'current': 0,
                'autocontinue': 1,
                'param1': 0.0,
                'param2': 0.0,
                'param3': 0.0,
                'param4': 0.0,
                'x': current_lat,
                'y': current_lon,
                'z': target_rel_alt_m,
            },
        ]

    def send_ekf_origin(self):
        if not getattr(self, 'is_fully_connected', False) or self.master is None:
            return
            
        if self.origin_sent_count >= 5:
            return
            
        self.origin_sent_count += 1
        self.get_logger().info(
            f'Sending EKF origin/home seed (attempt {self.origin_sent_count}/5): '
            f'lat={self.origin_lat_deg}, lon={self.origin_lon_deg}, alt={self.origin_alt_m}'
        )
        
        target_system = self.master.target_system
        target_component = self.master.target_component
        origin_lat = int(round(float(self.origin_lat_deg) * 1e7))
        origin_lon = int(round(float(self.origin_lon_deg) * 1e7))
        origin_alt_mm = int(round(float(self.origin_alt_m) * 1000.0))
        
        try:
            self.master.mav.set_gps_global_origin_send(
                target_system,
                origin_lat,
                origin_lon,
                origin_alt_mm,
                int(self._monotonic_now() * 1e6)
            )
            self.master.mav.command_int_send(
                target_system,
                target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                0,
                0,
                0.0,
                0.0,
                0.0,
                0.0,
                origin_lat,
                origin_lon,
                float(self.origin_alt_m),
            )
        except Exception as e:
            self.get_logger().error(f'Failed to send EKF Origin: {e}')

    def _normalize_mode_name(self, mode):
        return mode.strip().upper()

    def _send_mode_request(self, mode_name):
        if self.master is None:
            raise RuntimeError('ArduPilot connection is not available')

        normalized_mode = self._normalize_mode_name(mode_name)
        mode_mapping = self.master.mode_mapping() or {}
        if normalized_mode not in mode_mapping:
            available_modes = ', '.join(sorted(mode_mapping.keys()))
            raise RuntimeError(f'Unsupported mode "{normalized_mode}". Available modes: {available_modes}')

        self.master.set_mode(mode_mapping[normalized_mode])
        return normalized_mode

    def set_mode_callback(self, request, response):
        try:
            mode = self._send_mode_request(request.mode)
            response.success = True
            response.message = f'Flight mode change requested: {mode}'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f'Failed to set mode: {exc}')
        return response

    def arm_callback(self, request, response):
        if self.master is None:
            response.success = False
            response.message = 'ArduPilot connection is not available'
            return response

        expected_armed = bool(request.data)
        action = 'ARM' if expected_armed else 'DISARM'
        target_state_name = 'armed' if expected_armed else 'disarmed'

        if expected_armed and self.latest_armed:
            response.success = True
            response.message = 'Vehicle already armed'
            return response

        if not expected_armed and not self.latest_armed:
            response.success = True
            response.message = 'Vehicle already disarmed'
            return response

        try:
            self._clear_rc_overrides()
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1.0 if expected_armed else 0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )

            ack = self._wait_for_command_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
            if ack is None:
                response.success = False
                response.message = f'Timed out waiting for {action} acknowledgement'
                return response

            if ack.result not in (
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            ):
                response.success = False
                response.message = f'{action} rejected with {self._command_result_name(ack.result)}'
                return response

            if not self._wait_for_armed_state(expected_armed, timeout_sec=3.0):
                response.success = False
                response.message = f'Timed out waiting for vehicle to become {target_state_name}'
                return response

            response.success = True
            response.message = f'Vehicle {target_state_name}'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f'Failed to send arm/disarm command: {exc}')
        return response

    def _send_local_offset_altitude_target(self, altitude_m):
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
            ALTITUDE_ONLY_LOCAL_OFFSET_MASK,
            0.0,
            0.0,
            -float(altitude_m),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def takeoff_callback(self, request, response):
        if self.master is None:
            response.success = False
            response.message = 'ArduPilot connection is not available'
            return response

        if request.altitude_m <= 0.0:
            response.success = False
            response.message = 'Takeoff altitude must be positive'
            return response

        try:
            self.pending_guided_altitude_m = None
            self.pending_guided_altitude_deadline = 0.0
            self._clear_rc_overrides()
            self.get_logger().info(
                f'Uploading VTOL AUTO takeoff mission to {float(request.altitude_m):.2f} m'
            )
            mission_items = self._build_vtol_takeoff_mission(float(request.altitude_m))
            self._clear_mission()
            self._upload_mission_items(mission_items)

            auto_mode = self._send_mode_request('AUTO')
            if not self._wait_for_mode(auto_mode, timeout_sec=3.0):
                response.success = False
                response.message = 'Timed out waiting for AUTO mode confirmation'
                return response

            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_MISSION_START,
                0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )

            ack = self._wait_for_command_ack(mavutil.mavlink.MAV_CMD_MISSION_START)
            if ack is None:
                response.success = False
                response.message = 'Timed out waiting for mission start acknowledgement'
                return response

            if ack.result in (
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            ):
                response.success = True
                response.message = (
                    f'Uploaded VTOL AUTO takeoff mission to {float(request.altitude_m):.2f} m '
                    'and started AUTO mission'
                )
            else:
                response.success = False
                response.message = f'MISSION_START rejected with {self._command_result_name(ack.result)}'
        except Exception as exc:
            self.pending_guided_altitude_m = None
            self.pending_guided_altitude_deadline = 0.0
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f'Failed to request takeoff altitude: {exc}')
        return response

    def send_guided_altitude_target(self):
        if self.master is None or self.pending_guided_altitude_m is None:
            return

        if self._monotonic_now() > self.pending_guided_altitude_deadline:
            self.pending_guided_altitude_m = None
            self.pending_guided_altitude_deadline = 0.0
            return

        if math.isfinite(self.latest_relative_alt) and self.latest_relative_alt >= (self.pending_guided_altitude_m - 0.2):
            self.pending_guided_altitude_m = None
            self.pending_guided_altitude_deadline = 0.0
            return

        if self.latest_mode != 'GUIDED':
            self.pending_guided_altitude_m = None
            self.pending_guided_altitude_deadline = 0.0
            return

        try:
            self._send_local_offset_altitude_target(self.pending_guided_altitude_m)
        except Exception as exc:
            self.get_logger().warning(f'Failed to refresh guided altitude target: {exc}')

    def land_callback(self, request, response):
        del request

        self.pending_guided_altitude_m = None
        self.pending_guided_altitude_deadline = 0.0

        try:
            mode = self._send_mode_request('QLAND')
            response.success = True
            response.message = f'Landing mode requested: {mode}'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f'Failed to request QLAND: {exc}')
        return response

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

        sample_sec, usec = self._vision_sample_time(msg)
        if self.last_vision_sample_sec is not None:
            dt = sample_sec - self.last_vision_sample_sec
        else:
            dt = 0.0
        
        if 1e-6 < dt <= 0.5 and self.last_x is not None:
            vx = (x_ned - self.last_x) / dt
            vy = (y_ned - self.last_y) / dt
            vz = (z_ned - self.last_z) / dt
        else:
            vx, vy, vz = 0.0, 0.0, 0.0

        rot_ned = R.from_euler('xyz', [roll_ned, pitch_ned, yaw_ned], degrees=False)
        q_xyzw = rot_ned.as_quat()
        q_wxyz = [
            float(q_xyzw[3]),
            float(q_xyzw[0]),
            float(q_xyzw[1]),
            float(q_xyzw[2]),
        ]
        vx_body_frd, vy_body_frd, vz_body_frd = [
            float(value) for value in rot_ned.inv().apply([vx, vy, vz])
        ]
        pose_covariance = [float('nan')] * 21
        velocity_covariance = [float('nan')] * 21
        odometry_quality = 100
        odometry_payload = {
            'x': float(x_ned),
            'y': float(y_ned),
            'z': float(z_ned),
            'q': q_wxyz,
            'vx': vx_body_frd,
            'vy': vy_body_frd,
            'vz': vz_body_frd,
            'pose_covariance': pose_covariance,
            'velocity_covariance': velocity_covariance,
            'quality': odometry_quality,
            'sample_sec': float(sample_sec),
            'updated_monotonic': self._monotonic_now(),
        }
            
        self.last_x, self.last_y, self.last_z = x_ned, y_ned, z_ned
        self.last_vision_sample_sec = sample_sec
        self.cached_odometry = odometry_payload

        try:
            self._maybe_log_vision_payload(
                msg,
                sample_sec,
                usec,
                dt,
                x_ned,
                y_ned,
                z_ned,
                roll_ned,
                pitch_ned,
                yaw_ned,
                vx,
                vy,
                vz,
                q_wxyz,
                vx_body_frd,
                vy_body_frd,
                vz_body_frd,
            )
            self._send_odometry_message(odometry_payload, usec)
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
