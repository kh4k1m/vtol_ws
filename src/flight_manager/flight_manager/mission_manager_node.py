"""High-level mission state machine.

Subscribes to ``/nav/status`` (NavStatus) instead of the tag-specific
``/localizer_status`` so the same logic works for tags-only, DPVO-only
and hybrid configurations - any pose source publishes its health to a
single topic and the mission only cares whether the active source is in
a tracking-grade state.

Also escalates after repeated arm rejections instead of looping forever
and surfaces the last few PreArm/Arm STATUSTEXT lines reported by the
bridge.
"""

from __future__ import annotations

import math
import time

import rclpy
from flight_interfaces.srv import SetMode, SwitchEKFSource, Takeoff
from marker_interfaces.msg import NavStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import SetBool, Trigger


# State codes that count as "vision is good enough to fly on".
DEFAULT_TRACKING_STATES = (
    NavStatus.STATE_TRACKING,
    NavStatus.STATE_TRACKING_DEGRADED,
)


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('target_altitude_m', 7.0)
        self.declare_parameter('vision_ready_duration_sec', 1.5)
        self.declare_parameter('vision_timeout_sec', 0.75)
        self.declare_parameter('vision_loss_abort_sec', 2.0)

        # State codes (uint8) that the mission accepts as "ready".
        # Defaults to TRACKING + TRACKING_DEGRADED.
        self.declare_parameter(
            'vision_allowed_state_codes',
            [int(code) for code in DEFAULT_TRACKING_STATES],
        )
        self.declare_parameter('vision_min_quality_score', 0.0)
        self.declare_parameter('pre_takeoff_settle_sec', 5.0)
        self.declare_parameter('command_timeout_sec', 5.0)
        self.declare_parameter('takeoff_timeout_sec', 45.0)
        self.declare_parameter('hover_duration_sec', 3.0)
        self.declare_parameter('land_timeout_sec', 90.0)
        self.declare_parameter('takeoff_reached_margin_m', 0.3)
        self.declare_parameter('landing_complete_altitude_m', 0.3)
        self.declare_parameter('arm_max_attempts', 5)

        self.target_altitude_m = self.get_parameter('target_altitude_m').value
        self.vision_ready_duration_sec = self.get_parameter('vision_ready_duration_sec').value
        self.vision_timeout_sec = self.get_parameter('vision_timeout_sec').value
        self.vision_loss_abort_sec = self.get_parameter('vision_loss_abort_sec').value
        self.vision_allowed_state_codes = {
            int(code) for code in self.get_parameter('vision_allowed_state_codes').value
        }
        self.vision_min_quality_score = float(
            self.get_parameter('vision_min_quality_score').value
        )
        self.pre_takeoff_settle_sec = self.get_parameter('pre_takeoff_settle_sec').value
        self.command_timeout_sec = self.get_parameter('command_timeout_sec').value
        self.takeoff_timeout_sec = self.get_parameter('takeoff_timeout_sec').value
        self.hover_duration_sec = self.get_parameter('hover_duration_sec').value
        self.land_timeout_sec = self.get_parameter('land_timeout_sec').value
        self.takeoff_reached_margin_m = self.get_parameter('takeoff_reached_margin_m').value
        self.landing_complete_altitude_m = self.get_parameter('landing_complete_altitude_m').value
        self.arm_max_attempts = max(1, int(self.get_parameter('arm_max_attempts').value))

        self.ap_connected = False
        self.ap_armed = False
        self.ap_mode = 'UNKNOWN'
        self.ap_external_nav_ready = False
        self.relative_alt_m = math.nan

        self.nav_state_code = NavStatus.STATE_NO_DATA
        self.nav_quality_score = 0.0
        self.nav_source = 'unknown'
        self.last_nav_rx_monotonic = 0.0
        self.nav_valid_since = None
        self.vision_loss_since = None

        self.pending_future = None
        self.pending_command_name = None
        self.pending_started_monotonic = 0.0
        self.last_service_wait_log_monotonic = 0.0
        self.last_nav_wait_log_monotonic = 0.0
        self.last_external_nav_wait_log_monotonic = 0.0
        self.arm_attempts = 0

        self.state = ''
        self.state_entered_monotonic = self.get_clock().now().nanoseconds / 1e9
        self.state_pub = self.create_publisher(String, '/mission_state', 10)

        self.create_subscription(NavStatus, '/nav/status', self._nav_status_cb, 10)
        self.create_subscription(
            Float64, '/ap/relative_alt', self._relative_alt_cb, qos_profile_sensor_data
        )
        self.create_subscription(Bool, '/ap/connected', self._ap_connected_cb, 10)
        self.create_subscription(Bool, '/ap/armed', self._ap_armed_cb, 10)
        self.create_subscription(Bool, '/ap/external_nav_ready', self._ap_ext_nav_cb, 10)
        self.create_subscription(String, '/ap/flight_mode', self._ap_mode_cb, 10)

        self.set_mode_client = self.create_client(SetMode, '/ap/cmd/set_mode')
        self.arm_client = self.create_client(SetBool, '/ap/cmd/arm')
        self.takeoff_client = self.create_client(Takeoff, '/ap/cmd/takeoff')
        self.land_client = self.create_client(Trigger, '/ap/cmd/land')
        self.switch_ekf_client = self.create_client(SwitchEKFSource, '/ap/cmd/switch_ekf_source')

        self.timer = self.create_timer(0.2, self._tick)

        self.get_logger().info(
            f'Configured autonomous scenario: takeoff to {self.target_altitude_m:.1f} m and land'
        )
        self._set_state('WAIT_FOR_AP', 'Waiting for MAVLink bridge connection')

    # -- subscriptions -----------------------------------------------------

    def _nav_status_cb(self, msg: NavStatus):
        self.nav_state_code = int(msg.state)
        self.nav_quality_score = float(msg.quality_score)
        self.nav_source = str(msg.source) or 'unknown'
        self.last_nav_rx_monotonic = self.get_clock().now().nanoseconds / 1e9

    def _relative_alt_cb(self, msg: Float64):
        self.relative_alt_m = float(msg.data)

    def _ap_connected_cb(self, msg: Bool):
        self.ap_connected = bool(msg.data)

    def _ap_armed_cb(self, msg: Bool):
        self.ap_armed = bool(msg.data)

    def _ap_ext_nav_cb(self, msg: Bool):
        self.ap_external_nav_ready = bool(msg.data)

    def _ap_mode_cb(self, msg: String):
        self.ap_mode = str(msg.data)

    # -- main tick ---------------------------------------------------------

    def _tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        self._update_vision_state(now)

        if self._should_abort_to_land(now):
            return

        if self.state == 'WAIT_FOR_AP':
            if self.ap_connected:
                self._set_state('WAIT_FOR_NAV', 'Bridge connection is alive')
            return

        if self.state == 'WAIT_FOR_NAV':
            if self._vision_ready(now):
                self._set_state('WAIT_FOR_SETTLE', 'Stable visual lock acquired')
            elif (now - self.last_nav_wait_log_monotonic) > 2.0:
                self.get_logger().info(
                    'Still waiting for stable navigation source: '
                    f'source={self.nav_source}, '
                    f'state_code={self.nav_state_code}, '
                    f'quality_score={self.nav_quality_score:.2f}'
                )
                self.last_nav_wait_log_monotonic = now
            return

        if self.state == 'WAIT_FOR_SETTLE':
            if not self._vision_ready(now):
                self._set_state(
                    'WAIT_FOR_NAV',
                    'Lost stable visual lock during pre-takeoff settle',
                )
            elif self._state_elapsed(now) > self.pre_takeoff_settle_sec:
                self._set_state(
                    'REQUEST_SWITCH_EKF',
                    'Pre-takeoff settle completed, requesting switch to ExternalNav (Source 2)',
                )
            return

        if self.state == 'REQUEST_SWITCH_EKF':
            req = SwitchEKFSource.Request()
            req.source = 2
            self._drive_service_state(
                client=self.switch_ekf_client,
                request=req,
                command_name='switch_ekf_source_2',
                success_state='WAIT_FOR_EXTERNAL_NAV',
            )
            return

        if self.state == 'WAIT_FOR_EXTERNAL_NAV':
            if not self._vision_ready(now):
                self._set_state(
                    'WAIT_FOR_NAV',
                    'Lost stable visual lock while waiting for ArduPilot external-nav readiness',
                )
            elif self.ap_external_nav_ready:
                self._set_state(
                    'REQUEST_RESET_MODE',
                    'ArduPilot reports external navigation ready',
                )
            elif (now - self.last_external_nav_wait_log_monotonic) > 2.0:
                self.get_logger().info(
                    'Waiting for ArduPilot external-nav readiness before arming'
                )
                self.last_external_nav_wait_log_monotonic = now
            return

        if self.state == 'REQUEST_RESET_MODE':
            self._drive_service_state(
                client=self.set_mode_client,
                request=self._make_set_mode_request('QLOITER'),
                command_name='set_mode QLOITER',
                success_state='WAIT_RESET_MODE',
            )
            return

        if self.state == 'WAIT_RESET_MODE':
            if self.ap_mode == 'QLOITER':
                if self.ap_armed:
                    self._set_state(
                        'REQUEST_DISARM',
                        'Reset mode confirmed, forcing clean arm cycle before takeoff',
                    )
                else:
                    self._set_state(
                        'REQUEST_ARM',
                        'Reset mode confirmed, vehicle is disarmed and ready for clean arm cycle',
                    )
            elif self._state_elapsed(now) > self.command_timeout_sec:
                self._set_state('REQUEST_RESET_MODE', 'Retrying reset mode request')
            return

        if self.state == 'REQUEST_DISARM':
            self._drive_service_state(
                client=self.arm_client,
                request=self._make_arm_request(False),
                command_name='disarm',
                success_state='WAIT_DISARMED',
                rejected_state='REQUEST_ARM',
                rejected_reason='Disarm rejected, continuing with arm synchronization',
            )
            return

        if self.state == 'WAIT_DISARMED':
            if not self.ap_armed:
                self._set_state('REQUEST_ARM', 'Vehicle is disarmed and ready for mission start')
            elif self._state_elapsed(now) > self.command_timeout_sec:
                self._set_state('REQUEST_DISARM', 'Retrying disarm request')
            return

        if self.state == 'REQUEST_ARM':
            self._drive_service_state(
                client=self.arm_client,
                request=self._make_arm_request(True),
                command_name='arm',
                success_state='WAIT_ARMED',
                rejected_state='ARM_RETRY',
                rejected_reason='Arm rejected by ArduPilot',
            )
            return

        if self.state == 'ARM_RETRY':
            self.arm_attempts += 1
            if self.arm_attempts >= self.arm_max_attempts:
                self._set_state(
                    'ABORT',
                    f'Arm rejected {self.arm_attempts} times - aborting mission for human review',
                )
                return
            if self._state_elapsed(now) >= 1.0:
                self._set_state(
                    'REQUEST_ARM',
                    f'Retrying arm (attempt {self.arm_attempts + 1}/{self.arm_max_attempts})',
                )
            return

        if self.state == 'WAIT_ARMED':
            if self.ap_armed:
                self._set_state(
                    'REQUEST_PRE_TAKEOFF_MODE',
                    'Vehicle armed, reapplying QLOITER before takeoff',
                )
            elif self._state_elapsed(now) > self.command_timeout_sec:
                self._set_state('REQUEST_ARM', 'Retrying arm request')
            return

        if self.state == 'REQUEST_PRE_TAKEOFF_MODE':
            self._drive_service_state(
                client=self.set_mode_client,
                request=self._make_set_mode_request('QLOITER'),
                command_name='set_mode QLOITER',
                success_state='WAIT_PRE_TAKEOFF_MODE',
            )
            return

        if self.state == 'WAIT_PRE_TAKEOFF_MODE':
            if self.ap_mode == 'QLOITER':
                self._set_state(
                    'REQUEST_TAKEOFF',
                    'Clean arm cycle completed, pre-takeoff mode confirmed',
                )
            elif self._state_elapsed(now) > self.command_timeout_sec:
                self._set_state(
                    'REQUEST_PRE_TAKEOFF_MODE',
                    'Retrying pre-takeoff mode request',
                )
            return

        if self.state == 'REQUEST_TAKEOFF':
            self._drive_service_state(
                client=self.takeoff_client,
                request=self._make_takeoff_request(self.target_altitude_m),
                command_name='takeoff',
                success_state='ASCENDING',
            )
            return

        if self.state == 'ASCENDING':
            if math.isfinite(self.relative_alt_m) and self.relative_alt_m >= (
                self.target_altitude_m - self.takeoff_reached_margin_m
            ):
                self._set_state('HOLD', 'Target altitude reached')
            elif self._state_elapsed(now) > self.takeoff_timeout_sec:
                self._transition_to_landing('Takeoff timeout exceeded')
            return

        if self.state == 'HOLD':
            if self._state_elapsed(now) > self.hover_duration_sec:
                self._set_state('REQUEST_LAND', 'Hover dwell complete')
            return

        if self.state == 'REQUEST_LAND':
            self._drive_service_state(
                client=self.land_client,
                request=Trigger.Request(),
                command_name='land',
                success_state='LANDING',
            )
            return

        if self.state == 'LANDING':
            landed = not self.ap_armed
            low_altitude = (
                math.isfinite(self.relative_alt_m)
                and self.relative_alt_m <= self.landing_complete_altitude_m
            )
            if landed and (low_altitude or self._state_elapsed(now) > 3.0):
                self._set_state('DONE', 'Landing completed')
            elif self._state_elapsed(now) > self.land_timeout_sec:
                self._set_state('ABORT', 'Landing timeout exceeded')
            return

    # -- vision gating -----------------------------------------------------

    def _update_vision_state(self, now):
        recent = (now - self.last_nav_rx_monotonic) <= self.vision_timeout_sec
        in_allowed_state = self.nav_state_code in self.vision_allowed_state_codes
        meets_quality = self.nav_quality_score >= self.vision_min_quality_score
        available = recent and in_allowed_state and meets_quality

        if available:
            if self.nav_valid_since is None:
                self.nav_valid_since = now
            self.vision_loss_since = None
        else:
            if self.vision_loss_since is None:
                self.vision_loss_since = now
            
            if (now - self.vision_loss_since) > self.vision_timeout_sec:
                self.nav_valid_since = None

    def _vision_ready(self, now):
        return (
            self.nav_valid_since is not None
            and (now - self.nav_valid_since) >= self.vision_ready_duration_sec
        )

    def _should_abort_to_land(self, now):
        if self.state not in {'ASCENDING', 'HOLD'}:
            return False
        if self.vision_loss_since is None:
            return False
        if (now - self.vision_loss_since) <= self.vision_loss_abort_sec:
            return False
        self._transition_to_landing('Visual localization lost during autonomous flight')
        return True

    # -- service helpers ---------------------------------------------------

    def _drive_service_state(
        self,
        client,
        request,
        command_name,
        success_state,
        rejected_state=None,
        rejected_reason='',
    ):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.pending_future is None:
            if not client.wait_for_service(timeout_sec=0.0):
                if (now - self.last_service_wait_log_monotonic) > 2.0:
                    self.get_logger().info(f'Waiting for service backing command: {command_name}')
                    self.last_service_wait_log_monotonic = now
                return

            self.pending_future = client.call_async(request)
            self.pending_command_name = command_name
            self.pending_started_monotonic = now
            self.get_logger().info(f'Requested command: {command_name}')
            return

        if not self.pending_future.done():
            if (now - self.pending_started_monotonic) > self.command_timeout_sec:
                self._clear_pending_command()
                self._set_state('ABORT', f'Service timeout waiting for {command_name}')
            return

        try:
            response = self.pending_future.result()
        except Exception as exc:
            self._clear_pending_command()
            self._set_state('ABORT', f'Command {command_name} failed: {exc}')
            return

        self._clear_pending_command()

        if getattr(response, 'success', True):
            message = getattr(response, 'message', '')
            if message:
                self.get_logger().info(f'{command_name}: {message}')
            self._set_state(success_state)
        else:
            if rejected_state is not None:
                message = getattr(response, 'message', '')
                reason = rejected_reason or f'Command {command_name} rejected'
                if message:
                    reason = f'{reason}: {message}'
                self._set_state(rejected_state, reason)
            else:
                self._set_state('ABORT', f'Command {command_name} rejected: {response.message}')

    def _transition_to_landing(self, reason):
        if self.state in {'REQUEST_LAND', 'LANDING', 'DONE', 'ABORT'}:
            return
        self._clear_pending_command()
        self._set_state('REQUEST_LAND', reason)

    def _clear_pending_command(self):
        self.pending_future = None
        self.pending_command_name = None
        self.pending_started_monotonic = 0.0

    def _state_elapsed(self, now):
        return now - self.state_entered_monotonic

    def _set_state(self, new_state, reason=''):
        if new_state == self.state:
            return

        if new_state == 'REQUEST_ARM' and self.state != 'ARM_RETRY':
            self.arm_attempts = 0

        self._clear_pending_command()
        self.state = new_state
        self.state_entered_monotonic = self.get_clock().now().nanoseconds / 1e9
        self.state_pub.publish(String(data=self.state))

        if reason:
            self.get_logger().info(f'Mission state -> {self.state}: {reason}')
        else:
            self.get_logger().info(f'Mission state -> {self.state}')

    def _make_set_mode_request(self, mode):
        request = SetMode.Request()
        request.mode = mode
        return request

    def _make_arm_request(self, should_arm):
        request = SetBool.Request()
        request.data = bool(should_arm)
        return request

    def _make_takeoff_request(self, altitude_m):
        request = Takeoff.Request()
        request.altitude_m = float(altitude_m)
        return request


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
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
