"""ROS service backends for ArduPilot commands.

All long-running waits run on a ``ReentrantCallbackGroup`` so the
MAVLink read timer keeps draining incoming traffic while one service
callback is blocked on a COMMAND_ACK / MISSION_ACK / mode confirmation.

The ``MavlinkConnection`` from :mod:`mavlink_io` is the synchronization
boundary: it serializes ``send`` and ``recv_match`` and notifies waiting
threads when the right message type arrives.
"""

from __future__ import annotations

import math
import time
from typing import Iterable, Optional

from flight_interfaces.srv import (
    GoToGlobal,
    SetMode,
    SwitchEKFSource,
    Takeoff,
    UploadMission,
)
from pymavlink import mavutil
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool, Trigger


# Bitmask for SET_POSITION_TARGET_LOCAL_NED that ignores everything except z.
ALTITUDE_ONLY_LOCAL_OFFSET_MASK = 3579

# EKF source selectors used by SwitchEKFSource. Only EKF3 is supported.
#
# These describe a FULL source switch (position, velocity and yaw)
# rather than just position. Without switching velocity to ExternalNav
# the EKF would still pull horizontal/vertical velocity from GPS, and
# on a real airframe with no GPS that source becomes "no data" and
# EKF3 would refuse to use external nav. Yaw stays on the compass for
# GPS mode and moves to ExternalNav when we're flying on visuals.
#
# POSZ=1 is Baro (the canonical "altitude from barometer" choice for
# GPS-aided flight). VELZ=3 = GPS-vertical-velocity. For visual-only
# every source is 6 = ExternalNav.
EKF_SRC_GPS = {
    'POSXY': 3.0, 'POSZ': 1.0,
    'VELXY': 3.0, 'VELZ': 3.0,
    'YAW': 1.0,
}
EKF_SRC_EXTERNAL_NAV = {
    'POSXY': 6.0, 'POSZ': 6.0,
    'VELXY': 6.0, 'VELZ': 6.0,
    'YAW': 6.0,
}


class CommandServices:
    """Bundle of services backed by a single MAVLink connection."""

    DEFAULT_ACK_TIMEOUT_SEC = 5.0
    # After VTOL_TAKEOFF reaches its target altitude we switch the vehicle
    # into QLOITER so the trailing LOITER_UNLIM mission item never gets
    # executed in fixed-wing mode (otherwise an ArduPlane VTOL transitions
    # to forward flight and starts circling at WP_LOITER_RAD).
    POST_TAKEOFF_MARGIN_M = 0.5
    POST_TAKEOFF_TIMEOUT_SEC = 180.0
    POST_TAKEOFF_HOLD_MODE = 'QLOITER'

    def __init__(self, node, connection, telemetry):
        self._node = node
        self._logger = node.get_logger()
        self._conn = connection
        self._telemetry = telemetry

        # Post-takeoff watchdog state. Set inside _takeoff_cb when the
        # AUTO mission successfully starts; cleared either by the
        # watchdog (target altitude reached) or by _land_cb (manual
        # abort to land).
        self._post_takeoff_target_alt_m: Optional[float] = None
        self._post_takeoff_started_monotonic: float = 0.0

        cb_group = ReentrantCallbackGroup()

        self._set_mode_srv = node.create_service(
            SetMode, '/ap/cmd/set_mode', self._set_mode_cb, callback_group=cb_group
        )
        self._arm_srv = node.create_service(
            SetBool, '/ap/cmd/arm', self._arm_cb, callback_group=cb_group
        )
        self._takeoff_srv = node.create_service(
            Takeoff, '/ap/cmd/takeoff', self._takeoff_cb, callback_group=cb_group
        )
        self._land_srv = node.create_service(
            Trigger, '/ap/cmd/land', self._land_cb, callback_group=cb_group
        )
        self._goto_srv = node.create_service(
            GoToGlobal, '/ap/cmd/goto_global', self._goto_cb, callback_group=cb_group
        )
        self._upload_mission_srv = node.create_service(
            UploadMission,
            '/ap/cmd/upload_mission',
            self._upload_mission_cb,
            callback_group=cb_group,
        )
        self._switch_ekf_srv = node.create_service(
            SwitchEKFSource,
            '/ap/cmd/switch_ekf_source',
            self._switch_ekf_cb,
            callback_group=cb_group,
        )

        # Watchdog that promotes the vehicle to QLOITER once VTOL_TAKEOFF
        # reaches the requested altitude. Runs in the same MAVLink-IO
        # callback group as the read loop so it does not race with
        # service callbacks.
        self._post_takeoff_timer = node.create_timer(
            0.2, self._post_takeoff_tick
        )

    # -- helpers ----------------------------------------------------------

    def _send(self, sender, *args, **kwargs):
        with self._conn.lock():
            master = self._conn.master
            if master is None:
                raise RuntimeError('ArduPilot connection is not available')
            return sender(master, *args, **kwargs)

    def _wait_for_command_ack(self, expected_command, timeout_sec=DEFAULT_ACK_TIMEOUT_SEC):
        return self._conn.wait_for(
            'COMMAND_ACK',
            timeout_sec,
            predicate=lambda msg: int(msg.command) == int(expected_command),
        )

    def _wait_for_mode(self, expected_mode, timeout_sec=3.0):
        normalized = expected_mode.strip().upper()
        if self._telemetry.latest_mode == normalized:
            return True
        deadline = self._node.get_clock().now().nanoseconds / 1e9 + float(timeout_sec)
        while True:
            remaining = deadline - self._node.get_clock().now().nanoseconds / 1e9
            if remaining <= 0:
                break
            self._conn.wait_for('HEARTBEAT', min(remaining, 1.0))
            if self._telemetry.latest_mode == normalized:
                return True
        return self._telemetry.latest_mode == normalized

    def _wait_for_armed_state(self, expected_armed, timeout_sec=3.0):
        expected_armed = bool(expected_armed)
        deadline = self._node.get_clock().now().nanoseconds / 1e9 + float(timeout_sec)
        while True:
            if bool(self._telemetry.latest_armed) == expected_armed:
                return True
            remaining = deadline - self._node.get_clock().now().nanoseconds / 1e9
            if remaining <= 0:
                break
            self._conn.wait_for('HEARTBEAT', min(remaining, 0.5))
        return bool(self._telemetry.latest_armed) == expected_armed

    def _command_result_name(self, result) -> str:
        result_enum = mavutil.mavlink.enums.get('MAV_RESULT', {})
        return result_enum.get(result).name if result in result_enum else str(result)

    def _mission_result_name(self, result) -> str:
        result_enum = mavutil.mavlink.enums.get('MAV_MISSION_RESULT', {})
        return result_enum.get(result).name if result in result_enum else str(result)

    def _send_mode_request(self, mode_name):
        normalized = mode_name.strip().upper()
        with self._conn.lock():
            master = self._conn.master
            if master is None:
                raise RuntimeError('ArduPilot connection is not available')
            mapping = master.mode_mapping() or {}
            if normalized not in mapping:
                available = ', '.join(sorted(mapping.keys()))
                raise RuntimeError(
                    f'Unsupported mode "{normalized}". Available modes: {available}'
                )
            master.set_mode(mapping[normalized])
        return normalized

    def _clear_rc_overrides(self):
        def sender(master):
            master.mav.rc_channels_override_send(
                master.target_system, master.target_component,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            )

        self._send(sender)

    # -- service callbacks ------------------------------------------------

    def _set_mode_cb(self, request, response):
        try:
            requested = request.mode.strip().upper()
            # External mode change overrides the post-takeoff promotion;
            # the caller is taking explicit control of the flight mode.
            if requested != 'AUTO' and self._post_takeoff_target_alt_m is not None:
                self._post_takeoff_target_alt_m = None
                self._logger.info(
                    'Post-takeoff watchdog cancelled by external set_mode request'
                )
            mode = self._send_mode_request(request.mode)
            response.success = True
            response.message = f'Flight mode change requested: {mode}'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._logger.error(f'Failed to set mode: {exc}')
        return response

    def _arm_cb(self, request, response):
        if self._conn.master is None:
            response.success = False
            response.message = 'ArduPilot connection is not available'
            return response

        expected_armed = bool(request.data)
        action = 'ARM' if expected_armed else 'DISARM'
        target_state_name = 'armed' if expected_armed else 'disarmed'

        if expected_armed and self._telemetry.latest_armed:
            response.success = True
            response.message = 'Vehicle already armed'
            return response

        if not expected_armed and not self._telemetry.latest_armed:
            response.success = True
            response.message = 'Vehicle already disarmed'
            return response

        try:
            self._clear_rc_overrides()

            def sender(master):
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                    1.0 if expected_armed else 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                )

            self._send(sender)
            ack = self._wait_for_command_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
            if ack is None:
                response.success = False
                response.message = f'Timed out waiting for {action} acknowledgement'
                return response
            if ack.result not in (
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            ):
                pre_arm_hint = ''
                if expected_armed and self._telemetry.prearm_failures:
                    pre_arm_hint = (
                        ' Last pre-arm messages: '
                        + ' | '.join(self._telemetry.prearm_failures[-3:])
                    )
                response.success = False
                response.message = (
                    f'{action} rejected with {self._command_result_name(ack.result)}'
                    + pre_arm_hint
                )
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
            self._logger.error(f'Failed to send arm/disarm command: {exc}')
        return response

    def _takeoff_cb(self, request, response):
        if self._conn.master is None:
            response.success = False
            response.message = 'ArduPilot connection is not available'
            return response

        if request.altitude_m <= 0.0:
            response.success = False
            response.message = 'Takeoff altitude must be positive'
            return response

        try:
            self._clear_rc_overrides()
            self._logger.info(
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

            def sender(master):
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_MISSION_START, 0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                )

            self._send(sender)
            ack = self._wait_for_command_ack(mavutil.mavlink.MAV_CMD_MISSION_START)
            if ack is None:
                response.success = False
                response.message = 'Timed out waiting for mission start acknowledgement'
                return response

            if ack.result in (
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            ):
                self._arm_post_takeoff_watchdog(float(request.altitude_m))
                response.success = True
                response.message = (
                    f'Uploaded VTOL AUTO takeoff mission to {float(request.altitude_m):.2f} m '
                    'and started AUTO mission'
                )
            else:
                response.success = False
                response.message = (
                    f'MISSION_START rejected with {self._command_result_name(ack.result)}'
                )
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._logger.error(f'Failed to request takeoff altitude: {exc}')
        return response

    def _land_cb(self, request, response):
        del request
        # Cancel any pending post-takeoff promotion: we are about to land,
        # so we don't want the watchdog to reapply QLOITER on top of QLAND.
        self._post_takeoff_target_alt_m = None
        try:
            mode = self._send_mode_request('QLAND')
            response.success = True
            response.message = f'Landing mode requested: {mode}'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._logger.error(f'Failed to request QLAND: {exc}')
        return response

    def _arm_post_takeoff_watchdog(self, target_alt_m: float) -> None:
        self._post_takeoff_target_alt_m = float(target_alt_m)
        self._post_takeoff_started_monotonic = self._node.get_clock().now().nanoseconds / 1e9
        self._logger.info(
            f'Post-takeoff watchdog armed: will switch to '
            f'{self.POST_TAKEOFF_HOLD_MODE} once altitude '
            f'>= {target_alt_m - self.POST_TAKEOFF_MARGIN_M:.2f} m'
        )

    def _post_takeoff_tick(self) -> None:
        if self._post_takeoff_target_alt_m is None:
            return

        target = self._post_takeoff_target_alt_m
        rel_alt = self._telemetry.latest_relative_alt
        elapsed = (self._node.get_clock().now().nanoseconds / 1e9) - self._post_takeoff_started_monotonic

        if math.isfinite(rel_alt) and rel_alt >= (target - self.POST_TAKEOFF_MARGIN_M):
            self._post_takeoff_target_alt_m = None
            try:
                self._send_mode_request(self.POST_TAKEOFF_HOLD_MODE)
                self._logger.info(
                    f'Post-takeoff: altitude {rel_alt:.2f} m reached, switched to '
                    f'{self.POST_TAKEOFF_HOLD_MODE} (mission paused)'
                )
            except Exception as exc:
                self._logger.warning(
                    f'Post-takeoff: failed to switch to {self.POST_TAKEOFF_HOLD_MODE}: {exc}'
                )
            return

        if elapsed > self.POST_TAKEOFF_TIMEOUT_SEC:
            self._post_takeoff_target_alt_m = None
            self._logger.warning(
                f'Post-takeoff watchdog timed out after {elapsed:.1f}s '
                f'(altitude={rel_alt}, target={target:.2f} m); '
                'leaving the vehicle in AUTO LOITER_UNLIM'
            )

    def _goto_cb(self, request, response):
        if self._conn.master is None:
            response.success = False
            response.message = 'ArduPilot connection is not available'
            return response
        try:
            def sender(master):
                master.mav.mission_item_int_send(
                    master.target_system, master.target_component,
                    0,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    2,
                    0,
                    0.0, 0.0, 0.0, 0.0,
                    int(request.lat_deg * 1e7),
                    int(request.lon_deg * 1e7),
                    float(request.alt_m),
                )

            self._send(sender)
            response.success = True
            response.message = (
                f'GoTo target sent via MISSION_ITEM_INT: '
                f'{request.lat_deg}, {request.lon_deg}, {request.alt_m}m'
            )
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._logger.error(f'Failed to send goto_global: {exc}')
        return response

    def _switch_ekf_cb(self, request, response):
        if self._conn.master is None:
            response.success = False
            response.message = 'ArduPilot connection is not available'
            return response
        try:
            params = EKF_SRC_EXTERNAL_NAV if int(request.source) == 2 else EKF_SRC_GPS
            # All sources that define a complete navigation backend: the
            # EKF needs a position+velocity+yaw triple from the same
            # family or it falls back to whatever else is configured
            # (typically GPS), which defeats the point on a GPS-denied
            # platform.
            param_map = [
                (b'EK3_SRC1_POSXY', params['POSXY']),
                (b'EK3_SRC1_POSZ',  params['POSZ']),
                (b'EK3_SRC1_VELXY', params['VELXY']),
                (b'EK3_SRC1_VELZ',  params['VELZ']),
                (b'EK3_SRC1_YAW',   params['YAW']),
            ]

            def make_setter(name: bytes, value: float):
                def _send(master):
                    master.mav.param_set_send(
                        master.target_system, master.target_component,
                        name, float(value),
                        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                    )
                return _send

            def make_reader(name: bytes):
                def _send(master):
                    master.mav.param_request_read_send(
                        master.target_system, master.target_component,
                        name, -1,
                    )
                return _send

            for name, value in param_map:
                self._send(make_setter(name, value))
            # Explicitly ask AP to echo the new values. Without this, on
            # some firmware/stream-rate setups the only signal of an
            # EKF source change is the (sometimes silent) "EKF3 IMU0
            # is using external nav data" STATUSTEXT — and if AHRS is
            # configured to use a non-EKF3 backend (e.g. SITL cheat
            # backend, AHRS_EKF_TYPE=10) that STATUSTEXT never fires
            # and the mission state machine waits forever. The
            # PARAM_VALUE echo is parsed in TelemetryPublisher and used
            # as an alternative external-nav-ready signal.
            for name, _ in param_map:
                self._send(make_reader(name))
            response.success = True
            response.message = (
                f'EKF Source {request.source} requested via PARAM_SET '
                f'(POS+VEL+YAW; {len(param_map)} params)'
            )
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._logger.error(f'Failed to switch EKF source: {exc}')
        return response

    def _upload_mission_cb(self, request, response):
        if self._conn.master is None:
            response.success = False
            response.message = 'ArduPilot connection is not available'
            return response

        if (
            not request.lats
            or len(request.lats) != len(request.lons)
            or len(request.lats) != len(request.alts)
        ):
            response.success = False
            response.message = 'Invalid waypoint arrays'
            return response

        try:
            self._logger.info(f'Uploading mission with {len(request.lats)} waypoints')
            home = self._home_position_or_raise()
            items = [
                _waypoint_at(home.lat_int, home.lon_int, home.alt_msl_m),
                _vtol_takeoff_item(home.lat_int, home.lon_int, float(request.alts[0])),
            ]
            for lat, lon, alt in zip(request.lats, request.lons, request.alts):
                items.append(
                    _global_relative_waypoint(
                        int(lat * 1e7), int(lon * 1e7), float(alt)
                    )
                )
            items.append(_rtl_item())

            self._clear_mission()
            self._upload_mission_items(items)

            response.success = True
            response.message = f'Successfully uploaded mission with {len(request.lats)} waypoints'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self._logger.error(f'Failed to upload mission: {exc}')
        return response

    # -- mission helpers ---------------------------------------------------

    def _home_position_or_raise(self):
        if (
            self._telemetry.latest_global_lat_int is None
            or self._telemetry.latest_global_lon_int is None
            or not math.isfinite(self._telemetry.latest_global_alt_msl_m)
        ):
            raise RuntimeError('Current global position is unavailable for mission upload')
        return _HomePosition(
            lat_int=int(self._telemetry.latest_global_lat_int),
            lon_int=int(self._telemetry.latest_global_lon_int),
            alt_msl_m=float(self._telemetry.latest_global_alt_msl_m),
        )

    def _build_vtol_takeoff_mission(self, altitude_m):
        # ArduPlane VTOL AUTO takeoff mission. The trailing LOITER_UNLIM is
        # critical: without it the mission completes the instant the vehicle
        # reaches takeoff altitude and ArduPilot drops the throttle, so the
        # quad never actually leaves the ground.
        home = self._home_position_or_raise()
        return [
            _waypoint_at(home.lat_int, home.lon_int, home.alt_msl_m),
            _vtol_takeoff_item(home.lat_int, home.lon_int, float(altitude_m)),
            _loiter_unlim_item(home.lat_int, home.lon_int, float(altitude_m)),
        ]

    def _clear_mission(self, timeout_sec=2.0):
        def sender(master):
            master.mav.mission_clear_all_send(master.target_system, master.target_component)

        self._send(sender)
        ack = self._conn.wait_for('MISSION_ACK', timeout_sec)
        if ack is None:
            raise RuntimeError('Timed out waiting for MISSION_ACK after mission_clear_all')

    def _upload_mission_items(self, items, timeout_sec=10.0):
        # We listen for MISSION_REQUEST, MISSION_REQUEST_INT and MISSION_ACK
        # in a single waiter so we cannot lose the final MISSION_ACK while
        # a separate waiter is parked on a request type.
        def sender(master):
            master.mav.mission_count_send(
                master.target_system, master.target_component, len(items),
            )

        self._send(sender)

        deadline_ns = self._node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        sent = 0
        while True:
            remaining_sec = max(
                0.0, (deadline_ns - self._node.get_clock().now().nanoseconds) / 1e9
            )
            if remaining_sec <= 0.0:
                raise RuntimeError('Timed out waiting for mission upload handshake')

            msg = self._conn.wait_for(
                ('MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'),
                remaining_sec,
            )
            if msg is None:
                raise RuntimeError('Timed out waiting for mission upload handshake')

            msg_type = msg.get_type()
            if msg_type == 'MISSION_ACK':
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    return
                raise RuntimeError(
                    f'Mission upload rejected with {self._mission_result_name(msg.type)}'
                )

            seq = int(msg.seq)
            if seq < 0 or seq >= len(items):
                raise RuntimeError(f'Unexpected mission item request for seq={seq}')

            item = items[seq]

            def sender_item(master, item=item, seq=seq):
                master.mav.mission_item_int_send(
                    master.target_system, master.target_component,
                    seq, item['frame'], item['command'],
                    item['current'], item['autocontinue'],
                    item['param1'], item['param2'], item['param3'], item['param4'],
                    item['x'], item['y'], item['z'],
                )

            self._send(sender_item)
            sent += 1
            if sent > len(items) + 4:
                raise RuntimeError('Mission upload appears to be looping; aborting')


# -- mission item builders ------------------------------------------------

class _HomePosition:
    __slots__ = ('lat_int', 'lon_int', 'alt_msl_m')

    def __init__(self, lat_int, lon_int, alt_msl_m):
        self.lat_int = lat_int
        self.lon_int = lon_int
        self.alt_msl_m = alt_msl_m


def _waypoint_at(lat_int, lon_int, alt_abs_m):
    return {
        'frame': mavutil.mavlink.MAV_FRAME_GLOBAL,
        'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        'current': 0, 'autocontinue': 1,
        'param1': 0.0, 'param2': 0.0, 'param3': 0.0, 'param4': 0.0,
        'x': lat_int, 'y': lon_int, 'z': alt_abs_m,
    }


def _vtol_takeoff_item(lat_int, lon_int, alt_rel_m):
    return {
        'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        'command': mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF,
        'current': 0, 'autocontinue': 1,
        'param1': 0.0, 'param2': 0.0, 'param3': 0.0, 'param4': 0.0,
        'x': lat_int, 'y': lon_int, 'z': alt_rel_m,
    }


def _loiter_unlim_item(lat_int, lon_int, alt_rel_m):
    return {
        'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        'command': mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
        'current': 0, 'autocontinue': 1,
        'param1': 0.0, 'param2': 0.0, 'param3': 0.0, 'param4': 0.0,
        'x': lat_int, 'y': lon_int, 'z': alt_rel_m,
    }


def _global_relative_waypoint(lat_int, lon_int, alt_rel_m):
    return {
        'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        'current': 0, 'autocontinue': 1,
        'param1': 0.0, 'param2': 0.0, 'param3': 0.0, 'param4': 0.0,
        'x': lat_int, 'y': lon_int, 'z': alt_rel_m,
    }


def _rtl_item():
    return {
        'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        'command': mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        'current': 0, 'autocontinue': 1,
        'param1': 0.0, 'param2': 0.0, 'param3': 0.0, 'param4': 0.0,
        'x': 0, 'y': 0, 'z': 0,
    }
