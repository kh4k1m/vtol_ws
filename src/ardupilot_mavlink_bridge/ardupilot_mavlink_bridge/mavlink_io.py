"""Thread-safe MAVLink connection wrapper used by the bridge node.

This module owns the single pymavlink connection. All other components
(command services, vision forwarder, telemetry republisher) talk to
ArduPilot through this object. We:

* serialize concurrent ``send`` and ``recv_match`` calls behind one lock,
  because pymavlink's MAVFile is not thread-safe;
* dispatch incoming messages to per-type handlers without blocking the
  caller;
* expose a ``wait_for`` helper that lets a service callback block on a
  specific message arriving, while other callback groups keep running.

The reconnection logic is centralized here so the rest of the bridge
sees a single ``is_connected()`` boolean and a stream of dispatched
messages.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Dict, Iterable, List, Optional, Tuple, Union

from pymavlink import mavutil


MsgTypes = Union[str, Iterable[str]]


HandlerFn = Callable[[object], None]


class MavlinkConnection:
    """Wrap a pymavlink mavlink connection with locking and dispatch."""

    HEARTBEAT_TIMEOUT_SEC = 5.0

    def __init__(self, connection_string: str, baudrate: int, logger):
        self._conn_str = connection_string
        self._baud = int(baudrate)
        self._logger = logger
        self._lock = threading.Lock()
        self._handlers: Dict[str, List[HandlerFn]] = {}
        self._wildcard_handlers: List[HandlerFn] = []
        self._waiters: List["_Waiter"] = []
        self._waiters_lock = threading.Lock()

        self._master = None
        self._autopilot_system_id: Optional[int] = None
        self._autopilot_component_id: Optional[int] = None
        self._last_heartbeat_monotonic = 0.0
        self._fully_connected = False
        self._on_connect_cb: Optional[Callable[[], None]] = None

    @property
    def conn_str(self) -> str:
        return self._conn_str

    @property
    def baud(self) -> int:
        return self._baud

    @property
    def autopilot_system_id(self) -> Optional[int]:
        return self._autopilot_system_id

    @property
    def autopilot_component_id(self) -> Optional[int]:
        return self._autopilot_component_id

    @property
    def master(self):
        """Direct access to the pymavlink master. Caller must hold ``lock()``."""
        return self._master

    def lock(self):
        return self._lock

    def is_connected(self) -> bool:
        return (
            self._fully_connected
            and (time.monotonic() - self._last_heartbeat_monotonic) < 3.0
        )

    def add_handler(self, msg_type: str, callback: HandlerFn) -> None:
        self._handlers.setdefault(msg_type, []).append(callback)

    def add_wildcard_handler(self, callback: HandlerFn) -> None:
        self._wildcard_handlers.append(callback)

    def set_on_connect(self, callback: Callable[[], None]) -> None:
        self._on_connect_cb = callback

    def open_if_needed(self) -> None:
        """Open the connection if closed. Reconnect on heartbeat timeout."""
        now = time.monotonic()
        if (
            self._fully_connected
            and (now - self._last_heartbeat_monotonic) > self.HEARTBEAT_TIMEOUT_SEC
        ):
            self._logger.warning(
                'Lost connection to ArduPilot (heartbeat timeout). Reconnecting...'
            )
            self._teardown_connection()

        if self._master is None:
            try:
                self._master = mavutil.mavlink_connection(self._conn_str, baud=self._baud)
                self._logger.info(
                    f'Opened connection to {self._conn_str}, waiting for heartbeat...'
                )
                self._last_heartbeat_monotonic = now
            except Exception as exc:
                self._logger.error(f'Failed to open connection: {exc}')
                self._master = None

    def _teardown_connection(self) -> None:
        master = self._master
        self._master = None
        self._fully_connected = False
        self._autopilot_system_id = None
        self._autopilot_component_id = None
        if master is not None:
            try:
                master.close()
            except Exception:
                pass

    def poll(self) -> None:
        """Read all pending MAVLink messages and dispatch them to handlers.

        Safe to call from a periodic ROS timer.
        """
        if self._master is None:
            return

        while True:
            try:
                with self._lock:
                    if self._master is None:
                        return
                    msg = self._master.recv_match(blocking=False)
            except Exception as exc:
                self._logger.debug(f'MAVLink read error: {exc}')
                return

            if msg is None:
                return

            self._handle_internal(msg)
            self._dispatch(msg)

    def _handle_internal(self, msg) -> None:
        msg_type = msg.get_type()
        if msg_type != 'HEARTBEAT':
            return

        if not self._fully_connected and msg.get_srcComponent() == 1:
            self._autopilot_system_id = int(msg.get_srcSystem())
            self._autopilot_component_id = int(msg.get_srcComponent())
            self._fully_connected = True
            self._logger.info(
                f'Successfully connected to ArduPilot (System ID: {self._autopilot_system_id})'
            )
            self._request_default_streams()
            if self._on_connect_cb is not None:
                try:
                    self._on_connect_cb()
                except Exception as exc:
                    self._logger.warning(f'on_connect callback raised: {exc}')

        if self._is_primary_autopilot_message(msg):
            self._last_heartbeat_monotonic = time.monotonic()

    def _is_primary_autopilot_message(self, msg) -> bool:
        if self._autopilot_system_id is None:
            return False
        if int(msg.get_srcSystem()) != int(self._autopilot_system_id):
            return False
        if self._autopilot_component_id in (None, 0):
            self._autopilot_component_id = int(msg.get_srcComponent())
            return True
        return int(msg.get_srcComponent()) == int(self._autopilot_component_id)

    def is_primary_autopilot_message(self, msg) -> bool:
        return self._is_primary_autopilot_message(msg)

    def _dispatch(self, msg) -> None:
        msg_type = msg.get_type()

        with self._waiters_lock:
            still_active: List[_Waiter] = []
            for waiter in self._waiters:
                if waiter.consume(msg, msg_type):
                    continue
                still_active.append(waiter)
            self._waiters = still_active

        for handler in self._handlers.get(msg_type, ()):
            try:
                handler(msg)
            except Exception as exc:
                self._logger.warning(f'handler for {msg_type} raised: {exc}')
        for handler in self._wildcard_handlers:
            try:
                handler(msg)
            except Exception as exc:
                self._logger.warning(f'wildcard handler raised: {exc}')

    def _request_default_streams(self) -> None:
        if self._master is None:
            return
        with self._lock:
            try:
                self._master.mav.request_data_stream_send(
                    self._master.target_system,
                    self._master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                    10,
                    1,
                )
                self._master.mav.request_data_stream_send(
                    self._master.target_system,
                    self._master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                    50,
                    1,
                )
                self._master.mav.request_data_stream_send(
                    self._master.target_system,
                    self._master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                    10,
                    1,
                )
                self._master.mav.request_data_stream_send(
                    self._master.target_system,
                    self._master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    10,
                    1,
                )
            except Exception as exc:
                self._logger.warning(f'Failed to request MAVLink data streams: {exc}')

    def wait_for(
        self,
        msg_types: MsgTypes,
        timeout_sec: float,
        predicate: Optional[Callable[[object], bool]] = None,
    ):
        """Block the calling thread until a matching message is dispatched.

        ``msg_types`` may be a single MAVLink message type (e.g.
        ``'COMMAND_ACK'``) or an iterable of types (e.g.
        ``('MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK')``) -
        the waiter wakes up on whichever of them arrives first. Returns
        the matched message or ``None`` on timeout.
        """
        types: Tuple[str, ...]
        if isinstance(msg_types, str):
            types = (msg_types,)
        else:
            types = tuple(msg_types)
            if not types:
                raise ValueError('wait_for requires at least one message type')

        waiter = _Waiter(types, predicate)
        with self._waiters_lock:
            self._waiters.append(waiter)
        try:
            return waiter.wait(timeout_sec)
        finally:
            with self._waiters_lock:
                if waiter in self._waiters:
                    self._waiters.remove(waiter)


class _Waiter:
    """Single-shot wait condition used by ``MavlinkConnection.wait_for``."""

    def __init__(
        self,
        msg_types: Tuple[str, ...],
        predicate: Optional[Callable[[object], bool]],
    ):
        self._msg_types = msg_types
        self._predicate = predicate
        self._event = threading.Event()
        self._matched = None

    def consume(self, msg, msg_type: str) -> bool:
        if msg_type not in self._msg_types:
            return False
        if self._predicate is not None and not self._predicate(msg):
            return False
        self._matched = msg
        self._event.set()
        return True

    def wait(self, timeout_sec: float):
        triggered = self._event.wait(timeout=max(0.0, float(timeout_sec)))
        return self._matched if triggered else None
