"""Start/stop rosbag + optional video + telemetry CSV on /ap/safety_released edges.

Each cycle ``safety released (True) -> engaged (False)`` opens one
timestamped folder under ``log_root`` and gracefully stops the recorders
with SIGINT (so sqlite rosbag closes cleanly and MP4 finalizes its moov
atom). Companion artefacts from the long-running ``tag_localizer``
(per-flight discovered marker map, anchor log) are snapshotted into the
cycle directory at stop so the cycle dir is self-contained for
post-flight analysis.

The supervisor is launched only when
``flight.yaml -> logging.safety_gated_sessions.enabled: true``. When
disabled, the regular bag/video/csv recorders run continuously for the
whole launch lifetime and this node is not started.
"""

from __future__ import annotations

import glob
import os
import shutil
import signal
import subprocess
import time
from typing import List, Optional, Tuple

import rclpy
import yaml
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import Bool


# Per-process budget given for graceful SIGINT shutdown before the
# supervisor escalates to SIGKILL. Generous, because sqlite rosbag
# flushes can take >10 s on slow disks under WSL.
_GRACEFUL_STOP_TIMEOUT_SEC = 20.0

# How often to poll alive subprocesses while waiting for graceful exit.
_STOP_POLL_INTERVAL_SEC = 0.05


class SafetyLogSupervisor(Node):
    def __init__(self):
        super().__init__('safety_log_supervisor')

        self.declare_parameter('workspace_dir', '')
        self.declare_parameter('session_prefix', 'session')
        self.declare_parameter('log_root', 'logs')
        self.declare_parameter('safety_topic', '/ap/safety_released')
        # Use an explicit STRING_ARRAY descriptor so we can default to
        # the empty list. Without the descriptor rclpy rejects [] and
        # the legacy workaround was to default to ['']` then filter.
        self.declare_parameter(
            'rosbag_topics',
            [],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )
        self.declare_parameter('rosbag_storage_id', 'sqlite3')
        self.declare_parameter('rosbag_compression', '')
        self.declare_parameter('video_enabled', False)
        self.declare_parameter('video_fps', 30.0)
        self.declare_parameter('video_chunk_duration_sec', 60.0)
        self.declare_parameter('video_topic', '/camera/image_raw')
        self.declare_parameter('telemetry_csv_enabled', False)
        self.declare_parameter('telemetry_csv_rate_hz', 10.0)
        self.declare_parameter('stack_mode', '')
        self.declare_parameter('stack_navigation', '')
        self.declare_parameter('stack_environment', '')
        # Directory the tag_localizer streams its discovered_map.yaml +
        # anchor_*.yaml into. We snapshot those into each cycle folder
        # at stop so a cycle dir is self-contained. Empty string ->
        # snapshotting disabled.
        self.declare_parameter('localizer_runtime_dir', '')
        # Path to config/markers.yaml. Loaded once per cycle so the
        # manifest contains the same marker_detector_profile block the
        # non-safety-gated launch already writes (dictionary +
        # default_marker_size + markers list). Empty -> profile omitted.
        self.declare_parameter('markers_yaml_path', '')

        self._ws = self.get_parameter('workspace_dir').get_parameter_value().string_value
        self._prefix = self.get_parameter('session_prefix').get_parameter_value().string_value
        self._log_root = self.get_parameter('log_root').get_parameter_value().string_value
        t = self.get_parameter('rosbag_topics').get_parameter_value().string_array_value
        self._topics: List[str] = [s for s in t if s]
        self._storage = self.get_parameter('rosbag_storage_id').get_parameter_value().string_value
        comp = self.get_parameter('rosbag_compression').get_parameter_value().string_value
        self._compression = str(comp or '').strip().lower()

        self._video_on = self.get_parameter('video_enabled').get_parameter_value().bool_value
        self._video_fps = self.get_parameter('video_fps').get_parameter_value().double_value
        self._video_chunk = self.get_parameter('video_chunk_duration_sec').get_parameter_value().double_value
        self._video_topic = self.get_parameter('video_topic').get_parameter_value().string_value
        self._csv_on = self.get_parameter('telemetry_csv_enabled').get_parameter_value().bool_value
        self._csv_hz = self.get_parameter('telemetry_csv_rate_hz').get_parameter_value().double_value

        self._stack_mode = self.get_parameter('stack_mode').get_parameter_value().string_value
        self._stack_nav = self.get_parameter('stack_navigation').get_parameter_value().string_value
        self._stack_env = self.get_parameter('stack_environment').get_parameter_value().string_value

        self._localizer_runtime_dir = self.get_parameter(
            'localizer_runtime_dir'
        ).get_parameter_value().string_value
        self._markers_yaml_path = self.get_parameter(
            'markers_yaml_path'
        ).get_parameter_value().string_value

        self._last: Optional[bool] = None
        self._session_dir: Optional[str] = None
        self._bag_proc: Optional[subprocess.Popen] = None
        self._vid_proc: Optional[subprocess.Popen] = None
        self._csv_proc: Optional[subprocess.Popen] = None

        safety_topic = self.get_parameter('safety_topic').get_parameter_value().string_value
        self.create_subscription(Bool, safety_topic, self._safety_cb, 10)

        self.get_logger().info(
            f'Safety log supervisor: topic={safety_topic} topics={len(self._topics)} '
            f'video={self._video_on} csv={self._csv_on} '
            f'localizer_runtime_dir={self._localizer_runtime_dir or "(none)"}'
        )

    # -- safety edges -------------------------------------------------

    def _safety_cb(self, msg: Bool) -> None:
        s = bool(msg.data)
        if self._last is None:
            self._last = s
            if s:
                self._start_session('initial state: safety released')
            return
        if s and not self._last:
            self._start_session('safety released -> record')
        elif not s and self._last:
            self._stop_session('safety engaged -> stop')
        self._last = s

    # -- session lifecycle --------------------------------------------

    def _start_session(self, reason: str) -> None:
        if self._session_dir is not None:
            self.get_logger().warning(
                f'Start requested while already recording; ignoring ({reason})'
            )
            return
        if not self._topics:
            self.get_logger().error('rosbag_topics empty; check flight.yaml / launch')
            return

        stamp = time.strftime('%Y%m%d-%H%M%S')
        session_dir = os.path.join(self._ws, self._log_root, f'{self._prefix}_{stamp}')
        try:
            os.makedirs(session_dir, exist_ok=True)
            video_dir = os.path.join(session_dir, 'video')
            os.makedirs(video_dir, exist_ok=True)
        except OSError as exc:
            self.get_logger().error(f'Failed to create session dir {session_dir}: {exc}')
            return
        bag_dir = os.path.join(session_dir, 'bag')

        # Reserve the session slot AFTER directories exist so a partial
        # mkdir failure doesn't leave us thinking we have an active
        # session.
        self._session_dir = session_dir

        try:
            self._write_manifest(session_dir)

            self.get_logger().info(f'{reason}: session={session_dir}')

            env = os.environ.copy()
            cmd = ['ros2', 'bag', 'record', '-s', self._storage, '-o', bag_dir]
            if self._compression and self._storage != 'mcap':
                cmd += ['--compression-format', self._compression, '--compression-mode', 'message']
            cmd += self._topics
            self._bag_proc = subprocess.Popen(cmd, env=env)

            if self._video_on:
                vout = os.path.join(video_dir, 'flight_video.mp4')
                self._vid_proc = subprocess.Popen(
                    [
                        'ros2', 'run', 'camera', 'video_recorder_node',
                        '--ros-args',
                        '-p', f'output_path:={vout}',
                        '-p', f'fps:={self._video_fps}',
                        '-p', f'chunk_duration_sec:={self._video_chunk}',
                        '-r', f'{self._video_topic}:={self._video_topic}',
                    ],
                    env=env,
                )

            if self._csv_on:
                self._csv_proc = subprocess.Popen(
                    [
                        'ros2', 'run', 'ardupilot_mavlink_bridge', 'telemetry_logger_node',
                        '--ros-args',
                        '-p', f'log_dir:={session_dir}',
                        '-p', f'log_rate_hz:={self._csv_hz}',
                    ],
                    env=env,
                )
        except Exception as exc:
            self.get_logger().error(
                f'Failed to fully start session {session_dir}: {exc}; '
                'rolling back partial subprocess spawns'
            )
            self._stop_session('start failure rollback')

    def _stop_session(self, reason: str) -> None:
        if self._session_dir is None:
            return
        session_dir = self._session_dir
        self.get_logger().info(f'{reason}: flushing {session_dir}')

        procs: Tuple[Tuple[Optional[subprocess.Popen], str], ...] = (
            (self._bag_proc, 'rosbag'),
            (self._csv_proc, 'telemetry_csv'),
            (self._vid_proc, 'video'),
        )

        # Phase 1: SIGINT every still-running child in parallel so they
        # can flush concurrently. Order doesn't matter at this stage.
        for proc, name in procs:
            if proc is None or proc.poll() is not None:
                continue
            try:
                proc.send_signal(signal.SIGINT)
            except Exception as exc:
                self.get_logger().warning(f'{name}: SIGINT failed: {exc}')

        # Phase 2: per-process deadline so a slow rosbag flush can't
        # steal the entire budget from the MP4 finalize step (the whole
        # point of the safety-gated mode is to never have a corrupt MP4
        # because of an unclean shutdown).
        for proc, name in procs:
            if proc is None:
                continue
            self._wait_or_kill(proc, name, _GRACEFUL_STOP_TIMEOUT_SEC)

        self._bag_proc = None
        self._vid_proc = None
        self._csv_proc = None

        # Snapshot localizer artefacts (map + anchor logs) into the
        # cycle dir AFTER recorders have stopped, so the cycle folder
        # is self-contained for post-flight analysis. Done after the
        # process teardown so the file we copy is the latest write.
        self._snapshot_localizer_artefacts(session_dir)

        self._session_dir = None

    def _wait_or_kill(
        self,
        proc: subprocess.Popen,
        name: str,
        timeout_sec: float,
    ) -> None:
        deadline = time.monotonic() + timeout_sec
        while proc.poll() is None and time.monotonic() < deadline:
            time.sleep(_STOP_POLL_INTERVAL_SEC)
        if proc.poll() is None:
            self.get_logger().warning(
                f'{name} did not exit in {timeout_sec:.0f}s; SIGKILL'
            )
            proc.kill()

    # -- helpers ------------------------------------------------------

    def _write_manifest(self, session_dir: str) -> None:
        manifest = {
            'created_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
            'mode': self._stack_mode,
            'navigation': self._stack_nav,
            'environment': self._stack_env,
            'rosbag_topics': list(self._topics),
            'video_enabled': self._video_on,
            'telemetry_csv_enabled': self._csv_on,
            'safety_gated_session': True,
        }
        marker_profile = self._load_marker_profile()
        if marker_profile:
            manifest['marker_detector_profile'] = marker_profile
        try:
            with open(
                os.path.join(session_dir, 'manifest.yaml'), 'w', encoding='utf-8'
            ) as f:
                yaml.safe_dump(manifest, f, sort_keys=False, allow_unicode=True)
        except OSError as exc:
            self.get_logger().warning(f'Failed to write manifest: {exc}')

    def _load_marker_profile(self) -> dict:
        path = self._markers_yaml_path
        if not path or not os.path.exists(path):
            return {}
        try:
            with open(path, 'r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f) or {}
            return {
                'dictionary': cfg.get('dictionary'),
                'default_marker_size': float(cfg.get('default_marker_size', 0.2)),
                'markers': cfg.get('markers', []),
            }
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to load markers profile from {path}: {exc}'
            )
            return {}

    def _snapshot_localizer_artefacts(self, session_dir: str) -> None:
        """Copy discovered_map.yaml and anchor_*.yaml from the localizer's
        runtime dir into the cycle folder.

        The tag_localizer is one process per launch; with safety gating
        it spans many cycles. Its ``persist_map_path`` is fixed at
        launch time to ``<launch_session_dir>/discovered_map.yaml`` and
        it rewrites that file on every newly confirmed marker. We
        snapshot the latest contents here so the cycle folder ships
        with the map state at safety-engaged time.

        Anchor is set ONCE (first detection per localizer lifetime) so
        the same anchor_<ts>.yaml is copied into every cycle - that's
        intentional: every cycle of the same flight refers to the same
        georeference.
        """
        src_dir = self._localizer_runtime_dir
        if not src_dir or not os.path.isdir(src_dir):
            return

        map_src = os.path.join(src_dir, 'discovered_map.yaml')
        if os.path.exists(map_src):
            try:
                shutil.copy2(map_src, os.path.join(session_dir, 'discovered_map.yaml'))
            except OSError as exc:
                self.get_logger().warning(
                    f'Failed to copy discovered_map.yaml: {exc}'
                )

        for anchor_src in glob.glob(os.path.join(src_dir, 'anchor_*.yaml')):
            try:
                shutil.copy2(
                    anchor_src,
                    os.path.join(session_dir, os.path.basename(anchor_src)),
                )
            except OSError as exc:
                self.get_logger().warning(
                    f'Failed to copy {os.path.basename(anchor_src)}: {exc}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = SafetyLogSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_session('shutdown')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
