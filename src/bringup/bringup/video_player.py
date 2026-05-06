"""Video player node для воспроизведения записанного полёта.

Поддерживает два режима:

1. ``replay`` (рекомендуется для прогонки DPVO/ORB-SLAM3 по логам).
   Читает MP4 чанки + сидекар CSV ``<basename>_timestamps.csv`` от
   ``video_recorder_node``. Каждому опубликованному кадру выставляется его
   оригинальный ``header.stamp`` (тот, с которым он лежит в исходном
   камера-сообщении). Если ``use_sim_time=True``, нод ждёт пока
   ``rclpy`` clock (он же ``/clock`` от ``ros2 bag play``) дойдёт до
   timestamp'а кадра. Так видео синхронизируется с bag по оригинальному
   времени записи, и навигация видит ту же связку IMU↔Image, что в полёте.

2. ``loop`` (legacy / data collection). Просто крутит MP4 по кругу с
   фиксированным FPS, ставя ``now()``. Подходит для жирных кусков
   raw-видео без таймстемпов (``data/C0026/C0026.MP4``).

Параметры:

* ``session_dir`` — путь до session_*/, удобный shortcut. Если задан, ищем
  ``<session_dir>/video/*.mp4`` и сидекар CSV.
* ``video_dir`` — альтернатива: явная папка с MP4.
* ``video_path`` — единичный MP4 (legacy). При задании остальные игнорятся.
* ``timestamps_csv`` — путь к CSV. Если пусто, ищется автоматически.
* ``mode`` — ``replay`` | ``loop`` (default ``replay`` если есть CSV,
  иначе ``loop``).
* ``loop`` — bool, перезапускать ли последовательность в режиме ``loop``.
* ``fps`` — fallback FPS если нет CSV.
* ``playback_rate`` — множитель скорости в режиме ``loop`` (sim_time не
  затрагивает, для replay используется bag --rate).
* ``image_topic`` — куда публиковать (default ``/camera/image_raw``).
* ``frame_id`` — header.frame_id (default ``camera_optical_frame``).
"""

from __future__ import annotations

import csv
import glob
import os
from dataclasses import dataclass
from typing import List, Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image


@dataclass
class _FrameRecord:
    frame_index: int
    chunk_index: int
    stamp_sec: int
    stamp_nanosec: int

    @property
    def stamp_ns(self) -> int:
        return self.stamp_sec * 1_000_000_000 + self.stamp_nanosec


def _read_timestamps_csv(csv_path: str) -> List[_FrameRecord]:
    records: List[_FrameRecord] = []
    with open(csv_path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                records.append(_FrameRecord(
                    frame_index=int(row['frame_index']),
                    chunk_index=int(row['chunk_index']),
                    stamp_sec=int(row['ros_time_sec']),
                    stamp_nanosec=int(row['ros_time_nanosec']),
                ))
            except (KeyError, ValueError):
                continue
    records.sort(key=lambda r: r.frame_index)
    return records


def _discover_chunks(video_dir: str, basename_hint: Optional[str]) -> List[str]:
    """Find all MP4 chunks in ``video_dir`` and sort by chunk index."""
    if basename_hint:
        pattern = os.path.join(video_dir, f'{basename_hint}*.mp4')
    else:
        pattern = os.path.join(video_dir, '*.mp4')
    files = sorted(glob.glob(pattern))
    return files


class VideoPlayerNode(Node):
    def __init__(self):
        super().__init__('video_player_node')

        self.declare_parameter('session_dir', '')
        self.declare_parameter('video_dir', '')
        self.declare_parameter('video_path', '')
        self.declare_parameter('timestamps_csv', '')
        self.declare_parameter('mode', 'auto')
        self.declare_parameter('loop', True)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('playback_rate', 1.0)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('frame_id', 'camera_optical_frame')

        session_dir = str(self.get_parameter('session_dir').value or '')
        video_dir = str(self.get_parameter('video_dir').value or '')
        video_path = str(self.get_parameter('video_path').value or '')
        timestamps_csv = str(self.get_parameter('timestamps_csv').value or '')
        requested_mode = str(self.get_parameter('mode').value or 'auto').lower()
        self._loop = bool(self.get_parameter('loop').value)
        self._fps = float(self.get_parameter('fps').value)
        self._playback_rate = max(1e-3, float(self.get_parameter('playback_rate').value))
        image_topic = str(self.get_parameter('image_topic').value)
        self._frame_id = str(self.get_parameter('frame_id').value)

        # Resolve files.
        self._video_files: List[str] = []
        self._records: List[_FrameRecord] = []

        if video_path:
            if not os.path.exists(video_path):
                self.get_logger().error(f"video_path does not exist: {video_path}")
                return
            self._video_files = [video_path]
        else:
            if session_dir and not video_dir:
                video_dir = os.path.join(session_dir, 'video')
            if not video_dir:
                self.get_logger().error(
                    "Specify one of: video_path / video_dir / session_dir"
                )
                return
            if not os.path.isdir(video_dir):
                self.get_logger().error(f"video_dir does not exist: {video_dir}")
                return

            # Try to auto-detect basename via CSV name
            basename_hint = None
            if not timestamps_csv:
                csv_candidates = sorted(
                    glob.glob(os.path.join(video_dir, '*_timestamps.csv'))
                )
                if csv_candidates:
                    timestamps_csv = csv_candidates[0]
            if timestamps_csv and os.path.exists(timestamps_csv):
                csv_name = os.path.basename(timestamps_csv)
                basename_hint = csv_name[: -len('_timestamps.csv')] \
                    if csv_name.endswith('_timestamps.csv') else None

            self._video_files = _discover_chunks(video_dir, basename_hint)

        if not self._video_files:
            self.get_logger().error("No MP4 files found")
            return

        # Resolve mode.
        if timestamps_csv and os.path.exists(timestamps_csv):
            self._records = _read_timestamps_csv(timestamps_csv)
            if not self._records:
                self.get_logger().warn(
                    f"timestamps CSV {timestamps_csv} is empty; falling back to loop mode"
                )

        if requested_mode == 'auto':
            self._mode = 'replay' if self._records else 'loop'
        else:
            self._mode = requested_mode
            if self._mode == 'replay' and not self._records:
                self.get_logger().warn(
                    "Mode='replay' requested but no timestamps CSV; switching to 'loop'"
                )
                self._mode = 'loop'

        self._bridge = CvBridge()
        self._publisher = self.create_publisher(Image, image_topic, 10)

        self._chunk_caps = {}
        self._current_chunk = None
        self._current_cap: Optional[cv2.VideoCapture] = None
        self._next_frame_global = 0
        self._record_idx = 0

        # First chunk preload
        self._open_chunk(0)

        if self._mode == 'replay':
            # Tick at 200 Hz; we publish whichever frames are due.
            self._timer = self.create_timer(1.0 / 200.0, self._replay_tick)
            anchor_ns = self._records[0].stamp_ns
            self.get_logger().info(
                f"video_player: REPLAY mode, {len(self._records)} frames across "
                f"{len(self._video_files)} chunks, first stamp = "
                f"{anchor_ns / 1e9:.3f}s, use_sim_time="
                f"{self.get_parameter('use_sim_time').value}"
            )
        else:
            period = 1.0 / max(1.0, self._fps * self._playback_rate)
            self._timer = self.create_timer(period, self._loop_tick)
            self.get_logger().info(
                f"video_player: LOOP mode at {self._fps:.1f} fps "
                f"x{self._playback_rate:.2f}, files={len(self._video_files)}"
            )

    # -------------------- chunk helpers --------------------

    def _open_chunk(self, chunk_idx: int) -> bool:
        if chunk_idx < 0 or chunk_idx >= len(self._video_files):
            return False
        if self._current_cap is not None:
            self._current_cap.release()
        path = self._video_files[chunk_idx]
        self._current_cap = cv2.VideoCapture(path)
        if not self._current_cap.isOpened():
            self.get_logger().error(f"Could not open chunk: {path}")
            self._current_cap = None
            return False
        self._current_chunk = chunk_idx
        return True

    # -------------------- replay mode --------------------

    def _replay_tick(self) -> None:
        if self._record_idx >= len(self._records):
            return
        now_ns = self.get_clock().now().nanoseconds
        # Publish all due frames in this tick (catches up if we lagged).
        while self._record_idx < len(self._records):
            rec = self._records[self._record_idx]
            if rec.stamp_ns > now_ns:
                # In sim_time mode (use_sim_time=True) clock comes from /clock
                # and may be far behind wall clock - we just wait. In default
                # mode, all stamps are in the past (recording is older than
                # now), so the loop will publish them as fast as possible.
                break
            self._publish_record(rec)
            self._record_idx += 1
        if self._record_idx >= len(self._records):
            self.get_logger().info("video_player: end of replay")

    def _publish_record(self, rec: _FrameRecord) -> None:
        if rec.chunk_index != self._current_chunk:
            if not self._open_chunk(rec.chunk_index):
                return
            # Reset to start of new chunk; frame index inside chunk follows.
            # Note: cv2.VideoCapture is sequential, so we read in order.

        ret, frame = self._current_cap.read()
        if not ret or frame is None:
            self.get_logger().warn(
                f"failed to read frame {rec.frame_index} from chunk {rec.chunk_index}"
            )
            return

        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = Time(
            seconds=rec.stamp_sec,
            nanoseconds=rec.stamp_nanosec,
        ).to_msg()
        msg.header.frame_id = self._frame_id
        self._publisher.publish(msg)

    # -------------------- loop mode --------------------

    def _loop_tick(self) -> None:
        if self._current_cap is None:
            return
        ret, frame = self._current_cap.read()
        if not ret or frame is None:
            next_chunk = (self._current_chunk or 0) + 1
            if next_chunk >= len(self._video_files):
                if not self._loop:
                    self.get_logger().info("video_player: end of sequence")
                    self._timer.cancel()
                    return
                next_chunk = 0
            if not self._open_chunk(next_chunk):
                return
            ret, frame = self._current_cap.read()
            if not ret or frame is None:
                return

        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VideoPlayerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
