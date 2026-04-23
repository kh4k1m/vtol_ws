#!/usr/bin/env python3
"""Write GPS and DPVO predictions into a CSV file."""

from __future__ import annotations

import csv
from collections import deque
from pathlib import Path
from typing import Deque, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class TrajectoryLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_logger_node")

        self.declare_parameter(
            "gps_topic",
            "/mavros/global_position/raw/fix",
            ParameterDescriptor(
                description="NavSatFix (у нас с mavlink_bridge, не /gps/fix).",
            ),
        )
        self.declare_parameter(
            "pose_topic",
            "/vision_pose_enu",
            ParameterDescriptor(description="PoseStamped от DPVO (как в flight.launch)."),
        )
        self.declare_parameter("output_csv", "/tmp/dpvo_vs_gps.csv")
        self.declare_parameter("max_gps_buffer", 5000)
        self.declare_parameter("max_time_delta_sec", 0.25)
        self.declare_parameter("flush_every_n", 20)

        self.gps_topic = self.get_parameter("gps_topic").get_parameter_value().string_value
        self.pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        self.output_csv = self.get_parameter("output_csv").get_parameter_value().string_value
        self.max_gps_buffer = self.get_parameter("max_gps_buffer").get_parameter_value().integer_value
        self.max_time_delta_sec = self.get_parameter("max_time_delta_sec").get_parameter_value().double_value
        self.flush_every_n = max(1, self.get_parameter("flush_every_n").get_parameter_value().integer_value)

        self.gps_buffer: Deque[Tuple[int, NavSatFix]] = deque(maxlen=int(self.max_gps_buffer))
        self.rows_since_flush = 0

        output_path = Path(self.output_csv)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = output_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            [
                "pose_stamp_ns",
                "gps_stamp_ns",
                "gps_time_delta_sec",
                "gps_status",
                "gps_lat_deg",
                "gps_lon_deg",
                "gps_alt_m",
                "dpvo_x_m",
                "dpvo_y_m",
                "dpvo_z_m",
                "dpvo_qx",
                "dpvo_qy",
                "dpvo_qz",
                "dpvo_qw",
            ]
        )
        self.csv_file.flush()

        self.create_subscription(NavSatFix, self.gps_topic, self._gps_cb, 50)
        self.create_subscription(PoseStamped, self.pose_topic, self._pose_cb, 50)

        self.get_logger().info(
            f"Trajectory logger is up. gps_topic={self.gps_topic}, pose_topic={self.pose_topic}, output={self.output_csv}"
        )

    def _gps_cb(self, msg: NavSatFix) -> None:
        self.gps_buffer.append((stamp_to_ns(msg.header.stamp), msg))

    def _pose_cb(self, msg: PoseStamped) -> None:
        pose_ns = stamp_to_ns(msg.header.stamp)
        match = self._find_closest_gps(pose_ns)
        if match is None:
            return

        gps_ns, gps_msg = match
        delta_sec = abs(pose_ns - gps_ns) / 1e9
        if delta_sec > self.max_time_delta_sec:
            return

        self.writer.writerow(
            [
                pose_ns,
                gps_ns,
                f"{delta_sec:.6f}",
                int(gps_msg.status.status),
                f"{gps_msg.latitude:.9f}",
                f"{gps_msg.longitude:.9f}",
                f"{gps_msg.altitude:.4f}",
                f"{msg.pose.position.x:.6f}",
                f"{msg.pose.position.y:.6f}",
                f"{msg.pose.position.z:.6f}",
                f"{msg.pose.orientation.x:.6f}",
                f"{msg.pose.orientation.y:.6f}",
                f"{msg.pose.orientation.z:.6f}",
                f"{msg.pose.orientation.w:.6f}",
            ]
        )
        self.rows_since_flush += 1
        if self.rows_since_flush >= self.flush_every_n:
            self.csv_file.flush()
            self.rows_since_flush = 0

    def _find_closest_gps(self, pose_ns: int) -> Optional[Tuple[int, NavSatFix]]:
        if not self.gps_buffer:
            return None

        best_ns = None
        best_msg = None
        best_delta = None
        for gps_ns, gps_msg in self.gps_buffer:
            delta = abs(pose_ns - gps_ns)
            if best_delta is None or delta < best_delta:
                best_delta = delta
                best_ns = gps_ns
                best_msg = gps_msg

        if best_ns is None or best_msg is None:
            return None
        return best_ns, best_msg

    def destroy_node(self) -> bool:
        try:
            self.csv_file.flush()
            self.csv_file.close()
        finally:
            return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = TrajectoryLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
