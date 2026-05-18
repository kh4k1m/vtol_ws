#!/usr/bin/env python3
"""Persist a DPVO run to disk: TUM-format trajectory + GPS sidecar + PNG map.

Designed for the DPVO-only diagnostic launch (no autonomous mission, just
collect data and render a flyover map after the operator lands).

Outputs (paths configurable via parameters):

    <output_dir>/
        trajectory_tum.txt      - DPVO poses in TUM format
                                  (timestamp tx ty tz qx qy qz qw)
        dpvo_vs_gps.csv         - per-pose row joined with the closest
                                  GPS fix (lat/lon/alt + delta_sec)
        flight_map.png          - top-down 2D map rendered on shutdown
                                  (DPVO XY local + optional GPS XY ENU)

The PNG is rendered when the node is asked to stop (SIGINT / rclpy shutdown).
"""

from __future__ import annotations

import csv
import math
import os
import signal
import threading
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Deque, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

# matplotlib is imported lazily inside the render hook because it isn't
# strictly required when the operator only wants TUM+CSV outputs.


_EARTH_RADIUS_M = 6378137.0


def _stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _latlon_to_enu(lat_deg: float, lon_deg: float,
                   ref_lat_deg: float, ref_lon_deg: float) -> Tuple[float, float]:
    """Equirectangular ENU offset (meters) from (ref_lat, ref_lon)."""
    d_lat = math.radians(lat_deg - ref_lat_deg)
    d_lon = math.radians(lon_deg - ref_lon_deg)
    ref_lat_rad = math.radians(ref_lat_deg)
    north = d_lat * _EARTH_RADIUS_M
    east = d_lon * _EARTH_RADIUS_M * math.cos(ref_lat_rad)
    return east, north


class DPVOTrajectoryWriterNode(Node):
    def __init__(self) -> None:
        super().__init__("dpvo_trajectory_writer_node")

        self.declare_parameter(
            "pose_topic", "/vision/dpvo/pose",
            ParameterDescriptor(description="DPVO pose topic (PoseStamped)."),
        )
        self.declare_parameter(
            "gps_topic", "/ap/gps/fix",
            ParameterDescriptor(description="Optional GPS topic (NavSatFix)."),
        )
        self.declare_parameter(
            "output_dir", "",
            ParameterDescriptor(
                description="Directory for trajectory+csv+png. Empty -> ~/vtol_ws/logs/dpvo_<ts>.",
            ),
        )
        self.declare_parameter("max_gps_buffer", 5000)
        self.declare_parameter("max_time_delta_sec", 0.5)
        self.declare_parameter("flush_every_n", 20)
        self.declare_parameter(
            "render_png", True,
            ParameterDescriptor(description="Render top-down PNG map on shutdown."),
        )

        self.pose_topic = self.get_parameter("pose_topic").value
        self.gps_topic = self.get_parameter("gps_topic").value
        output_dir = self.get_parameter("output_dir").value
        self.max_gps_buffer = int(self.get_parameter("max_gps_buffer").value)
        self.max_time_delta_sec = float(self.get_parameter("max_time_delta_sec").value)
        self.flush_every_n = max(1, int(self.get_parameter("flush_every_n").value))
        self.render_png = bool(self.get_parameter("render_png").value)

        if not output_dir:
            workspace = os.environ.get(
                "VTOL_WS_ROOT", os.path.expanduser("~/vtol_ws")
            )
            ts = datetime.now().strftime("%Y%m%d-%H%M%S")
            output_dir = os.path.join(workspace, "logs", f"dpvo_{ts}")
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.tum_path = self.output_dir / "trajectory_tum.txt"
        self.csv_path = self.output_dir / "dpvo_vs_gps.csv"
        self.png_path = self.output_dir / "flight_map.png"

        # IO state
        self._render_lock = threading.Lock()
        self._rendered = False

        self.tum_file = self.tum_path.open("w", encoding="utf-8")
        self.tum_file.write(
            "# DPVO trajectory in TUM format. Columns: timestamp_sec tx ty tz qx qy qz qw\n"
        )

        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
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
        ])
        self.csv_file.flush()

        # In-memory trajectories for the final PNG render. We keep them
        # capped so a 30 min flight at 30 Hz doesn't blow up RAM.
        self.max_in_memory = 200_000
        self.dpvo_xy: List[Tuple[float, float, float]] = []   # (x, y, z)
        self.gps_xy: List[Tuple[float, float, float]] = []    # (east, north, alt)

        self.gps_buffer: Deque[Tuple[int, NavSatFix]] = deque(maxlen=self.max_gps_buffer)
        self.gps_ref: Optional[Tuple[float, float]] = None    # (lat, lon) anchor

        self.rows_since_flush = 0
        self.gps_messages_seen = 0
        self.pose_messages_seen = 0

        self.create_subscription(PoseStamped, self.pose_topic, self._pose_cb, 50)
        self.create_subscription(NavSatFix, self.gps_topic, self._gps_cb, 50)

        self.get_logger().info(
            f"Trajectory writer up. pose={self.pose_topic} gps={self.gps_topic} "
            f"out_dir={self.output_dir}"
        )

        # Run finalize() on SIGINT / SIGTERM even if rclpy.shutdown was
        # called from elsewhere (launch teardown).
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    # -- callbacks ------------------------------------------------------

    def _gps_cb(self, msg: NavSatFix) -> None:
        self.gps_messages_seen += 1
        # Filter "no fix" status (status == -1) so the GPS anchor doesn't
        # snap to garbage.
        if msg.status.status < 0:
            return
        if self.gps_ref is None:
            self.gps_ref = (float(msg.latitude), float(msg.longitude))
            self.get_logger().info(
                f"GPS reference anchored at lat={msg.latitude:.7f} lon={msg.longitude:.7f}"
            )
        ns = _stamp_to_ns(msg.header.stamp)
        self.gps_buffer.append((ns, msg))

        if self.gps_ref is not None and len(self.gps_xy) < self.max_in_memory:
            east, north = _latlon_to_enu(
                msg.latitude, msg.longitude, self.gps_ref[0], self.gps_ref[1]
            )
            self.gps_xy.append((east, north, float(msg.altitude)))

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.pose_messages_seen += 1
        pose_ns = _stamp_to_ns(msg.header.stamp)
        pose_sec = pose_ns / 1e9

        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)
        qx = float(msg.pose.orientation.x)
        qy = float(msg.pose.orientation.y)
        qz = float(msg.pose.orientation.z)
        qw = float(msg.pose.orientation.w)

        # TUM line
        self.tum_file.write(
            f"{pose_sec:.6f} {x:.6f} {y:.6f} {z:.6f} "
            f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n"
        )

        if len(self.dpvo_xy) < self.max_in_memory:
            self.dpvo_xy.append((x, y, z))

        # Match with GPS
        gps_match = self._find_closest_gps(pose_ns)
        if gps_match is not None:
            gps_ns, gps_msg = gps_match
            delta = abs(pose_ns - gps_ns) / 1e9
            if delta <= self.max_time_delta_sec:
                self.csv_writer.writerow([
                    pose_ns,
                    gps_ns,
                    f"{delta:.6f}",
                    int(gps_msg.status.status),
                    f"{gps_msg.latitude:.9f}",
                    f"{gps_msg.longitude:.9f}",
                    f"{gps_msg.altitude:.4f}",
                    f"{x:.6f}",
                    f"{y:.6f}",
                    f"{z:.6f}",
                    f"{qx:.6f}",
                    f"{qy:.6f}",
                    f"{qz:.6f}",
                    f"{qw:.6f}",
                ])
                self.rows_since_flush += 1

        if self.rows_since_flush >= self.flush_every_n:
            self.tum_file.flush()
            self.csv_file.flush()
            self.rows_since_flush = 0

    def _find_closest_gps(self, pose_ns: int) -> Optional[Tuple[int, NavSatFix]]:
        if not self.gps_buffer:
            return None
        best = None
        best_delta = None
        for gps_ns, gps_msg in self.gps_buffer:
            delta = abs(pose_ns - gps_ns)
            if best_delta is None or delta < best_delta:
                best_delta = delta
                best = (gps_ns, gps_msg)
        return best

    # -- finalize -------------------------------------------------------

    def _signal_handler(self, signum, _frame) -> None:
        self.get_logger().info(f"Received signal {signum}, finalizing outputs...")
        self.finalize()
        # Re-raise default behavior so rclpy.spin() exits cleanly.
        rclpy.try_shutdown()

    def finalize(self) -> None:
        with self._render_lock:
            if self._rendered:
                return
            self._rendered = True

            try:
                self.tum_file.flush()
                self.tum_file.close()
            except Exception:
                pass
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception:
                pass

            self.get_logger().info(
                f"Flushed {self.pose_messages_seen} DPVO poses and "
                f"{self.gps_messages_seen} GPS fixes to {self.output_dir}"
            )

            if self.render_png:
                self._render_flight_map()

    def _render_flight_map(self) -> None:
        if not self.dpvo_xy and not self.gps_xy:
            self.get_logger().warn(
                "Nothing to render - no DPVO poses and no GPS fixes received."
            )
            return
        try:
            import matplotlib
            matplotlib.use("Agg")  # headless backend
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().error(
                "matplotlib is not installed in this Python env - skipping PNG render. "
                "Install with: pip install matplotlib"
            )
            return

        fig, axes = plt.subplots(1, 2, figsize=(14, 7))

        ax_xy = axes[0]
        if self.dpvo_xy:
            xs, ys, _ = zip(*self.dpvo_xy)
            ax_xy.plot(xs, ys, "-", color="tab:blue", linewidth=1.2, label="DPVO (local)")
            ax_xy.scatter(
                [xs[0]], [ys[0]], color="tab:green", s=80, zorder=5, label="DPVO start"
            )
            ax_xy.scatter(
                [xs[-1]], [ys[-1]], color="tab:red", s=80, marker="X",
                zorder=5, label="DPVO end",
            )
        if self.gps_xy:
            gxs, gys, _ = zip(*self.gps_xy)
            ax_xy.plot(
                gxs, gys, "--", color="tab:orange", linewidth=1.2,
                label=(
                    f"GPS (ENU @ "
                    f"{self.gps_ref[0]:.5f},{self.gps_ref[1]:.5f})"
                    if self.gps_ref else "GPS (ENU)"
                ),
            )
        ax_xy.set_xlabel("X / East [m]")
        ax_xy.set_ylabel("Y / North [m]")
        ax_xy.set_title("Top-down flight map")
        ax_xy.set_aspect("equal", adjustable="datalim")
        ax_xy.grid(True, linestyle=":", alpha=0.5)
        ax_xy.legend(loc="best", fontsize=9)

        ax_z = axes[1]
        if self.dpvo_xy:
            zs = [p[2] for p in self.dpvo_xy]
            ax_z.plot(range(len(zs)), zs, color="tab:blue", label="DPVO z (local)")
        if self.gps_xy:
            alts = [p[2] for p in self.gps_xy]
            ax_z.plot(range(len(alts)), alts, color="tab:orange", label="GPS altitude")
        ax_z.set_xlabel("Sample index")
        ax_z.set_ylabel("Z / altitude [m]")
        ax_z.set_title("Altitude over time")
        ax_z.grid(True, linestyle=":", alpha=0.5)
        ax_z.legend(loc="best", fontsize=9)

        fig.suptitle(f"DPVO session - {self.output_dir.name}", fontsize=12)
        fig.tight_layout()
        fig.savefig(self.png_path, dpi=130)
        plt.close(fig)

        self.get_logger().info(f"Wrote {self.png_path}")

    def destroy_node(self) -> bool:
        try:
            self.finalize()
        finally:
            return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = DPVOTrajectoryWriterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
