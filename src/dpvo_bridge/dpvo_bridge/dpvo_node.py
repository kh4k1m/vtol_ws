#!/usr/bin/env python3
"""ROS 2 node wrapping DPVO as a proper ROS 2 system.

Topic contract (vision_interfaces standard - see
src/vision_interfaces/README.md):

* /vision/dpvo/pose    (geometry_msgs/PoseStamped)         - latest pose, local frame
* /vision/dpvo/status  (vision_interfaces/VisionStatus)    - health + uncertainty
* /vision/dpvo/path    (nav_msgs/Path)                     - rolling window for viz
* /vision/dpvo/reset   (vision_interfaces/ResetVision)     - drop + re-init

Legacy topics published in parallel for backwards compatibility with the
existing flight.launch.py (uses marker_interfaces/NavStatus):

* /dpvo/pose           (geometry_msgs/PoseStamped)
* /dpvo/nav_status     (marker_interfaces/NavStatus)
* /dpvo/path           (nav_msgs/Path)

Deployment note
---------------
DPVO ships with custom CUDA extensions and is pinned to pytorch 2.3.1 +
cuda 12.1 (see ~/DPVO/environment.yml). That stack is not compatible with
the system Python that ROS 2 humble uses out of the box, so this node is
intended to run from a conda env that has BOTH DPVO and rclpy installed.

See scripts/run_dpvo_node.sh + dpvo_bridge/README.md for the recommended
setup (conda env `dpvo` + RoboStack rclpy + workspace overlay sourced).
"""

from __future__ import annotations

import os
import sys
import threading
import traceback
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

# vision_interfaces is the SOURCE OF TRUTH for the visual navigation
# topic contract. NavStatus is kept only for backwards compatibility
# with mission_manager / single_source_router which haven't migrated yet.
from vision_interfaces.msg import VisionStatus
from vision_interfaces.srv import ResetVision

try:
    from marker_interfaces.msg import NavStatus  # legacy mirror topic
    _HAVE_LEGACY_NAV_STATUS = True
except ImportError:
    _HAVE_LEGACY_NAV_STATUS = False


def _add_dpvo_path_from_env() -> None:
    """Allow running without `pip install .` from inside ~/DPVO."""
    extra = os.environ.get("DPVO_PYTHON_PATH", "").strip()
    if extra and extra not in sys.path:
        sys.path.insert(0, extra)


def _image_msg_to_bgr(msg: Image) -> np.ndarray:
    """Minimal sensor_msgs/Image -> BGR8 ndarray converter.

    We avoid the cv_bridge dependency because cv_bridge requires a
    matching libopencv build that is painful to install inside a conda
    env. The supported encodings cover every camera we actually ship
    (sim + USB + RTSP).
    """
    enc = msg.encoding
    height, width, step = int(msg.height), int(msg.width), int(msg.step)
    if height <= 0 or width <= 0 or step <= 0:
        raise ValueError("Image has zero/invalid geometry")
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc == "bgr8":
        return buf.reshape(height, step)[:, : width * 3].reshape(height, width, 3).copy()
    if enc == "rgb8":
        rgb = buf.reshape(height, step)[:, : width * 3].reshape(height, width, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if enc == "mono8":
        mono = buf.reshape(height, step)[:, :width].reshape(height, width)
        return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
    if enc == "bgra8":
        bgra = buf.reshape(height, step)[:, : width * 4].reshape(height, width, 4)
        return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
    if enc == "rgba8":
        rgba = buf.reshape(height, step)[:, : width * 4].reshape(height, width, 4)
        return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
    raise ValueError(f"Unsupported Image encoding: {enc!r}")


class DPVONode(Node):
    def __init__(self) -> None:
        super().__init__("dpvo_node")
        _add_dpvo_path_from_env()

        # --- topic names (vision_interfaces standard + legacy mirror) ---
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("vision_pose_topic", "/vision/dpvo/pose")
        self.declare_parameter("vision_status_topic", "/vision/dpvo/status")
        self.declare_parameter("vision_path_topic", "/vision/dpvo/path")
        self.declare_parameter("vision_reset_service", "/vision/dpvo/reset")
        # Legacy mirror. Empty = disable legacy publishing.
        self.declare_parameter("legacy_pose_topic", "/dpvo/pose")
        self.declare_parameter("legacy_nav_status_topic", "/dpvo/nav_status")
        self.declare_parameter("legacy_path_topic", "/dpvo/path")

        self.declare_parameter("frame_id", "dpvo_local")
        self.declare_parameter(
            "source_label", "dpvo",
            ParameterDescriptor(description="Goes into VisionStatus.source."),
        )

        # --- noise + path tuning ---
        self.declare_parameter("position_std_m", 0.20)
        self.declare_parameter("rotation_std_rad", 0.10)
        self.declare_parameter("velocity_std_m_s", 0.50)
        self.declare_parameter("publish_path", True)
        self.declare_parameter("max_path_length", 2000)
        self.declare_parameter("process_every_n", 1)
        self.declare_parameter("status_publish_period_sec", 0.2)
        self.declare_parameter("log_every_n", 30)

        # --- DPVO config ---
        self.declare_parameter("weights_path", "")
        self.declare_parameter("config_path", "")
        self.declare_parameter("config_overrides", [])
        self.declare_parameter("use_cuda", True)
        self.declare_parameter("undistort", True)
        self.declare_parameter("enable_viz", False)

        # ---- pull params into locals ----
        gp = self.get_parameter
        self.image_topic = gp("image_topic").value
        self.camera_info_topic = gp("camera_info_topic").value
        self.vision_pose_topic = gp("vision_pose_topic").value
        self.vision_status_topic = gp("vision_status_topic").value
        self.vision_path_topic = gp("vision_path_topic").value
        self.vision_reset_service = gp("vision_reset_service").value
        self.legacy_pose_topic = gp("legacy_pose_topic").value
        self.legacy_nav_status_topic = gp("legacy_nav_status_topic").value
        self.legacy_path_topic = gp("legacy_path_topic").value
        self.frame_id = gp("frame_id").value
        self.source_label = gp("source_label").value
        self.position_std_m = float(gp("position_std_m").value)
        self.rotation_std_rad = float(gp("rotation_std_rad").value)
        self.velocity_std_m_s = float(gp("velocity_std_m_s").value)
        self.publish_path = bool(gp("publish_path").value)
        self.max_path_length = int(gp("max_path_length").value)
        self.process_every_n = max(1, int(gp("process_every_n").value))
        self.status_publish_period_sec = float(gp("status_publish_period_sec").value)
        self.log_every_n = max(1, int(gp("log_every_n").value))
        self.weights_path = gp("weights_path").value
        self.config_path = gp("config_path").value
        self.config_overrides = list(gp("config_overrides").value)
        self.use_cuda = bool(gp("use_cuda").value)
        self.undistort = bool(gp("undistort").value)
        self.enable_viz = bool(gp("enable_viz").value)

        # ---- publishers ----
        self.vision_pose_pub = self.create_publisher(PoseStamped, self.vision_pose_topic, 10)
        self.vision_status_pub = self.create_publisher(VisionStatus, self.vision_status_topic, 10)
        if self.publish_path:
            self.vision_path_pub = self.create_publisher(PathMsg, self.vision_path_topic, 10)
        else:
            self.vision_path_pub = None

        self.legacy_pose_pub = (
            self.create_publisher(PoseStamped, self.legacy_pose_topic, 10)
            if self.legacy_pose_topic else None
        )
        self.legacy_path_pub = (
            self.create_publisher(PathMsg, self.legacy_path_topic, 10)
            if self.legacy_path_topic and self.publish_path else None
        )
        self.legacy_nav_status_pub = None
        if self.legacy_nav_status_topic and _HAVE_LEGACY_NAV_STATUS:
            self.legacy_nav_status_pub = self.create_publisher(
                NavStatus, self.legacy_nav_status_topic, 10
            )
        elif self.legacy_nav_status_topic and not _HAVE_LEGACY_NAV_STATUS:
            self.get_logger().warn(
                "marker_interfaces is not importable - legacy NavStatus mirror is OFF. "
                "single_source_router_node will not see /dpvo/nav_status."
            )

        # ---- path buffer + state ----
        self.path_msg = PathMsg()
        self.path_msg.header.frame_id = self.frame_id

        # camera intrinsics state
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.received_camera_info = False

        # lazy DPVO state
        self.device: Optional[str] = None
        self.torch = None
        self.lietorch = None
        self.dpvo_cls = None
        self.cfg = None
        self.slam = None

        # frame book-keeping
        self.frame_counter = 0
        self.processed_counter = 0
        self.last_pose_publish_time: Optional[float] = None
        self.last_pose_vec: Optional[np.ndarray] = None
        self.last_position: Optional[np.ndarray] = None
        self.state: int = VisionStatus.STATE_NO_DATA
        self.detail: str = "waiting for camera_info"

        self.processing_lock = threading.Lock()
        self.processing = False

        # ---- subscriptions ----
        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(Image, self.image_topic, self._image_cb, 10)

        # ---- service ----
        self.create_service(ResetVision, self.vision_reset_service, self._reset_cb)

        # ---- periodic VisionStatus heartbeat (so /nav/status doesn't go silent
        # while DPVO is still bootstrapping or waiting for camera_info) ----
        self.create_timer(self.status_publish_period_sec, self._status_heartbeat)

        self.get_logger().info(
            f"DPVO node is up. image_topic={self.image_topic} "
            f"vision_pose_topic={self.vision_pose_topic} "
            f"legacy_pose_topic={self.legacy_pose_topic or '<off>'} "
            f"weights_path={self.weights_path or '<unset>'}"
        )

    # -- lazy imports ---------------------------------------------------

    def _lazy_import_dpvo(self) -> None:
        if self.dpvo_cls is not None:
            return
        try:
            import torch  # type: ignore
            import lietorch  # type: ignore
            from dpvo.config import cfg as base_cfg  # type: ignore
            from dpvo.dpvo import DPVO  # type: ignore
        except Exception as exc:
            raise RuntimeError(
                "Could not import DPVO. Either install DPVO into this Python env "
                "(see ~/DPVO/README.md), or run dpvo_node from a conda env that has "
                "DPVO + rclpy (see src/dpvo_bridge/README.md)."
            ) from exc

        if self.use_cuda and not torch.cuda.is_available():
            raise RuntimeError("use_cuda=true but torch.cuda.is_available() is False.")

        self.torch = torch
        self.lietorch = lietorch
        self.dpvo_cls = DPVO
        self.cfg = base_cfg.clone()
        self.device = "cuda" if self.use_cuda else "cpu"

        if self.config_path:
            p = Path(self.config_path)
            if not p.exists():
                raise FileNotFoundError(f"config_path does not exist: {p}")
            self.cfg.merge_from_file(str(p))
        if self.config_overrides:
            self.cfg.merge_from_list(self.config_overrides)

        if not self.weights_path:
            raise RuntimeError("weights_path parameter is empty.")
        if not Path(self.weights_path).exists():
            raise FileNotFoundError(f"weights_path does not exist: {self.weights_path}")

    # -- callbacks ------------------------------------------------------

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if not self.received_camera_info:
            self.received_camera_info = True
            self._set_state(
                VisionStatus.STATE_INITIALIZING,
                "got camera_info, waiting for first frame",
            )
        self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        self.dist_coeffs = (
            np.array(msg.d, dtype=np.float32) if msg.d else np.zeros((0,), dtype=np.float32)
        )

    def _image_cb(self, msg: Image) -> None:
        self.frame_counter += 1
        if not self.received_camera_info:
            if self.frame_counter % self.log_every_n == 0:
                self.get_logger().warn(
                    f"Waiting for {self.camera_info_topic} before processing images."
                )
            return

        if (self.frame_counter - 1) % self.process_every_n != 0:
            return

        if self.processing:
            return

        with self.processing_lock:
            if self.processing:
                return
            self.processing = True

        try:
            self._process_frame(msg)
        except Exception as exc:
            self.get_logger().error(
                f"DPVO processing failed: {exc}\n{traceback.format_exc()}"
            )
            self._set_state(VisionStatus.STATE_LOST, f"processing error: {exc}")
        finally:
            self.processing = False

    def _reset_cb(
        self,
        request: ResetVision.Request,
        response: ResetVision.Response,
    ) -> ResetVision.Response:
        try:
            self.get_logger().warn(
                "ResetVision requested - dropping current DPVO state."
            )
            self.slam = None
            self.processed_counter = 0
            self.last_pose_vec = None
            self.last_position = None
            self.path_msg = PathMsg()
            self.path_msg.header.frame_id = self.frame_id
            self._set_state(
                VisionStatus.STATE_INITIALIZING,
                "reset() requested by service",
            )
            response.ok = True
            response.detail = "DPVO state dropped"
        except Exception as exc:
            response.ok = False
            response.detail = f"reset failed: {exc}"
        return response

    # -- core processing -----------------------------------------------

    def _process_frame(self, msg: Image) -> None:
        self._lazy_import_dpvo()

        image_bgr = _image_msg_to_bgr(msg)
        image_bgr = np.ascontiguousarray(image_bgr)

        intrinsics = self._build_intrinsics(image_bgr.shape)
        image_bgr, intrinsics = self._preprocess_image(image_bgr, intrinsics)

        image_tensor = self.torch.from_numpy(image_bgr).permute(2, 0, 1).to(self.device)
        intrinsics_tensor = self.torch.from_numpy(intrinsics).to(self.device)

        if self.slam is None:
            _, height, width = image_tensor.shape
            self.slam = self.dpvo_cls(
                self.cfg, self.weights_path,
                ht=int(height), wd=int(width), viz=self.enable_viz,
            )
            self.get_logger().info(
                f"DPVO initialized. image_size={width}x{height} frame_id={self.frame_id}"
            )
            self._set_state(VisionStatus.STATE_SETTLING, "DPVO instantiated")

        frame_idx = self.processed_counter
        self.slam(frame_idx, image_tensor, intrinsics_tensor)
        pose_vec = self._extract_pose_vector(frame_idx)
        self.processed_counter += 1

        now = self.get_clock().now()
        self.last_pose_publish_time = now.nanoseconds * 1e-9
        prev_position = self.last_position
        self.last_pose_vec = pose_vec
        self.last_position = pose_vec[:3].copy()

        # promote SETTLING -> TRACKING after a few processed frames so
        # mission_manager can ARM. Adjust threshold once we collect real
        # SETTLING / divergence statistics.
        if self.processed_counter > 10 and self.state in (
            VisionStatus.STATE_INITIALIZING, VisionStatus.STATE_SETTLING,
        ):
            self._set_state(VisionStatus.STATE_TRACKING, "tracking")

        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = float(pose_vec[0])
        pose_msg.pose.position.y = float(pose_vec[1])
        pose_msg.pose.position.z = float(pose_vec[2])
        pose_msg.pose.orientation.x = float(pose_vec[3])
        pose_msg.pose.orientation.y = float(pose_vec[4])
        pose_msg.pose.orientation.z = float(pose_vec[5])
        pose_msg.pose.orientation.w = float(pose_vec[6])

        self.vision_pose_pub.publish(pose_msg)
        if self.legacy_pose_pub is not None:
            self.legacy_pose_pub.publish(pose_msg)

        if self.publish_path:
            self.path_msg.header.stamp = msg.header.stamp
            self.path_msg.poses.append(pose_msg)
            if len(self.path_msg.poses) > self.max_path_length:
                self.path_msg.poses = self.path_msg.poses[-self.max_path_length:]
            if self.vision_path_pub is not None:
                self.vision_path_pub.publish(self.path_msg)
            if self.legacy_path_pub is not None:
                self.legacy_path_pub.publish(self.path_msg)

        # publish a fresh status on every frame so mission_manager always
        # has a stamp <= one frame old.
        self._publish_status(msg.header.stamp, prev_position=prev_position)

        if self.processed_counter % self.log_every_n == 0:
            self.get_logger().info(
                "DPVO frame=%d xyz=(%.3f, %.3f, %.3f)"
                % (self.processed_counter, pose_vec[0], pose_vec[1], pose_vec[2])
            )

    # -- VisionStatus publishing ---------------------------------------

    def _set_state(self, state: int, detail: str) -> None:
        if state != self.state:
            self.get_logger().info(f"DPVO state -> {self._state_name(state)} ({detail})")
        self.state = state
        self.detail = detail

    @staticmethod
    def _state_name(state: int) -> str:
        names = {
            VisionStatus.STATE_UNKNOWN: "UNKNOWN",
            VisionStatus.STATE_NO_DATA: "NO_DATA",
            VisionStatus.STATE_INITIALIZING: "INITIALIZING",
            VisionStatus.STATE_SETTLING: "SETTLING",
            VisionStatus.STATE_TRACKING_DEGRADED: "TRACKING_DEGRADED",
            VisionStatus.STATE_TRACKING: "TRACKING",
            VisionStatus.STATE_LOST: "LOST",
            VisionStatus.STATE_REJECTED: "REJECTED",
        }
        return names.get(state, f"STATE_{state}")

    def _publish_status(self, stamp, prev_position: Optional[np.ndarray] = None) -> None:
        status = VisionStatus()
        status.header.stamp = stamp
        status.header.frame_id = self.frame_id
        status.state = self.state
        status.source = self.source_label
        status.detail = self.detail
        status.quality_score = 1.0 if self.state == VisionStatus.STATE_TRACKING else 0.0
        status.position_std_m = float(self.position_std_m)
        status.rotation_std_rad = float(self.rotation_std_rad)
        status.velocity_std_m_s = float(self.velocity_std_m_s)
        status.has_velocity = False
        status.n_map_points = int(getattr(self.slam, "m", 0) or 0) if self.slam else 0
        status.n_keyframes = int(getattr(self.slam, "n", 0) or 0) if self.slam else 0
        status.scale_factor = 1.0  # pure mono VO; downstream tooling should be aware
        status.has_global_anchor = False
        status.visible_ids = []
        self.vision_status_pub.publish(status)

        if self.legacy_nav_status_pub is not None:
            nav = NavStatus()
            nav.header.stamp = stamp
            nav.header.frame_id = self.frame_id
            nav.state = self.state  # same state codes by construction
            nav.source = self.source_label
            nav.detail = self.detail
            nav.quality_score = status.quality_score
            nav.position_std_m = status.position_std_m
            nav.rotation_std_rad = status.rotation_std_rad
            nav.velocity_std_m_s = status.velocity_std_m_s
            nav.has_velocity = False
            nav.visible_ids = []
            self.legacy_nav_status_pub.publish(nav)

    def _status_heartbeat(self) -> None:
        # Don't spam status if a frame just produced one.
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (
            self.last_pose_publish_time is not None
            and now_sec - self.last_pose_publish_time < self.status_publish_period_sec * 0.9
        ):
            return
        self._publish_status(self.get_clock().now().to_msg())

    # -- intrinsics + preprocessing -------------------------------------

    def _build_intrinsics(self, _image_shape: Tuple[int, int, int]) -> np.ndarray:
        if self.camera_matrix is None:
            raise RuntimeError("camera_matrix is not available yet")
        fx = float(self.camera_matrix[0, 0])
        fy = float(self.camera_matrix[1, 1])
        cx = float(self.camera_matrix[0, 2])
        cy = float(self.camera_matrix[1, 2])
        return np.array([fx, fy, cx, cy], dtype=np.float32)

    def _preprocess_image(
        self, image_bgr: np.ndarray, intrinsics: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        image = image_bgr
        if (
            self.undistort
            and self.dist_coeffs is not None
            and self.dist_coeffs.size > 0
            and np.any(np.abs(self.dist_coeffs) > 1e-12)
        ):
            image = cv2.undistort(image, self.camera_matrix, self.dist_coeffs)

        height, width = image.shape[:2]
        # DPVO's CUDA kernels need a multiple-of-16 image size.
        cropped_h = height - (height % 16)
        cropped_w = width - (width % 16)
        if cropped_h <= 0 or cropped_w <= 0:
            raise RuntimeError(
                f"Image size too small after 16-pixel alignment: {width}x{height}"
            )
        image = image[:cropped_h, :cropped_w]
        return np.ascontiguousarray(image), intrinsics

    def _extract_pose_vector(self, frame_idx: int) -> np.ndarray:
        # Primary path: DPVO's own frame-pose interpolator.
        try:
            pose = (
                self.slam.get_pose(frame_idx)
                .inv().data.detach().cpu().numpy().reshape(-1)
            )
            if pose.shape[0] == 7:
                return pose.astype(np.float64)
        except Exception:
            pass
        # Fallback: latest pose in the active pose graph.
        latest_index = max(0, int(self.slam.n - 1))
        pose = (
            self.lietorch.SE3(self.slam.pg.poses_[latest_index])
            .inv().data.detach().cpu().numpy().reshape(-1)
        )
        if pose.shape[0] != 7:
            raise RuntimeError(f"DPVO returned pose vec of shape {pose.shape}")
        return pose.astype(np.float64)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = DPVONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
