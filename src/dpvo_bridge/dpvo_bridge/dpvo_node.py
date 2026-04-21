#!/usr/bin/env python3
"""ROS 2 node that wraps the unofficial real-time use of DPVO.

Notes
-----
DPVO's public demo path is offline/video oriented. This node mirrors that demo loop
frame-by-frame and uses `slam.get_pose(t).inv()` to extract the current pose in the
same x y z qx qy qz qw convention that DPVO's `terminate()` returns.
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
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def _add_dpvo_path_from_env() -> None:
    extra = os.environ.get("DPVO_PYTHON_PATH", "").strip()
    if extra and extra not in sys.path:
        sys.path.insert(0, extra)


class DPVONode(Node):
    def __init__(self) -> None:
        super().__init__("dpvo_node")
        _add_dpvo_path_from_env()

        self.declare_parameter(
            "image_topic",
            "/camera/image_raw",
            ParameterDescriptor(description="Input image topic."),
        )
        self.declare_parameter(
            "camera_info_topic",
            "/camera/camera_info",
            ParameterDescriptor(description="Camera calibration topic."),
        )
        self.declare_parameter("pose_topic", "/dpvo/pose")
        self.declare_parameter("path_topic", "/dpvo/path")
        self.declare_parameter("frame_id", "dpvo_map")
        self.declare_parameter("weights_path", "")
        self.declare_parameter("config_path", "")
        self.declare_parameter("config_overrides", [])
        self.declare_parameter("use_cuda", True)
        self.declare_parameter("undistort", True)
        self.declare_parameter("publish_path", True)
        self.declare_parameter("max_path_length", 2000)
        self.declare_parameter("process_every_n", 1)
        self.declare_parameter("log_every_n", 30)
        self.declare_parameter("enable_viz", False)

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        self.path_topic = self.get_parameter("path_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.weights_path = self.get_parameter("weights_path").get_parameter_value().string_value
        self.config_path = self.get_parameter("config_path").get_parameter_value().string_value
        self.config_overrides = list(
            self.get_parameter("config_overrides").get_parameter_value().string_array_value
        )
        self.use_cuda = self.get_parameter("use_cuda").get_parameter_value().bool_value
        self.undistort = self.get_parameter("undistort").get_parameter_value().bool_value
        self.publish_path = self.get_parameter("publish_path").get_parameter_value().bool_value
        self.max_path_length = self.get_parameter("max_path_length").get_parameter_value().integer_value
        self.process_every_n = max(1, self.get_parameter("process_every_n").get_parameter_value().integer_value)
        self.log_every_n = max(1, self.get_parameter("log_every_n").get_parameter_value().integer_value)
        self.enable_viz = self.get_parameter("enable_viz").get_parameter_value().bool_value

        self.bridge = CvBridge()
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.path_pub = self.create_publisher(PathMsg, self.path_topic, 10)
        self.path_msg = PathMsg()
        self.path_msg.header.frame_id = self.frame_id

        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.device: Optional[str] = None
        self.torch = None
        self.lietorch = None
        self.dpvo_cls = None
        self.cfg = None
        self.slam = None

        self.frame_counter = 0
        self.processed_counter = 0
        self.received_camera_info = False
        self.processing_lock = threading.Lock()
        self.processing = False

        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(Image, self.image_topic, self._image_cb, 10)

        self.get_logger().info(
            f"DPVO node is up. image_topic={self.image_topic}, camera_info_topic={self.camera_info_topic}"
        )

    def _lazy_import_dpvo(self) -> None:
        if self.dpvo_cls is not None:
            return

        try:
            import torch  # type: ignore
            import lietorch  # type: ignore
            from dpvo.config import cfg as base_cfg  # type: ignore
            from dpvo.dpvo import DPVO  # type: ignore
        except Exception as exc:  # pragma: no cover - import error path
            raise RuntimeError(
                "Could not import DPVO. Install the official DPVO package into the same Python "
                "environment as ROS 2, or export DPVO_PYTHON_PATH=/path/to/DPVO."
            ) from exc

        if self.use_cuda and not torch.cuda.is_available():
            raise RuntimeError("use_cuda=true, but torch.cuda.is_available() is false.")

        self.torch = torch
        self.lietorch = lietorch
        self.dpvo_cls = DPVO
        self.cfg = base_cfg.clone()
        self.device = "cuda" if self.use_cuda else "cpu"

        if self.config_path:
            cfg_path = Path(self.config_path)
            if not cfg_path.exists():
                raise FileNotFoundError(f"config_path does not exist: {cfg_path}")
            self.cfg.merge_from_file(str(cfg_path))

        if self.config_overrides:
            self.cfg.merge_from_list(self.config_overrides)

        if not self.weights_path:
            raise RuntimeError("weights_path parameter is empty.")

        if not Path(self.weights_path).exists():
            raise FileNotFoundError(f"weights_path does not exist: {self.weights_path}")

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self.received_camera_info = True
        self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float32) if msg.d else np.zeros((0,), dtype=np.float32)

    def _image_cb(self, msg: Image) -> None:
        self.frame_counter += 1
        if not self.received_camera_info:
            if self.frame_counter % self.log_every_n == 0:
                self.get_logger().warn("Waiting for /camera/camera_info before processing images.")
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
            self.get_logger().error(f"DPVO processing failed: {exc}\n{traceback.format_exc()}")
        finally:
            self.processing = False

    def _process_frame(self, msg: Image) -> None:
        self._lazy_import_dpvo()

        image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        image_bgr = np.ascontiguousarray(image_bgr)

        intrinsics = self._build_intrinsics(image_bgr.shape)
        image_bgr, intrinsics = self._preprocess_image(image_bgr, intrinsics)

        image_tensor = self.torch.from_numpy(image_bgr).permute(2, 0, 1).to(self.device)
        intrinsics_tensor = self.torch.from_numpy(intrinsics).to(self.device)

        if self.slam is None:
            _, height, width = image_tensor.shape
            self.slam = self.dpvo_cls(
                self.cfg,
                self.weights_path,
                ht=int(height),
                wd=int(width),
                viz=self.enable_viz,
            )
            self.get_logger().info(
                f"DPVO initialized. image_size={width}x{height}, frame_id={self.frame_id}"
            )

        frame_idx = self.processed_counter
        self.slam(frame_idx, image_tensor, intrinsics_tensor)
        pose_vec = self._extract_pose_vector(frame_idx)
        self.processed_counter += 1

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
        self.pose_pub.publish(pose_msg)

        if self.publish_path:
            self.path_msg.header.stamp = msg.header.stamp
            self.path_msg.poses.append(pose_msg)
            if len(self.path_msg.poses) > self.max_path_length:
                self.path_msg.poses = self.path_msg.poses[-self.max_path_length :]
            self.path_pub.publish(self.path_msg)

        if self.processed_counter % self.log_every_n == 0:
            self.get_logger().info(
                "Published DPVO pose frame=%d xyz=(%.3f, %.3f, %.3f)"
                % (self.processed_counter, pose_vec[0], pose_vec[1], pose_vec[2])
            )

    def _build_intrinsics(self, image_shape: Tuple[int, int, int]) -> np.ndarray:
        if self.camera_matrix is None:
            raise RuntimeError("camera_matrix is not available yet")

        fx = float(self.camera_matrix[0, 0])
        fy = float(self.camera_matrix[1, 1])
        cx = float(self.camera_matrix[0, 2])
        cy = float(self.camera_matrix[1, 2])
        return np.array([fx, fy, cx, cy], dtype=np.float32)

    def _preprocess_image(self, image_bgr: np.ndarray, intrinsics: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        image = image_bgr
        if self.undistort and self.dist_coeffs is not None and self.dist_coeffs.size > 0:
            if np.any(np.abs(self.dist_coeffs) > 1e-12):
                image = cv2.undistort(image, self.camera_matrix, self.dist_coeffs)

        height, width = image.shape[:2]
        cropped_h = height - (height % 16)
        cropped_w = width - (width % 16)
        if cropped_h <= 0 or cropped_w <= 0:
            raise RuntimeError(f"Image size is too small after 16-pixel alignment: {width}x{height}")

        image = image[:cropped_h, :cropped_w]
        return np.ascontiguousarray(image), intrinsics

    def _extract_pose_vector(self, frame_idx: int) -> np.ndarray:
        # Primary path: use the frame pose interpolation helper from DPVO itself.
        try:
            pose = self.slam.get_pose(frame_idx).inv().data.detach().cpu().numpy().reshape(-1)
            if pose.shape[0] == 7:
                return pose.astype(np.float64)
        except Exception:
            pass

        # Fallback path: use the newest pose in the active pose graph.
        try:
            latest_index = int(self.slam.n - 1)
            pose = (
                self.lietorch.SE3(self.slam.pg.poses_[latest_index])
                .inv()
                .data.detach()
                .cpu()
                .numpy()
                .reshape(-1)
            )
            if pose.shape[0] == 7:
                return pose.astype(np.float64)
        except Exception as exc:
            raise RuntimeError("Could not extract a live pose from DPVO.") from exc

        raise RuntimeError("DPVO returned an invalid pose vector shape.")


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
