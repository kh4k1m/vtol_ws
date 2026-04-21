import csv
import json
import math
import os
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from marker_interfaces.msg import MarkerDetectionArray, LocalizerStatus
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.parameter import Parameter
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64

# Import geo.py from the current package
from .geo import tagDB

# Adapter class to convert ROS 2 MarkerDetection to the format expected by geo.py
class DetectedTag:
    def __init__(self, tag_id, pose_R, pose_t):
        self.tag_id = tag_id
        self.pose_R = pose_R
        self.pose_t = pose_t

class TagLocalizerNode(Node):
    def __init__(self):
        super().__init__('tag_localizer')
        
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('cam_trans', [0.0, 0.0, -0.18])
        self.declare_parameter('cam_rot', [180.0, 0.0, -90.0]) # base_link -> camera_optical_frame, roll/pitch/yaw in degrees
        self.declare_parameter('markers', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('height_log_dir', os.path.expanduser('~/vtol_ws/logs'))
        self.declare_parameter('stable_window_size', 5)
        self.declare_parameter('stable_xy_stddev_m', 0.08)
        self.declare_parameter('stable_z_stddev_m', 0.12)
        
        # Initialize the tag database from geo.py
        self.tag_db = tagDB(debug=False, slidingWindow=5)
        self.height_log_file = None
        self.height_log_writer = None
        self.latest_reference_height = math.nan
        self.latest_sim_time = math.nan
        self.pending_estimated_height = 0.0
        self.has_pending_estimate = False
        self.stable_window_size = max(2, int(self.get_parameter('stable_window_size').value))
        self.stable_xy_stddev_m = float(self.get_parameter('stable_xy_stddev_m').value)
        self.stable_z_stddev_m = float(self.get_parameter('stable_z_stddev_m').value)
        self.recent_pose_samples = deque(maxlen=self.stable_window_size)
        
        # Transform from base_link to camera
        trans = self.get_parameter('cam_trans').value
        rot_deg = self.get_parameter('cam_rot').value
        self.T_base_camera = np.eye(4)
        self.T_base_camera[0:3, 3] = trans
        self.T_base_camera[0:3, 0:3] = R.from_euler('xyz', rot_deg, degrees=True).as_matrix()
        self.get_logger().info(f"Camera offset: trans={trans}, rot={rot_deg} deg")
        self._load_marker_map()
        self._init_height_logger()
        
        from rclpy.qos import qos_profile_sensor_data
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_cb, qos_profile_sensor_data)
        self.sub = self.create_subscription(MarkerDetectionArray, '/tag_detections', self.det_cb, qos_profile_sensor_data)
        self.reference_height_sub = self.create_subscription(
            Float64,
            '/ap/relative_alt',
            self.reference_height_cb,
            qos_profile_sensor_data
        )
        self.pose_pub = self.create_publisher(PoseStamped, '/vision_pose_enu', 10)
        self.status_pub = self.create_publisher(LocalizerStatus, '/localizer_status', 10)

    def det_cb(self, msg: MarkerDetectionArray):
        status = LocalizerStatus()
        status.header.stamp = msg.header.stamp
        status.pose_valid = False
        status.active_marker_source = ''
        status.visible_ids = []
        status.quality_score = 0.0
        status.state = "NO_DETECTIONS"
        
        if not msg.detections:
            self._reset_stability_window()
            self.status_pub.publish(status)
            return

        # 1. Start a new frame for geo.py
        self.tag_db.newFrame()
        
        # 2. Iterate through detections and add them to tag_db
        for det in msg.detections:
            # Skip invalid NaNs
            if math.isnan(det.pose_camera_marker.position.x):
                continue
                
            T_cam_mark = self.pose_to_matrix(det.pose_camera_marker)
            if T_cam_mark is None:
                continue
                
            pose_R = T_cam_mark[0:3, 0:3]
            pose_t = T_cam_mark[0:3, 3]
            detected_tag = DetectedTag(det.id, pose_R, pose_t)
            
            # Add tag to database
            self.tag_db.addTag(detected_tag, self.T_base_camera)
            status.visible_ids.append(det.id)

        if not status.visible_ids:
            status.state = "INVALID_DETECTIONS"
            self._reset_stability_window()
            self.status_pub.publish(status)
            return

        status.active_marker_source = f"markers_{status.visible_ids}"
        status.quality_score = float(len(status.visible_ids))

        # 3. Generate reported location (Kalman filter + Z-score)
        # Use message timestamp in seconds
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.tag_db.getBestTransform(current_time)

        if len(self.tag_db.timestamps) == 0:
            status.state = "INITIALIZING"
            self._reset_stability_window()
            self.status_pub.publish(status)
            return

        latest_timestamp = float(self.tag_db.timestamps[-1])
        if abs(latest_timestamp - current_time) > 1e-3:
            status.state = "REJECTED_POSE"
            self._reset_stability_window()
            self.status_pub.publish(status)
            return

        reported_pos = self._coerce_vector(self.tag_db.reportedPos, 3)
        reported_rot = self._coerce_vector(self.tag_db.reportedRot, 3)
        if reported_pos is None or reported_rot is None:
            status.state = "INVALID_POSE"
            self._reset_stability_window()
            self.status_pub.publish(status)
            return

        self.recent_pose_samples.append(np.array(reported_pos, dtype=float))
        tracking_mode = "SINGLE_TAG" if len(status.visible_ids) == 1 else "MULTI_TAG"
        if not self._tracking_is_stable():
            status.state = f"TRACKING_{tracking_mode}_SETTLING"
            self.status_pub.publish(status)
            return

        # 4. Publish the final filtered pose once the estimate has settled.
        status.pose_valid = True
        status.state = f"TRACKING_{tracking_mode}"

        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = self.get_parameter('map_frame').value

        pose_msg.pose.position.x = float(reported_pos[0])
        pose_msg.pose.position.y = float(reported_pos[1])
        pose_msg.pose.position.z = float(reported_pos[2])

        # geo.py returns Euler angles in radians.
        rot = R.from_euler('xyz', reported_rot, degrees=False)
        quat = rot.as_quat()
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])

        self.pending_estimated_height = float(reported_pos[2])
        self.has_pending_estimate = True
        self.pose_pub.publish(pose_msg)
        self.status_pub.publish(status)

    def pose_to_matrix(self, pose):
        T = np.eye(4)
        T[0, 3] = pose.position.x
        T[1, 3] = pose.position.y
        T[2, 3] = pose.position.z
        
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm < 1e-6:
            return None
            
        rot = R.from_quat([qx, qy, qz, qw])
        T[0:3, 0:3] = rot.as_matrix()
        return T

    def clock_cb(self, msg: Clock):
        self.latest_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def reference_height_cb(self, msg: Float64):
        self.latest_reference_height = float(msg.data)
        timestamp_sec = self.latest_sim_time if math.isfinite(self.latest_sim_time) else time.time()
        estimated_height_m = self.pending_estimated_height if self.has_pending_estimate else 0.0

        # Log every reference sample so gaps in vision predictions show up as zeros.
        self._log_height_sample(timestamp_sec, estimated_height_m, self.latest_reference_height)
        self.has_pending_estimate = False

    def _load_marker_map(self):
        raw_markers = self.get_parameter('markers').value
        if not raw_markers:
            self.get_logger().info('No marker map provided; localizer will learn unknown tags from observations.')
            return

        loaded_count = 0
        for raw_entry in raw_markers:
            try:
                marker = json.loads(raw_entry) if isinstance(raw_entry, str) else raw_entry
                pose = marker['pose']
                if len(pose) != 6:
                    raise ValueError(f"expected 6 pose values, got {len(pose)}")

                pose = [float(value) for value in pose]
                tag_id = int(marker['id'])

                T_map_tag = np.eye(4)
                T_map_tag[0:3, 3] = pose[:3]
                T_map_tag[0:3, 0:3] = R.from_euler('xyz', pose[3:], degrees=False).as_matrix()
                self.tag_db.tagPlacement[tag_id] = T_map_tag
                loaded_count += 1
            except Exception as exc:
                self.get_logger().warning(f"Skipping invalid marker map entry {raw_entry!r}: {exc}")

        if loaded_count > 0:
            self.get_logger().info(f"Loaded {loaded_count} known markers into the map frame.")
        else:
            self.get_logger().warning('Marker map parameter was provided, but no valid entries were loaded.')

    def _init_height_logger(self):
        log_dir = self.get_parameter('height_log_dir').value
        try:
            os.makedirs(log_dir, exist_ok=True)
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            log_path = os.path.join(log_dir, f'localizer_height_{timestamp}.csv')
            self.height_log_file = open(log_path, 'w', newline='')
            self.height_log_writer = csv.writer(self.height_log_file)
            self.height_log_writer.writerow(['timestamp', 'estimated_height_m', 'reference_height_m'])
            self.height_log_file.flush()
            self.get_logger().info(f'Logging computed height to {log_path}')
        except Exception as exc:
            self.height_log_file = None
            self.height_log_writer = None
            self.get_logger().warning(f'Failed to initialize height logger: {exc}')

    def _log_height_sample(self, timestamp_sec, estimated_height_m, reference_height_m):
        if self.height_log_writer is None or self.height_log_file is None:
            return

        self.height_log_writer.writerow([
            float(timestamp_sec),
            float(estimated_height_m),
            float(reference_height_m) if math.isfinite(reference_height_m) else float('nan')
        ])
        self.height_log_file.flush()

    def _coerce_vector(self, value, expected_size):
        array = np.asarray(value, dtype=float)
        if array.size != expected_size or not np.all(np.isfinite(array)):
            return None
        return array.reshape(expected_size)

    def _reset_stability_window(self):
        self.recent_pose_samples.clear()

    def _tracking_is_stable(self):
        if len(self.recent_pose_samples) < self.stable_window_size:
            return False

        samples = np.asarray(self.recent_pose_samples, dtype=float)
        xy_std = np.std(samples[:, :2], axis=0)
        z_std = float(np.std(samples[:, 2]))
        return (
            max(float(xy_std[0]), float(xy_std[1])) <= self.stable_xy_stddev_m
            and z_std <= self.stable_z_stddev_m
        )

    def destroy_node(self):
        if self.height_log_file is not None:
            self.height_log_file.close()
            self.height_log_file = None
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TagLocalizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
