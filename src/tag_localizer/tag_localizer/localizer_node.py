import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from marker_interfaces.msg import MarkerDetectionArray, LocalizerStatus
from scipy.spatial.transform import Rotation as R
import math
import sys
import os
# Import geo.py from the current package
from .geo import tagDB, getTransform

# Wrapper to convert ROS 2 MarkerDetection to the format expected by geo.py
class MockTag:
    def __init__(self, tag_id, pose_R, pose_t):
        self.tag_id = tag_id
        self.pose_R = pose_R
        self.pose_t = pose_t


class TagLocalizerNode(Node):
    def __init__(self):
        super().__init__('tag_localizer')
        
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('base_link_frame', 'base_link')
        
        self.declare_parameter('markers', [''])
        markers_param = self.get_parameter('markers').value
        
        import json
        self.marker_map = {}
        for m_str in markers_param:
            if not m_str: continue
            try:
                m = json.loads(m_str) if isinstance(m_str, str) else m_str
                if isinstance(m, dict):
                    m_id = m.get('id')
                    pose = m.get('pose') # [x, y, z, roll, pitch, yaw]
                    if m_id is not None and pose is not None and len(pose) == 6:
                        T = np.eye(4)
                        T[0:3, 3] = pose[0:3]
                        rot = R.from_euler('xyz', pose[3:6]).as_matrix()
                        T[0:3, 0:3] = rot
                        self.marker_map[m_id] = T
            except json.JSONDecodeError:
                self.get_logger().error(f"Failed to parse marker JSON: {m_str}")
        
        self.get_logger().info(f"Loaded {len(self.marker_map)} markers into map.")
        
        self.T_base_camera = np.eye(4)
        
        from rclpy.qos import qos_profile_sensor_data
        self.sub = self.create_subscription(MarkerDetectionArray, '/tag_detections', self.det_cb, qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, '/vision_pose_enu', 10)
        self.status_pub = self.create_publisher(LocalizerStatus, '/localizer_status', 10)
        
        # Initialize the tag database from geo.py
        self.tag_db = tagDB(debug=False, slidingWindow=5)


    def det_cb(self, msg: MarkerDetectionArray):
        status = LocalizerStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.pose_valid = False
        status.visible_ids = []
        
        best_pose_map_base = None
        best_score = -1
        active_id = -1
        
        # --- GEO.PY INTEGRATION TODO ---
        # 1. Start a new frame for geo.py:
        # self.tag_db.newFrame()
        #
        # 2. Iterate through detections and add them to tag_db:
        # for det in msg.detections:
        #     T_cam_mark = self.pose_to_matrix(det.pose_camera_marker)
        #     pose_R = T_cam_mark[0:3, 0:3]
        #     pose_t = T_cam_mark[0:3, 3]
        #     mock_tag = MockTag(det.id, pose_R, pose_t)
        #     # You also need to pass T_CamtoVeh (transform from camera to vehicle)
        #     self.tag_db.addTag(mock_tag, self.T_base_camera)
        #
        # 3. Generate reported location:
        # self.tag_db.generateReportedLoc()
        #
        # 4. Use self.tag_db.reportedPos and self.tag_db.reportedRot 
        #    to publish the final filtered pose.
        # -------------------------------
        
        for det in msg.detections:
            if det.id not in self.marker_map:
                continue
                
            # Skip invalid NaNs
            if math.isnan(det.pose_camera_marker.position.x) or math.isnan(det.pose_camera_marker.position.y) or math.isnan(det.pose_camera_marker.position.z):
                continue
                
            T_cam_mark = self.pose_to_matrix(det.pose_camera_marker)
            if T_cam_mark is None:
                continue
                
            try:
                T_mark_cam = np.linalg.inv(T_cam_mark)
                T_map_mark = self.marker_map[det.id]
                T_map_cam = np.dot(T_map_mark, T_mark_cam)
                
                T_cam_base = np.linalg.inv(self.T_base_camera)
                T_map_base = np.dot(T_map_cam, T_cam_base)
            except np.linalg.LinAlgError:
                continue
            
            score = det.size_m * 100 + det.decision_margin
            if det.id in [34, 6] and best_score == -1:
                score = 50 
                
            if score > best_score:
                best_score = score
                best_pose_map_base = T_map_base
                active_id = det.id
                
            status.visible_ids.append(det.id)

        if best_pose_map_base is not None:
            status.pose_valid = True
            status.active_marker_source = f"marker_{active_id}"
            status.quality_score = float(best_score)
            status.state = "TRACKING"
            
            pose_msg = PoseStamped()
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.header.frame_id = self.get_parameter('map_frame').value
            pose_msg.pose = self.matrix_to_pose(best_pose_map_base)
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

    def matrix_to_pose(self, matrix):
        pose = Pose()
        pose.position.x = matrix[0, 3]
        pose.position.y = matrix[1, 3]
        pose.position.z = matrix[2, 3]
        rot = R.from_matrix(matrix[0:3, 0:3])
        quat = rot.as_quat() # [x, y, z, w]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

def main(args=None):
    rclpy.init(args=args)
    node = TagLocalizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
