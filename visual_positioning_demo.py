import cv2
import numpy as np
import math

def get_gps_from_offset(lat0, lon0, dx, dy):
    # dx is East, dy is North (in meters)
    r_earth = 6378137.0
    d_lat = dy / r_earth
    d_lon = dx / (r_earth * math.cos(math.pi * lat0 / 180.0))
    return lat0 + (d_lat * 180.0 / math.pi), lon0 + (d_lon * 180.0 / math.pi)

def main():
    video_path = "data/C0026/C0026.MP4"
    cap = cv2.VideoCapture(video_path)
    
    if not cap.isOpened():
        print("Error opening video")
        return

    # ArUco / AprilTag setup
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    # Approximate camera matrix (assuming 4K video from Sony A6700)
    width, height = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    focal_length = 3000.0  # approximate
    camera_matrix = np.array([[focal_length, 0, width/2], 
                              [0, focal_length, height/2], 
                              [0, 0, 1]], dtype=float)
    dist_coeffs = np.zeros((5, 1))

    marker_size = 0.8  # meters (approximate size of the marker)

    # Known anchor (id 4)
    anchor_id = 4
    anchor_lat, anchor_lon = 65.988106, 117.622628

    # Relative positions of markers 3 and 5 to marker 4 (in meters, ENU frame)
    # Assuming they are placed along a line, e.g., 3 is 5 meters West, 5 is 5 meters East
    relative_positions = {
        4: np.array([0.0, 0.0, 0.0]),
        3: np.array([-5.0, 0.0, 0.0]),
        5: np.array([5.0, 0.0, 0.0])
    }

    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret: break
        frame_count += 1
        
        # Process every 10th frame for speed
        if frame_count % 10 != 0:
            continue

        corners, ids, rejected = detector.detectMarkers(frame)
        
        if ids is not None:
            # OpenCV 4.7+ pose estimation
            obj_points = np.array([
                [-marker_size/2, marker_size/2, 0],
                [marker_size/2, marker_size/2, 0],
                [marker_size/2, -marker_size/2, 0],
                [-marker_size/2, -marker_size/2, 0]
            ], dtype=np.float32)

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in relative_positions:
                    success, rvec, tvec = cv2.solvePnP(obj_points, corners[i][0], camera_matrix, dist_coeffs)
                    if success:
                        # tvec is the position of the marker in the camera frame
                        # Camera position in the marker frame is -R^T * tvec
                        R, _ = cv2.Rodrigues(rvec)
                        camera_pos_marker_frame = -np.dot(R.T, tvec).flatten()
                        
                        # Drone position relative to anchor (marker 4)
                        # We add the marker's relative position to the camera's position relative to the marker
                        marker_offset = relative_positions[marker_id]
                        
                        # Simplified: assuming marker frame X is East, Y is North (or similar)
                        # Let's just use the X, Y translation for demonstration
                        dx = camera_pos_marker_frame[0] + marker_offset[0]
                        dy = camera_pos_marker_frame[1] + marker_offset[1]
                        dz = camera_pos_marker_frame[2] # Altitude
                        
                        # Calculate GPS
                        drone_lat, drone_lon = get_gps_from_offset(anchor_lat, anchor_lon, dx, dy)
                        
                        print(f"Frame {frame_count} | Detected ID: {marker_id} | "
                              f"Drone Pos (rel to anchor): X={dx:.2f}m, Y={dy:.2f}m, Alt={dz:.2f}m | "
                              f"Estimated GPS: {drone_lat:.6f}, {drone_lon:.6f}")
                        
                        # Draw for visualization (optional, we won't show it here, just print)
                        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_size/2)

    cap.release()
    print("Processing complete.")

if __name__ == "__main__":
    main()
