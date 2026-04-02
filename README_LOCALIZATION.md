# Visual Navigation System

This system provides GPS-denied visual localization using an AprilTag/ArUco field.

## Architecture

1. **`camera`**: Publishes raw images and camera intrinsics.
2. **`tag_detector`**: Detects all visible markers, estimates their pose relative to the camera, and publishes `MarkerDetectionArray`.
3. **`tag_localizer`**: Loads a YAML map of marker positions. Computes the vehicle's global pose in the `map` (ENU) frame by fusing detections.
4. **`ardupilot_mavlink_bridge`**: Converts the ENU pose to NED and sends `VISION_POSITION_ESTIMATE` to ArduPilot.
5. **`tag_nav_stub` (Mission Manager)**: A high-level state machine that orchestrates flight scenarios based on the current localizer status.

## Scenarios

### Scenario A: Takeoff and Land on Pad 34
- **Launch**: `ros2 launch bringup scenario_takeoff_land_pad34.launch.py`
- **Description**: The drone searches for the near-field pad marker (ID 34), acquires it, aligns, and lands.

### Scenario B: Ascend (34 -> 1 -> 55)
- **Launch**: `ros2 launch bringup scenario_ascend_34_1_55.launch.py`
- **Description**: Demonstrates multi-level localization. Starts on marker 34, ascends to see marker 1, and continues ascending until marker 55 is visible.

### Scenario C: Full Marker Route
- **Launch**: `ros2 launch bringup scenario_full_marker_route.launch.py`
- **Description**: Follows the complete route: 34 -> 1 -> 55 -> 56 -> 57 -> 55. Triggers simulated photo captures at markers 56 and 57.

## Testing

### Bench Testing
1. Print markers 34 (4cm) and 1 (16cm).
2. Run `ros2 launch bringup system_localization.launch.py`.
3. Move the camera by hand over the markers.
4. Use `rviz2` to visualize the `map` -> `base_link` TF and the `/vision_pose_enu` topic. The vehicle pose should move smoothly without jumps when transitioning between markers.

### SITL Simulation
1. Place the marker models in Gazebo according to `full_map.yaml`.
2. Run `ros2 launch bringup scenario_full_marker_route.launch.py`.
3. A simulation harness should read `/mission_state` and send GUIDED setpoints to ArduPilot.
4. Listen to `/photo_trigger` to capture simulated images when requested by the mission manager.
