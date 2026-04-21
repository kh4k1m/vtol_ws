# Visual Navigation System

This system provides GPS-denied visual localization using an AprilTag/ArUco field.

## Architecture

1. **`camera`**: Publishes raw images and camera intrinsics.
2. **`tag_detector`**: Detects all visible markers, estimates their pose relative to the camera, and publishes `MarkerDetectionArray`.
3. **`tag_localizer`**: Loads a YAML map of marker positions. Computes the vehicle's global pose in the `map` (ENU) frame by fusing detections.
4. **`ardupilot_mavlink_bridge`**: Converts the ENU pose to NED and sends `VISION_POSITION_ESTIMATE` to ArduPilot.
5. **`flight_manager` (Mission Manager)**: A high-level state machine that waits for stable visual localization, commands GUIDED/ARM/TAKEOFF/LAND through the MAVLink bridge, and monitors the scenario end-to-end.

## Scenarios

### Scenario A: Takeoff to 7m and Land
- **Launch**: `ros2 launch bringup scenario_takeoff_land_7m.launch.py`
- **Description**: The drone waits for stable AprilTag-based localization, switches to GUIDED, arms, climbs to 7 meters, holds briefly, and then lands with `QLAND`.

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
2. Start ArduPilot with the base vehicle profile plus the vision overlay:
   `python3 ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane --model JSON --add-param-file=$HOME/SITL_Models/Gazebo/config/alti_transition_quad.param --add-param-file=$HOME/SITL_Models/Gazebo/config/alti_transition_quad_vision_nav.param --console --map`
   If you switch between manual GPS-based tests and vision-only tests, start SITL with `-w` (or remove the saved EEPROM) so persisted parameters do not override the selected profile.
3. Run `ros2 launch bringup scenario_takeoff_land_7m.launch.py`.
4. The `flight_manager` node reads `/localizer_status` and `/ap/relative_alt`, then issues `GUIDED`, `ARM`, takeoff, and `QLAND` commands through `ardupilot_mavlink_bridge`.
5. Keep manual and bench tests on the base `alti_transition_quad.param` only, so ordinary `QLOITER` behavior stays intact even if visual localization is not ready yet.
