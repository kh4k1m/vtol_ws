# Visual Navigation System

GPS-denied visual localization and navigation for ArduPilot-based VTOL
using AprilTag/ArUco fields, Deep Patch Visual Odometry (DPVO), or a
hybrid of the two.

## Architecture

The system is modular and the active scenario is selected via
`config/flight.yaml`. The MAVROS dependency was removed - the
`ardupilot_mavlink_bridge` talks to ArduPilot directly over MAVLink and
republishes telemetry on `/ap/...` ROS topics.

### Core packages

| Package | Role |
|---|---|
| `camera` (C++) / `bringup.video_player` (Py) / `ros_gz_bridge` | Camera source for `real`, `from_log`, `gazebo` respectively. |
| `ardupilot_mavlink_bridge` | MAVLink owner. Forwards `/vision_pose_enu` as `ODOMETRY` (NED/FRD) with proper covariances and republishes IMU/GPS/state on `/ap/...`. |
| `tag_detector` (C++) | OpenCV ArUco detector. Publishes `MarkerDetectionArray`. |
| `tag_localizer` | Pose from a YAML map of known markers. Publishes pose, `LocalizerStatus` (tag-specific) and `NavStatus` (unified). |
| `dpvo_bridge` | ROS 2 wrapper around DPVO. Publishes pose + `NavStatus` (default `/dpvo/pose` + `/dpvo/nav_status`; or `/vision_pose_enu` + `/nav/status` in standalone DPVO scenario). |
| `orb_slam3_bridge` (C++) | ROS 2 wrapper around ORB-SLAM3 (mono / IMU-mono). Same contract as DPVO: pose + `NavStatus`. Built on demand once `ORB_SLAM3_ROOT` is exported (see `scripts/install_orb_slam3.sh`). |
| `flight_manager.vision_fusion_node` | Multiplexer between tags and a generic VO source (`vo_pose_topic` / `vo_nav_status_topic`). SE(3) alignment with smooth-return on tag re-acquisition. Publishes the active `NavStatus`. |
| `flight_manager.landmark_mapper_node` | Persists newly observed markers to `logs/discovered_tags_<ts>.yaml` (atomic write, median over N frames). |
| `flight_manager.mission_manager_node` | High-level state machine. Gates on `NavStatus`, escalates after repeated arm rejections. |
| `flight_manager.tf_broadcaster_node` | Publishes `map -> base_link` (dynamic) and `base_link -> camera_optical_frame` (static) so RViz / TF consumers work in every scenario. |

### Topic contract

* **`/vision_pose_enu`** (`geometry_msgs/PoseStamped`) - pose to forward
  to ArduPilot. Frame: `map`, body: ENU/FLU.
* **`/nav/status`** (`marker_interfaces/NavStatus`) - unified status from
  the active source. Carries `state` (uint8 enum), `quality_score`,
  per-axis std-devs and an optional velocity sample.
* **`/ap/relative_alt`**, **`/ap/connected`**, **`/ap/armed`**,
  **`/ap/external_nav_ready`**, **`/ap/flight_mode`** - autopilot state.
* **`/ap/imu/raw`** (`sensor_msgs/Imu`),
  **`/ap/gps/fix`** (`sensor_msgs/NavSatFix`) - raw telemetry. (Replaces
  the legacy `/mavros/...` topic names.)

### Frame conventions

Vision pipeline operates in ENU/FLU (REP-103). The bridge converts to
ArduPilot's NED/FRD via fixed rotation matrices (no Euler-angle round
trip), and reports body-frame velocities derived from the full SE(3)
orientation. Pose covariance is filled from `NavStatus`'s std-devs
(per-axis), velocity covariance and quality propagate from the same
source - never NaN.

## Configuration

`$VTOL_WS_ROOT/config/flight.yaml` (defaults to `~/vtol_ws`):

* `environment`: `real` | `gazebo` | `from_log`
* `navigation`: `tags` | `dpvo` | `orb_slam3` | `hybrid` | `none`
* `vo_backend` (used when `navigation: hybrid`): `dpvo` | `orb_slam3`
* `orb_slam3.sensor_type`: `MONOCULAR` | `IMU_MONOCULAR`
* `orb_slam3.vocabulary_path`, `orb_slam3.settings_path`: optional
  overrides; empty falls back to `$ORB_SLAM3_ROOT/Vocabulary/ORBvoc.txt`
  and the per-mode template in `orb_slam3_bridge/config/`.
* `real_connection` / `gazebo_connection` / `log_connection`: pymavlink
  connection strings.

Other configs:

* `config/camera.yaml` - intrinsics + camera_offset (translation + rotation).
* `config/markers.yaml` - per-marker IDs and physical sizes.
* `config/mission.yaml` - target altitude, hover duration.
* `src/tag_localizer/config/*.yaml` - the active marker map (loaded as
  ROS parameters via the launch file).

## Usage

```bash
ros2 launch bringup flight.launch.py
```

### Scenarios

#### AprilTag flight (SITL or real)

1. `config/flight.yaml`: `navigation: "tags"`, `environment: "gazebo"` or `"real"`.
2. `ros2 launch bringup flight.launch.py`.
3. The mission manager waits for `NavStatus.state >= TRACKING_DEGRADED`,
   switches EKF to ExternalNav, arms, takes off via VTOL AUTO mission,
   hovers, lands.

#### DPVO visual odometry

1. Place the DPVO checkpoint at `~/vtol_ws/dpvo.pth`.
2. `navigation: "dpvo"`.
3. Pose is published to `/vision_pose_enu`; the trajectory logger writes
   `logs/dpvo_vs_gps_<timestamp>.csv`.

#### ORB-SLAM3 visual odometry

1. Install ORB-SLAM3 once (see "ORB-SLAM3 install" below).
2. `navigation: "orb_slam3"`. Defaults assume a downward-looking mono camera;
   for metric scale use `orb_slam3.sensor_type: "IMU_MONOCULAR"` and verify
   that `IMU.T_b_c1` in the settings template matches your rig.
3. Pose is published to `/vision_pose_enu`, `NavStatus` to `/nav/status`.
   Note: in monocular mode scale is up to a constant - prefer IMU-mono on
   the real Jetson rig.

#### Hybrid (tags + VO)

1. `navigation: "hybrid"`. Pick the VO backend with `vo_backend: "dpvo"` or
   `vo_backend: "orb_slam3"`.
2. `tag_localizer` publishes on `/tag_localizer/pose` and `/tag_localizer/nav_status`.
3. The VO node publishes on `/<vo>/pose` + `/<vo>/nav_status`
   (`/dpvo/pose`+`/dpvo/nav_status` or `/orb_slam3/pose`+`/orb_slam3/nav_status`).
4. `vision_fusion_node` aligns VO to the tag pose with a full SE(3) offset
   and publishes the multiplexed `/vision_pose_enu` and `/nav/status`. The
   active backend label is forwarded into `NavStatus.source` so the bridge
   sees a non-zero covariance during tag-lost intervals.
5. `landmark_mapper_node` writes new markers to
   `logs/discovered_tags_<ts>.yaml` (atomic, median over N frames).

#### Data-collection-only

1. `navigation: "none"`. Camera + bridge only - safe to record bags or
   run experimental nodes (`gps_takeoff_land_node`, `gps_waypoints_node`)
   in parallel terminals against the live bridge.

## Testing

### Bench (AprilTags)

1. `environment: "real"`, `navigation: "tags"`.
2. Move the camera by hand over a printed marker field.
3. RViz on `map` frame: the `vision_tf_broadcaster` publishes
   `map -> base_link` and the static `base_link -> camera_optical_frame`,
   so you can visualize pose continuity directly.

### SITL

1. Place the marker models in Gazebo according to your active map.
2. Start ArduPilot with the base vehicle profile plus the vision overlay:
   ```bash
   python3 ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane --model JSON \
     --add-param-file=$HOME/SITL_Models/Gazebo/config/alti_transition_quad.param \
     --add-param-file=$HOME/SITL_Models/Gazebo/config/alti_transition_quad_vision_nav.param \
     --console --map
   ```
   Use `-w` if switching between GPS and vision-only tests to wipe old
   parameters.
3. `ros2 launch bringup flight.launch.py`.

## ORB-SLAM3 install

ORB-SLAM3 ships as a C++ library and is built from source via the bundled
installer. The same script works on x86_64 Ubuntu 22.04 (laptop, Gazebo)
and on Jetson Orin NX 16GB / JetPack 6 (Ubuntu 22.04). On Jetson the
script preserves JetPack's CUDA-enabled OpenCV by skipping
`libopencv-dev` if it is already installed.

```bash
cd ~/vtol_ws
./scripts/install_orb_slam3.sh           # installs to ~/vtol_ws/thirdparty/ORB_SLAM3
echo 'export ORB_SLAM3_ROOT="$HOME/vtol_ws/thirdparty/ORB_SLAM3"' >> ~/.bashrc
source ~/.bashrc
colcon build --packages-select orb_slam3_bridge
```

Useful flags: `--reinstall` (rebuild even if libs exist),
`--no-pangolin` (use system Pangolin). Build can be parallelized via
`ORB_SLAM3_BUILD_JOBS=N ./scripts/install_orb_slam3.sh`. On Jetson Orin
NX the full build takes ~15 min.

If `orb_slam3_node` exits at startup with "ORB_SLAM3_ROOT is not set",
re-source the shell where `colcon build` is invoked - the variable is
read at CMake configure time. The wrapper also checks that
`Vocabulary/ORBvoc.txt` exists; if it doesn't, run the installer with
`--reinstall` to re-extract it.

### Camera settings for ORB-SLAM3

Copy a template into `config/` and edit intrinsics + image size to match
your sensor (`/camera/camera_info` in real, `/world/.../camera_info` in
Gazebo):

```bash
cp src/orb_slam3_bridge/config/orb_slam3_monocular_template.yaml \
   config/orb_slam3.yaml
# edit Camera1.fx/fy/cx/cy, Camera.width/height, ORBextractor.* if needed
```

Then point `flight.yaml` at it:

```yaml
flight_config:
  navigation: "orb_slam3"
  orb_slam3:
    sensor_type: "MONOCULAR"
    settings_path: "config/orb_slam3.yaml"
```

For metric flight on the Jetson, switch to IMU-mono and adjust the
`IMU.T_b_c1` block in the IMU template to match your camera<->IMU
extrinsics.
