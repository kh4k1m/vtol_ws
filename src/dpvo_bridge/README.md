# dpvo_bridge

ROS 2 Python package with:
- `dpvo_node`: subscribes to `/camera/image_raw` and `/camera/camera_info`, runs DPVO, publishes `/dpvo/pose` and `/dpvo/path`.
- `trajectory_logger_node`: subscribes to `/dpvo/pose` and GPS (`sensor_msgs/NavSatFix`), saves both into CSV.

## What this package assumes

1. Official DPVO is already installed into the same Python environment that launches ROS 2,
   or you export `DPVO_PYTHON_PATH=/absolute/path/to/DPVO` before `ros2 run` / `ros2 launch`.
2. `weights_path` points to the downloaded DPVO checkpoint.
3. `config_path` points to `DPVO/config/default.yaml` or your own copied config.

## Build inside vtol_ws/src

```bash
cd ~/vtol_ws/src
cp -r /path/to/dpvo_bridge .
cd ~/vtol_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dpvo_bridge
source install/setup.bash
```

## Run only DPVO

```bash
ros2 run dpvo_bridge dpvo_node --ros-args \
  -p image_topic:=/camera/image_raw \
  -p camera_info_topic:=/camera/camera_info \
  -p weights_path:=/ABS/PATH/TO/dpvo.pth \
  -p config_path:=/ABS/PATH/TO/DPVO/config/default.yaml
```

## Run DPVO + CSV logger

```bash
ros2 launch dpvo_bridge dpvo_with_logger.launch.py \
  weights_path:=/ABS/PATH/TO/dpvo.pth \
  config_path:=/ABS/PATH/TO/DPVO/config/default.yaml \
  gps_topic:=/gps/fix \
  output_csv:=/tmp/dpvo_vs_gps.csv
```

## CSV columns

- `pose_stamp_ns`, `gps_stamp_ns`, `gps_time_delta_sec`
- `gps_lat_deg`, `gps_lon_deg`, `gps_alt_m`
- `dpvo_x_m`, `dpvo_y_m`, `dpvo_z_m`
- `dpvo_qx`, `dpvo_qy`, `dpvo_qz`, `dpvo_qw`

## Important limitation

This is an integration wrapper around DPVO's research code. The official repo documents the
video/image-directory demo path, not a supported ROS real-time API, so the live pose extraction
uses `get_pose(frame_idx).inv()` with a fallback to the newest pose graph state.
