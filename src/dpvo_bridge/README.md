# dpvo_bridge

Run [DPVO / DPV-SLAM](https://github.com/princeton-vl/DPVO) as a proper
ROS 2 system that publishes on the `vision_interfaces` contract.

What this package provides:

* `dpvo_node` - subscribes to `/camera/image_raw` + `/camera/camera_info`,
  publishes:
  * `/vision/dpvo/pose`   (`geometry_msgs/PoseStamped`)
  * `/vision/dpvo/status` (`vision_interfaces/VisionStatus`)
  * `/vision/dpvo/path`   (`nav_msgs/Path`)
  * `/vision/dpvo/reset`  (`vision_interfaces/ResetVision` service)
  * Legacy mirror: `/dpvo/pose`, `/dpvo/nav_status`, `/dpvo/path`
    (for backwards compatibility with `flight.launch.py`).
* `dpvo_trajectory_writer_node` - subscribes to `/vision/dpvo/pose` and
  `/ap/gps/fix`, writes a TUM-format trajectory + GPS sidecar CSV, and on
  shutdown renders a top-down `flight_map.png` of the whole session.
* `trajectory_logger_node` - the legacy GPS-vs-DPVO CSV writer
  (still used by `flight.launch.py`).
* `scripts/run_dpvo_node.sh` - launches `dpvo_node` from a conda env
  with both DPVO and rclpy installed.

---

## Why DPVO lives in its own Python env

DPVO requires `pytorch 2.x` + matching CUDA + custom CUDA kernels
(`altcorr`, `fastba`, `lietorch`). That stack is **not** compatible
with the system Python that ROS 2 humble uses on Ubuntu 22.04 (desktop)
out of the box - PyTorch versions clash with what ROS 2 expects.

The architecture is the same on both platforms:

* **System Python** runs ROS 2 humble + the rest of the flight stack
  (camera, tag detector, mission manager, MAVLink bridge).
* **A separate Python env** runs `dpvo_node`. The two processes talk
  over normal ROS 2 topics. From the flight stack's point of view DPVO
  is just "another node that publishes on `/vision/dpvo/*`".

The "separate env" implementation differs per platform:

| Platform                                | Env type   | rclpy source             | Helper script                  |
|-----------------------------------------|------------|--------------------------|--------------------------------|
| Desktop x86_64                          | conda      | RoboStack conda channel  | `run_dpvo_node.sh`             |
| Jetson aarch64 (JetPack 6.x, Ubuntu 22) | plain venv | system `/opt/ros/humble` | `run_dpvo_node_jetson.sh`      |

The launch picks the right script via the `DPVO_RUNTIME` env var
(`conda` default, `venv` on Jetson). Override the chosen script
explicitly with `DPVO_LAUNCHER_SCRIPT=/abs/path/to/script.sh`.

---

## One-time setup - desktop (x86_64)

```bash
# 1. DPVO env from upstream
cd ~/DPVO
conda env create -f environment.yml         # creates env `dpvo`
conda activate dpvo

# install DPVO into the env (uses thirdparty/eigen-3.4.0 if you've unzipped it)
pip install .

# 2. ROS 2 humble client libs into the same env (RoboStack)
mamba install -c conda-forge -c robostack-staging \
    ros-humble-rclpy ros-humble-sensor-msgs ros-humble-geometry-msgs \
    ros-humble-nav-msgs ros-humble-std-msgs ros-humble-rcl-interfaces

# 3. matplotlib for the PNG flight map (optional)
pip install matplotlib

# 4. Build the workspace (system Python; from a fresh shell with no conda active)
cd ~/vtol_ws
conda deactivate         # important - colcon must see system python
colcon build --packages-select vision_interfaces marker_interfaces dpvo_bridge

# Sanity check
source install/setup.bash
ros2 interface show vision_interfaces/msg/VisionStatus | head -5
```

---

## One-time setup - Jetson (aarch64, JetPack 6.2 / CUDA 12.6)

> **Do NOT use `conda env create -f environment.yml` on Jetson.** The
> DPVO env pins `pytorch-cuda=12.1` from the conda `nvidia` channel,
> which only has x86_64 / win-64 builds. On aarch64 conda silently
> falls back to **CPU-only** PyTorch, so DPVO's CUDA extensions
> (`cuda_corr`, `fastba`, `lietorch`) fail to compile and import. The
> right approach on Jetson is plain venv + NVIDIA Jetson wheel + system
> `rclpy` from `/opt/ros/humble`.

### JetPack version check

```bash
dpkg -l | grep nvidia-l4t-core
# nvidia-l4t-core 36.4.x  =>  JetPack 6.2   (CUDA 12.6)
# nvidia-l4t-core 36.3.x  =>  JetPack 6.1   (CUDA 12.6)
# nvidia-l4t-core 36.2.x  =>  JetPack 6.0   (CUDA 12.2)
```

The wheels below are for **JetPack 6.2 / CUDA 12.6**. For other JetPack
releases see <https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048>.

### Install

```bash
# 0. Drop any conda envs (auto-activated 'base' or your old 'dpvo')
conda deactivate || true
conda deactivate || true
conda env remove -n dpvo -y 2>/dev/null || true
conda config --set auto_activate_base false   # optional but recommended

# 1. System deps
sudo apt update
sudo apt install -y python3-dev python3-pip python3-venv \
    cmake build-essential git libeigen3-dev libopencv-dev \
    libsuitesparse-dev ninja-build

# 2. Isolated venv with system site-packages so /opt/ros/humble Python
#    bindings (rclpy, sensor_msgs, ...) stay importable inside it
python3 -m venv ~/venvs/dpvo --system-site-packages
source ~/venvs/dpvo/bin/activate
pip install --upgrade pip wheel setuptools

# 3. PyTorch 2.3.0 + torchvision 0.18 - precompiled for JP 6.2 / CUDA 12.6
#    (community wheels from Shattered217; mirrors NVIDIA's PyTorch for Jetson
#     thread guidance)
cd /tmp
wget https://github.com/Shattered217/Jetson-Orin-Nano-Wheels/releases/download/6.2.1rc1/torch-2.3.0a0+git97ff6cf-cp310-cp310-linux_aarch64.whl
wget https://github.com/Shattered217/Jetson-Orin-Nano-Wheels/releases/download/6.2.1rc1/torchvision-0.18.0-cp310-cp310-linux_aarch64.whl
pip install --no-cache ./torch-*.whl ./torchvision-*.whl

# Verify CUDA on Jetson:
python -c "import torch; print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_device_name(0))"
# expected: 2.3.0a0+git97ff6cf True Orin
```

Fallback if Shattered217 release is unavailable: install a newer torch
from jetson-ai-lab. This **may** break DPVO's custom CUDA extensions
(the C++ ABI shifts between PyTorch major versions), so try this only
if the 2.3 wheel above is dead:

```bash
pip install torch torchvision --index-url https://pypi.jetson-ai-lab.io/jp6/cu126
```

### Continue with DPVO

```bash
# 4. DPVO python deps (pytorch-scatter has to compile under aarch64)
pip install 'numpy<2' numba tqdm einops pypose kornia plyfile evo \
            opencv-python yacs tensorboard matplotlib
pip install torch-scatter==2.1.2 --no-build-isolation

# 5. DPVO itself (15-40 min on Jetson Orin - CUDA extensions compile)
[ -d ~/DPVO ] || git clone https://github.com/princeton-vl/DPVO.git --recursive ~/DPVO
cd ~/DPVO
[ -d thirdparty/eigen-3.4.0 ] || { \
    wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip; \
    unzip eigen-3.4.0.zip -d thirdparty; }

# If you previously tried to build under conda, clean stale artifacts:
rm -rf build dist *.egg-info dpvo/altcorr/*.so dpvo/fastba/*.so dpvo/lietorch/*.so

pip install . --no-build-isolation
./download_models_and_data.sh

# Sanity check - this is what fails if CUDA extensions weren't built:
python -c "from dpvo.dpvo import DPVO; print('DPVO import OK')"
```

### Wire DPVO into the workspace

```bash
# 6. Build the workspace (deactivate venv first - colcon must use system python
#    with system ROS 2 humble; the venv is only for the DPVO subprocess)
deactivate
cd ~/vtol_ws
colcon build --packages-select vision_interfaces marker_interfaces dpvo_bridge bringup

# 7. Tell every shell which DPVO launcher to use
cat >> ~/.bashrc <<'EOF'
export DPVO_RUNTIME=venv
export DPVO_REPO=$HOME/DPVO
export DPVO_VENV=$HOME/venvs/dpvo
EOF
source ~/.bashrc
```

### Troubleshooting

**`ModuleNotFoundError: No module named 'cuda_corr'`** - CUDA
extensions did not compile. Usually because PyTorch in the active env
is CPU-only (typical when `environment.yml` was used on aarch64), or
because old `.so` files from a previous failed build are still around.
Fix: ensure `torch.cuda.is_available()` returns `True`, clean
`build/`, `*.egg-info/`, `dpvo/*/*.so`, then `pip install . --no-build-isolation`
again.

**`pytorch-cuda` resolution error from conda** - confirms you tried
the conda path on aarch64. Stop, drop the conda env, switch to venv as
above.

---

## Running

### Standalone

Desktop:

```bash
~/vtol_ws/src/dpvo_bridge/scripts/run_dpvo_node.sh
```

Jetson:

```bash
~/vtol_ws/src/dpvo_bridge/scripts/run_dpvo_node_jetson.sh
```

Useful overrides (work on both scripts):

```bash
DPVO_WEIGHTS=~/DPVO/dpvo.pth \
DPVO_CONFIG=~/DPVO/config/default.yaml \
~/vtol_ws/src/dpvo_bridge/scripts/run_dpvo_node.sh \
    --ros-args -p enable_viz:=false -p process_every_n:=2
```

### From a launch file

`bringup/launch/flight.launch.py` already picks the right script based
on `$DPVO_RUNTIME` (`conda` by default, set to `venv` once per machine
on Jetson). Direct usage example if you want a custom launch:

```python
from launch.actions import ExecuteProcess

ExecuteProcess(
    cmd=['/home/dragon/vtol_ws/src/dpvo_bridge/scripts/run_dpvo_node_jetson.sh',
         '--ros-args',
         '-p', 'vision_pose_topic:=/vision/dpvo/pose',
         '-p', 'enable_viz:=false'],
    output='screen',
)
```

### DPVO-only diagnostic session

Use `flight.launch.py` with `navigation: dpvo` and `mode: log_only` (or
`fly_and_viz` if you want PNG plots auto-rendered):

```bash
# config/flight.yaml -> navigation: "dpvo", mode: "log_only"
ros2 launch bringup flight.launch.py
# ... fly the drone manually ...
# Ctrl+C the launch -> bag + telemetry CSV land under
# ~/vtol_ws/logs/session_<timestamp>/.  Render plots offline with:
python3 -m flight_viz --session ~/vtol_ws/logs/session_<timestamp> --format png
```

---

## Topic contract summary

See `src/vision_interfaces/README.md` for the full contract. Every
visual navigation backend (DPVO, ORB-SLAM3, VINS, global-map binder,
...) is required to publish the same family of topics under its own
`/vision/<bk>/` prefix. The flight stack only ever subscribes to the
unified `/vision_pose_enu` + `/nav/status`, which are produced by
`single_source_router_node` (single-backend launches) or
`vision_fusion_node` (hybrid launches).

This means: **adding a new VO / SLAM / VIO backend to the flight stack
is "publish to `/vision/<bk>/*` and add one entry in a launch file".**
No flight code changes.
