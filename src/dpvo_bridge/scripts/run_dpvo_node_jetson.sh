#!/usr/bin/env bash
# Launch dpvo_node on a Jetson (JetPack 6.x / Ubuntu 22.04 / system ROS 2 humble).
#
# Differences from the desktop script (run_dpvo_node.sh):
#   * No conda. PyTorch on Jetson comes from NVIDIA's wheel index and lives
#     in a plain Python venv.
#   * rclpy / sensor_msgs / etc. come from the system /opt/ros/humble apt
#     packages, not from RoboStack. The venv is created with
#     --system-site-packages specifically so those system Python bindings
#     stay importable from inside the venv.
#   * Workspace overlay (vision_interfaces, marker_interfaces) is sourced
#     after the venv is activated.
#
# Usage:
#   run_dpvo_node_jetson.sh                       # uses defaults
#   run_dpvo_node_jetson.sh --ros-args -p ...     # forwards args to the node
#
# Override env (typically in ~/.bashrc):
#   DPVO_VENV            path to the venv.        Default: "$HOME/venvs/dpvo".
#   DPVO_REPO            path to the DPVO repo.   Default: "$HOME/DPVO".
#   DPVO_WEIGHTS         path to dpvo.pth.        Default: "$DPVO_REPO/dpvo.pth".
#   DPVO_CONFIG          path to config yaml.     Default: "$DPVO_REPO/config/default.yaml".
#   VTOL_WS_ROOT         workspace root.          Default: "$HOME/vtol_ws".
#   ROS_DISTRO           ROS 2 distro.            Default: "humble".

set -euo pipefail

: "${DPVO_VENV:=$HOME/venvs/dpvo}"
: "${DPVO_REPO:=$HOME/DPVO}"
: "${DPVO_WEIGHTS:=$DPVO_REPO/dpvo.pth}"
: "${DPVO_CONFIG:=$DPVO_REPO/config/default.yaml}"
: "${VTOL_WS_ROOT:=$HOME/vtol_ws}"
: "${ROS_DISTRO:=humble}"

if [[ ! -d "${DPVO_VENV}" ]]; then
    cat >&2 <<EOF
[run_dpvo_node_jetson] ERROR: venv '${DPVO_VENV}' does not exist.

Create it first (one-time setup; this assumes JetPack 6.2 / CUDA 12.6).
Do NOT use 'conda env create -f environment.yml' from the DPVO repo on
Jetson - it pins pytorch-cuda=12.1 from a channel that has no aarch64
build, and conda will silently install CPU-only PyTorch. CUDA extensions
(cuda_corr / fastba / lietorch) will then not compile properly.

    # 0. Make sure no conda envs are active
    conda deactivate || true
    conda deactivate || true

    sudo apt update
    sudo apt install -y python3-dev python3-pip python3-venv \\
        cmake build-essential git libeigen3-dev libopencv-dev \\
        libsuitesparse-dev ninja-build

    python3 -m venv "${DPVO_VENV}" --system-site-packages
    source "${DPVO_VENV}/bin/activate"
    pip install --upgrade pip wheel setuptools

    # 1. PyTorch + torchvision pre-built for JP 6.2 / CUDA 12.6 (community wheels)
    cd /tmp
    wget https://github.com/Shattered217/Jetson-Orin-Nano-Wheels/releases/download/6.2.1rc1/torch-2.3.0a0+git97ff6cf-cp310-cp310-linux_aarch64.whl
    wget https://github.com/Shattered217/Jetson-Orin-Nano-Wheels/releases/download/6.2.1rc1/torchvision-0.18.0-cp310-cp310-linux_aarch64.whl
    pip install --no-cache ./torch-*.whl ./torchvision-*.whl

    python -c "import torch; assert torch.cuda.is_available(), 'no CUDA!'"

    # 2. DPVO Python deps:
    pip install 'numpy<2' numba tqdm einops pypose kornia plyfile evo \\
                opencv-python yacs tensorboard matplotlib
    pip install torch-scatter==2.1.2 --no-build-isolation

    # 3. DPVO itself (15-40 min on Orin - CUDA extensions compile):
    [ -d "${DPVO_REPO}" ] || git clone https://github.com/princeton-vl/DPVO.git --recursive "${DPVO_REPO}"
    cd "${DPVO_REPO}"
    [ -d thirdparty/eigen-3.4.0 ] || { \\
        wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip; \\
        unzip eigen-3.4.0.zip -d thirdparty; }
    rm -rf build dist *.egg-info dpvo/altcorr/*.so dpvo/fastba/*.so dpvo/lietorch/*.so
    pip install . --no-build-isolation
    ./download_models_and_data.sh

See src/dpvo_bridge/README.md (section "One-time setup - Jetson") for details
and fallback wheel sources (jetson-ai-lab) if Shattered217 is unavailable.
EOF
    exit 2
fi

# Activate the venv. --system-site-packages keeps /opt/ros/humble Python
# bindings importable inside it.
# shellcheck disable=SC1091
source "${DPVO_VENV}/bin/activate"

# Source ROS 2 (system humble) and the workspace overlay so the venv
# Python sees vision_interfaces / marker_interfaces.
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
    echo "[run_dpvo_node_jetson] ERROR: /opt/ros/${ROS_DISTRO}/setup.bash not found." >&2
    exit 3
fi

if [[ -f "${VTOL_WS_ROOT}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${VTOL_WS_ROOT}/install/setup.bash"
else
    echo "[run_dpvo_node_jetson] WARN: ${VTOL_WS_ROOT}/install/setup.bash not found - " \
         "vision_interfaces will not be importable. Run colcon build first." >&2
fi

export DPVO_PYTHON_PATH="${DPVO_REPO}"
export PYTHONUNBUFFERED=1

exec python -m dpvo_bridge.dpvo_node \
    --ros-args \
        -p "weights_path:=${DPVO_WEIGHTS}" \
        -p "config_path:=${DPVO_CONFIG}" \
        "$@"
