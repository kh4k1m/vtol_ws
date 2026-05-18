#!/usr/bin/env bash
# Launch dpvo_node from the dpvo conda env with the workspace overlay sourced.
#
# Usage (from launch file or shell):
#   run_dpvo_node.sh                       # uses defaults
#   run_dpvo_node.sh --ros-args -p ...     # forwards args to the node
#
# Required env (override at call site as needed):
#   DPVO_CONDA_ENV       conda env name to use. Default: "dpvo".
#   DPVO_REPO            path to the DPVO repo. Default: "$HOME/DPVO".
#   DPVO_WEIGHTS         path to dpvo.pth.    Default: "$HOME/DPVO/dpvo.pth".
#   DPVO_CONFIG          path to config yaml. Default: "$HOME/DPVO/config/default.yaml".
#   VTOL_WS_ROOT         workspace root.      Default: "$HOME/vtol_ws".
#   ROS_DISTRO           ROS 2 distro.        Default: "humble".

set -euo pipefail

: "${DPVO_CONDA_ENV:=dpvo}"
: "${DPVO_REPO:=$HOME/DPVO}"
: "${DPVO_WEIGHTS:=$HOME/DPVO/dpvo.pth}"
: "${DPVO_CONFIG:=$HOME/DPVO/config/default.yaml}"
: "${VTOL_WS_ROOT:=$HOME/vtol_ws}"
: "${ROS_DISTRO:=humble}"

if ! command -v conda >/dev/null 2>&1; then
    echo "[run_dpvo_node] ERROR: conda is not on PATH. Install miniconda first." >&2
    exit 1
fi

CONDA_BASE="$(conda info --base)"
# shellcheck disable=SC1091
source "${CONDA_BASE}/etc/profile.d/conda.sh"

if ! conda env list | awk '{print $1}' | grep -qx "${DPVO_CONDA_ENV}"; then
    cat >&2 <<EOF
[run_dpvo_node] ERROR: conda env '${DPVO_CONDA_ENV}' does not exist.

Create it first:
    cd "${DPVO_REPO}"
    conda env create -f environment.yml      # creates env 'dpvo' with DPVO deps
    conda activate ${DPVO_CONDA_ENV}
    pip install .
    # add ROS 2 client libraries via robostack:
    mamba install -c conda-forge -c robostack-staging \
        ros-humble-rclpy ros-humble-sensor-msgs ros-humble-geometry-msgs \
        ros-humble-nav-msgs ros-humble-std-msgs ros-humble-rcl-interfaces
    pip install matplotlib   # for dpvo_trajectory_writer_node PNG output

See src/dpvo_bridge/README.md for the full setup recipe.
EOF
    exit 2
fi

conda activate "${DPVO_CONDA_ENV}"

# Source the workspace overlay AFTER conda activation so the conda
# Python is used but vision_interfaces / marker_interfaces are visible.
if [[ -f "${VTOL_WS_ROOT}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${VTOL_WS_ROOT}/install/setup.bash"
else
    echo "[run_dpvo_node] WARN: ${VTOL_WS_ROOT}/install/setup.bash not found - " \
         "vision_interfaces will not be importable. Run colcon build first." >&2
fi

export DPVO_PYTHON_PATH="${DPVO_REPO}"
export PYTHONUNBUFFERED=1

# Forward all CLI args to the Python entry point.
exec python -m dpvo_bridge.dpvo_node \
    --ros-args \
        -p "weights_path:=${DPVO_WEIGHTS}" \
        -p "config_path:=${DPVO_CONFIG}" \
        "$@"
