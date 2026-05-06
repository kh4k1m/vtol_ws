#!/usr/bin/env bash
# install_orb_slam3.sh
#
# One-shot installer for ORB-SLAM3 + its Pangolin dependency.
# Works on:
#   * x86_64 Ubuntu 22.04 (laptop, Gazebo)
#   * aarch64 Jetson Orin NX 16GB on JetPack 6 (Ubuntu 22.04)
#
# After successful execution, you must export ORB_SLAM3_ROOT before
# building the workspace, e.g.:
#
#   echo 'export ORB_SLAM3_ROOT="$HOME/vtol_ws/thirdparty/ORB_SLAM3"' >> ~/.bashrc
#   source ~/.bashrc
#   cd ~/vtol_ws && colcon build --packages-select orb_slam3_bridge
#
# Usage:
#   ./scripts/install_orb_slam3.sh                 # full install
#   ./scripts/install_orb_slam3.sh --reinstall     # rebuild even if libs already exist
#   ./scripts/install_orb_slam3.sh --no-pangolin   # skip Pangolin (e.g. system already has it)
#
# Notes for Jetson:
#   On JetPack 6 OpenCV is shipped with CUDA already enabled by NVIDIA.
#   We do *not* run apt-get install libopencv-dev when libopencv-dev is
#   already present, to avoid pulling the non-CUDA build over JetPack's
#   curated one.

set -euo pipefail

REINSTALL=0
SKIP_PANGOLIN=0
for arg in "$@"; do
  case "$arg" in
    --reinstall)    REINSTALL=1 ;;
    --no-pangolin)  SKIP_PANGOLIN=1 ;;
    -h|--help)
      sed -n '1,30p' "$0"
      exit 0
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      exit 1
      ;;
  esac
done

WS_DIR="${VTOL_WS_ROOT:-$HOME/vtol_ws}"
THIRDPARTY_DIR="$WS_DIR/thirdparty"
PANGOLIN_DIR="$THIRDPARTY_DIR/Pangolin"
ORB_SLAM3_DIR="$THIRDPARTY_DIR/ORB_SLAM3"

PANGOLIN_REPO="https://github.com/stevenlovegrove/Pangolin.git"
PANGOLIN_TAG="v0.8"
ORB_SLAM3_REPO="https://github.com/UZ-SLAMLab/ORB_SLAM3.git"
ORB_SLAM3_BRANCH="master"

NPROC_DEFAULT="$(nproc)"
BUILD_JOBS="${ORB_SLAM3_BUILD_JOBS:-$NPROC_DEFAULT}"

# --- helpers --------------------------------------------------------------
say()  { echo -e "\033[1;36m==> $*\033[0m"; }
warn() { echo -e "\033[1;33m!! $*\033[0m" >&2; }
err()  { echo -e "\033[1;31m** $*\033[0m" >&2; }

is_jetson() {
  if [ -f /etc/nv_tegra_release ]; then return 0; fi
  if [ -f /sys/firmware/devicetree/base/model ]; then
    grep -qi 'jetson\|nvidia' /sys/firmware/devicetree/base/model && return 0
  fi
  return 1
}

dpkg_installed() {
  dpkg -s "$1" >/dev/null 2>&1
}

# --- preflight ------------------------------------------------------------
ARCH="$(dpkg --print-architecture)"
say "Target: arch=$ARCH, jetson=$(is_jetson && echo yes || echo no), build_jobs=$BUILD_JOBS"
say "Workspace dir: $WS_DIR"
say "Thirdparty dir: $THIRDPARTY_DIR"
mkdir -p "$THIRDPARTY_DIR"

if ! grep -qi 'ubuntu' /etc/os-release; then
  warn "Non-Ubuntu OS detected. Script was tested on Ubuntu 22.04 only - proceed at your own risk."
fi

UBUNTU_VERSION="$(. /etc/os-release && echo "$VERSION_ID")"
if [ "$UBUNTU_VERSION" != "22.04" ]; then
  warn "Detected Ubuntu $UBUNTU_VERSION (expected 22.04). Continuing anyway."
fi

# --- step 1: apt dependencies --------------------------------------------
say "Installing apt dependencies"

APT_PKGS=(
  build-essential cmake git pkg-config wget ca-certificates
  libeigen3-dev
  libboost-all-dev
  libssl-dev
  libglew-dev libepoxy-dev
  libgl1-mesa-dev libglu1-mesa-dev libxkbcommon-dev
  libpython3-dev python3-numpy
)

# Only pull libopencv-dev if it's not already installed (preserve JetPack's
# CUDA-enabled OpenCV on Jetson).
if dpkg_installed libopencv-dev; then
  say "libopencv-dev already installed - skipping (good for Jetson with JetPack OpenCV)."
else
  APT_PKGS+=( libopencv-dev )
fi

sudo apt-get update
sudo apt-get install -y --no-install-recommends "${APT_PKGS[@]}"

# --- step 2: Pangolin -----------------------------------------------------
PANGOLIN_LIB="$(ldconfig -p | awk '/libpango_core/ {print $4; exit}' || true)"
if [ "$SKIP_PANGOLIN" = "1" ]; then
  say "Skipping Pangolin (per --no-pangolin)"
elif [ -n "$PANGOLIN_LIB" ] && [ "$REINSTALL" != "1" ]; then
  say "Pangolin already installed system-wide ($PANGOLIN_LIB) - skipping."
else
  if [ ! -d "$PANGOLIN_DIR" ]; then
    say "Cloning Pangolin $PANGOLIN_TAG"
    git clone --depth 1 --branch "$PANGOLIN_TAG" "$PANGOLIN_REPO" "$PANGOLIN_DIR"
  fi

  say "Building Pangolin"
  cmake -S "$PANGOLIN_DIR" -B "$PANGOLIN_DIR/build" \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_PANGOLIN_PYTHON=OFF \
        -DBUILD_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF
  cmake --build "$PANGOLIN_DIR/build" -j"$BUILD_JOBS"

  say "Installing Pangolin"
  sudo cmake --install "$PANGOLIN_DIR/build"
  sudo ldconfig
fi

# --- step 3: ORB-SLAM3 ----------------------------------------------------
if [ ! -d "$ORB_SLAM3_DIR" ]; then
  say "Cloning ORB-SLAM3"
  git clone --depth 1 --branch "$ORB_SLAM3_BRANCH" "$ORB_SLAM3_REPO" "$ORB_SLAM3_DIR"
fi

# Patches required to build UZ-SLAMLab/ORB_SLAM3 master on Ubuntu 22.04
# (newer GCC/OpenCV/Pangolin). Keep them idempotent (grep before sed).
say "Applying Ubuntu 22.04 compatibility patches (idempotent)"

# Force C++14 - master targets c++11 in places, but newer OpenCV/Pangolin
# headers need at least 14.
ORB_CMAKE="$ORB_SLAM3_DIR/CMakeLists.txt"
if [ -f "$ORB_CMAKE" ]; then
  sed -i 's/-std=c++11/-std=c++14/g' "$ORB_CMAKE" || true
fi
DBOW_CMAKE="$ORB_SLAM3_DIR/Thirdparty/DBoW2/CMakeLists.txt"
if [ -f "$DBOW_CMAKE" ]; then
  sed -i 's/-std=c++11/-std=c++14/g' "$DBOW_CMAKE" || true
fi
G2O_CMAKE="$ORB_SLAM3_DIR/Thirdparty/g2o/CMakeLists.txt"
if [ -f "$G2O_CMAKE" ]; then
  sed -i 's/-std=c++11/-std=c++14/g' "$G2O_CMAKE" || true
fi

# CameraModels.h forward-declares ostream operators with monostate types;
# old header layout breaks with newer libstdc++. Add explicit include if
# missing (no-op when the upstream patch is already merged).
SERIAL_HDR="$ORB_SLAM3_DIR/include/CameraModels/GeometricCamera.h"
if [ -f "$SERIAL_HDR" ] && ! grep -q '#include <boost/serialization/access.hpp>' "$SERIAL_HDR"; then
  sed -i '1i #include <boost/serialization/access.hpp>' "$SERIAL_HDR" || true
fi

# Bunch of files use cv::DescriptorMatcher::create("BruteForce-Hamming"),
# which OpenCV 4.5+ wants as cv::DescriptorMatcher::BRUTEFORCE_HAMMING.
# We try to be conservative and only patch when build fails - skipped for
# now, the upstream master usually compiles on 4.5+ already.

# --- step 3b: build ORB-SLAM3 -------------------------------------------
ORB_LIB="$ORB_SLAM3_DIR/lib/libORB_SLAM3.so"
if [ -f "$ORB_LIB" ] && [ "$REINSTALL" != "1" ]; then
  say "ORB-SLAM3 already built ($ORB_LIB) - skipping. Use --reinstall to rebuild."
else
  cd "$ORB_SLAM3_DIR"

  say "Building ORB-SLAM3 thirdparty (DBoW2 + g2o)"
  mkdir -p Thirdparty/DBoW2/build
  cmake -S Thirdparty/DBoW2 -B Thirdparty/DBoW2/build -DCMAKE_BUILD_TYPE=Release
  cmake --build Thirdparty/DBoW2/build -j"$BUILD_JOBS"

  mkdir -p Thirdparty/g2o/build
  cmake -S Thirdparty/g2o -B Thirdparty/g2o/build -DCMAKE_BUILD_TYPE=Release
  cmake --build Thirdparty/g2o/build -j"$BUILD_JOBS"

  say "Building Sophus (header-only, but it carries a CMake project)"
  mkdir -p Thirdparty/Sophus/build
  cmake -S Thirdparty/Sophus -B Thirdparty/Sophus/build -DCMAKE_BUILD_TYPE=Release
  cmake --build Thirdparty/Sophus/build -j"$BUILD_JOBS" || true

  say "Uncompressing ORB vocabulary"
  if [ -f "$ORB_SLAM3_DIR/Vocabulary/ORBvoc.txt.tar.gz" ] && \
     [ ! -f "$ORB_SLAM3_DIR/Vocabulary/ORBvoc.txt" ]; then
    tar -xzf "$ORB_SLAM3_DIR/Vocabulary/ORBvoc.txt.tar.gz" \
        -C "$ORB_SLAM3_DIR/Vocabulary"
  fi

  say "Building ORB-SLAM3 itself (this may take 10-20 minutes on Jetson)"
  mkdir -p build
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
  cmake --build build -j"$BUILD_JOBS"
fi

# --- step 4: sanity --------------------------------------------------------
say "Verifying ORB-SLAM3 install"
test -f "$ORB_SLAM3_DIR/lib/libORB_SLAM3.so" \
  || { err "libORB_SLAM3.so was not produced - build failed."; exit 1; }
test -f "$ORB_SLAM3_DIR/Vocabulary/ORBvoc.txt" \
  || { err "ORBvoc.txt is missing - the wrapper will refuse to start."; exit 1; }

# --- final hint ------------------------------------------------------------
cat <<EOF

\033[1;32mDone.\033[0m

Add to your shell init (~/.bashrc or similar):

    export ORB_SLAM3_ROOT="$ORB_SLAM3_DIR"

Then re-source the shell and rebuild the ROS workspace:

    source ~/.bashrc
    cd "$WS_DIR"
    colcon build --packages-select orb_slam3_bridge

If colcon prints "ORB_SLAM3_ROOT is not set", the env var did not propagate
into the build shell. Run \`echo \$ORB_SLAM3_ROOT\` in the same shell where
you invoke colcon and confirm it points at $ORB_SLAM3_DIR.
EOF
