#!/bin/bash
# QB3rt reference-unit build — STEP 0: base ROS + Qualcomm QIRP packages (apt).
#
# Runs ON the reference RB3 (Ubuntu 24.04, user `ubuntu` with sudo). This is
# the first of three reference/ scripts; run them in order, once, on the ONE
# unit you will capture as the golden image:
#     reference/00_apt_base.sh      # this file: apt ROS + qrb_ros_*
#     reference/10_build_custom.sh  # colcon-build the custom packages
#     reference/20_fleet_config.sh  # fleet-wide (identity-independent) config
#
# Background (why apt ROS and not RoboStack): the ORB-SLAM3 VIO path consumes
# the Qualcomm hardware nodes qrb_ros_camera (on-board OV9282 via the ISP) and
# qrb_ros_imu (on-board lsm6dst). Those ship ONLY as ros-jazzy-qrb-ros-* apt
# debs built against the system ROS in /opt/ros/jazzy — they cannot install
# into a conda/RoboStack prefix. So the base ROS here is apt ROS Jazzy plus the
# Qualcomm IoT PPAs.
#
# Idempotent: safe to re-run (apt sources are added only if missing).
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"

log() { printf '\n=== %s ===\n' "$*"; }

if [ "$(id -u)" -eq 0 ]; then
    SUDO=""
else
    SUDO="sudo"
fi

# --- sanity: this must be the Ubuntu image, not the old QIRP/Yocto one -------
if ! grep -qi ubuntu /etc/os-release 2>/dev/null; then
    echo "ERROR: /etc/os-release is not Ubuntu — are you on the new image?" >&2
    exit 1
fi
. /etc/os-release
echo "Target: ${PRETTY_NAME:-unknown}  ($(uname -m))"
if [ "$(uname -m)" != "aarch64" ]; then
    echo "WARNING: expected aarch64 (arm64); got $(uname -m). Continuing anyway." >&2
fi

# --- prerequisites for managing apt sources ---------------------------------
log "Installing apt source-management prerequisites"
$SUDO apt-get update
$SUDO apt-get install -y software-properties-common curl ca-certificates gnupg lsb-release

# --- ROS 2 apt repository (packages.ros.org) --------------------------------
# Use the official ros2-apt-source helper package so the key/repo stay current.
# Skip if a ros2 source already exists (image may ship it, or a prior run added).
# NB: check each glob independently — `ls a* b*` returns non-zero if EITHER is
# unmatched even when the other exists, which would wrongly re-add the repo.
if ! compgen -G '/etc/apt/sources.list.d/ros2*.list' >/dev/null \
   && ! compgen -G '/etc/apt/sources.list.d/ros2*.sources' >/dev/null; then
    log "Adding ROS 2 apt repository"
    $SUDO curl -fsSL -o /usr/share/keyrings/ros-archive-keyring.gpg \
        https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
    ROS_APT_VER="$(curl -fsSL "https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest" \
        | grep -oP '"tag_name":\s*"\K[^"]+' || true)"
    if [ -n "$ROS_APT_VER" ]; then
        DEB="/tmp/ros2-apt-source.deb"
        curl -fsSL -o "$DEB" \
            "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_VER}/ros2-apt-source_${ROS_APT_VER}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
        $SUDO apt-get install -y "$DEB"
        rm -f "$DEB"
    else
        # Fallback: classic sources.list entry.
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") main" \
            | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null
    fi
else
    echo "ROS 2 apt repo already present — skipping."
fi

# --- Qualcomm IoT + QIRP PPAs (qrb_ros_* hardware nodes) ---------------------
# The RB3 Gen2 Ubuntu image MAY already ship the qcom PPA enabled; add-apt-
# repository is a no-op if the source already exists, so this is safe.
log "Adding Qualcomm IoT + QIRP PPAs"
$SUDO add-apt-repository -y ppa:ubuntu-qcom-iot/qcom-ppa
$SUDO add-apt-repository -y ppa:ubuntu-qcom-iot/qirp

$SUDO apt-get update

# --- stock ROS stack --------------------------------------------------------
log "Installing stock ROS $ROS_DISTRO stack"
$SUDO apt-get install -y \
    "ros-${ROS_DISTRO}-ros-base" \
    "ros-${ROS_DISTRO}-navigation2" \
    "ros-${ROS_DISTRO}-nav2-bringup" \
    "ros-${ROS_DISTRO}-slam-toolbox" \
    "ros-${ROS_DISTRO}-rplidar-ros" \
    "ros-${ROS_DISTRO}-robot-localization" \
    "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" \
    "ros-${ROS_DISTRO}-ros2launch" \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# --- OAK-D / DepthAI (v3) from apt ------------------------------------------
# The RB3 Gen2 image's PPAs ship depthai v3 (matches the depthai-ros v3_jazzy
# line), so no source build of depthai-core/depthai-ros is needed. The driver
# metapackage pulls bridge-v3, ros-msgs-v3, and depthai-v3 (core).
log "Installing DepthAI (OAK-D) v3 from apt"
$SUDO apt-get install -y \
    "ros-${ROS_DISTRO}-depthai-ros-driver-v3" \
    || echo "WARNING: depthai v3 driver not found via apt — check 'apt search ros-${ROS_DISTRO}-depthai'." >&2

# --- Qualcomm hardware ROS nodes (QIRP) -------------------------------------
# The on-board OV9282 camera + IMU driver nodes ORB-SLAM3 depends on. Confirmed
# present on the RB3 Gen2 Ubuntu image (qrb-ros-camera 2.0.2, qrb-ros-imu 1.3.0);
# their transport-type deps (qrb-ros-transport-image/imu-type) come in as deps.
# Names can drift between QIRP releases — enumerate with:
#     apt search ros-${ROS_DISTRO}-qrb-ros
log "Installing Qualcomm qrb_ros hardware nodes"
QRB_PKGS=(
    "ros-${ROS_DISTRO}-qrb-ros-camera"
    "ros-${ROS_DISTRO}-qrb-ros-imu"
)
# Install what actually resolves; warn (don't abort) on any that don't so a
# renamed package doesn't block the whole base install.
for p in "${QRB_PKGS[@]}"; do
    if apt-cache show "$p" >/dev/null 2>&1; then
        $SUDO apt-get install -y "$p"
    else
        echo "WARNING: $p not found in apt — check 'apt search ros-${ROS_DISTRO}-qrb-ros' and update QRB_PKGS." >&2
    fi
done

# --- native build dependencies for the custom packages (step 10) ------------
log "Installing native build dependencies"
$SUDO apt-get install -y \
    build-essential cmake git ninja-build pkg-config \
    libeigen3-dev libopencv-dev \
    libboost-all-dev libssl-dev \
    libusb-1.0-0-dev \
    libgl1-mesa-dev libglew-dev   # Pangolin (ORB-SLAM3 viewer) deps

# rosdep bootstrap (used by 10_build_custom.sh to resolve package.xml deps).
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    $SUDO rosdep init || true
fi
rosdep update || true

log "STEP 0 complete"
echo "ROS $ROS_DISTRO + qrb_ros installed. Next: reference/10_build_custom.sh"
echo "Verify qrb_ros availability:  source /opt/ros/${ROS_DISTRO}/setup.bash && ros2 pkg list | grep qrb"
