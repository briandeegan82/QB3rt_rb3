#!/bin/bash
# QB3rt reference-unit build — STEP 1: build the custom (non-apt) packages.
#
# Runs ON the reference RB3, after 00_apt_base.sh. Builds natively against apt
# ROS Jazzy and installs into a system overlay:  /opt/qb3rt/install
#
# Packages (from reference/qb3rt.repos): orb_slam3_ros, wave_rover_controller.
# DepthAI/OAK-D and the qrb_ros camera/IMU nodes come from apt (see
# 00_apt_base.sh) — they are NOT built here.
#
# This replaces the old cross-compile flow (qirp-xcompile-toolkit). All the
# Yocto/Hunter/bzip2/sysroot workarounds from those recipes are GONE — a native
# aarch64 build on Ubuntu needs none of them.
#
# Env knobs:
#   WS               workspace dir            (default: /opt/qb3rt/src_ws)
#   OVERLAY          install prefix           (default: /opt/qb3rt/install)
#   BUILD_JOBS       colcon/make parallelism  (default: nproc, capped by RAM below)
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WS="${WS:-/opt/qb3rt/src_ws}"
OVERLAY="${OVERLAY:-/opt/qb3rt/install}"
REPO_MANIFEST="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/qb3rt.repos"

log() { printf '\n=== %s ===\n' "$*"; }

if [ "$(id -u)" -eq 0 ]; then SUDO=""; else SUDO="sudo"; fi

# RAM-aware default: C++ heavy (ORB-SLAM3) can need ~1-2 GB/job.
NPROC="$(nproc)"
MEM_GB="$(awk '/MemTotal/ {printf "%d", $2/1024/1024}' /proc/meminfo)"
# ORB-SLAM3's heaviest TUs (Tracking.cc, Optimizer.cc, LocalMapping.cc) can peak
# ~3-4 GB EACH. Budget ~3 GB/job so two heavy compiles don't collide and get
# OOM-killed (that failure cost a 5.2 GB box the whole build). => 1 job here.
DEFAULT_JOBS=$(( MEM_GB/3 > 0 ? MEM_GB/3 : 1 ))
[ "$DEFAULT_JOBS" -gt "$NPROC" ] && DEFAULT_JOBS="$NPROC"
BUILD_JOBS="${BUILD_JOBS:-$DEFAULT_JOBS}"
echo "Build parallelism: ${BUILD_JOBS} jobs (nproc=${NPROC}, mem=${MEM_GB}GB)"

# --- temporary swap headroom (safety net for the peak TUs on <8 GB boards) ---
# Removed on exit; also excluded from the golden image by make_golden.sh.
SWAPFILE="/swapfile.qb3rt"
SWAP_ADDED=""
cleanup_swap() {
    [ -n "$SWAP_ADDED" ] || return 0
    $SUDO swapoff "$SWAPFILE" 2>/dev/null || true
    $SUDO rm -f "$SWAPFILE" 2>/dev/null || true
}
trap cleanup_swap EXIT
CUR_SWAP_KB="$(awk '/SwapTotal/{print $2}' /proc/meminfo)"
if [ "$MEM_GB" -lt 8 ] && [ "${CUR_SWAP_KB:-0}" -eq 0 ]; then
    SWAP_GB="${SWAP_GB:-6}"
    echo "Adding temporary ${SWAP_GB}G swap for build headroom (removed at end)"
    if $SUDO fallocate -l "${SWAP_GB}G" "$SWAPFILE" 2>/dev/null \
       || $SUDO dd if=/dev/zero of="$SWAPFILE" bs=1M count=$((SWAP_GB*1024)) status=none; then
        $SUDO chmod 600 "$SWAPFILE"
        $SUDO mkswap "$SWAPFILE" >/dev/null && $SUDO swapon "$SWAPFILE" && SWAP_ADDED=1
    fi
    [ -n "$SWAP_ADDED" ] || echo "WARNING: could not enable swap; continuing at ${BUILD_JOBS} job(s)." >&2
fi

# --- source apt ROS ---------------------------------------------------------
# ROS setup.bash references unbound vars (AMENT_TRACE_SETUP_FILES, …); relax
# nounset around it, then restore.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

$SUDO mkdir -p "$(dirname "$OVERLAY")" "$WS/src"
$SUDO chown -R "$(id -u):$(id -g)" /opt/qb3rt

# --- 1. import custom ROS packages ------------------------------------------
log "Importing custom package sources (vcs)"
if ! command -v vcs >/dev/null 2>&1; then
    $SUDO apt-get install -y python3-vcstool
fi
# Guard against the placeholder URLs shipping unedited.
if grep -q 'TODO confirm' "$REPO_MANIFEST"; then
    echo "ERROR: reference/qb3rt.repos still has 'TODO confirm' URLs." >&2
    echo "       Edit it to point at your real orb_slam3_ros / wave_rover_controller repos." >&2
    exit 1
fi
vcs import "$WS/src" < "$REPO_MANIFEST"

# vcs import does NOT fetch git submodules — orb_slam3_ros carries the
# ORB-SLAM3 core as modules/ORB_SLAM3. Pull all submodules recursively.
log "Fetching git submodules"
vcs custom "$WS/src" --git --args submodule update --init --recursive

# ORB-SLAM3 ships its vocabulary as a tarball that must be extracted before the
# colcon build (per the upstream Dockerfile). Extract any ORBvoc.txt.tar.gz in
# place if the .txt is not already present.
log "Extracting ORB-SLAM3 vocabulary (if present)"
while IFS= read -r voc; do
    d="$(dirname "$voc")"
    if [ ! -f "$d/ORBvoc.txt" ]; then
        echo "  $voc"
        tar --no-same-owner -xzf "$voc" -C "$d"
    fi
done < <(find "$WS/src" -name 'ORBvoc.txt.tar.gz' 2>/dev/null)

# --- 2. resolve remaining deps via rosdep (pulls binaries from apt) ----------
log "Resolving dependencies (rosdep)"
rosdep install --from-paths "$WS/src" --ignore-src -y -r \
    --rosdistro "$ROS_DISTRO" || \
    echo "WARNING: rosdep reported unresolved keys — review above; continuing."

# --- 3. build into the overlay ----------------------------------------------
log "colcon build -> ${OVERLAY}"
cd "$WS"
export MAKEFLAGS="-j${BUILD_JOBS}"
export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS}"
# Fail loudly on any package failure (colcon returns non-zero, but a partial
# --merge-install still writes setup.bash — so the verify below must not treat
# setup.bash's mere existence as success).
if ! colcon build \
    --merge-install \
    --install-base "$OVERLAY" \
    --executor sequential \
    --event-handlers console_direct+ \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF
then
    echo "ERROR: colcon build failed. Inspect ${WS}/log/latest_build for the failing package." >&2
    echo "  (If a compile was 'Killed', it OOM-ed — lower BUILD_JOBS or raise SWAP_GB and re-run.)" >&2
    exit 1
fi

# --- 4. verify --------------------------------------------------------------
log "Verifying overlay"
[ -f "$OVERLAY/setup.bash" ] || { echo "ERROR: no ${OVERLAY}/setup.bash" >&2; exit 1; }
set +u
# shellcheck disable=SC1090
source "$OVERLAY/setup.bash"
set -u
# Every expected package must actually be present (not just setup.bash).
EXPECT=(orb_slam3_ros orb_slam3_msgs orb_slam3_bringup wave_rover_controller)
missing=()
present="$(ros2 pkg list 2>/dev/null)"
for p in "${EXPECT[@]}"; do
    printf '%s\n' "$present" | grep -qx "$p" && echo "  OK   $p" || { echo "  MISS $p"; missing+=("$p"); }
done
if [ "${#missing[@]}" -ne 0 ]; then
    echo "ERROR: overlay missing packages: ${missing[*]}" >&2
    exit 1
fi

log "STEP 1 complete"
echo "Custom packages installed to ${OVERLAY}. Next: reference/20_fleet_config.sh"
