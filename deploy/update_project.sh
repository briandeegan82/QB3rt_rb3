#!/bin/bash
# Push QB3rt project code to a live unit over SSH — no reflash, no rebuild.
# The frequent "layer 3" op: iterate on launch/config/scripts and update a
# running robot in seconds. Replaces the adb deploy.sh round-trip.
#
#   deploy/update_project.sh --unit 46927088
#   deploy/update_project.sh --host ubuntu@192.168.0.55
#
# Syncs the project's runtime payload into the installed overlay package
# (/opt/qb3rt/install/share/QB3rt), re-applies the wave_rover_controller vendor
# override, and refreshes the /etc system configs. RELAUNCH nodes afterward.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"          # the QB3rt/ project root
UNITS_DIR="$REPO/units"

OVERLAY="${OVERLAY:-/opt/qb3rt/install}"
DST_SHARE="$OVERLAY/share/QB3rt"
WRC_LIB="$OVERLAY/lib/wave_rover_controller"
WRC_SHARE="$OVERLAY/share/wave_rover_controller"

UNIT="" HOST_OVERRIDE=""
usage() { sed -n '2,10p' "$0" | sed 's/^# \?//'; exit "${1:-0}"; }
while [ $# -gt 0 ]; do
    case "$1" in
        --unit) UNIT="$2"; shift 2 ;;
        --host) HOST_OVERRIDE="$2"; shift 2 ;;
        -h|--help) usage 0 ;;
        *) echo "Unknown arg: $1" >&2; usage 1 ;;
    esac
done
if [ -n "$UNIT" ]; then
    CONF="$UNITS_DIR/$UNIT.conf"; [ -f "$CONF" ] || { echo "ERROR: no $CONF" >&2; exit 1; }
    # shellcheck disable=SC1090
    source "$CONF"
fi
SSH_TARGET="${HOST_OVERRIDE:-${SSH_HOST:?need --host or a unit profile with SSH_HOST}}"
# Dedicated fleet key for both ssh and rsync (works on first-contact-by-IP too).
FLEET_KEY="${QB3RT_SSH_KEY:-$HOME/.ssh/qb3rt_fleet}"
SSH_ID=(-o StrictHostKeyChecking=accept-new -o UserKnownHostsFile=/dev/null); [ -f "$FLEET_KEY" ] && SSH_ID+=(-i "$FLEET_KEY" -o IdentitiesOnly=yes)
[ -f "$FLEET_KEY" ] && export RSYNC_RSH="ssh ${SSH_ID[*]}"
SSHO=("${SSH_ID[@]}" -o ConnectTimeout=10)

echo "Updating QB3rt on $SSH_TARGET"
# QB3rt is a data-only ament package (no CMakeLists — installed by copying its
# share tree + the ament index marker, as the old deploy.sh did). Bootstrap the
# install dirs on first run so this handles both first-install and updates.
ssh "${SSHO[@]}" "$SSH_TARGET" "test -d $OVERLAY" || {
    echo "ERROR: overlay $OVERLAY missing — run reference/10_build_custom.sh first." >&2
    exit 1; }
ssh "${SSHO[@]}" "$SSH_TARGET" "mkdir -p $DST_SHARE"

# --- 1. project runtime payload -> installed share/QB3rt --------------------
# Whole-tree with --delete so files removed in the repo disappear on-device too
# (stale-launch-file drift bit the old setup; see deploy.sh history).
for d in launch config urdf scripts behavior_trees rviz docs; do
    [ -d "$REPO/$d" ] || continue
    rsync -rz --delete --exclude='__pycache__' \
        "$REPO/$d/" "$SSH_TARGET:$DST_SHARE/$d/"
done
rsync -z "$REPO/package.xml" "$SSH_TARGET:$DST_SHARE/package.xml"

# ament index marker (so `ros2 launch QB3rt ...` resolves after a bare sync).
ssh "${SSHO[@]}" "$SSH_TARGET" \
    "mkdir -p $OVERLAY/share/ament_index/resource_index/packages && \
     touch $OVERLAY/share/ament_index/resource_index/packages/QB3rt && \
     find $DST_SHARE -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null || true"

# --- 2. wave_rover_controller vendor override (calibrated bridge) ------------
OVR="$REPO/vendor_overrides/wave_rover_controller"
if [ -d "$OVR" ]; then
    ssh "${SSHO[@]}" "$SSH_TARGET" "test -d $WRC_LIB" && {
        rsync -z "$OVR/wave_rover_bridge.py"   "$SSH_TARGET:$WRC_LIB/wave_rover_bridge.py"
        ssh "${SSHO[@]}" "$SSH_TARGET" "chmod +x $WRC_LIB/wave_rover_bridge.py"
        rsync -z "$OVR/wave_rover_bridge.yaml" "$SSH_TARGET:$WRC_SHARE/config/wave_rover_bridge.yaml"
        echo "  vendor override applied"
    } || echo "  NOTE: $WRC_LIB not present — skipping vendor override."
fi

# --- 3. system configs (/etc) — need sudo -----------------------------------
SYS="$REPO/system"
if [ -d "$SYS" ]; then
    ssh "${SSHO[@]}" "$SSH_TARGET" "mkdir -p /tmp/qb3rt-sys"
    rsync -z "$SYS/cyclonedds.xml" "$SYS/98-agv-dds.conf" "$SYS/80-movidius.rules" \
        "$SYS/qb3rt-ros-env.sh" "$SSH_TARGET:/tmp/qb3rt-sys/"
    ssh "${SSHO[@]}" "$SSH_TARGET" 'sudo sh -c "
        cp /tmp/qb3rt-sys/cyclonedds.xml   /etc/qb3rt/cyclonedds.xml &&
        cp /tmp/qb3rt-sys/98-agv-dds.conf  /etc/sysctl.d/98-agv-dds.conf &&
        cp /tmp/qb3rt-sys/80-movidius.rules /etc/udev/rules.d/80-movidius.rules &&
        cp /tmp/qb3rt-sys/qb3rt-ros-env.sh /etc/profile.d/qb3rt-ros-env.sh &&
        sysctl --system >/dev/null && udevadm control --reload-rules" && rm -rf /tmp/qb3rt-sys'
    echo "  system configs refreshed"
fi

echo "Done. Relaunch the stack on the unit to pick up changes."
