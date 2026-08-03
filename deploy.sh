#!/bin/bash
# Deploy the QB3rt project from the canonical workspace (/root/QB3rt) into the
# ROS install space (/usr/share/QB3rt + /usr/lib/wave_rover_controller).
#
# /root/QB3rt is the PROJECT - edit here; it persists across ostree reflashes.
# /usr/share/QB3rt is the INSTALL - running nodes load from it; never edit it
# directly (that is exactly the bidirectional drift that kept biting us).
#
# Runs either ON THE ROBOT:
#     bash /root/QB3rt/deploy.sh
# or ON THE LAPTOP against the sshfs mount:
#     bash ~/mnt/rb3/root/QB3rt/deploy.sh
#
# /usr is a read-only ostree mount. Remounting it read-write requires a shell
# ON THE DEVICE (the laptop sshfs write shows the same EPERM until then):
#     source /root/rover_env.sh      # does: mount -o remount,rw /usr
#
# After deploying, RELAUNCH the affected nodes - they load the installed copies.
set -e

# Locate the tree root: on the laptop everything lives under the sshfs mount.
if [ -d /home/brian/mnt/rb3/usr/share/QB3rt ]; then
    ROOT=/home/brian/mnt/rb3
else
    ROOT=""
fi

SRC="$ROOT/root/QB3rt"
DST="$ROOT/usr/share/QB3rt"
WRC_LIB="$ROOT/usr/lib/wave_rover_controller"
WRC_SHARE="$ROOT/usr/share/wave_rover_controller"

# Writability preflight (clearer than a mid-copy EPERM).
if ! touch "$DST/.deploy_write_test" 2>/dev/null; then
    echo "ERROR: $DST is not writable." >&2
    echo "/usr is a read-only ostree mount. In a shell ON THE DEVICE run:" >&2
    echo "    source /root/rover_env.sh" >&2
    echo "then re-run this script." >&2
    exit 1
fi
rm -f "$DST/.deploy_write_test"

# Ensure ament can find the packages. ros2 looks up
# share/ament_index/resource_index/packages/<name> — the share/<name> tree
# alone is not enough (seen after a bootstrap that omitted the index).
AMENT_PKGS="$ROOT/usr/share/ament_index/resource_index/packages"
mkdir -p "$AMENT_PKGS"
for pkg in QB3rt wave_rover_controller orb_slam3_ros orb_slam3_msgs slam_toolbox; do
    [ -e "$AMENT_PKGS/$pkg" ] || touch "$AMENT_PKGS/$pkg"
done

copy_tree() {  # copy_tree <src_dir> <dst_dir>  (whole-tree, skips __pycache__)
    local src="$1" dst="$2"
    mkdir -p "$dst"
    if command -v rsync >/dev/null 2>&1; then
        # --delete: files removed/renamed in the workspace disappear from the
        # install too. Without it, deleted launch/config files lingered in
        # /usr/share forever and could still be launched (audit 2026-07-16).
        rsync -r --delete --exclude='__pycache__' "$src/" "$dst/"
    else
        (cd "$src" && find . -name __pycache__ -prune -o -type f -print) | \
        while IFS= read -r f; do
            mkdir -p "$dst/$(dirname "$f")"
            cp "$src/$f" "$dst/$f"
        done
    fi
    echo "  $src -> $dst"
}

echo "Deploying QB3rt project -> install space"

# The whole package payload, not a hand-maintained file list (stale-list drift
# is how the calibrated bridge got lost once already). 'results' and
# 'vendor_overrides' and 'laptop' are workspace-only on purpose.
for d in launch config urdf scripts behavior_trees rviz docs; do
    [ -d "$SRC/$d" ] && copy_tree "$SRC/$d" "$DST/$d"
done
cp "$SRC/package.xml" "$DST/package.xml"
echo "  package.xml"

# Stale-bytecode guard: a launch .pyc newer than its .py wins with some
# interpreters; nuke the install-side caches.
find "$DST" -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null || true

# Vendor overrides: the wave_rover_controller bridge + calibrated config.
# The vendor package has no source on the robot, so a reflash reverts it -
# re-running this deploy restores the calibrated controller.
OVR="$SRC/vendor_overrides/wave_rover_controller"
if [ -d "$OVR" ]; then
    cp "$OVR/wave_rover_bridge.py" "$WRC_LIB/wave_rover_bridge.py"
    chmod +x "$WRC_LIB/wave_rover_bridge.py"
    cp "$OVR/wave_rover_bridge.yaml" "$WRC_SHARE/config/wave_rover_bridge.yaml"
    echo "  vendor override: wave_rover_bridge.py + wave_rover_bridge.yaml"
fi

# System configs (system/): hand-installed once, they used to live ONLY in
# /opt and /etc and were lost on reflash (audit 2026-07-16). The project copy
# is canonical; deploy re-installs them. After a reflash re-run this deploy,
# then `sysctl --system` and `udevadm control --reload` (or just reboot).
SYS="$SRC/system"
if [ -d "$SYS" ]; then
    cp "$SYS/cyclonedds.xml" "$ROOT/opt/cyclonedds.xml"
    mkdir -p "$ROOT/etc/sysctl.d" "$ROOT/etc/udev/rules.d"
    cp "$SYS/98-agv-dds.conf" "$ROOT/etc/sysctl.d/98-agv-dds.conf"
    cp "$SYS/99-agv-serial.rules" "$ROOT/etc/udev/rules.d/99-agv-serial.rules"
    echo "  system configs: cyclonedds.xml + sysctl DDS buffers + udev serial rules"
fi

echo "Done. Relaunch the stack to pick up the deployed files."
