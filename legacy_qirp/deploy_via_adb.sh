#!/bin/bash
# One-click deploy/bootstrap of QB3rt onto an RB3 over adb (USB).
#
# Run from this repo (the git root):
#
#   ./deploy_via_adb.sh --unit 46927088 --clean --bootstrap
#       Class handoff: wipe student WiFi + /root projects, then full bootstrap.
#
#   ./deploy_via_adb.sh --unit 46927088 --clean-only
#       Wipe only (no redeploy) — between classes when image is already good.
#
#   ./deploy_via_adb.sh --unit 46927088
#       Routine update: push project, run deploy.sh, reload udev/sysctl.
#
# Bootstrap needs the slim overlay tree (share/ + lib/ of custom packages).
# That tree is NOT in git (~0.5–1 GB). Obtain it with:
#   ./fetch_overlay.sh          # from GitHub Release, or
#   ./pack_overlay.sh           # from a local full share/lib dump
# See SETUP.md.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$HERE"
PKG_LIST="$HERE/bootstrap_packages.list"
UNITS_DIR="$HERE/units"
CLEAN_SCRIPT="$HERE/clean_rb3_on_device.sh"

BOOTSTRAP=0
DOMAIN_ID=""
SERIAL=""
UNIT=""
SKIP_VERIFY=0
DO_CLEAN=0
CLEAN_ONLY=0

usage() {
    sed -n '2,24p' "$0" | sed 's/^# \?//'
    exit "${1:-0}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --bootstrap) BOOTSTRAP=1; shift ;;
        --domain-id) DOMAIN_ID="$2"; shift 2 ;;
        -s|--serial) SERIAL="$2"; shift 2 ;;
        --unit) UNIT="$2"; shift 2 ;;
        --skip-verify) SKIP_VERIFY=1; shift ;;
        --clean) DO_CLEAN=1; shift ;;
        --clean-only) DO_CLEAN=1; CLEAN_ONLY=1; shift ;;
        -h|--help) usage 0 ;;
        *) echo "Unknown argument: $1" >&2; usage 1 ;;
    esac
done

# --- load unit profile -------------------------------------------------
USB_RPLIDAR_SERIAL=""
USB_WAVE_ROVER_SERIAL=""
WIFI_KEEP=""
if [ -n "$UNIT" ]; then
    UNIT_FILE=""
    for cand in "$UNITS_DIR/$UNIT.conf" "$UNITS_DIR/$UNIT"; do
        [ -f "$cand" ] && UNIT_FILE="$cand" && break
    done
    if [ -z "$UNIT_FILE" ]; then
        echo "ERROR: no unit profile units/$UNIT.conf" >&2
        echo "Available:" >&2
        ls -1 "$UNITS_DIR"/*.conf 2>/dev/null | xargs -n1 basename >&2 || true
        exit 1
    fi
    # shellcheck disable=SC1090
    source "$UNIT_FILE"
    [ -n "${ADB_SERIAL:-}" ] && [ -z "$SERIAL" ] && SERIAL="$ADB_SERIAL"
    echo "Unit profile: $UNIT_FILE"
fi

[ -z "$DOMAIN_ID" ] && DOMAIN_ID=0

ADB=(adb)
[ -n "$SERIAL" ] && ADB=(adb -s "$SERIAL")

adb_shell() { "${ADB[@]}" shell "$@"; }
adb_push()  { "${ADB[@]}" push "$@"; }

# Slim overlay install-space (share/ + lib/) used by --bootstrap.
# Prefer OVERLAY_ROOT, then ./overlay (gitignored), then legacy ../share+../lib.
resolve_overlay() {
    if [ -n "${OVERLAY_ROOT:-}" ]; then
        echo "$OVERLAY_ROOT"
        return
    fi
    if [ -d "$HERE/overlay/share" ] || [ -d "$HERE/overlay/lib" ]; then
        echo "$HERE/overlay"
        return
    fi
    if [ -d "$HERE/../share" ] && [ -d "$HERE/../lib" ]; then
        echo "$HERE/.."
        return
    fi
    echo ""
}

# --- device sanity -----------------------------------------------------
NDEVICES=$("${ADB[@]}" devices | grep -c $'\tdevice$' || true)
if [ "$NDEVICES" -eq 0 ]; then
    echo "ERROR: no adb device attached (check USB cable / 'adb devices -l')." >&2
    exit 1
elif [ "$NDEVICES" -gt 1 ] && [ -z "$SERIAL" ]; then
    echo "ERROR: multiple adb devices attached - pass -s <serial> or --unit <name>:" >&2
    "${ADB[@]}" devices -l >&2
    exit 1
fi

echo "Target device:"
adb_shell "hostname; ip link show wlan0 2>/dev/null | grep -o 'permaddr [0-9a-f:]*' || true"

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

# --- optional: class-handoff clean (WiFi + /root projects) --------------
if [ "$DO_CLEAN" -eq 1 ]; then
    if [ ! -f "$CLEAN_SCRIPT" ]; then
        echo "ERROR: missing $CLEAN_SCRIPT" >&2
        exit 1
    fi
    echo "Cleaning student state (WiFi except WIFI_KEEP=[$WIFI_KEEP], /root projects)..."
    adb_push "$CLEAN_SCRIPT" /tmp/clean_rb3_on_device.sh
    adb_shell "WIFI_KEEP=$(printf '%q' "$WIFI_KEEP") bash /tmp/clean_rb3_on_device.sh && rm -f /tmp/clean_rb3_on_device.sh"
    if [ "$CLEAN_ONLY" -eq 1 ]; then
        echo
        echo "Clean-only done. Reconnect lab WiFi on the device if needed, then redeploy."
        exit 0
    fi
fi

adb_sourced() {
    adb_shell "bash -c 'source /root/rover_env.sh >/dev/null && $*'"
}

# --- generate /root/rover_env.sh early (needed before any /usr write) ----
echo "Generating /root/rover_env.sh (ROS_DOMAIN_ID=$DOMAIN_ID)..."
sed -E "s/^export ROS_DOMAIN_ID=.*/export ROS_DOMAIN_ID=$DOMAIN_ID/" \
    "$PROJECT/QB3rt_env.sh" > "$TMP/rover_env.sh"
adb_shell "mkdir -p /root"
adb_push "$TMP/rover_env.sh" /root/rover_env.sh
adb_shell "chmod 644 /root/rover_env.sh"
echo "  /root/rover_env.sh written."

if ! adb_sourced "touch /usr/.deploy_write_test && rm -f /usr/.deploy_write_test" >/dev/null 2>&1; then
    echo "ERROR: /usr is not writable on the device after sourcing /root/rover_env.sh." >&2
    exit 1
fi
echo "/usr remounted rw (via source /root/rover_env.sh)."

RULES_SRC="$PROJECT/system/99-agv-serial.rules"
if [ -n "$USB_RPLIDAR_SERIAL" ] && [ -n "$USB_WAVE_ROVER_SERIAL" ]; then
    cat > "$TMP/99-agv-serial.rules" <<EOF
# Generated by deploy_via_adb.sh from unit profile (do not hand-edit on device).
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="$USB_RPLIDAR_SERIAL", SYMLINK+="rplidar"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="$USB_WAVE_ROVER_SERIAL", SYMLINK+="wave_rover"
EOF
    RULES_SRC="$TMP/99-agv-serial.rules"
    mkdir -p "$TMP/system_overlay"
    cp "$RULES_SRC" "$TMP/system_overlay/99-agv-serial.rules"
    echo "USB serial rules: rplidar=$USB_RPLIDAR_SERIAL wave_rover=$USB_WAVE_ROVER_SERIAL"
fi

collect_ament_paths() {
    local root="$1" pkg="$2" f idx
    for idx in packages package_run_dependencies parent_prefix_path \
               rosidl_interfaces rclcpp_components; do
        f="share/ament_index/resource_index/$idx/$pkg"
        [ -e "$root/$f" ] && printf '%s\n' "$f"
    done
    for f in "$root"/share/ament_index/resource_index/*__pluginlib__plugin/"$pkg"; do
        [ -e "$f" ] && printf '%s\n' "${f#"$root"/}"
    done
}

# --- bootstrap: restore overlay install-space --------------------------
if [ "$BOOTSTRAP" -eq 1 ]; then
    OVERLAY="$(resolve_overlay)"
    if [ -z "$OVERLAY" ]; then
        echo "ERROR: no overlay tree found for --bootstrap." >&2
        echo "  Run:  ./fetch_overlay.sh   # download GitHub Release asset" >&2
        echo "  or:   ./pack_overlay.sh    # build ./overlay from a local dump" >&2
        echo "  or set OVERLAY_ROOT=/path/to/dir containing share/ and lib/" >&2
        exit 1
    fi
    echo "Overlay root: $OVERLAY"
    if [ ! -f "$PKG_LIST" ]; then
        echo "ERROR: missing $PKG_LIST" >&2
        exit 1
    fi
    mapfile -t BOOT_PKGS < <(grep -vE '^\s*(#|$)' "$PKG_LIST")
    BOOT_PATHS=()
    for pkg in "${BOOT_PKGS[@]}"; do
        [ -d "$OVERLAY/share/$pkg" ] && BOOT_PATHS+=("share/$pkg")
        [ -d "$OVERLAY/lib/$pkg" ] && BOOT_PATHS+=("lib/$pkg")
        while IFS= read -r p; do BOOT_PATHS+=("$p"); done < <(collect_ament_paths "$OVERLAY" "$pkg")
    done
    # Optional second copy of ORBvoc (prefer share/orb_slam3_ros/Vocabulary).
    [ -d "$OVERLAY/share/orb_slam3" ] && BOOT_PATHS+=("share/orb_slam3")

    while IFS= read -r -d '' so; do
        BOOT_PATHS+=("${so#"$OVERLAY"/}")
    done < <(find "$OVERLAY/lib" -maxdepth 1 -type f -name '*.so' -print0 2>/dev/null)

    for py in orb_slam3_msgs slam_toolbox; do
        [ -d "$OVERLAY/lib/python3.12/site-packages/$py" ] && \
            BOOT_PATHS+=("lib/python3.12/site-packages/$py")
    done

    if [ ${#BOOT_PATHS[@]} -eq 0 ]; then
        echo "ERROR: bootstrap path list empty — check overlay share/ lib/." >&2
        exit 1
    fi

    echo "Bootstrapping ${#BOOT_PKGS[@]} packages (${#BOOT_PATHS[@]} paths)..."
    for pkg in "${BOOT_PKGS[@]}"; do
        if [ ! -d "$OVERLAY/share/$pkg" ]; then
            echo "WARNING: share/$pkg missing from overlay." >&2
        fi
    done
    if [ ! -x "$OVERLAY/lib/orb_slam3_ros/orb_slam3_ros_mono_imu" ]; then
        echo "WARNING: lib/orb_slam3_ros/orb_slam3_ros_mono_imu missing." >&2
    fi
    if [ ! -x "$OVERLAY/lib/slam_toolbox/async_slam_toolbox_node" ]; then
        echo "WARNING: lib/slam_toolbox/async_slam_toolbox_node missing." >&2
    fi

    tar -C "$OVERLAY" -czf "$TMP/bootstrap.tar.gz" \
        --exclude='__pycache__' --exclude='*.bak.*' \
        "${BOOT_PATHS[@]}"
    adb_push "$TMP/bootstrap.tar.gz" /tmp/qb3rt_bootstrap.tar.gz
    adb_sourced "tar -C /usr -xzf /tmp/qb3rt_bootstrap.tar.gz && rm -f /tmp/qb3rt_bootstrap.tar.gz"
    echo "  bootstrap extract done."
fi

# --- push project source to /root/QB3rt --------------------------------
echo "Pushing project source -> /root/QB3rt..."
mkdir -p "$TMP/proj/QB3rt"
# Repo IS the project; exclude overlay cache, git metadata, local tarballs.
if command -v rsync >/dev/null 2>&1; then
    rsync -a \
        --exclude='.git/' \
        --exclude='overlay/' \
        --exclude='__pycache__/' \
        --exclude='results/' \
        --exclude='*.tar.gz' \
        --exclude='.deploy_write_test' \
        "$HERE"/ "$TMP/proj/QB3rt/"
else
    tar -C "$HERE" -cf - \
        --exclude='.git' --exclude='overlay' --exclude='__pycache__' \
        --exclude='results' --exclude='*.tar.gz' \
        . | tar -C "$TMP/proj/QB3rt" -xf -
fi
if [ -d "$TMP/system_overlay" ]; then
    cp "$TMP/system_overlay/99-agv-serial.rules" "$TMP/proj/QB3rt/system/"
fi
tar -C "$TMP/proj" -czf "$TMP/project.tar.gz" QB3rt
adb_push "$TMP/project.tar.gz" /tmp/qb3rt_project.tar.gz
adb_shell "tar -C /root -xzf /tmp/qb3rt_project.tar.gz && rm -f /tmp/qb3rt_project.tar.gz"
echo "  /root/QB3rt updated."

echo "Running /root/QB3rt/deploy.sh on-device..."
adb_sourced "bash /root/QB3rt/deploy.sh"

adb_shell "sysctl --system >/dev/null 2>&1; udevadm control --reload-rules; udevadm trigger -c add -s tty" || true

if [ "$SKIP_VERIFY" -eq 0 ]; then
    echo "Verifying deploy..."
    VERIFY_PKGS="QB3rt wave_rover_controller orb_slam3_ros"
    [ "$BOOTSTRAP" -eq 1 ] && VERIFY_PKGS="$VERIFY_PKGS slam_toolbox orb_slam3_msgs"
    FAIL=0
    for pkg in $VERIFY_PKGS; do
        if adb_sourced "ros2 pkg prefix $pkg" >/dev/null 2>&1; then
            echo "  OK  ros2 pkg prefix $pkg"
        else
            echo "  FAIL ros2 pkg prefix $pkg" >&2
            FAIL=1
        fi
    done
    if adb_shell "test -e /dev/wave_rover && test -e /dev/rplidar"; then
        echo "  OK  /dev/wave_rover + /dev/rplidar"
    else
        echo "  WARN /dev/wave_rover or /dev/rplidar missing (plug USB adapters / check unit USB serials)." >&2
    fi
    if adb_shell "test -x /usr/lib/orb_slam3_ros/orb_slam3_ros_mono_imu"; then
        echo "  OK  orb_slam3_ros_mono_imu"
    else
        echo "  FAIL orb_slam3_ros_mono_imu missing (re-run with --bootstrap after fetch_overlay)." >&2
        FAIL=1
    fi
    if [ "$BOOTSTRAP" -eq 1 ]; then
        if adb_shell "test -x /usr/lib/slam_toolbox/async_slam_toolbox_node"; then
            echo "  OK  async_slam_toolbox_node"
        else
            echo "  FAIL async_slam_toolbox_node missing." >&2
            FAIL=1
        fi
    fi
    if [ "$FAIL" -ne 0 ]; then
        echo "ERROR: verify failed." >&2
        exit 1
    fi
fi

echo
echo "Done."
echo "Next: adb shell -> 'source /root/rover_env.sh' ->"
echo "      ros2 launch QB3rt full_stack.launch.py enable_nav:=false"
[ "$BOOTSTRAP" -eq 0 ] && echo "(First time / after reflash? ./fetch_overlay.sh && re-run with --bootstrap.)"
