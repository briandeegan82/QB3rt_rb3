#!/bin/bash
# On-device class-handoff wipe. Pushed and run as root by handoff/clean_unit.sh.
# Ported from the old clean_rb3_on_device.sh: on the Ubuntu image the user is
# `ubuntu` (home /home/ubuntu), not root/`/var/roothome`. Safe to re-run.
#
# Env (optional):
#   WIFI_KEEP    comma-separated NetworkManager connection names to retain
#                (default empty = delete ALL saved Wi-Fi profiles)
#   USER_HOME    home dir to wipe project state from (default /home/ubuntu)
set -euo pipefail

WIFI_KEEP="${WIFI_KEEP:-}"
USER_HOME="${USER_HOME:-/home/ubuntu}"

echo "== Cleaning Wi-Fi (NetworkManager) =="
keep_match() {
    local name="$1" k
    [ -z "$WIFI_KEEP" ] && return 1
    IFS=',' read -r -a _keeps <<< "$WIFI_KEEP"
    for k in "${_keeps[@]}"; do
        k="${k#"${k%%[![:space:]]*}"}"; k="${k%"${k##*[![:space:]]}"}"
        [ -n "$k" ] && [ "$name" = "$k" ] && return 0
    done
    return 1
}
while IFS=: read -r name uuid type; do
    [ "$type" = "802-11-wireless" ] || continue
    if keep_match "$name"; then
        echo "  keep   $name"
    else
        echo "  delete $name"
        nmcli connection delete uuid "$uuid" >/dev/null
    fi
done < <(nmcli -t -f NAME,UUID,TYPE connection show)
nmcli connection reload >/dev/null 2>&1 || true

echo "== Cleaning ${USER_HOME} (student project + history) =="
# Wipe visible projects so the next class does not inherit prior code.
# Keep .ssh (lab keys). Preserve the profile.d ROS env (system-wide, not here).
shopt -s nullglob dotglob
for p in "$USER_HOME"/*; do
    case "$(basename "$p")" in
        .ssh) echo "  keep   $p"; continue ;;
    esac
    echo "  rm     $p"
    rm -rf "$p"
done
shopt -u nullglob dotglob

# History often contains typed wifi passwords / tokens.
if [ -f "$USER_HOME/.bash_history" ]; then
    : > "$USER_HOME/.bash_history"; echo "  cleared .bash_history"
fi
# ROS logs / bags left by previous students.
[ -d "$USER_HOME/.ros" ] && { rm -rf "$USER_HOME/.ros"; echo "  removed .ros"; }

echo "Clean done."
