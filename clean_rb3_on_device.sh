#!/bin/bash
# On-device student handoff wipe. Invoked by deploy_via_adb.sh --clean (pushed
# over adb and run as root). Safe to re-run.
#
# Env (optional):
#   WIFI_KEEP   comma-separated NetworkManager connection names to retain
#               (default empty = delete ALL saved WiFi profiles)
set -euo pipefail

WIFI_KEEP="${WIFI_KEEP:-}"

echo "== Cleaning WiFi (NetworkManager) =="
# Delete every 802-11-wireless connection not in WIFI_KEEP. Prefer nmcli so
# NM's runtime state stays consistent (never print psk= / passwords).
keep_match() {
    local name="$1" k
    [ -z "$WIFI_KEEP" ] && return 1
    IFS=',' read -r -a _keeps <<< "$WIFI_KEEP"
    for k in "${_keeps[@]}"; do
        k="${k#"${k%%[![:space:]]*}"}"
        k="${k%"${k##*[![:space:]]}"}"
        [ -n "$k" ] && [ "$name" = "$k" ] && return 0
    done
    return 1
}

while IFS=: read -r name uuid type; do
    [ "$type" = "802-11-wireless" ] || continue
    if keep_match "$name"; then
        echo "  keep  $name"
    else
        echo "  delete $name"
        nmcli connection delete uuid "$uuid" >/dev/null
    fi
done < <(nmcli -t -f NAME,UUID,TYPE connection show)
nmcli connection reload >/dev/null 2>&1 || true

echo "== Cleaning /root (student project + history) =="
# /root -> /var/roothome on these units; wipe visible projects so the next
# class does not inherit prior code. Keep .ssh (lab keys) and .local.
ROOT_HOME=/var/roothome
[ -d "$ROOT_HOME" ] || ROOT_HOME=/root

shopt -s nullglob
for p in "$ROOT_HOME"/*; do
    echo "  rm  $p"
    rm -rf "$p"
done
shopt -u nullglob

# History often contains typed wifi passwords / tokens.
if [ -f "$ROOT_HOME/.bash_history" ]; then
    : > "$ROOT_HOME/.bash_history"
    echo "  cleared .bash_history"
fi
# ROS logs / bags left by previous students.
if [ -d "$ROOT_HOME/.ros" ]; then
    rm -rf "$ROOT_HOME/.ros"
    echo "  removed .ros"
fi

echo "Clean done."
