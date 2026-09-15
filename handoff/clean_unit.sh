#!/bin/bash
# Class-handoff wipe over SSH (laptop side). Deletes student Wi-Fi profiles
# (except WIFI_KEEP) and home-dir project state, keeping .ssh lab keys.
#
#   handoff/clean_unit.sh --unit 46927088
#   handoff/clean_unit.sh --host ubuntu@192.168.0.55 --keep ros_net_5G,ROS_NET
#
# Replaces `deploy_via_adb.sh --clean` (adb -> SSH).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNITS_DIR="$(cd "$HERE/../units" && pwd)"
ON_DEVICE="$HERE/_clean_on_device.sh"

UNIT="" HOST_OVERRIDE="" KEEP_OVERRIDE=""
usage() { sed -n '2,9p' "$0" | sed 's/^# \?//'; exit "${1:-0}"; }
while [ $# -gt 0 ]; do
    case "$1" in
        --unit) UNIT="$2"; shift 2 ;;
        --host) HOST_OVERRIDE="$2"; shift 2 ;;
        --keep) KEEP_OVERRIDE="$2"; shift 2 ;;
        -h|--help) usage 0 ;;
        *) echo "Unknown arg: $1" >&2; usage 1 ;;
    esac
done

if [ -n "$UNIT" ]; then
    CONF="$UNITS_DIR/$UNIT.conf"
    [ -f "$CONF" ] || { echo "ERROR: no unit profile $CONF" >&2; exit 1; }
    # shellcheck disable=SC1090
    source "$CONF"
fi
SSH_TARGET="${HOST_OVERRIDE:-${SSH_HOST:?need --host or a unit profile with SSH_HOST}}"
WIFI_KEEP="${KEEP_OVERRIDE:-${WIFI_KEEP:-}}"

FLEET_KEY="${QB3RT_SSH_KEY:-$HOME/.ssh/qb3rt_fleet}"
SSH_ID=(); [ -f "$FLEET_KEY" ] && SSH_ID=(-i "$FLEET_KEY" -o IdentitiesOnly=yes)

echo "Cleaning $SSH_TARGET  (keep Wi-Fi: [${WIFI_KEEP}])"
ssh "${SSH_ID[@]}" -o ConnectTimeout=10 "$SSH_TARGET" \
    "sudo env WIFI_KEEP=$(printf '%q' "$WIFI_KEEP") bash -s" < "$ON_DEVICE"
echo "Done. Reconnect lab Wi-Fi on the unit if it was not in the keep-list."
