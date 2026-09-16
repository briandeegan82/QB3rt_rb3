#!/bin/bash
# Per-unit identity stamping over SSH (laptop side). Run AFTER a unit has been
# flashed with the golden image and is reachable on the network.
#
#   stamp/stamp_unit.sh --unit 46927088            # hostname/domain/udev
#   stamp/stamp_unit.sh --unit 46927088 --wifi     # also (re)join lab Wi-Fi
#   stamp/stamp_unit.sh --unit 46927088 --host ubuntu@192.168.0.55
#
# Reads units/<unit>.conf. Replaces the adb path of the old deploy_via_adb.sh
# with SSH. Assumes key-based SSH to the `ubuntu` user and working sudo (see
# README for the one-time NOPASSWD note if you want unattended fleet stamping).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNITS_DIR="$(cd "$HERE/../units" && pwd)"
ON_DEVICE="$HERE/_stamp_on_device.sh"

UNIT="" HOST_OVERRIDE="" DO_WIFI=0
usage() { sed -n '2,12p' "$0" | sed 's/^# \?//'; exit "${1:-0}"; }
while [ $# -gt 0 ]; do
    case "$1" in
        --unit) UNIT="$2"; shift 2 ;;
        --host) HOST_OVERRIDE="$2"; shift 2 ;;
        --wifi) DO_WIFI=1; shift ;;
        -h|--help) usage 0 ;;
        *) echo "Unknown arg: $1" >&2; usage 1 ;;
    esac
done
[ -n "$UNIT" ] || { echo "ERROR: --unit required" >&2; usage 1; }

CONF="$UNITS_DIR/$UNIT.conf"
[ -f "$CONF" ] || { echo "ERROR: no unit profile $CONF" >&2; ls "$UNITS_DIR"/*.conf; exit 1; }
# shellcheck disable=SC1090
source "$CONF"

SSH_TARGET="${HOST_OVERRIDE:-${SSH_HOST:?SSH_HOST not set in $CONF}}"
: "${HOSTNAME:?HOSTNAME not set in $CONF}" "${DOMAIN_ID:?DOMAIN_ID not set in $CONF}"

# Use the dedicated fleet key explicitly so first contact by DHCP IP (before the
# unit is named, when ~/.ssh/config's `Host qb3rt-*` can't match) still works.
FLEET_KEY="${QB3RT_SSH_KEY:-$HOME/.ssh/qb3rt_fleet}"
SSH_ID=(-o StrictHostKeyChecking=accept-new -o UserKnownHostsFile=/dev/null); [ -f "$FLEET_KEY" ] && SSH_ID+=(-i "$FLEET_KEY" -o IdentitiesOnly=yes)
SSH=(ssh "${SSH_ID[@]}" -o ConnectTimeout=10 "$SSH_TARGET")
echo "Stamping $UNIT via $SSH_TARGET  (hostname=$HOSTNAME domain=$DOMAIN_ID)"
"${SSH[@]}" true || { echo "ERROR: cannot SSH to $SSH_TARGET" >&2; exit 1; }

# --- run the on-device stamp (as root) --------------------------------------
# NB: if STATIC_IP is set, the unit switches to it ~3s after this closes, so the
# next stamp/deploy should target that IP (set SSH_HOST in the profile to match).
"${SSH[@]}" "sudo env \
    HOSTNAME_NEW=$(printf '%q' "$HOSTNAME") \
    DOMAIN_ID=$(printf '%q' "$DOMAIN_ID") \
    USB_RPLIDAR_SERIAL=$(printf '%q' "${USB_RPLIDAR_SERIAL:-}") \
    USB_WAVE_ROVER_SERIAL=$(printf '%q' "${USB_WAVE_ROVER_SERIAL:-}") \
    STATIC_IP=$(printf '%q' "${STATIC_IP:-}") \
    GATEWAY=$(printf '%q' "${GATEWAY:-}") \
    DNS=$(printf '%q' "${DNS:-}") \
    PREFIX=$(printf '%q' "${PREFIX:-}") \
    bash -s" < "$ON_DEVICE"
[ -n "${STATIC_IP:-}" ] && echo "NOTE: unit switching to ${STATIC_IP} shortly; reconnect there."

# --- optional: (re)join lab Wi-Fi -------------------------------------------
if [ "$DO_WIFI" -eq 1 ]; then
    if [ -n "${WIFI_SSID:-}" ] && [ -n "${WIFI_PSK:-}" ]; then
        echo "Joining Wi-Fi '$WIFI_SSID'..."
        "${SSH[@]}" "sudo nmcli device wifi connect $(printf '%q' "$WIFI_SSID") password $(printf '%q' "$WIFI_PSK")" \
            && echo "  connected" || echo "  WARNING: Wi-Fi join failed (check SSID/PSK/radio)." >&2
    else
        echo "  --wifi given but WIFI_SSID/WIFI_PSK empty in $CONF — skipping." >&2
    fi
fi

echo "Done. Verify:  ssh $SSH_TARGET 'ls -l /dev/rplidar /dev/wave_rover; cat /etc/qb3rt/domain_id'"
