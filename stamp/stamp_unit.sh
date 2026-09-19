#!/bin/bash
# Per-unit identity stamping over SSH (laptop side). Run AFTER a unit has been
# flashed with the golden image and is reachable on the network.
#
# Simplest (no pre-made conf needed) — derive everything from an id N:
#   stamp/stamp_unit.sh --id 16 --host ubuntu@<dhcp-ip>
#     => hostname qb3rt-16, ROS_DOMAIN_ID 16, static IP 192.168.0.16 on ALL lab
#        Wi-Fi nets; creates units/16.conf for later deploy/handoff.
# Existing-profile mode:
#   stamp/stamp_unit.sh --unit 46927088 [--host ...] [--wifi]
#
# If the profile sets WIFI_SSID+WIFI_PSK (dedicated per-unit network, e.g. a
# load-balanced fleet split across two APs), STATIC_IP is pinned to ONLY that
# SSID's connection profile (auto-created if needed) instead of every saved
# Wi-Fi net — no --wifi flag needed for that case.
#
# Sets hostname + ROS_DOMAIN_ID + /dev/rplidar,/dev/wave_rover udev + static IP.
# Assumes key-based SSH to `ubuntu` + working sudo (fleet key auto-used).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNITS_DIR="$(cd "$HERE/../units" && pwd)"
ON_DEVICE="$HERE/_stamp_on_device.sh"

UNIT="" ID="" HOST_OVERRIDE="" DO_WIFI=0
usage() { sed -n '2,18p' "$0" | sed 's/^# \?//'; exit "${1:-0}"; }
while [ $# -gt 0 ]; do
    case "$1" in
        --id)   ID="$2"; shift 2 ;;
        --unit) UNIT="$2"; shift 2 ;;
        --host) HOST_OVERRIDE="$2"; shift 2 ;;
        --wifi) DO_WIFI=1; shift ;;
        -h|--help) usage 0 ;;
        *) echo "Unknown arg: $1" >&2; usage 1 ;;
    esac
done
[ -n "$ID" ] || [ -n "$UNIT" ] || { echo "ERROR: --id N (or --unit NAME) required" >&2; usage 1; }

NAME="${UNIT:-$ID}"
CONF="$UNITS_DIR/$NAME.conf"

# --id N needs no pre-existing conf: derive identity and auto-create the conf so
# later deploy/handoff/read_serials can target it by --unit N.
if [ -n "$ID" ] && [ ! -f "$CONF" ]; then
    cat > "$CONF" <<EOF
# Auto-created by: stamp_unit.sh --id $ID
SSH_HOST=ubuntu@192.168.0.$ID
HOSTNAME=qb3rt-$ID
DOMAIN_ID=$ID
STATIC_IP=192.168.0.$ID
USB_RPLIDAR_SERIAL=
USB_WAVE_ROVER_SERIAL=
WIFI_KEEP=ros_net_5G,ROS_NET,ROS_NET_G037
EOF
    echo "created $CONF"
fi
[ -f "$CONF" ] || { echo "ERROR: no profile $CONF — use --id $NAME to create one" >&2; exit 1; }
# shellcheck disable=SC1090
source "$CONF"

# --id fills the identity (conf may still supply serials/wifi/overrides).
if [ -n "$ID" ]; then
    HOSTNAME="qb3rt-$ID"; : "${DOMAIN_ID:=$ID}" "${STATIC_IP:=192.168.0.$ID}"
fi

SSH_TARGET="${HOST_OVERRIDE:-${SSH_HOST:?SSH_HOST not set in $CONF}}"
: "${HOSTNAME:?HOSTNAME not set in $CONF}" "${DOMAIN_ID:?DOMAIN_ID not set in $CONF}"

# Use the dedicated fleet key explicitly so first contact by DHCP IP (before the
# unit is named, when ~/.ssh/config's `Host qb3rt-*` can't match) still works.
FLEET_KEY="${QB3RT_SSH_KEY:-$HOME/.ssh/qb3rt_fleet}"
SSH_ID=(-o StrictHostKeyChecking=accept-new -o UserKnownHostsFile=/dev/null); [ -f "$FLEET_KEY" ] && SSH_ID+=(-i "$FLEET_KEY" -o IdentitiesOnly=yes)
SSH=(ssh "${SSH_ID[@]}" -o ConnectTimeout=10 "$SSH_TARGET")
echo "Stamping $NAME via $SSH_TARGET  (hostname=$HOSTNAME domain=$DOMAIN_ID static=${STATIC_IP:-dhcp})"
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
    WIFI_SSID=$(printf '%q' "${WIFI_SSID:-}") \
    WIFI_PSK=$(printf '%q' "${WIFI_PSK:-}") \
    WIFI_RESERVE_SSID=$(printf '%q' "${WIFI_RESERVE_SSID:-}") \
    WIFI_RESERVE_PSK=$(printf '%q' "${WIFI_RESERVE_PSK:-}") \
    bash -s" < "$ON_DEVICE"
[ -n "${STATIC_IP:-}" ] && echo "NOTE: unit switching to ${STATIC_IP} shortly; reconnect there."

# --- optional: (re)join lab Wi-Fi -------------------------------------------
# Only needed for a plain DHCP join (no STATIC_IP): when STATIC_IP + WIFI_SSID
# are both set, the on-device stamp above already created/pinned that SSID's
# connection profile — running a DHCP join here too would create a duplicate,
# conflicting profile for the same network.
if [ "$DO_WIFI" -eq 1 ]; then
    if [ -n "${STATIC_IP:-}" ] && [ -n "${WIFI_SSID:-}" ]; then
        echo "  --wifi given but STATIC_IP+WIFI_SSID already pinned by the stamp above — skipping DHCP join."
    elif [ -n "${WIFI_SSID:-}" ] && [ -n "${WIFI_PSK:-}" ]; then
        echo "Joining Wi-Fi '$WIFI_SSID'..."
        "${SSH[@]}" "sudo nmcli device wifi connect $(printf '%q' "$WIFI_SSID") password $(printf '%q' "$WIFI_PSK")" \
            && echo "  connected" || echo "  WARNING: Wi-Fi join failed (check SSID/PSK/radio)." >&2
    else
        echo "  --wifi given but WIFI_SSID/WIFI_PSK empty in $CONF — skipping." >&2
    fi
fi

echo "Done. Verify:  ssh $SSH_TARGET 'ls -l /dev/rplidar /dev/wave_rover; cat /etc/qb3rt/domain_id'"
