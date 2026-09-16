#!/bin/bash
# Read CP2102N USB-serial adapter serials from a unit and map them to the
# /dev/rplidar and /dev/wave_rover roles. Both adapters are Silicon Labs CP2102N
# (10c4:ea60), so the burned-in serial is the only stable discriminator.
#
#   stamp/read_serials.sh --unit 16                    # list attached adapters + serials
#   stamp/read_serials.sh --unit 16 --assign           # guided (unplug-to-identify) mapping
#   stamp/read_serials.sh --unit 16 --assign --write   # also write USB_* into units/16.conf
#   stamp/read_serials.sh --host ubuntu@<ip> [--assign] # target by IP instead of a profile
#
# After --write, run:  stamp/stamp_unit.sh --unit <id>   to install the udev rule.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNITS_DIR="$(cd "$HERE/../units" && pwd)"

UNIT="" HOST_OVERRIDE="" ASSIGN=0 WRITE=0
usage() { sed -n '2,12p' "$0" | sed 's/^# \?//'; exit "${1:-0}"; }
while [ $# -gt 0 ]; do
    case "$1" in
        --unit) UNIT="$2"; shift 2 ;;
        --host) HOST_OVERRIDE="$2"; shift 2 ;;
        --assign) ASSIGN=1; shift ;;
        --write) WRITE=1; shift ;;
        -h|--help) usage 0 ;;
        *) echo "Unknown arg: $1" >&2; usage 1 ;;
    esac
done

CONF=""
if [ -n "$UNIT" ]; then
    CONF="$UNITS_DIR/$UNIT.conf"
    [ -f "$CONF" ] || { echo "ERROR: no unit profile $CONF" >&2; exit 1; }
    # shellcheck disable=SC1090
    source "$CONF"
fi
SSH_TARGET="${HOST_OVERRIDE:-${SSH_HOST:-}}"
[ -n "$SSH_TARGET" ] || { echo "ERROR: need --host or a unit profile with SSH_HOST" >&2; exit 1; }
[ "$WRITE" -eq 1 ] && [ -z "$CONF" ] && { echo "ERROR: --write requires --unit (a profile to write to)" >&2; exit 1; }

FLEET_KEY="${QB3RT_SSH_KEY:-$HOME/.ssh/qb3rt_fleet}"
SSH_ID=(-o StrictHostKeyChecking=accept-new -o UserKnownHostsFile=/dev/null); [ -f "$FLEET_KEY" ] && SSH_ID+=(-i "$FLEET_KEY" -o IdentitiesOnly=yes)
SSH=(ssh "${SSH_ID[@]}" -o BatchMode=yes -o ConnectTimeout=10 "$SSH_TARGET")

# Remote: print "dev<TAB>serial" for each attached CP2102N (10c4:ea60) tty.
REMOTE_LIST='
for d in /dev/ttyUSB*; do
  [ -e "$d" ] || continue
  eval "$(udevadm info -q property --export -n "$d" 2>/dev/null | grep -E "^ID_(VENDOR_ID|MODEL_ID|SERIAL_SHORT)=")"
  [ "${ID_VENDOR_ID:-}" = "10c4" ] && [ "${ID_MODEL_ID:-}" = "ea60" ] && printf "%s\t%s\n" "$d" "${ID_SERIAL_SHORT:-}"
  unset ID_VENDOR_ID ID_MODEL_ID ID_SERIAL_SHORT
done'

list_serials() { "${SSH[@]}" "bash -c '$REMOTE_LIST'" 2>/dev/null; }

# Read serials only (sorted, unique) into a newline list.
serials_only() { list_serials | awk -F'\t' 'NF{print $2}' | sort -u; }

echo "Reading CP2102N adapters on $SSH_TARGET ..."
"${SSH[@]}" true 2>/dev/null || { echo "ERROR: cannot SSH to $SSH_TARGET" >&2; exit 1; }

mapfile -t ROWS < <(list_serials)
if [ "${#ROWS[@]}" -eq 0 ]; then
    echo "  No CP2102N (10c4:ea60) adapters attached. Plug in the lidar + rover and retry." >&2
    exit 1
fi
echo "Attached adapters:"
printf '  %s\n' "${ROWS[@]}" | sed 's/\t/  serial=/'

RPLIDAR="" WAVE_ROVER=""

if [ "$ASSIGN" -eq 1 ]; then
    echo
    echo "Guided mapping (both chips are identical, so identify by unplugging):"
    mapfile -t BEFORE < <(serials_only)
    if [ "${#BEFORE[@]}" -lt 2 ]; then
        echo "  Need both adapters plugged in to auto-map (found ${#BEFORE[@]}). Aborting --assign." >&2
        exit 1
    fi
    printf '  >>> Unplug the RPLIDAR adapter now, then press Enter... '
    read -r _ < /dev/tty
    sleep 1
    mapfile -t AFTER < <(serials_only)
    # The serial present BEFORE but absent AFTER is the RPLIDAR.
    RPLIDAR="$(comm -23 <(printf '%s\n' "${BEFORE[@]}") <(printf '%s\n' "${AFTER[@]}") | head -1)"
    if [ -z "$RPLIDAR" ]; then
        echo "  Could not detect a removed adapter — did the RPLIDAR unplug? Aborting." >&2
        exit 1
    fi
    # The remaining serial (BEFORE minus the rplidar) is the wave_rover (assumes 2 adapters).
    WAVE_ROVER="$(printf '%s\n' "${BEFORE[@]}" | grep -vx "$RPLIDAR" | head -1)"
    printf '  Detected  rplidar=%s  wave_rover=%s\n' "$RPLIDAR" "$WAVE_ROVER"
    printf '  >>> Plug the RPLIDAR back in, then press Enter... '
    read -r _ < /dev/tty
fi

echo
echo "# Paste into units/<id>.conf:"
echo "USB_RPLIDAR_SERIAL=${RPLIDAR}"
echo "USB_WAVE_ROVER_SERIAL=${WAVE_ROVER}"

if [ "$WRITE" -eq 1 ]; then
    [ -n "$RPLIDAR" ] && [ -n "$WAVE_ROVER" ] || { echo "ERROR: --write needs both serials (use --assign)." >&2; exit 1; }
    sed -i -E "s|^USB_RPLIDAR_SERIAL=.*|USB_RPLIDAR_SERIAL=${RPLIDAR}|" "$CONF"
    sed -i -E "s|^USB_WAVE_ROVER_SERIAL=.*|USB_WAVE_ROVER_SERIAL=${WAVE_ROVER}|" "$CONF"
    echo
    echo "Wrote serials into $CONF. Next: stamp/stamp_unit.sh --unit $UNIT"
fi
