#!/bin/bash
# Fleet stamping status from units/*.conf (offline — no SSH). Shows which units
# are still missing USB serials (lidar/rover not yet attached) or WIFI_MAC.
#
#   stamp/status.sh
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNITS_DIR="$(cd "$HERE/../units" && pwd)"

printf '%-6s %-16s %-7s %-7s %-19s %s\n' UNIT STATIC_IP RPLIDAR ROVER WIFI_MAC STATUS
out="$(for conf in $(ls "$UNITS_DIR"/*.conf | grep -v '/_' | sort -t/ -k2 -V); do
    (
        # shellcheck disable=SC1090
        source "$conf"
        unit="$(basename "$conf" .conf)"
        lid="${USB_RPLIDAR_SERIAL:+yes}"; rov="${USB_WAVE_ROVER_SERIAL:+yes}"
        missing=()
        [ -n "$lid" ] || missing+=(rplidar)
        [ -n "$rov" ] || missing+=(rover)
        [ -n "${WIFI_MAC:-}" ] || missing+=(mac)
        if [ ${#missing[@]} -eq 0 ]; then st="complete"; else st="PARTIAL — missing: ${missing[*]}"; fi
        printf '%-6s %-16s %-7s %-7s %-19s %s\n' "$unit" "${STATIC_IP:-dhcp}" "${lid:--}" "${rov:--}" "${WIFI_MAC:--}" "$st"
    )
done)"
printf '%s\n' "$out"
n="$(grep -c PARTIAL <<<"$out")"
echo
echo "$n unit(s) partial. Finish with: stamp/read_serials.sh --unit <id> --assign --write && stamp/stamp_unit.sh --unit <id>"
