#!/bin/bash
# Discover QB3rt fleet units on the local subnet (laptop side).
# Scans for live hosts, probes each with the dedicated fleet key, and reports
# which are freshly-flashed (hostname qb3rt-unconfigured / domain 0) vs already
# stamped. Useful for first contact when mDNS (.local) is unreliable.
#
#   stamp/discover.sh                        # scan 192.168.0.0/24 (default)
#   stamp/discover.sh --subnet 10.42.0.0/24
set -uo pipefail

SUBNET="192.168.0.0/24"
while [ $# -gt 0 ]; do
    case "$1" in
        --subnet) SUBNET="$2"; shift 2 ;;
        -h|--help) sed -n '2,11p' "$0" | sed 's/^# \?//'; exit 0 ;;
        *) echo "Unknown arg: $1" >&2; exit 1 ;;
    esac
done

FLEET_KEY="${QB3RT_SSH_KEY:-$HOME/.ssh/qb3rt_fleet}"
[ -f "$FLEET_KEY" ] || { echo "ERROR: fleet key $FLEET_KEY not found (set QB3RT_SSH_KEY)." >&2; exit 1; }
BASE="${SUBNET%.*}"                       # 192.168.0  (assumes a /24)

echo "Scanning ${BASE}.0/24 for live hosts..."
hosts=()
if command -v nmap >/dev/null 2>&1; then
    mapfile -t hosts < <(nmap -sn -n --host-timeout 3s "${BASE}.0/24" -oG - 2>/dev/null | awk '/Status: Up/{print $2}')
else
    # Dependency-free: parallel ping sweep populates the neighbour table.
    for i in $(seq 1 254); do ping -c1 -W1 "${BASE}.$i" >/dev/null 2>&1 & done; wait
    mapfile -t hosts < <(ip neigh show 2>/dev/null | awk -v b="${BASE}." 'index($1,b)==1 && $0 !~ /FAILED|INCOMPLETE/{print $1}' | sort -u -t. -k4 -n)
fi
echo "  ${#hosts[@]} live host(s); probing with the fleet key (only our units respond)..."

probe() {  # prints "ip\thostname\tdomain" for a QB3rt unit; nothing otherwise
    local ip="$1" out
    out="$(ssh -i "$FLEET_KEY" -o IdentitiesOnly=yes -o BatchMode=yes \
              -o ConnectTimeout=4 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null \
              ubuntu@"$ip" \
              'test -f /etc/qb3rt/domain_id && printf "%s|%s" "$(hostname)" "$(cat /etc/qb3rt/domain_id)"' 2>/dev/null)" || return 0
    [ -n "$out" ] && printf '%s\t%s\t%s\n' "$ip" "${out%%|*}" "${out##*|}"
}

tmp="$(mktemp)"
for ip in "${hosts[@]}"; do probe "$ip" >> "$tmp" & done
wait

printf '\n%-16s %-22s %-7s %s\n' "IP" "HOSTNAME" "DOMAIN" "STATUS"
printf '%-16s %-22s %-7s %s\n' "----------------" "----------------------" "-------" "-------------------"
n=0
while IFS=$'\t' read -r ip host dom; do
    [ -n "$ip" ] || continue
    n=$((n+1))
    if [ "$host" = "qb3rt-unconfigured" ] || [ "${dom:-0}" = "0" ]; then
        st="FRESH — needs stamp"
    else
        st="stamped"
    fi
    printf '%-16s %-22s %-7s %s\n' "$ip" "$host" "${dom:-0}" "$st"
done < <(sort -t. -k4 -n "$tmp")
rm -f "$tmp"
echo
echo "$n QB3rt unit(s) found. First-stamp a FRESH one with:"
echo "  stamp/stamp_unit.sh --unit <id> --host ubuntu@<ip> --wifi"
