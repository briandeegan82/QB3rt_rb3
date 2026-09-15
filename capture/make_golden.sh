#!/bin/bash
# Capture a flashable golden rootfs image from the configured reference RB3.
#
# Runs ON the reference unit (needs root). Produces an EXT4 image you flash to
# every other unit via fastboot; per-unit identity is then applied with
# stamp/stamp_unit.sh. Identity is reset in the image COPY, so the reference
# unit itself is left untouched and still usable.
#
#   sudo capture/make_golden.sh --out /media/usb/qb3rt-golden.img
#   sudo capture/make_golden.sh --out ... --size 24G      # force image size
#
# IMPORTANT — write the image to EXTERNAL storage (USB stick / NVMe), NOT the
# rootfs you are copying. Default out dir is /media if you don't pass --out.
set -euo pipefail

OUT=""
SIZE=""                       # e.g. 24G; default computed from used space +35%
AUTH_KEYS=""                  # optional: replace image's ubuntu authorized_keys with this file
KEEP_WIFI=""                  # optional: comma-list of NM connection names to KEEP (else wipe all)
MNT="/mnt/qb3rt-golden"
SRC="/"

usage() { sed -n '2,14p' "$0" | sed 's/^# \?//'; exit "${1:-0}"; }
while [ $# -gt 0 ]; do
    case "$1" in
        --out) OUT="$2"; shift 2 ;;
        --size) SIZE="$2"; shift 2 ;;
        --authorized-keys) AUTH_KEYS="$2"; shift 2 ;;
        --keep-wifi) KEEP_WIFI="$2"; shift 2 ;;
        -h|--help) usage 0 ;;
        *) echo "Unknown arg: $1" >&2; usage 1 ;;
    esac
done
[ -z "$AUTH_KEYS" ] || [ -f "$AUTH_KEYS" ] || { echo "ERROR: --authorized-keys file not found: $AUTH_KEYS" >&2; exit 1; }

[ "$(id -u)" -eq 0 ] || { echo "ERROR: run with sudo." >&2; exit 1; }
[ -n "$OUT" ] || { echo "ERROR: --out <path> required (put it on EXTERNAL storage)." >&2; exit 1; }

# Refuse to write the image onto the filesystem we're about to copy.
OUT_DEV="$(df --output=source "$(dirname "$OUT")" | tail -1)"
ROOT_DEV="$(df --output=source / | tail -1)"
if [ "$OUT_DEV" = "$ROOT_DEV" ]; then
    echo "ERROR: --out is on the rootfs ($ROOT_DEV). Use an external disk." >&2
    exit 1
fi

# --- size the image ---------------------------------------------------------
if [ -z "$SIZE" ]; then
    USED_KB="$(df --output=used / | tail -1)"
    SIZE_MB=$(( USED_KB/1024 * 135 / 100 ))     # used + 35% headroom
    SIZE="${SIZE_MB}M"
fi
echo "Creating ${SIZE} EXT4 image at ${OUT}"

# --- create + mount the empty image -----------------------------------------
rm -f "$OUT"
truncate -s "$SIZE" "$OUT"
mkfs.ext4 -F -L QB3RT_ROOT "$OUT" >/dev/null
mkdir -p "$MNT"
mount -o loop "$OUT" "$MNT"
cleanup() { umount "$MNT" 2>/dev/null || true; }
trap cleanup EXIT

# --- copy the live rootfs (excluding volatile + capture artifacts) ----------
echo "Copying rootfs (rsync)..."
rsync -aHAXx --numeric-ids \
    --exclude='/proc/*' --exclude='/sys/*' --exclude='/dev/*' \
    --exclude='/run/*'  --exclude='/tmp/*' --exclude='/mnt/*' \
    --exclude='/media/*' --exclude='/lost+found' \
    --exclude='/swap.img' --exclude='/swapfile' --exclude='/swapfile.qb3rt' \
    --exclude='/opt/qb3rt/.build' --exclude='/opt/qb3rt/src_ws' \
    --exclude='/var/cache/apt/archives/*.deb' \
    --exclude='/opt/qb3rt/install/share/QB3rt' \
    --exclude='/opt/qb3rt/install/share/ament_index/resource_index/packages/QB3rt' \
    --exclude="$OUT" \
    "$SRC" "$MNT/"

# recreate the excluded mount points
for d in proc sys dev run tmp mnt media; do mkdir -p "$MNT/$d"; done
chmod 1777 "$MNT/tmp"

# --- reset per-unit identity IN THE COPY (reference unit untouched) ----------
echo "Resetting identity in the image..."
# machine-id (regenerated on first boot)
: > "$MNT/etc/machine-id"
rm -f "$MNT/var/lib/dbus/machine-id"
# SSH host keys (regenerated on first boot by ssh.service)
rm -f "$MNT"/etc/ssh/ssh_host_*
# per-unit ROS/udev identity (re-applied by stamp_unit.sh)
echo 0 > "$MNT/etc/qb3rt/domain_id" 2>/dev/null || { mkdir -p "$MNT/etc/qb3rt"; echo 0 > "$MNT/etc/qb3rt/domain_id"; }
rm -f "$MNT/etc/udev/rules.d/99-agv-serial.rules"
# template hostname
echo "qb3rt-unconfigured" > "$MNT/etc/hostname"
sed -i -E 's/^(\s*127\.0\.1\.1\s+).*/\1qb3rt-unconfigured/' "$MNT/etc/hosts" 2>/dev/null || true
# saved Wi-Fi: keep only the provisioning connection(s) in --keep-wifi so flashed
# units auto-join the network on boot; wipe the rest (no student creds shipped).
NMDIR="$MNT/etc/NetworkManager/system-connections"
if [ -n "$KEEP_WIFI" ]; then
    IFS=',' read -r -a _keepw <<< "$KEEP_WIFI"
    for f in "$NMDIR"/*; do
        [ -e "$f" ] || continue
        cid="$(awk -F= '/^id=/{print $2; exit}' "$f" 2>/dev/null)"
        [ -z "$cid" ] && cid="$(basename "$f" .nmconnection)"
        keep=0
        for k in "${_keepw[@]}"; do k="$(echo "$k" | xargs)"; [ "$cid" = "$k" ] && keep=1 && break; done
        if [ "$keep" = 1 ]; then echo "  keep provisioning Wi-Fi: $cid"; else rm -f "$f"; fi
    done
else
    rm -f "$NMDIR"/* 2>/dev/null || true
fi
# authorized_keys: optionally ship ONLY the provided (fleet) key, so a personal
# key used to build the reference unit is not spread across the fleet.
if [ -n "$AUTH_KEYS" ]; then
    U_UID="$(id -u ubuntu)"; U_GID="$(id -g ubuntu)"
    install -d -m 700 -o "$U_UID" -g "$U_GID" "$MNT/home/ubuntu/.ssh"
    install -m 600 -o "$U_UID" -g "$U_GID" "$AUTH_KEYS" "$MNT/home/ubuntu/.ssh/authorized_keys"
    echo "  authorized_keys replaced with $(basename "$AUTH_KEYS") (fleet key only, ${U_UID}:${U_GID})"
fi
# logs, histories, ROS state
find "$MNT/var/log" -type f -exec truncate -s 0 {} + 2>/dev/null || true
rm -rf "$MNT"/home/ubuntu/.ros "$MNT"/root/.ros 2>/dev/null || true
: > "$MNT/home/ubuntu/.bash_history" 2>/dev/null || true
# cloud-init re-run on first boot (fresh instance identity), if present
rm -rf "$MNT"/var/lib/cloud/instances/* 2>/dev/null || true

# --- finalize ---------------------------------------------------------------
sync
umount "$MNT"; trap - EXIT
echo "Checking + shrinking filesystem..."
e2fsck -fy "$OUT" >/dev/null || true
resize2fs -M "$OUT" >/dev/null 2>&1 || true
e2fsck -fy "$OUT" >/dev/null || true
# Truncate the image FILE down to the shrunk filesystem size so it isn't left at
# the oversized initial allocation (reclaims space + shrinks the flash transfer).
FS_BLOCKS="$(dumpe2fs -h "$OUT" 2>/dev/null | awk -F: '/Block count/{gsub(/ /,"",$2);print $2}')"
FS_BSIZE="$(dumpe2fs -h "$OUT" 2>/dev/null | awk -F: '/Block size/{gsub(/ /,"",$2);print $2}')"
if [ -n "$FS_BLOCKS" ] && [ -n "$FS_BSIZE" ]; then
    truncate -s "$((FS_BLOCKS * FS_BSIZE))" "$OUT"
fi

echo
echo "Golden image ready: $OUT ($(du -h "$OUT" | cut -f1), apparent $(ls -lh "$OUT" | awk '{print $5}'))"
cat <<'EOF'

FLASH to each unit (units are already on stock Canonical Ubuntu; we overwrite
ONLY the rootfs, leaving boot/firmware partitions as flashed. Put the board in
fastboot first):
  1) Confirm the rootfs partition name (this reference unit's rootfs PARTLABEL is
     "writable"):
        fastboot getvar all 2>&1 | grep -iE 'partition-(size|type):writable'
  2) Ensure the image fits that partition (shrunk above; grow --size if needed).
     The rootfs grows to fill the partition on first boot (growpart/cloud-init).
  3) Flash:
        fastboot flash writable  qb3rt-golden.img    # use the confirmed name
        fastboot reboot
  4) Boot, then from the laptop:  stamp/stamp_unit.sh --unit <id>
EOF
