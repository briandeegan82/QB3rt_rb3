#!/bin/bash
# Build a whole-disk (LUN0) golden image for EDL/QDL flashing = the stock
# Canonical Ubuntu GPT + EFI/GRUB + persist, with our customized rootfs swapped
# into the "writable" partition. Drop-in replacement for the stock
# iot-carmel-...img, so Qualcomm Launcher/QDL flashes our system in one pass
# (needed because RB3 Gen2 has no working fastboot — EDL only).
#
#   sudo capture/make_fulldisk.sh \
#        --stock  /path/iot-carmel-...img \
#        --rootfs /path/qb3rt-golden.img \
#        --out    /path/qb3rt-fulldisk.img
#
# Native 4K sectors. The rootfs grows to fill the real disk on first boot
# (growpart/cloud-init), exactly like the stock image does.
#
# Env: ROOT_UUID (default below) + ROOT_LABEL must match the content's
#      /boot/grub/grub.cfg (root=UUID=...) and /etc/fstab (LABEL=writable).
set -euo pipefail

STOCK="" ROOTFS="" OUT=""
ROOT_UUID="${ROOT_UUID:-c5de3ef5-e2b1-47ae-a2eb-34267d2d1441}"
ROOT_LABEL="${ROOT_LABEL:-writable}"
SLACK_GB="${SLACK_GB:-2}"
SECT=4096

need() { [ -n "${2:-}" ] || { echo "ERROR: $1 needs a value (did the command line get split across lines?)." >&2; exit 1; }; }
while [ $# -gt 0 ]; do case "$1" in
    --stock)  need "$1" "${2:-}"; STOCK="$2";    shift 2 ;;
    --rootfs) need "$1" "${2:-}"; ROOTFS="$2";   shift 2 ;;
    --out)    need "$1" "${2:-}"; OUT="$2";      shift 2 ;;
    --slack)  need "$1" "${2:-}"; SLACK_GB="$2"; shift 2 ;;
    -h|--help) sed -n '2,18p' "$0" | sed 's/^# \?//'; exit 0 ;;
    *) echo "Unknown arg: $1" >&2; exit 1 ;;
esac; done

[ "$(id -u)" = 0 ] || { echo "ERROR: run with sudo." >&2; exit 1; }
for f in "$STOCK" "$ROOTFS"; do [ -f "$f" ] || { echo "ERROR: missing file: $f" >&2; exit 1; }; done
[ -n "$OUT" ] || { echo "ERROR: --out required." >&2; exit 1; }
for t in sgdisk losetup e2fsck resize2fs tune2fs e2label blkid partprobe; do
    command -v "$t" >/dev/null || { echo "ERROR: need '$t' (install gdisk/parted/util-linux/e2fsprogs)." >&2; exit 1; }
done

WRMNT="$(mktemp -d)"
LOOP=""
cleanup() {
    mountpoint -q "$WRMNT" 2>/dev/null && umount "$WRMNT"
    [ -n "$LOOP" ] && losetup -d "$LOOP" 2>/dev/null || true
    rmdir "$WRMNT" 2>/dev/null || true
}
trap cleanup EXIT

log() { printf '\n== %s ==\n' "$*"; }

log "1. Copy stock whole-disk image -> $OUT"
cp --reflink=auto "$STOCK" "$OUT"

log "2. Inspect stock GPT (writable = last partition)"
LOOP="$(losetup -b $SECT -f --show "$OUT")"
sgdisk -p "$LOOP"
PN="$(sgdisk -p "$LOOP" | awk '/^[[:space:]]+[0-9]+[[:space:]]/{n=$1} END{print n}')"
[ -n "$PN" ] || { echo "ERROR: no partitions found in stock GPT." >&2; exit 1; }
WR_START="$(sgdisk -i "$PN" "$LOOP" | awk -F'[ (]' '/First sector/{print $3}')"
WR_NAME="$(sgdisk -i "$PN"  "$LOOP" | sed -n "s/.*Partition name: '\(.*\)'/\1/p")"
WR_TYPE="$(sgdisk -i "$PN"  "$LOOP" | awk '/Partition GUID code/{print $4}')"
WR_UGUID="$(sgdisk -i "$PN" "$LOOP" | awk '/Partition unique GUID/{print $4}')"
echo "  writable = partition #$PN  start=$WR_START  name='$WR_NAME'"
[ "$WR_NAME" = "$ROOT_LABEL" ] || echo "  NOTE: partition name '$WR_NAME' != '$ROOT_LABEL' (continuing; fs label is what fstab uses)."
losetup -d "$LOOP"; LOOP=""

log "3. Grow image file to hold rootfs + ${SLACK_GB}G slack + backup GPT"
ROOTFS_BYTES="$(stat -c %s "$ROOTFS")"
NEW_BYTES=$(( WR_START*SECT + ROOTFS_BYTES + SLACK_GB*1024*1024*1024 + 8*1024*1024 ))
NEW_BYTES=$(( (NEW_BYTES + SECT - 1) / SECT * SECT ))
truncate -s "$NEW_BYTES" "$OUT"
echo "  image now $(numfmt --to=iec "$NEW_BYTES")"

log "4. Relocate backup GPT to new end + extend writable to fill"
LOOP="$(losetup -b $SECT -f --show "$OUT")"
sgdisk -e "$LOOP"                                   # move backup GPT to end
sgdisk -d "$PN" "$LOOP"                             # delete + recreate spanning to end
sgdisk -n "$PN:$WR_START:0" -t "$PN:$WR_TYPE" -u "$PN:$WR_UGUID" -c "$PN:$WR_NAME" "$LOOP"
sgdisk -v "$LOOP"
partprobe "$LOOP" 2>/dev/null || true
losetup -d "$LOOP"; LOOP=""

log "5. Align UUID to ESP, write rootfs, patch grub + password expiry"
LOOP="$(losetup -b $SECT -P -f --show "$OUT")"
ESPDEV="${LOOP}p1"; WRDEV="${LOOP}p${PN}"
[ -b "$WRDEV" ] || { echo "ERROR: $WRDEV not present after partprobe." >&2; exit 1; }

# 5a. Discover the UUID the stock ESP's GRUB auto-searches for. This is the
#     authoritative boot target — if the rootfs fs UUID doesn't match it, GRUB's
#     `search --fs-uuid` finds nothing and drops to the `grub>` prompt.
ESPMNT="$(mktemp -d)"
mount -o ro "$ESPDEV" "$ESPMNT" 2>/dev/null || mount "$ESPDEV" "$ESPMNT"
ESP_UUID="$(grep -rhoE '[0-9a-f]{8}(-[0-9a-f]{4}){3}-[0-9a-f]{12}' "$ESPMNT"/EFI/*/grub.cfg 2>/dev/null | head -1)"
umount "$ESPMNT"; rmdir "$ESPMNT"
if [ -n "$ESP_UUID" ]; then
    echo "  ESP GRUB search UUID: $ESP_UUID (aligning writable + grub.cfg to it)"
    ROOT_UUID="$ESP_UUID"
else
    echo "  WARNING: could not read ESP grub UUID; using default $ROOT_UUID" >&2
fi

# 5b. Write our rootfs; set its fs UUID to the ESP target + label + fill partition.
echo "  dd rootfs ($(numfmt --to=iec "$ROOTFS_BYTES")) -> $WRDEV"
dd if="$ROOTFS" of="$WRDEV" bs=4M conv=fsync status=none
e2fsck -fy "$WRDEV" >/dev/null 2>&1 || true
tune2fs -U "$ROOT_UUID" "$WRDEV" >/dev/null
e2label "$WRDEV" "$ROOT_LABEL"
resize2fs "$WRDEV" >/dev/null 2>&1 || true
e2fsck -fy "$WRDEV" >/dev/null 2>&1 || true

# 5c. Patch the rootfs: make grub.cfg agree with the fs UUID, and clear the
#     forced 'ubuntu' password change (Ubuntu/cloud-init expires it, which blocks
#     key-based SSH command execution — our whole toolchain).
mount "$WRDEV" "$WRMNT"
OLD_UUID="$(grep -m1 -oE 'root=UUID=[0-9a-f-]{36}' "$WRMNT/boot/grub/grub.cfg" 2>/dev/null | cut -d= -f3)"
if [ -n "$OLD_UUID" ] && [ "$OLD_UUID" != "$ROOT_UUID" ]; then
    sed -i "s/$OLD_UUID/$ROOT_UUID/g" "$WRMNT/boot/grub/grub.cfg"
    echo "  grub.cfg root UUID: $OLD_UUID -> $ROOT_UUID"
fi
if [ -f "$WRMNT/etc/shadow" ]; then
    awk -F: 'BEGIN{OFS=":"} $1=="ubuntu"{ if($3==""||$3=="0") $3="20000"; if($5=="0") $5="99999" } {print}' \
        "$WRMNT/etc/shadow" > "$WRMNT/etc/shadow.qb3rt" \
        && cat "$WRMNT/etc/shadow.qb3rt" > "$WRMNT/etc/shadow" && rm -f "$WRMNT/etc/shadow.qb3rt"
    echo "  cleared forced password change on 'ubuntu'"
fi
mkdir -p "$WRMNT/etc/cloud/cloud.cfg.d"
# Primary fix: have cloud-init itself un-expire the account. runcmd executes in
# cloud-final, AFTER cc_set_passwords expires it — so this always wins. (The
# systemd unit below is a backup; the chpasswd:expire:false alone did not stick.)
cat > "$WRMNT/etc/cloud/cloud.cfg.d/99-qb3rt-unlock.cfg" <<'EOF'
chpasswd:
  expire: false
runcmd:
  - [ chage, -d, '20000', -M, '-1', ubuntu ]
  - [ passwd, -u, ubuntu ]
EOF
# Bulletproof: a boot-time oneshot that clears the expiry AFTER cloud-init (which
# re-expires it on first boot). Enabled offline via the wants/ symlink.
cat > "$WRMNT/etc/systemd/system/qb3rt-unlock-ubuntu.service" <<'EOF'
[Unit]
Description=QB3rt: clear forced password change on the ubuntu account
After=cloud-final.service
[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/bin/chage -d 20000 -M -1 ubuntu
ExecStart=/usr/bin/passwd -u ubuntu
[Install]
WantedBy=multi-user.target
EOF
mkdir -p "$WRMNT/etc/systemd/system/multi-user.target.wants"
ln -sf ../qb3rt-unlock-ubuntu.service \
    "$WRMNT/etc/systemd/system/multi-user.target.wants/qb3rt-unlock-ubuntu.service"
echo "  installed + enabled qb3rt-unlock-ubuntu.service (post-cloud-init)"
umount "$WRMNT"
e2fsck -fy "$WRDEV" >/dev/null 2>&1 || true

log "6. Verify"
sgdisk -v "$LOOP" | sed 's/^/  gpt: /'
echo "  partitions:"; sgdisk -p "$LOOP" | awk 'NF>=6 && $1 ~ /^[0-9]+$/{print "    "$0}'
echo "  writable fs: UUID=$(blkid -s UUID -o value "$WRDEV")  LABEL=$(e2label "$WRDEV")  (want UUID=$ROOT_UUID LABEL=$ROOT_LABEL)"
mount "$WRDEV" "$WRMNT"
ok=1
for p in opt/qb3rt/install/setup.bash etc/qb3rt/cyclonedds.xml etc/profile.d/qb3rt-ros-env.sh boot/grub/grub.cfg; do
    if [ -e "$WRMNT/$p" ]; then echo "    ok   /$p"; else echo "    MISS /$p"; ok=0; fi
done
echo "    hostname: $(cat "$WRMNT/etc/hostname" 2>/dev/null)   authorized_keys lines: $(wc -l < "$WRMNT/home/ubuntu/.ssh/authorized_keys" 2>/dev/null)"
GRUB_ROOT="$(grep -m1 -oE 'root=UUID=[0-9a-f-]+' "$WRMNT/boot/grub/grub.cfg" 2>/dev/null | cut -d= -f3)"
FS_UUID="$(blkid -s UUID -o value "$WRDEV")"
echo "    grub root=UUID: $GRUB_ROOT   fs UUID: $FS_UUID"
if [ "$GRUB_ROOT" = "$FS_UUID" ]; then echo "    ok   grub root UUID == fs UUID (auto-boot)"; else echo "    MISS grub root UUID != fs UUID"; ok=0; fi
echo "    ubuntu pw-expiry cleared: $(awk -F: '$1=="ubuntu"{print ($3!="0" && $3!="")?"yes":"NO"}' "$WRMNT/etc/shadow")"
echo "    netplan wifi files: $(ls "$WRMNT"/etc/netplan/90-NM-*.yaml 2>/dev/null | wc -l)"
umount "$WRMNT"
losetup -d "$LOOP"; LOOP=""

grub_uuid="$(printf '%s' "root=UUID=$ROOT_UUID")"
echo
if [ "$ok" = 1 ]; then
    echo "SUCCESS: $OUT ($(du -h "$OUT" | cut -f1))"
    echo "  Writable fs UUID matches the ESP GRUB search + grub.cfg (auto-boots), label=writable"
    echo "  (fstab), and the 'ubuntu' password expiry is cleared (key-based SSH works)."
    echo "  Flash it in place of the stock rootfs image via QDL/Launcher (LUN0, label=disk)."
    echo "  TEST ON ONE UNIT before the fleet."
else
    echo "WARNING: some expected content missing — inspect before flashing." >&2
    exit 1
fi
