#!/bin/bash
# QB3rt reference-unit build — STEP 2: fleet-wide (identity-independent) config.
#
# Runs ON the reference RB3, after 10_build_custom.sh. Installs everything that
# is IDENTICAL across the fleet, so it gets baked into the golden image. Anything
# PER-UNIT (hostname, ROS_DOMAIN_ID, USB serial udev rules, Wi-Fi) is NOT set
# here — that is done later by stamp/stamp_unit.sh over SSH.
#
# Installs:
#   /etc/qb3rt/cyclonedds.xml            DDS transport tuning
#   /etc/qb3rt/domain_id                 default 0 (per-unit value set by stamp)
#   /etc/sysctl.d/98-agv-dds.conf        UDP socket buffer limits for DDS
#   /etc/udev/rules.d/80-movidius.rules  OAK-D / DepthAI USB access
#   /etc/profile.d/qb3rt-ros-env.sh      ROS + overlay + DDS env for login shells
#   systemd-timesyncd NTP -> host        (RB3 has no battery-backed RTC)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYS="$(cd "$HERE/../system" && pwd)"
NTP_HOST="${NTP_HOST:-192.168.0.101}"   # host laptop serving chrony on the AP subnet

log() { printf '\n=== %s ===\n' "$*"; }
if [ "$(id -u)" -eq 0 ]; then SUDO=""; else SUDO="sudo"; fi

# --- locale (image ships without en_US.UTF-8; ROS + tooling warn loudly) -----
log "Ensuring a UTF-8 locale"
if ! locale -a 2>/dev/null | grep -qiE '^en_US\.utf-?8$'; then
    $SUDO locale-gen en_US.UTF-8 2>/dev/null || true
fi
$SUDO update-locale LANG=en_US.UTF-8 2>/dev/null || true

# --- DDS config + default domain id -----------------------------------------
log "Installing DDS config -> /etc/qb3rt"
$SUDO mkdir -p /etc/qb3rt
$SUDO cp "$SYS/cyclonedds.xml" /etc/qb3rt/cyclonedds.xml
# domain_id is per-unit; seed a default so the env file is valid pre-stamp.
[ -f /etc/qb3rt/domain_id ] || echo 0 | $SUDO tee /etc/qb3rt/domain_id >/dev/null

# --- passwordless sudo for fleet automation (stamp + future Ansible) ---------
# Explicit, image-owned drop-in so `become`/unattended sudo does not depend on
# cloud-init regenerating 90-cloud-init-users on each flashed unit. Staged with
# a dotted name (sudo ignores files containing '.') and validated before install.
log "Installing passwordless sudo drop-in (/etc/sudoers.d/90-qb3rt)"
$SUDO cp "$SYS/90-qb3rt-sudoers" /etc/sudoers.d/90-qb3rt.staged
$SUDO chmod 0440 /etc/sudoers.d/90-qb3rt.staged
if $SUDO visudo -cf /etc/sudoers.d/90-qb3rt.staged >/dev/null; then
    $SUDO mv /etc/sudoers.d/90-qb3rt.staged /etc/sudoers.d/90-qb3rt
    echo "  installed + validated"
else
    $SUDO rm -f /etc/sudoers.d/90-qb3rt.staged
    echo "ERROR: sudoers drop-in failed validation — not installed." >&2
    exit 1
fi

# --- don't let the ubuntu password expiry block key-based SSH ----------------
# Ubuntu/cloud-init ships the `ubuntu` account with an expired password (forced
# change on first login). That blocks NON-interactive key-based SSH command
# execution ("password change required... no TTY") — i.e. all of stamp/deploy/
# discover/Ansible. Clear the forced change and stop cloud-init re-expiring it.
log "Clearing forced password change on 'ubuntu' (unblocks key-based SSH)"
$SUDO chage -d "$(date +%Y-%m-%d)" -M -1 ubuntu 2>/dev/null || true
$SUDO mkdir -p /etc/cloud/cloud.cfg.d
# runcmd runs in cloud-final, AFTER cloud-init expires the account on first boot,
# so it reliably un-expires it (chpasswd:expire:false alone did not stick).
$SUDO tee /etc/cloud/cloud.cfg.d/99-qb3rt-unlock.cfg >/dev/null <<'EOF'
chpasswd:
  expire: false
runcmd:
  - [ chage, -d, '20000', -M, '-1', ubuntu ]
  - [ passwd, -u, ubuntu ]
EOF
# Bulletproof: a oneshot that re-clears it AFTER cloud-init on every boot
# (cloud-init re-expires the account on a fresh instance's first boot).
$SUDO tee /etc/systemd/system/qb3rt-unlock-ubuntu.service >/dev/null <<'EOF'
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
$SUDO systemctl enable qb3rt-unlock-ubuntu.service >/dev/null 2>&1 || true

# --- kernel UDP buffers for DDS large messages ------------------------------
log "Installing sysctl DDS buffer limits"
$SUDO cp "$SYS/98-agv-dds.conf" /etc/sysctl.d/98-agv-dds.conf
$SUDO sysctl --system >/dev/null

# --- OAK-D / DepthAI udev ----------------------------------------------------
log "Installing OAK-D udev rules"
$SUDO cp "$SYS/80-movidius.rules" /etc/udev/rules.d/80-movidius.rules
$SUDO udevadm control --reload-rules && $SUDO udevadm trigger || true

# --- login-shell ROS environment --------------------------------------------
log "Installing /etc/profile.d/qb3rt-ros-env.sh"
$SUDO cp "$SYS/qb3rt-ros-env.sh" /etc/profile.d/qb3rt-ros-env.sh
$SUDO chmod 644 /etc/profile.d/qb3rt-ros-env.sh

# --- clock sync (no RTC on the RB3) -----------------------------------------
log "Configuring systemd-timesyncd NTP -> ${NTP_HOST}"
$SUDO mkdir -p /etc/systemd/timesyncd.conf.d
printf '[Time]\nNTP=%s\n' "$NTP_HOST" | \
    $SUDO tee /etc/systemd/timesyncd.conf.d/qb3rt.conf >/dev/null
$SUDO systemctl enable --now systemd-timesyncd || true
$SUDO timedatectl set-ntp true || true

log "STEP 2 complete"
echo "Fleet-wide config installed. The reference unit is now ready to:"
echo "  1) verify:  log out/in, then  ros2 pkg list | grep -E 'qrb|orb_slam3|depthai|wave_rover'"
echo "  2) capture: capture/make_golden.sh   (produces the flashable image)"
