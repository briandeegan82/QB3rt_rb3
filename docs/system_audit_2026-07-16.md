# QB3rt System Audit — 2026-07-16

> **Historical snapshot (2026-07-16).** This is a point-in-time device audit, not
> a live runbook. Several findings have since changed in the repo (for example
> `system/` now versions cyclonedds / udev / sysctl; VIO is ORB-SLAM3 on OV9282,
> not OAK-D Basalt; deploy tooling includes adb bootstrap). Security and network
> items (default root password, `startup-network.service` → eduroam, etc.) may
> still apply on a given robot until verified. Prefer [README.md](../README.md)
> and [SETUP.md](../SETUP.md) for current procedures.

Full audit of the QB3rt AGV (Qualcomm RB3 Gen 2 vision kit, QIRP 1.5 / ROS 2 Jazzy).
Scope: code & config review, system/deploy hygiene, security & robustness, plus the
boot-time wifi-drop investigation. Static audit — the ROS stack was not running.
Evidence was snapshotted to the laptop before analysis (device wifi was dropping
mid-session; see finding N1, which is also why).

**Method note:** all on-device evidence was collected read-only. Nothing on the
device was changed by this audit except installing the laptop's ssh public key
in `/root/.ssh/authorized_keys` (done by the user, to enable collection).

---

## Executive summary

1. **The boot wifi drop is self-inflicted**: a custom systemd unit
   (`startup-network.service`) runs ~2 min after boot and executes
   `nmcli connection up eduroam`, yanking the robot off `ros_net_5G` onto the
   campus network. Then its next step fails (`/usr/bin/tailscale` doesn't exist),
   leaving the unit `failed` after having done the damage. Your bash history is
   the receipt: dozens of manual `nmcli device wifi connect ros_net_5G` retries.
2. **Security posture is poor for a device that auto-joins a public network**:
   root ssh password login enabled with the password unchanged since **2011**
   (i.e. the Yocto image default), MQTT broker open on all interfaces, dnsmasq
   answering DNS on a public university IP it no longer holds.
3. **The QB3rt project itself is in good shape** — deploy discipline works
   (source and install space match), configs are exceptionally well documented,
   and the calibrated drive model is consistent everywhere. The main code finding
   is a joystick runaway edge case in the bridge watchdog.
4. **Three network stacks + two time daemons fight over the box**, the clock is
   6 days behind with no working NTP, and the journal is drowned by a
   1 msg/sec dnsmasq warning loop.

---

## N. Network / the wifi drop

### N1 — CRITICAL: `startup-network.service` switches wifi to eduroam after boot
`/etc/systemd/system/startup-network.service`:
```ini
ExecStart=/usr/bin/nmcli connection up eduroam   ← the drop, verbatim
ExecStart=/bin/systemctl start tailscaled
ExecStart=/usr/bin/tailscale up                  ← 203/EXEC, wrong path
```
Boot journal (2026-07-10 19:27:41–49): unit starts → wlan0 loses carrier →
reassociates to `eduroam` (BSSID 9c:d5:7d:19:e8:a0) → DHCP gives 140.203.212.99
(University of Galway space) → robot vanishes from 192.168.0.x. `tailscale up`
then fails because the binary is at `/usr/local/bin/tailscale`, not `/usr/bin`.
This exactly reproduces the reported "connects at boot, drops after 30–60 s,
must reconnect manually" symptom (the delay is the unit's start time in the
boot sequence, not an RF problem).

### N2 — HIGH: all six saved wifi profiles autoconnect at equal priority
`ros_net_5G`, `eduroam`, `UGV`, `UoGPSK`, `robot_hotspot`, `Brians iPhone` are
all `autoconnect: yes`, `autoconnect-priority: 0`. The boot log shows NM picked
**eduroam first** at 19:25:52 and something had to switch it to `ros_net_5G` at
19:26:02. On campus, boot-time SSID selection is effectively random.

### N3 — MEDIUM: three network stacks are active simultaneously
`NetworkManager`, `systemd-networkd`, and `dhcpcd` are all enabled+active
(plus `dnsmasq` and vendor `wlan_daemon`). networkd churns Link UP/DOWN on
wlan0 until it "unmanages" it each connect cycle, matches eth0/eth1 with its
own DHCP config, and `systemd-networkd-wait-online.service` times out (failed
unit) every boot because networkd never completes. dhcpcd solicits on
tailscale0. One owner (NetworkManager) should manage everything.

### N4 — MEDIUM: dnsmasq spams the journal ~1 msg/sec
`LOUD WARNING: listening on 140.203.212.99 may accept requests via interfaces
other than wlan0` — dnsmasq bound the eduroam-era address and warns every
second, forever (netstat confirms it still holds 140.203.212.99:53 sockets).
This flooded the journal so completely that my first 400-line filtered pull
contained *zero* useful wifi events. It also answers DNS on every interface.

### N5 — MEDIUM: NetworkManager at ~48% CPU
13m52s CPU in a 28-minute uptime. Likely the constant active-scan loop: the
hidden-SSID profile (`UoGPSK`, `hidden=true`) forces periodic active scans
(NM logs the "This makes you trackable" warning), and five other autoconnect
candidates keep re-evaluation busy. Re-check after N1/N2/pruning; on a CPU
that also runs VIO + Nav2 this is real headroom being burned.

---

## S. Security & robustness

### S1 — CRITICAL: root password login enabled, password unchanged since 2011
`sshd -T`: `permitrootlogin yes`, `passwordauthentication yes`. `passwd -S root`
shows the password set **2011-04-05** — the QIRP image default (publicly
documented for these boards). Combined with N1 auto-joining eduroam, this is a
root-login-with-known-default-password box on a public university network.
Your ed25519 key is now installed, so password auth can be turned off.

### S2 — HIGH: unnecessary services listening on all interfaces
- `mosquitto` MQTT on `0.0.0.0:1883` (and `:::1883`) — no obvious consumer in
  the QB3rt stack; anonymous by default on this image.
- `rpcbind` on `:111` (its service is in a failed state anyway).
- `dnsmasq` DNS on every address including the stale public one (N4).
- `tftp_server`, `avahi`, `adbd` (vendor defaults) also enabled.
- Qualcomm `qwesd` ports 20803–20808 on 0.0.0.0 (vendor; harder to remove).

### S3 — MEDIUM: tailscale is logged out (dead fallback path)
`tailscale status` → "Logged out." The tailscaled daemon runs (correct unit,
`/usr/local/bin`), but the `startup-network.service` `tailscale up` that would
have surfaced the login URL has been failing (N1). Re-authenticating would give
you a stable access path independent of which wifi the robot lands on — which
would have made this audit (and every future debug session) far less painful.

### S4 — MEDIUM: eduroam profile carries personal credentials on disk
`eduroam.nmconnection` stores identity `0109491s@universityofgalway.ie` with an
MSCHAPv2 password (file perms 600/root — correct, but it's another reason not
to leave password-auth root ssh open). If the robot doesn't need eduroam,
delete the profile; that also removes the N1/N2 failure mode at the root.

### S5 — LOW: robustness gaps
- Failed units every boot: `gstd`, `var-usbfw.mount`, `rpcbind`, `uefi_sec`,
  `systemd-networkd-wait-online`, `startup-network` (vendor noise mostly, but
  they mask real failures and slow boot).
- Journal is volatile (`/var/volatile` tmpfs): previous-boot logs are gone —
  the N1 diagnosis was only possible because the current boot showed it live.
  Consider persistent journald (`Storage=persistent`, small `SystemMaxUse`).
- No RTC battery (RTC reads 1970) **and no working NTP**: `chronyd.service` is
  enabled but chrony isn't installed; `systemd-timesyncd` is enabled but
  `System clock synchronized: no`; device clock is ~6 days behind real time.
  Consequences: TLS/cert validation (tailscale login!), rosbag timestamps,
  robot↔laptop TF sync for remote Nav2 (a documented prerequisite in
  laptop/README.md). Pick ONE time source and verify it actually syncs.

---

## D. Deploy & project hygiene

### D1 — GOOD: source ↔ install space are in sync
`diff -r /root/QB3rt/{launch,config,urdf,scripts,behavior_trees} /usr/share/QB3rt/…`
is clean, bridge + yaml match `/usr/lib` / `/usr/share` copies. The deploy
workflow is being followed and works.

### D2 — MEDIUM: three orphan files exist ONLY in the install space
`/usr/share/QB3rt/launch/basalt_vio_only.launch.py`,
`config/basalt_vio_rb3.json`, `README_vio_tuning.md` were created directly in
the install space (bash history shows a scripted session heredoc-ing them into
`/usr/share`) — exactly the anti-pattern the README forbids. They are not in
`/root/QB3rt`, so the next reflash (or a deploy with `--delete`) silently
destroys them. Decide: adopt into the project or delete. Related:
`deploy.sh`'s rsync has no `--delete`, so removed/renamed source files linger
in the install space forever (this is how these were able to hide).

### D3 — MEDIUM: critical system configs live outside the project
`rover_env.sh` claims `/opt/cyclonedds.xml` is "deployed from this repo" — it
is not: no copy exists in `/root/QB3rt`, and `deploy.sh` doesn't touch it. Same
for `/etc/sysctl.d/98-agv-dds.conf` (the DDS buffer fix that took a day to
find), `/etc/udev/rules.d/99-agv-serial.rules` (stable serial names), and the
(to-be-fixed) `startup-network.service`. All are load-bearing, all are
hand-installed, all vanish on reflash. Add a `system/` dir to the project and
a deploy.sh section that installs them.

### D4 — LOW: stale defaults & comments
- `wave_rover_bridge.py` `declare_parameter` defaults still carry the pre-recal
  values (`max_speed 0.42`, `spin_boost 4.0`, `spin_boost_max 5.0`) vs the
  calibrated yaml (0.56 / 6.0 / 8.0). Running the node without the yaml
  silently reverts the calibration — and 5.0 `spin_boost_max` is the exact
  silent-clamp that already ate a calibration session once (the yaml warns
  about it). Sync the defaults.
- `odometry.launch.py` docstring still says the EKF fuses VIO (removed
  2026-07-13); bridge covariance comment says "ekf.yaml odom1" (it's odom0).
- `/root` clutter: 6.0 GB `ov9282_calibration` bag + ORB-SLAM3 `init_*.txt`
  droppings at `/root` top level; disk at 82%. Move bags off-device.

---

## C. Code review (bridge, relay, launch, configs)

### C1 — HIGH (safety): joystick runaway if /joy stream dies while deadman held
`wave_rover_bridge.py:435` — `_watchdog_cb` returns immediately when
`self._joy_active`. `_joy_active` only clears when a *new* /joy message arrives
with the deadman released. If the joystick disconnects / joy_node dies / wifi
teleop link drops **while the deadman is held and sticks deflected**, no
further /joy messages ever arrive, the watchdog stays muzzled, and the rover
drives at the last commanded wheel speeds indefinitely. Fix: record
`_last_joy_time` in `_joy_cb` and have the watchdog treat a stale joy stream
(> cmd_timeout) as deadman-released (send stop, clear `_joy_active`).
(Note: `joy_node` typically publishes at a steady rate while a device is
present, so the window is specifically device death — which is precisely when
you want the watchdog live.)

### C2 — LOW: odometry integrates the new velocity across arbitrary time gaps
`_integrate_odom` applies the *incoming* velocity over the entire interval
since `_last_odom_time`. After a watchdog stop or a joy-priority period,
that interval can be minutes, producing a large one-step pose jump in `/odom`.
The EKF is immune (fuses twist.vx only), but `odom_square_test.py` and any
debugging that trusts `/odom` pose are not. Fix: clamp dt (e.g. ignore
integration when dt > 0.5 s) or reset `_last_odom_time` on stop.

### C3 — INFO: reviewed clean
- `ekf.yaml` — wiring matches the declared architecture (wheel vx odom0 + gyro
  yaw-rate imu0, `base_link_frame: base_footprint`, VIO correctly parked with
  re-enable instructions). Frequency/covariance rationale documented. Good.
- `orbslam3_pose_to_odom.py` — R_REBASE conjugation + Kalibr-derived R_MOUNT
  match the live-validated 2026-07-14 state; `apply_mount_correction` default
  and the calibration-history docs are consistent.
- `deploy.sh` — correct source/install separation, laptop/on-device detection,
  writability preflight, pycache purge. (Add `--delete`, see D2.)
- `full_stack.launch.py` scoping trick (leaky `use_composition`) and the
  `bringup_base` single-foundation pattern are sound.
- CycloneDDS config + sysctl + udev rules verified present and correct on the
  device (content matches their documented intent).
- nav2.yaml / laptop split consistent with the 2026-07-14 tuning history.

---

## Prioritized fix list

Nothing below has been applied — your call, in this order:

1. **[N1] Defuse startup-network.service.** Either disable it
   (`systemctl disable --now startup-network`) or fix it: change
   `nmcli connection up eduroam` → `ros_net_5G` (or drop the line — NM
   autoconnect handles it once priorities are set) and
   `/usr/bin/tailscale` → `/usr/local/bin/tailscale`.
2. **[N2] Make wifi choice deterministic:**
   `nmcli con mod ros_net_5G connection.autoconnect-priority 100`, and
   `nmcli con mod eduroam connection.autoconnect no` (repeat for UGV/UoGPSK/
   robot_hotspot/iPhone, or delete unused profiles — S4 argues for deleting
   eduroam outright).
3. **[S1] Close root password ssh:** verify key login works, then in
   `/etc/ssh/sshd_config`: `PermitRootLogin prohibit-password`,
   `PasswordAuthentication no`; restart sshd. Change the root password anyway.
4. **[C1] Bridge joystick watchdog** (small patch in
   `vendor_overrides/wave_rover_controller/wave_rover_bridge.py` + deploy).
5. **[S5/N-clock] Fix time:** `systemctl disable chronyd` (binary absent),
   keep `systemd-timesyncd`, confirm `timedatectl` shows synchronized on
   ros_net_5G; then **[S3]** `tailscale up` and log in (needs the clock fixed
   first for TLS).
6. **[N3] One network owner:** `systemctl disable --now systemd-networkd
   systemd-networkd-wait-online dhcpcd` (NetworkManager stays).
7. **[N4/S2] dnsmasq + listeners:** `systemctl disable --now dnsmasq rpcbind`
   unless you use them; bind mosquitto to 127.0.0.1 (`listener 1883
   127.0.0.1` in mosquitto.conf) or disable it.
8. **[D3] Version the system configs in the project** (`system/` dir +
   deploy.sh install step for cyclonedds.xml, sysctl, udev rules, the fixed
   startup-network.service) so a reflash restores them.
9. **[D2] Resolve install-space orphans** (adopt basalt files into
   `/root/QB3rt` or delete) and add `--delete` to deploy.sh's rsync.
10. **[D4] Sync bridge defaults to calibrated values; fix the two stale
    comments; move the 6 GB calibration bag off-device.**
11. **[S5] Optional hygiene:** persistent journald, disable failing vendor
    units you don't use (`gstd`, `rpcbind`), prune `/root` clutter.

Wifi-drop verification after (1)+(2): reboot, don't touch anything, watch
`journalctl -fu NetworkManager` — the robot should associate to ros_net_5G
within ~30 s and *stay there* through the 2-minute mark where it used to jump.
