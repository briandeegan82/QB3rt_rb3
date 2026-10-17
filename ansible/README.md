# QB3rt fleet — Ansible

First Ansible content in this repo. `system/90-qb3rt-sudoers` already grants
the `ubuntu` user passwordless sudo specifically so `become` runs unattended
here, and `stamp/` already establishes the conventions this reuses: SSH as
`ubuntu` with the fleet key (`~/.ssh/qb3rt_fleet`), one unit reachable at a
time over the lab Wi-Fi.

## The one playbook to run

**`playbooks/base-config.yml`** applies every role below to `qb3rt_fleet` in
the right order (plus `agent_stack_deps` to `[qb3rt_agent]` only). It converges
a running unit of any image vintage to the current fleet config — use it for
new units, after re-flashes, and to repair drift:

```bash
ansible-playbook playbooks/base-config.yml            # fleet-wide
ansible-playbook playbooks/base-config.yml -l qb3rt-7 # one unit first
```

Per-unit identity (hostname, `ROS_DOMAIN_ID`, USB-serial udev symlinks, Wi-Fi
profiles) is *never* set by Ansible — that stays with `stamp/stamp_unit.sh`.

## Roles

### `qb3rt_base`

Fleet-wide, identity-independent convergence — the Ansible mirror of
`reference/20_fleet_config.sh`, so units flashed from an older image catch up
to the current one: en_US.UTF-8 locale, `/etc/qb3rt/cyclonedds.xml` (plus a
`domain_id` seed *only if absent*), the passwordless-sudo drop-in (staged and
`visudo`-validated), the cloud-init/`ubuntu` password-expiry unlock, DDS sysctl
buffers, OAK-D udev rules, `/etc/profile.d/qb3rt-ros-env.sh`, removal of the
`rplidar.rules` catch-all (it grabs `/dev/rplidar` for *any* CP2102N — including
the rover adapter — and sorts after the per-serial rules stamp writes), and a
warning when `/opt/qb3rt/install` lacks the `QB3rt` ament marker.

### Per-unit hardware convention probe

`qb3rt_base` also installs **`qb3rt-convention-probe`** (`/usr/local/sbin/`),
but never runs it — it drives the wheels. Run it once per unit after
base-config, interactively:

```bash
ansible-playbook playbooks/convention-probe.yml -l qb3rt-7
```

The playbook first runs **`qb3rt-preflight`** — a raw-serial health check
(no ROS) that kills every stale stack/bridge, verifies `/dev/wave_rover`
and `/dev/rplidar` resolve and stay bound, and confirms the lidar answers
`GET_INFO` and the ESP32 answers a reset banner. It fails fast on the
failure modes that used to surface as "the robot didn't move" mid-probe.

Then it starts `qb3rt-stack@odometry` + `qb3rt-stack@rplidar_c1` (the
systemd services — no transient `systemd-run` leftovers fighting over the
serial port), pauses for you to floor the robot and place a calibration
object at its nose, and runs the probe. It measures:

- **`drive_inverted`** — pulses `+linear.x` and discriminates direction by
  the nose obstacle's scan-range delta (IMU launch accel is fallback only;
  IMU-x vs body-x sign is itself a per-unit convention, qb3rt-17).
- **yaw convention** — `+angular.z` vs. gyro sign; a *negative* result can't
  be auto-fixed and flags the unit for manual wiring inspection.
- **`lidar_yaw`** — locates the nose-placed obstacle in the raw scan; its
  angle reveals which way scan-zero physically points.

Finally the playbook **verifies the result end-to-end**: it restarts the
bridge under the written conventions, pulses `+x` again, and asks you to
confirm — answering `b` flips `drive_inverted` and re-tests once. The
fleet default in `wave_rover_bridge.yaml` is `false` (most chassis) — the
conventions file overrides it per unit.

Results land in **`/etc/qb3rt/unit_conventions.yaml`**, which
`wave_rover_bridge.py` (`drive_inverted`) and `static_transforms.launch.py`
(`lidar_yaw` default) read at every startup — so the measured baseline
survives `update_project.sh` redeploys. A copy is fetched back to
`ansible/convention-results/<host>.yaml` for the record.

### `https_time_sync`

Deploys the HTTPS Date-header clock sync hack described in
[`README.md`](../README.md#operational-gotchas) gotcha #1 and
[`laptop/README.md`](../laptop/README.md): the RB3 has no working RTC, boots
with a wrong clock, and a mid-run NTP step breaks the EKF. Instead of relying
on NTP, it curls a few well-known HTTPS endpoints, takes the first one whose
`Date` response header is both parseable and close to the current clock, and
sets the kernel clock (+ hwclock if an RTC exists) from it — then repeats
every 15 minutes via a systemd timer.

Differences from a hand-run version of this script:

- **Bootstrap is automatic, not a manual sed round-trip.** A fresh unit's
  clock can be years off (no RTC battery), which the normal ±1h sanity check
  correctly rejects. The role runs the script once with `--bootstrap`
  (widens the check to 7 days) right after install, then leaves the deployed
  script at the tight 1-hour check for the timer's regular runs. Re-running
  the playbook is safe — the bootstrap step is idempotent-ish (it's a
  `command`, but only reports `changed` when it actually resets the clock).
- **chrony/timesyncd are stopped+disabled, not left racing the new timer**,
  and doing so is tolerant of either daemon being absent (some units never
  had chrony installed — see `docs/system_audit_2026-07-16.md`).
- Endpoint list, sanity-check threshold, and timer cadence are role
  variables (`roles/https_time_sync/defaults/main.yml`), not hardcoded.

### `wifi_powersave_off`

Fleet-wide. Installs `system/99-wifi-powersave-off.conf` as a NetworkManager
default (the same file `reference/20_fleet_config.sh` bakes into new golden
images) and turns power save off on the live interface. With power save on,
a strong link measured 35% loss and 2–5 s RTT, which makes every ROS-over-WiFi
workflow look broken.

It deliberately doesn't restart NetworkManager, because that would drop the
SSH session the play is running over. Instead it runs `nmcli general reload
conf` plus a runtime `iw ... power_save off`. To check by hand, run
`iw dev wlan0 get power_save` and expect `Power save: off`. The per-profile
`802-11-wireless.powersave` field will still say `default`. That's expected,
because the setting is the global default, not a per-profile value.

```bash
ansible-playbook playbooks/wifi-powersave-off.yml -l qb3rt_odd
```

### `wifi-ros-net-5` (playbook, not a role)

Adds ROS_NET_5 as every unit's **preferred** Wi-Fi while keeping the unit's
dedicated net (ROS_NET_ODD/EVEN) as the autoconnect fallback:

- `ROS_NET_5` profile pinned to the unit's static IP (`ansible_host`),
  gateway `<ip>.1`, `autoconnect-priority 20`
- dedicated net re-armed `autoconnect yes`, priority 10
- nothing forces a roam — NM picks up ROS_NET_5 on the next association or
  reboot; `-e switch_now=true` schedules a deferred `nmcli connection up`
  ~5 s out (same trick as stamp's IP switch)
- `-e ros_net_5_laptop_ip=<ip>` also writes `/etc/qb3rt/dds_peer_ros_net_5`;
  `qb3rt-ros-env.sh` prefers it whenever the unit is associated to ROS_NET_5
- PSK comes from `units/secrets.env` (`WIFI_PSK_ROS_NET_5`), never the repo

```bash
ansible-playbook playbooks/wifi-ros-net-5.yml -l qb3rt-14   # one unit
ansible-playbook playbooks/wifi-ros-net-5.yml               # fleet
```

Deployed 2026-10-17 to even units 8, 10, 12, 16, 24, 42.

### `agent_stack_deps`

Opt-in, for units in `[qb3rt_agent]` only. Installs the LLM agent stack's
runtime deps that aren't in the golden image: `ros-jazzy-twist-mux` (without
it, agent `cmd_vel_manual` commands are silently dropped), `python3-scipy` and
`python3-openai`. Afterwards it checks that `dpkg --audit` is clean, that
`twist_mux` is on the package path and that both Python modules import.

On a network with no internet uplink, put the arm64 `.deb`s in
`roles/agent_stack_deps/files/debs/` (they're gitignored). The role then
copies them over and installs them all in one `apt-get` call, which lets apt
handle the twist-mux → twist-mux-msgs → diagnostic-updater ordering.

```bash
ansible-playbook playbooks/agent-stack-deps.yml
```

## Test run against one unit

`inventory/hosts.ini` lists the whole fleet (`qb3rt_odd` / `qb3rt_even`, one
per network). Limit any run that changes state to one unit first, e.g.
`-l qb3rt-7`.

```bash
cd ansible
ansible-playbook playbooks/https-time-sync.yml
```

Then verify on the device:

```bash
ssh -i ~/.ssh/qb3rt_fleet ubuntu@192.168.2.2 \
  'systemctl status https-time-sync.timer --no-pager; date -u'
```

## Rolling out to the rest of the fleet

Once `qb3rt-2` looks good, add the other units to
`[qb3rt_time_sync_test]` in `inventory/hosts.ini` (or a new group) using
each unit's `SSH_HOST`/`STATIC_IP` from `units/<id>.conf`, then re-run with
`-l <group_or_host>` to control blast radius.

## Deploy + health check

```bash
ansible-playbook playbooks/deploy.yml -l qb3rt-7                                      # stop, sync, leave stopped
ansible-playbook playbooks/deploy.yml -l qb3rt_odd -e qb3rt_restart_profile=full_stack  # stop, sync, restart, check
ansible-playbook playbooks/stack-health.yml -l qb3rt-7                                # read-only: units, orphans, topics
```

`deploy.yml` wraps `deploy/update_project.sh` (one copy of the sync logic) and
adds the lifecycle around it: `qb3rt-down --force` before files change, then
`qb3rt-up <profile>` and `stack-health.yml` when a restart profile is given.
The convention probe always ends with `qb3rt-down --force`, even when a
task fails.
