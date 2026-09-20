# QB3rt — teaching AGV on Qualcomm RB3 Gen 2

**QB3rt** is a classroom / lab AGV stack for a fleet of skid-steer rovers. Each unit is a [WAVE ROVER](https://www.waveshare.com/) base with a **Qualcomm Robotics RB3 Gen 2** running **Ubuntu 24.04 + ROS 2 Jazzy** as the onboard computer. Students and instructors use it to explore ROS 2 perception, localization, mapping, and Nav2 navigation on real hardware.

This repository is the project source: launch files, configs, calibration, deploy tooling, and docs.

> **Fleet runs Ubuntu 24.04, not the old Qualcomm Linux/QIRP Yocto image.**
> Provisioning is a golden-image + SSH flow — see
> **[`docs/UBUNTU_MIGRATION.md`](docs/UBUNTU_MIGRATION.md)**. The `reference/`,
> `stamp/`, `handoff/`, `deploy/`, `capture/`, and `ansible/` directories are
> the current tooling; the old adb/overlay flow is archived, deprecated, and
> **not** part of this doc — see `legacy_qirp/README.md` if you need history.
> Base ROS is apt `ros-jazzy-*` + the QIRP PPAs (`qrb_ros_camera`/`qrb_ros_imu`
> — "QIRP" here names the apt package source, not the old image); custom
> packages build on-device into `/opt/qb3rt/install`.

![QB3rt fleet — WAVE ROVER base + Qualcomm RB3 Gen 2 with RPLIDAR](images/QB3rt.jpeg)

![Nav2 in RViz — mapped space, costmap, and planned path](images/QB3rt_map.png)

**Repo:** [briandeegan82/QB3rt_rb3](https://github.com/briandeegan82/QB3rt_rb3)

---

## What it does

| Layer | Role |
|-------|------|
| **Drive** | WAVE ROVER motors via a calibrated HTTP bridge (`/cmd_vel` → wheel commands) |
| **VIO** | ORB-SLAM3 mono-inertial on the OV9282 tracking camera + RB3 IMU |
| **Odometry** | `robot_localization` EKF fuses VIO deltas, IMU yaw rate, and wheel `vx` |
| **2D SLAM** | RPLIDAR C1 + `slam_toolbox` builds `/map` and owns `map→odom` |
| **Nav** | Nav2 (planner / controller / BT) — onboard **or** remote on a laptop |

Typical teaching split: the robot runs sensing + SLAM; the laptop runs Nav2 +
RViz so students can watch the map and send goals over Wi‑Fi.

---

## Hardware (per robot)

- WAVE ROVER skid-steer chassis (open-loop drive; cannot pivot in place)
- Qualcomm RB3 Gen 2 running Ubuntu 24.04 + ROS 2 Jazzy
- OV9282 tracking camera + onboard IMU → ORB-SLAM3 VIO
- RPLIDAR C1 on USB (`/dev/rplidar`)
- Chassis serial bridge on USB (`/dev/wave_rover`)

USB device nodes come from udev rules (`system/99-agv-serial.rules`) keyed by
each unit’s CP2102N chip serials in `units/<id>.conf`.

---

## Software layout

| Path | Role |
|------|------|
| This git clone (laptop) | **Canonical project source. Edit here**, then push with `deploy/update_project.sh --unit <id>`. |
| `/opt/qb3rt/install/share/QB3rt` (on-device) | Install / runtime copy, synced by `deploy/update_project.sh`. Nodes load from here. **Do not edit by hand** — the next deploy overwrites it. |
| `vendor_overrides/` | Patches for vendor packages that have no editable source on-device (wave_rover bridge + calibration); also synced by `deploy/update_project.sh`. |
| `laptop/` | Remote-Nav2 bundle for the instructor/student laptop. |
| `units/` | Per-robot profiles (SSH host, `ROS_DOMAIN_ID`, static IP, USB serials, Wi‑Fi SSID/MAC). |
| `ansible/`, `reference/`, `stamp/`, `deploy/`, `handoff/`, `capture/` | Fleet provisioning/config/deploy tooling — see `docs/UBUNTU_MIGRATION.md`. |
| GitHub | Off-robot backup / class distribution of this tree. |

The on-device rootfs is a normal writable filesystem — no remount needed. The
ROS/DDS environment (`ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`)
auto-loads at SSH login via `/etc/profile.d/qb3rt-ros-env.sh`; a non-interactive
`ssh host "command"` should source it explicitly first, since profile.d isn't
guaranteed to run for non-login shells.

---

## Quick start

### A. Fresh unit — flash + provision (from a laptop)

Needs: this git clone on the laptop, the fleet SSH key (`~/.ssh/qb3rt_fleet`),
and a unit flashed with the golden Ubuntu 24.04 image (or stock Ubuntu for a
new reference unit). Full details: **[docs/UBUNTU_MIGRATION.md](docs/UBUNTU_MIGRATION.md)**.

```bash
stamp/discover.sh                            # find fresh units + IPs on the lab Wi-Fi
stamp/stamp_unit.sh --unit <id> --host ubuntu@<discovered-ip> --wifi
deploy/update_project.sh --unit <id>         # install the QB3rt project itself
```

For class handoff (wipe student Wi‑Fi + `/home/ubuntu` state, keep the
provisioning profile):

```bash
handoff/clean_unit.sh --unit <id>
```

### B. Routine update (project already installed)

```bash
cd /path/to/QB3rt                # this git clone, edit here
deploy/update_project.sh --unit <id>   # push launch/config/scripts over SSH, no reflash
# then relaunch affected nodes on the robot
```

Fleet-wide config changes (DDS tuning, clock sync, package backfills) go
through Ansible instead — see [`ansible/README.md`](ansible/README.md).

---

## Bring-up (on the robot)

**Before launching:** let the RB3 clock sync (or freeze NTP). The board has no working RTC; a mid-run clock step breaks the EKF. See
[laptop/README.md](laptop/README.md) and the note below.

### Recommended: two-step (you gate SLAM)

Start odometry, confirm VIO is healthy, *then* start lidar SLAM so a warming-up VIO never corrupts the map:

SSH in as `ubuntu@<unit-ip>` — the ROS/DDS env auto-loads at login. Then:

```bash
# 1) base + VIO + EKF (no SLAM, no Nav2)
ros2 launch QB3rt odometry_bringup.launch.py
#    • wait for /odometry/filtered
#    • drive a small figure-8 for ORB-SLAM3 init, then STAND STILL until
#      the relay logs "VIO metric convergence confirmed" (/vio/ready=true)
#    • parked: ros2 topic echo /odometry/filtered --field twist.twist  → ~0

# 2) RPLIDAR + slam_toolbox → map→odom
ros2 launch QB3rt slam_toolbox.launch.py

# 3) Nav2 on the laptop — follow laptop/README.md
```

### One-shot (automatic `/vio/ready` gate)

```bash
# full stack onboard (SLAM + Nav2 on the robot)
ros2 launch QB3rt full_stack.launch.py

# robot sensing + SLAM only; Nav2 remote on the laptop
ros2 launch QB3rt full_stack.launch.py enable_nav:=false
```

Other launches: `perception.launch.py` (camera/VIO), `lidar_slam.launch.py` (camera-free lidar SLAM), `odom_square_test.launch.py` (drive calibration — see [docs/odometry_calibration.md](docs/odometry_calibration.md)).

---

## Architecture (REP-105 frames)

```
map ──(slam_toolbox)──> odom ──(EKF)──> base_footprint ──(URDF)──> base_link
                                                              └─> wheels / laser / imu / cams
```

| Transform | Owner | Notes |
|-----------|--------|--------|
| `map→odom` | slam_toolbox | From `/scan` |
| `odom→base_footprint` | EKF only | Fuses VIO (differential), IMU yaw rate, wheel `vx` |
| `base_footprint→…` | URDF / static TF | Sensors and links |

**VIO trust gate** (`scripts/orbslam3_pose_to_odom.py`): lateral motion is VIO-only-observable. The relay feeds the EKF only after VIO is gravity-aligned *and* metrically converged (wheels ~0 and VIO ~0 for a few seconds). It also drops frames implying >0.6 m/s (above the ~0.42 m/s platform top speed). End the figure-8 init with a brief stop so the gate can latch.

**Bridge:** `publish_tf: false` on the wave_rover driver — otherwise two parents claim `base_link` and Nav2 breaks.

**Drive model:** open-loop calibrated feedforward (deadband, max speed, skid turn gain, left/right trim). Firmware `motor_cmd_max: 0.5`. The base cannot spin in place; Nav2 is configured for arc-only motion (no rotate-to-heading /Spin recovery). Details: [docs/odometry_calibration.md](docs/odometry_calibration.md).

---

## Laptop Nav2 + RViz

Install Nav2, copy the laptop bundle, and peer CycloneDDS with the robot — see **[laptop/README.md](laptop/README.md)**. Summary:

1. Robot: `full_stack.launch.py enable_nav:=false` (or the two-step above)
2. Laptop: Nav2 + RViz; `/cmd_vel` goes over Wi‑Fi to the bridge
3. Bridge `cmd_timeout: 0.5` stops the wheels if the link drops

---

## Operational gotchas

1. **Clock** — Sync (or disable NTP after sync) *before* launching. A boot-time timesync step while the EKF is running causes wild odometry drift. Fleet-wide this is handled by the `https_time_sync` Ansible role (RB3 has no working RTC).
2. **DDS discovery** — Lab APs have multicast off, so CycloneDDS needs explicit unicast peers on both the robot (`/etc/qb3rt/cyclonedds.xml`) and laptop side — see `docs/network_architecture.html`.
3. **`ROS_DOMAIN_ID`** — Unique per robot on the same Wi‑Fi (`units/*.conf`).
4. **VIO init** — Figure-8, then stand still until `/vio/ready` before trusting maps / Nav2.
5. **USB serials** — After swapping lids or lids/bases, update `units/<id>.conf` and udev rules so `/dev/rplidar` and `/dev/wave_rover`
   point at the right CP2102Ns.

---

## Documentation map

| Doc | Contents |
|-----|----------|
| [SETUP.md](SETUP.md) | Quick-reference: provision a fresh unit, routine updates, class handoff |
| [docs/UBUNTU_MIGRATION.md](docs/UBUNTU_MIGRATION.md) | Full golden-image + SSH provisioning flow (canonical) |
| [docs/network_architecture.html](docs/network_architecture.html) | Fleet network layout, DDS/CycloneDDS unicast setup, Ansible |
| [ansible/README.md](ansible/README.md) | Fleet-wide config roles (clock sync, package backfills) |
| [laptop/README.md](laptop/README.md) | Remote Nav2, DDS, clock sync |
| [docs/slam_nav2.md](docs/slam_nav2.md) | Live SLAM + Nav2 (onboard vs laptop) |
| [docs/odometry_calibration.md](docs/odometry_calibration.md) | Wheel / bridge calibration |
| [docs/orbslam3_calibration.md](docs/orbslam3_calibration.md) | Camera / VIO calibration |
| [docs/system_audit_2026-07-16.md](docs/system_audit_2026-07-16.md) | Historical device audit (dated) |
| [legacy_qirp/README.md](legacy_qirp/README.md) | Archived adb/Yocto provisioning flow — deprecated, history only |

---

## License

Apache-2.0 — see `package.xml`.