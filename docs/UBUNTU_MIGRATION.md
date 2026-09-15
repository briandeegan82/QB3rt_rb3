# QB3rt on Ubuntu 24.04 — fleet provisioning

The RB3 Gen 2 fleet now runs the **Canonical Ubuntu 24.04 LTS** image instead of
the old Qualcomm Linux / QIRP Yocto image. Provisioning uses a **golden-image**
model: configure one reference unit, capture its rootfs, flash the fleet, then
stamp per-unit identity over SSH.

## What changed from the QIRP flow

| | Old (QIRP/Yocto) | New (Ubuntu 24.04) |
|---|---|---|
| Login | `root` | `ubuntu` (sudo), home `/home/ubuntu` |
| Deploy transport | adb over USB | **SSH / rsync** over the network |
| `/usr` | read-only ostree (`remount,rw`) | normal writable rootfs |
| ROS source | baked into image | **apt** `ros-jazzy-*` **+ QIRP PPAs** (`qrb_ros_*`) |
| Custom pkgs | ~1 GB overlay tarball + `--bootstrap` | built on-device with `colcon` → `/opt/qb3rt/install` |
| Env | `QB3rt_env.sh` (`qirp-setup.sh`) | `/etc/profile.d/qb3rt-ros-env.sh` |

The on-board **OV9282 camera + IMU** ORB-SLAM3 VIO path depends on the Qualcomm
**`qrb_ros_camera` / `qrb_ros_imu`** nodes, which ship only as apt debs from the
QIRP PPAs — that is why the base ROS is apt (not RoboStack/conda).

## Three phases

### Phase A — build the reference unit (once)

Flash ONE RB3 with stock Ubuntu 24.04, get SSH working as `ubuntu`, then on that
unit (clone this repo to it first):

```bash
reference/00_apt_base.sh        # ROS + QIRP apt sources; install stack + qrb_ros_* + build deps
#   -> confirm qrb_ros names:  apt search ros-jazzy-qrb-ros
# Edit reference/qb3rt.repos: set the real orb_slam3_ros / wave_rover_controller URLs
reference/10_build_custom.sh    # colcon-build custom pkgs -> /opt/qb3rt/install
reference/20_fleet_config.sh    # DDS/sysctl/udev/env/NTP (fleet-wide config)
```

Verify (after logging out/in so `/etc/profile.d` is sourced):

```bash
ros2 pkg list | grep -E 'qrb_ros|orb_slam3|depthai|wave_rover|navigation2|slam_toolbox'
ros2 launch QB3rt full_stack.launch.py enable_nav:=false    # camera+IMU up, ORB-SLAM3 tracks
```

### Phase B — capture + flash the golden image (once)

```bash
sudo capture/make_golden.sh --out /media/usb/qb3rt-golden.img
# then per the printed instructions:
fastboot flash system qb3rt-golden.img   # confirm partition name first (see script)
```

Identity is reset **in the image copy**, so the reference unit stays usable.
Flash the image to every other unit.

### Phase C — stamp each unit (per unit, from the laptop)

Fill in `units/<id>.conf` (SSH host, hostname, `DOMAIN_ID`, USB CP2102N serials,
Wi-Fi), then:

```bash
stamp/stamp_unit.sh --unit 46927088          # hostname + ROS_DOMAIN_ID + /dev/rplidar,/dev/wave_rover
stamp/stamp_unit.sh --unit 46927088 --wifi   # also join lab Wi-Fi
```

## Day-to-day ops

```bash
deploy/update_project.sh --unit 46927088     # push launch/config/script edits (no reflash)
handoff/clean_unit.sh   --unit 46927088      # class handoff: wipe student Wi-Fi + home state
```

## Prerequisites / notes

- **SSH**: fleet automation uses a **dedicated passphraseless key** `~/.ssh/qb3rt_fleet`
  (public half baked into the golden image's `~ubuntu/.ssh/authorized_keys`). An
  `~/.ssh/config` block (`Host qb3rt-*` → that key, user `ubuntu`) makes `stamp`/
  `deploy`/`handoff` use it automatically. The same key is Ansible's control key.
- **Passwordless sudo** for `become` is installed deterministically by
  `20_fleet_config.sh` as `/etc/sudoers.d/90-qb3rt` (not reliant on cloud-init).
- **Ansible-ready**: nodes already have Python 3.12 + `python3-apt`, SSH via
  `ssh.socket` at boot, the fleet key, and NOPASSWD sudo — Ansible itself installs
  only on the laptop control node (agentless).
- **qrb_ros package names** can drift between QIRP releases — `00_apt_base.sh`
  warns (not fails) on any missing; adjust `QRB_PKGS` in it if renamed.
- **`reference/qb3rt.repos`** ships placeholder URLs for `orb_slam3_ros` and
  `wave_rover_controller`; `10_build_custom.sh` refuses to run until you replace
  the `TODO confirm` entries with your real repos.
- **Fastboot partition name** for the rootfs varies by image build — confirm via
  `fastboot getvar all` and the Canonical RB3 image Release Notes before flashing.
- **Host side** (Nav2/RViz): see the `qb3rt_host` repo; it connects to
  `ubuntu@<unit>` now (was `root@`).
```
