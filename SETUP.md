# One-click RB3 setup / update

Laptop entry point (this git repo):

```bash
cd /path/to/QB3rt          # git clone of briandeegan82/QB3rt_rb3
./fetch_overlay.sh         # once: download slim share/lib from GitHub Release
./deploy_via_adb.sh --unit 46927088 --clean --bootstrap
```

If `./fetch_overlay.sh` fails (no Release / no `overlay_release.env` yet), either:

- copy a local `qb3rt-overlay.tar.gz` next to the script and extract into
  `./overlay/`, or
- point `OVERLAY_ROOT` at an existing slim `share/` + `lib/` tree, or
- build one with `./pack_overlay.sh` from a machine that still has the dump
  (see below).

Then re-run deploy with `--bootstrap`.

## Layout

| path | in git? | role |
|------|---------|------|
| `deploy_via_adb.sh` | yes | USB one-click clean / bootstrap / update |
| `clean_rb3_on_device.sh` | yes | student WiFi + `/root` wipe |
| `deploy.sh` | yes | on-robot sync into `/usr/share/QB3rt` |
| `units/*.conf` | yes | per-robot DOMAIN_ID, USB serials, WIFI_KEEP |
| `bootstrap_packages.list` | yes | which overlay packages `--bootstrap` restores |
| `overlay/` | **no** | local cache of slim `share/` + `lib/` (~0.5 GB) |
| `qb3rt-overlay.tar.gz` | **no** | packed overlay for GitHub Releases |

## Why `share/` / `lib/` are not in git

A full `/usr/share` dump is ~1 GB (mostly stock QIRP ROS packages). Bootstrap
only needs custom overlay packages (ORB-SLAM, slam_toolbox, wave_rover, …) —
still hundreds of MB of binaries / ORBvoc, which exceeds GitHub's git file
limits and would bloat clones.

**Store them as a GitHub Release asset**, not in the git history:

1. On a machine that has the dump (or after `./pack_overlay.sh` from
   `../share` + `../lib`):
   ```bash
   ./pack_overlay.sh --tarball    # writes ./overlay/ + qb3rt-overlay.tar.gz
   gh release create overlay-2026-08-03 qb3rt-overlay.tar.gz \
     --title "QB3rt bootstrap overlay" \
     --notes "Slim share/lib for deploy_via_adb --bootstrap"
   ```
2. Set the tag in `overlay_release.env` (from `overlay_release.env.example`).
3. Anyone else: `./fetch_overlay.sh` then deploy with `--bootstrap`.

Alternatives if Releases are inconvenient: university object storage / S3 /
shared NAS — point `OVERLAY_ROOT=/mnt/...` or drop a tarball URL into a
forked `fetch_overlay.sh`. Avoid Git LFS for this; Release assets are simpler
and do not inflate every clone.

## Class handoff (`--clean`)

```bash
./deploy_via_adb.sh --unit 46927088 --clean --bootstrap
./deploy_via_adb.sh --unit 46927088 --clean-only
```

| action | detail |
|--------|--------|
| WiFi | Deletes NetworkManager profiles except `WIFI_KEEP` (unit profile). |
| `/root` | Removes non-hidden `/var/roothome` entries (`QB3rt`, etc.). Keeps `.ssh`. |
| History | Clears `.bash_history`; removes `.ros`. |

## Routine update

```bash
./deploy_via_adb.sh --unit 46927088
```

## After deploy (on device)

```bash
adb shell
source /root/rover_env.sh
ros2 launch QB3rt full_stack.launch.py enable_nav:=false
```

## Per-unit profiles

`units/<name>.conf`: `ADB_SERIAL`, `DOMAIN_ID`, `USB_RPLIDAR_SERIAL`,
`USB_WAVE_ROVER_SERIAL`, `WIFI_KEEP`. Discover USB serials with
`udevadm info -q property -n /dev/ttyUSB*`.

## `/usr` and `rover_env.sh`

`/usr` is ostree read-only until `source /root/rover_env.sh`. The deploy script
does this over adb before writing; interactive shells must still source it
before `ros2 launch`.
