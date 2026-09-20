# RB3 setup / update — quick reference

Fleet runs **Ubuntu 24.04**, provisioned golden-image + SSH (not adb). This
page is a quick command reference; the full walkthrough (flashing a golden
image, per-unit stamping, prerequisites) is
**[docs/UBUNTU_MIGRATION.md](docs/UBUNTU_MIGRATION.md)** — read that first if
you haven't provisioned a unit before.

Laptop entry point (this git repo, cloned locally):

```bash
cd /path/to/QB3rt          # git clone of briandeegan82/QB3rt_rb3
```

## Fresh unit (flashed with the golden image, not yet stamped)

```bash
stamp/discover.sh                                  # find fresh units + IPs on the lab Wi-Fi
# fill in units/<id>.conf: SSH host, hostname, DOMAIN_ID, USB serials, Wi-Fi
stamp/stamp_unit.sh --unit <id> --host ubuntu@<discovered-ip> --wifi
deploy/update_project.sh --unit <id>                # install the QB3rt project (not baked into the golden image)
```

## Routine update (project already installed)

```bash
deploy/update_project.sh --unit <id>
# or by IP directly, no unit profile needed:
deploy/update_project.sh --host ubuntu@<ip>
```

Syncs `launch/`, `config/`, `urdf/`, `scripts/`, `behavior_trees/`, `rviz/`,
`docs/`, the `vendor_overrides/wave_rover_controller` bridge, and `/etc`
system configs (CycloneDDS, sysctl, udev, env) over SSH. No reflash, no
rebuild. Relaunch affected nodes afterward.

## Class handoff

```bash
handoff/clean_unit.sh --unit <id>
```

| action | detail |
|--------|--------|
| Wi-Fi | Deletes NetworkManager profiles except the unit's primary SSID |
| `/home/ubuntu` | Removes project/state directories; keeps `.ssh` |
| History | Clears `.bash_history`, removes `.ros` |

## Fleet-wide config changes

Per-unit deploy is for *this project's* code. For config that should apply
identically across the whole fleet (clock sync, package backfills, DDS
tuning), use Ansible instead — see **[ansible/README.md](ansible/README.md)**:

```bash
cd ansible
ansible-playbook playbooks/<name>.yml -l qb3rt_fleet   # or -l qb3rt_odd / qb3rt_even / a single host
```

## Per-unit profiles

`units/<id>.conf`: `SSH_HOST`, `HOSTNAME`, `DOMAIN_ID`, `STATIC_IP`,
`USB_RPLIDAR_SERIAL`, `USB_WAVE_ROVER_SERIAL`, `WIFI_SSID`/`WIFI_PSK`,
`WIFI_MAC`, `WIFI_RESERVE_SSID`/`WIFI_RESERVE_PSK`. Discover USB serials with
`udevadm info -q property -n /dev/ttyUSB*`; discover the Wi-Fi MAC with
`ip link show wlan0 | grep ether` (over SSH).

## Prerequisites

- Fleet SSH key: `~/.ssh/qb3rt_fleet` (public half baked into the golden
  image's `authorized_keys`). Login is `ubuntu@<ip>`, not `root@`.
- The ROS/DDS environment auto-loads at login via
  `/etc/profile.d/qb3rt-ros-env.sh` — nothing to source by hand for an
  interactive shell.

## Looking for the old adb-based flow?

Provisioning used to be adb-over-USB against a read-only ostree image
(`deploy_via_adb.sh`, `fetch_overlay.sh`, `/root/QB3rt`, `rover_env.sh`).
That flow is **deprecated** and archived under `legacy_qirp/` — see
[`legacy_qirp/README.md`](legacy_qirp/README.md) for the old↔new command
mapping. Do not run those scripts against a fleet unit; they target the
Yocto image the fleet no longer runs.
