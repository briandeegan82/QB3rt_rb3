# Legacy QIRP (Qualcomm Linux / Yocto) provisioning — DEPRECATED

These scripts provisioned the RB3 fleet when it ran the **Qualcomm Linux / QIRP
Yocto image** (read-only ostree `/usr`, `root` login, deploy over **adb/USB**,
stock ROS baked into the image, custom packages restored from a ~1 GB overlay
tarball).

The fleet has moved to the **Canonical Ubuntu 24.04** image. The replacement
workflow lives one level up:

| Old (here) | New (Ubuntu) |
|---|---|
| `deploy_via_adb.sh` | `stamp/stamp_unit.sh` + `deploy/update_project.sh` (SSH) |
| `deploy.sh` | `deploy/update_project.sh` |
| `clean_rb3_on_device.sh` | `handoff/clean_unit.sh` + `handoff/_clean_on_device.sh` |
| `fetch_overlay.sh` / `pack_overlay.sh` / `bootstrap_packages.list` | gone — stock via `apt`, custom via `reference/10_build_custom.sh` |
| `QB3rt_env.sh` | `system/qb3rt-ros-env.sh` |

See `docs/UBUNTU_MIGRATION.md`. Kept for reference/history only; do not run.
