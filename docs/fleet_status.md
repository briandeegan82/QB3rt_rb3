# QB3rt fleet provisioning status

Generated 2026-10-15. "Fully provisioned" = stamped (serials pinned) +
`base-config.yml` converged + `update_project.sh` payload + convention
probe verified (baseline in `ansible/convention-results/`).

Units 1–15 were stamped under the legacy process before the current
ansible pipeline existed; their serial pinning is real but they still
need `base-config` + payload + probe to reach current standard.

| Unit | Net | Profile | Serials | base-config | Payload | Probe | Conventions | Notes |
|---:|---|---|---|---|---|---|---|---|
| 1  | odd  | yes | pinned | legacy | ? | no | — | legacy-stamped; needs current pipeline |
| 2  | even | yes | pinned | legacy | ? | no | — | legacy-stamped; needs current pipeline |
| 3  | odd  | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=0` | **fully provisioned** — verified by eye (weak IMU signal on its floor) |
| 4  | even | yes | none | no | no | no | — | profile only, never stamped |
| 5  | odd  | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=π` | **fully provisioned** — verified by eye; rover adapter was flapping on USB (re-seated), renesas fw added, lidar rear-facing |
| 6  | even | yes | pinned | ~95% — interrupted at QAIRT staging | no | no | — | rerun base-config to converge, then payload + probe |
| 7  | odd  | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=0` | **fully provisioned** — verified by eye (forward + CCW); original source of fleet `drive_inverted: true` default, but with current board mounting `false` is correct |
| 8  | even | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=0` | **fully provisioned** |
| 9  | odd  | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=0` | **fully provisioned** — verified by eye (probe misclassified; lidar tracked a non-nose obstacle); shipped without `renesas_usb_fw.mem` — dead USB fixed |
| 10 | even | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=π` | **fully provisioned** — reference unit (SLAM/Nav2/YOLOE verified) |
| 11 | odd  | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=0` | **fully provisioned** — manual verify (battery issue, unit held): wheels forward under `false`; yaw from scan cluster at nose obstacle. Adapter serials re-captured (hardware swap) |
| 12 | even | yes | pinned | done | done | no | — | base-config + payload done; **needs floor probe** (clock-skew prone — verify clock before driving) |
| 13 | odd  | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=0` | **fully provisioned** — verified by eye; Wi-Fi radio was disabled (only dock ethernet); renesas fw was missing but ROM fallback kept USB alive |
| 14 | even | yes | pinned | done | done | no | — | base-config + payload done; **needs floor probe** |
| 15 | odd  | yes | pinned | legacy | ? | no | — | legacy-stamped; needs current pipeline |
| 16 | even | yes | pinned | done | done | no | — | base-config + payload done (QAIRT 2.49 fresh); **needs floor probe** |
| 17 | odd  | yes | pinned | done | done | done | `drive_inverted=true, lidar_yaw=0` | **fully provisioned** — IMU-x flipped vs body-x |
| 18 | even | yes | none | no | no | no | — | profile only, never stamped |
| 19 | odd  | yes | none | no | no | no | — | profile only, never stamped |
| 20 | even | yes | none | no | no | no | — | profile only, never stamped |
| 21 | odd  | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=0` | **fully provisioned** — verified by eye; stale serial-TX commands replayed as multi-phase motion (bridge now writes non-blocking) |
| 22 | even | yes | pinned | done | done | done | `drive_inverted=true, lidar_yaw=0` | **fully provisioned** — board mounted backwards, conventions compensate |
| 23 | odd  | yes | none | no | no | no | — | profile only, never stamped |
| 24 | even | yes | pinned | done | done | done | `drive_inverted=true, lidar_yaw=0` | **fully provisioned** — verified by eye (weak IMU signal on carpet) |
| 25 | odd  | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=0` | **fully provisioned** — probe misclassified on a stray return (0.83 m vs the ~0.5 m obstacle) + likely flipped IMU-x; eyeball check caught it |
| 26 | even | yes | none | no | no | no | — | profile only, never stamped |
| 27 | odd  | yes | none | no | no | no | — | profile only, never stamped |
| 28 | even | yes | none | no | no | no | — | profile only, never stamped |
| 29 | odd  | yes | none | no | no | no | — | profile only, never stamped |
| 30 | — | no | — | no | no | no | — | no profile |
| 31 | — | no | — | no | no | no | — | no profile |
| 32 | — | no | — | no | no | no | — | no profile |

Stray: `units/42.conf` exists with no serials — probably a test artifact,
not a real unit.

## Fleet-wide known issues

- **No WAN on ROS_NET_EVEN or ROS_NET_ODD** — `https-time-sync` fails on
  every unit; clocks drift ~2 weeks stale and must be set manually (or a
  local NTP/chrony peer configured). TLS and DDS time alignment suffer.
- **`drive_inverted` is mostly `false`** (true: 17, 22, 24; false: 3, 5,
  7, 8, 9, 10, 11, 13, 21, 25). Fleet default in `wave_rover_bridge.yaml` is now
  `false`; the eyeball verify step in the probe is still not optional —
  lidar delta has misread non-nose obstacles (unit 9) and IMU-x sign is
  not fleet-consistent (unit 17).
- **Missing `renesas_usb_fw.mem` = dead USB bus** — unit 9's image lacked
  the Renesas uPD720201 host-controller firmware; no ttyUSB devices at
  all. Fixed by installing the blob + PCI rebind; `qb3rt_base` now stages
  `renesas_usb_fw.mem` fleet-wide. If a unit shows zero `/dev/ttyUSB*`,
  check `dmesg | grep renesas` first.
- **Unit 17's IMU-x is flipped vs body-x** — IMU accel sign is not a
  reliable forward indicator fleet-wide; the probe's lidar-range delta is
  the primary discriminant for exactly this reason.
- **Carpet/high-friction floors** produce sub-threshold motion in the
  probe pulse — classification then falls back to IMU which may read the
  wrong sign. Verify by eye on soft floors.
