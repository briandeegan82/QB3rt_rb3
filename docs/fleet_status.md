# QB3rt fleet provisioning status

Generated 2026-10-15. "Fully provisioned" = stamped (serials pinned) +
`base-config.yml` converged + `update_project.sh` payload + convention
probe verified (baseline in `ansible/convention-results/`).

Units 1–15 were stamped under the legacy process before the current
ansible pipeline existed; their serial pinning is real but they still
need `base-config` + payload + probe to reach current standard.

| Unit | Net | Profile | Serials | base-config | Payload | Probe | Conventions | Notes |
|---:|---|---|---|---|---|---|---|---|
| 1  | odd  | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=0` | **fully provisioned** — `.2.101` on ROS_NET_5 (special-case — `.2.1` is the gateway; playbook var-shadowing bug fixed), ODD fallback; serials verified (rplidar→ttyUSB1, wave_rover→ttyUSB0); probe verified by eye (forward + CCW correct; IMU under threshold — probable flipped IMU-x, lidar delta decisive) |
| 2  | even | yes | re-pinned 2026-10-21 (hw swapped; serials re-captured) | done | done | done | `drive_inverted=true, lidar_yaw=π` | **fully provisioned** — ROS_NET_5 profile was stamp-deleted mid-process (WIFI_KEEP lacked it — fixed fleet-wide), recovered via serial + `nmcli connect`; `.2.2` on ROS_NET_5, EVEN fallback; probe verified by eye (weak IMU signal: launch/rotation under threshold, lidar + eyeball decisive); lidar rear-facing |
| 3  | odd  | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=0` | **fully provisioned** — verified by eye (weak IMU signal on its floor) |
| 4  | even | yes | none | no | no | no | — | profile only, never stamped |
| 5  | odd  | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=π` | **fully provisioned** — verified by eye; rover adapter was flapping on USB (re-seated), renesas fw added, lidar rear-facing |
| 6  | even | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=0` | **fully provisioned** — base-config + payload + ROS_NET_5 (`.2.6`, EVEN fallback created) 2026-10-21; probe verified by eye — IMU-x flipped vs body-x (probe wrote `true`, lidar delta showed motion toward nose obstacle; eyeball caught it, re-pulse forward under `false`) |
| 7  | odd  | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=0` | **fully provisioned** — verified by eye (forward + CCW); original source of fleet `drive_inverted: true` default, but with current board mounting `false` is correct |
| 8  | even | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=0` | **fully provisioned** |
| 9  | odd  | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=0` | **fully provisioned** — verified by eye (probe misclassified; lidar tracked a non-nose obstacle); shipped without `renesas_usb_fw.mem` — dead USB fixed |
| 10 | even | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=π` | **fully provisioned** — reference unit (SLAM/Nav2/YOLOE verified) |
| 11 | odd  | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=0` | **fully provisioned** — manual verify (battery issue, unit held): wheels forward under `false`; yaw from scan cluster at nose obstacle. Adapter serials re-captured (hardware swap) |
| 12 | even | yes | pinned | done | done | done | `drive_inverted=true, lidar_yaw=0` | **fully provisioned** — verified by eye; probe sensors under threshold (soft floor) but two backward pulses under `false` + forward under `true` were unambiguous |
| 13 | odd  | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=0` | **fully provisioned** — verified by eye; Wi-Fi radio was disabled (only dock ethernet); renesas fw was missing but ROM fallback kept USB alive |
| 14 | even | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=π` | **fully provisioned** — clean probe run; all pulses forward under `false`; nose object at raw +117° (rear hemisphere) → lidar rear-facing |
| 15 | odd  | yes | pinned (verified vs hardware 2026-10-21) | done | done | done | `drive_inverted=false, lidar_yaw=π` | **fully provisioned** — provisioned over phone hotspot (`Brians iPhone` profile); `ROS_NET_5` staged at `.2.15` priority 20 — auto-migrates in range; ODD fallback; base-config full completion incl. https-time-sync; serials verified (rplidar→ttyUSB0, wave_rover→ttyUSB1); probe verified by eye — IMU-x flipped vs body-x; lidar rear-facing |
| 16 | even | yes | pinned | done | done | done | `drive_inverted=true, lidar_yaw=π` | **fully provisioned** — full-stack verified: eyeball + RF2O bidirectional + raw scan; lidar rear-facing; dead IMU healed by sensor-watch |
| 17 | odd  | yes | pinned | done | done | done | `drive_inverted=true, lidar_yaw=0` | **fully provisioned** — IMU-x flipped vs body-x |
| 18 | even | yes | pinned 2026-10-22 (first capture — conf serials were blank) | done | done | done | `drive_inverted=false, lidar_yaw=π` | **fully provisioned** — `.2.18` on ROS_NET_5, EVEN fallback; apt tree repaired via new role task; serials verified (rplidar→ttyUSB1, wave_rover→ttyUSB0); probe: linear pulse drove forward into the nose object (crashed it — eyeball confirms `false`); lidar yaw measured manually from standing scan (object at raw −177°) — rear-facing lidar |
| 19 | odd  | yes | pinned 2026-10-21 (first capture — conf serials were blank) | done | done | done | `drive_inverted=false, lidar_yaw=0` | **fully provisioned** — `.2.19` on ROS_NET_5, ODD fallback; stamp stranded it on ODD (fixed in `_stamp_on_device.sh`), recovered via UART; probe verified by eye — IMU-x flipped vs body-x (probe wrote `true`; eyeball + lidar delta → `false`, forward confirmed on carpet with stronger pulse) |
| 20 | even | yes | pinned 2026-10-22 (first capture — conf serials were blank) | done | done | done | — | provisioned 2026-10-22; `.2.20` on ROS_NET_5 (was DHCP `.2.209`), EVEN fallback; apt tree repaired via role task; serials verified (rplidar→ttyUSB1, wave_rover→ttyUSB0); **probe remaining** |
| 21 | odd  | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=0` | **fully provisioned** — verified by eye; stale serial-TX commands replayed as multi-phase motion (bridge now writes non-blocking) |
| 22 | even | yes | pinned | done | done | done | `drive_inverted=true, lidar_yaw=0` | **fully provisioned** — board mounted backwards, conventions compensate |
| 23 | odd  | yes | pinned 2026-10-21 (first capture — conf serials were blank) | done | done | done | — | provisioned 2026-10-21 over phone hotspot → migrated to `.2.23` on ROS_NET_5 automatically after stamp; ODD fallback; first unit through the fixed stamp (kept ROS_NET_5 preferred — no stranding); apt tree repaired (QAIRT 2.49 phantom deps — role task added, fleet-wide repair pushed); serials verified (rplidar→ttyUSB1, wave_rover→ttyUSB0); **probe remaining** |
| 24 | even | yes | pinned | done | done | done | `drive_inverted=true, lidar_yaw=0` | **fully provisioned** — verified by eye (weak IMU signal on carpet) |
| 25 | odd  | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=0` | **fully provisioned** — probe misclassified on a stray return (0.83 m vs the ~0.5 m obstacle) + likely flipped IMU-x; eyeball check caught it |
| 26 | even | yes | none | no | no | no | — | profile only, never stamped |
| 27 | odd  | yes | none | no | no | no | — | profile only, never stamped |
| 28 | even | yes | none | no | no | no | — | profile only, never stamped |
| 29 | odd  | yes | none | no | no | no | — | profile only, never stamped |
| 30 | — | no | — | no | no | no | — | no profile |
| 31 | — | no | — | no | no | no | — | no profile |
| 32 | — | no | — | no | no | no | — | no profile |
| 42 | even | yes | pinned | done | done | done | `drive_inverted=false, lidar_yaw=π` | **fully provisioned** — golden sample / dev target (domain 42); serials captured + stamped same day; IMU-x flipped vs body-x (probe trusted lidar delta); lidar rear-facing |

## Pending fleet actions

- **Compressed-camera republish payload** — `perception.launch.py` gained an
  `image_transport republish` node (`/image_raw/compressed`, arg
  `enable_compressed` default true) on 2026-10-17. Deployed and verified on
  units 7 and 42 only; the other 15 provisioned units need a payload rerun
  (`deploy/update_project.sh --host ubuntu@192.168.2.<N>`) when next online.

## Network layout (2026-10-17)

- **All 17 provisioned units are on ROS_NET_5** at `192.168.2.<unit#>` —
  profile `ROS_NET_5` (priority 20, static IP, gw 192.168.2.1) pushed via
  `playbooks/wifi-ros-net-5.yml` and switched fleet-wide (`switch_now=true`).
  Inventory `ansible_host`s now reflect the .2 addresses.
- **Fallbacks**: each unit's dedicated net stays autoconnect at priority 10 —
  evens on ROS_NET_EVEN (same .2.x fabric as ROS_NET_5), odds on ROS_NET_ODD
  at their original `192.168.1.<id>` statics. ROS_NET_ODD's AP now DHCPs on
  192.168.3.0/24 but .1.x still works at L2 — to reach a fallen-back odd
  unit, join ROS_NET_ODD and add a .1 alias:
  `nmcli device modify wlp4s0 +ipv4.addresses 192.168.1.175/24`.
- **Unit 1 exception**: `192.168.2.1` is the ROS_NET_5 gateway, so its
  ROS_NET_5 address is `192.168.2.101` (host var `ros_net_5_static_ip` in
  inventory).
- **DDS peer**: `/etc/qb3rt/dds_peer_ros_net_5` = `192.168.2.175` (laptop's
  ROS_NET_5 DHCP lease) written on all 17 units. If the laptop's lease
  changes, re-run `wifi-ros-net-5.yml -e ros_net_5_laptop_ip=<new>` — or
  reserve the laptop's IP on the AP.

## Fleet-wide known issues

- **No WAN on ROS_NET_EVEN or ROS_NET_ODD** — `https-time-sync` fails on
  every unit; clocks drift ~2 weeks stale and must be set manually (or a
  local NTP/chrony peer configured). TLS and DDS time alignment suffer.
- **`drive_inverted` is mostly `false`** (true: 2, 12, 16, 17, 22, 24; false:
  1, 3, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15, 19, 21, 25, 42). Fleet default in `wave_rover_bridge.yaml` is now
  `false`; the eyeball verify step in the probe is still not optional —
  lidar delta has misread non-nose obstacles (unit 9) and IMU-x sign is
  not fleet-consistent (unit 17).
- **Missing `renesas_usb_fw.mem` = dead USB bus** — unit 9's image lacked
  the Renesas uPD720201 host-controller firmware; no ttyUSB devices at
  all. Fixed by installing the blob + PCI rebind; `qb3rt_base` now stages
  `renesas_usb_fw.mem` fleet-wide. If a unit shows zero `/dev/ttyUSB*`,
  check `dmesg | grep renesas` first.
- **Units 17 and 42 have IMU-x flipped vs body-x** — IMU accel sign is not a
  reliable forward indicator fleet-wide; the probe's lidar-range delta is
  the primary discriminant for exactly this reason.
- **Carpet/high-friction floors** produce sub-threshold motion in the
  probe pulse — classification then falls back to IMU which may read the
  wrong sign. Verify by eye on soft floors.
