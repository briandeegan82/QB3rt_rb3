# Live SLAM + Nav2 (QB3rt)

Live 2D SLAM (`slam_toolbox`) and Nav2 for the QB3rt skid-steer AGV. The robot
maps while it drives; there is no pre-saved map requirement. Motors are driven
by the calibrated HTTP/serial bridge (`vendor_overrides/wave_rover_controller/`).

For first-time setup see [README.md](../README.md) and [SETUP.md](../SETUP.md).
For remote Nav2 on a laptop see [laptop/README.md](../laptop/README.md).

## Preflight

SSH in as `ubuntu@<unit-ip>` (`ssh -i ~/.ssh/qb3rt_fleet ubuntu@<ip>`) — the
ROS/DDS environment (domain id, RMW, CycloneDDS URI) auto-loads at login via
`/etc/profile.d/qb3rt-ros-env.sh`, nothing to source by hand. Let the RB3
clock sync (or freeze NTP) **before** launching — see laptop/README.md.

## Architecture / TF ownership

| Transform | Owner | Source |
|-----------|--------|--------|
| `map → odom` | `slam_toolbox` | RPLIDAR `/scan` |
| `odom → base_footprint` | EKF (`robot_localization`) | `config/ekf.yaml` |
| `base_footprint → …` | URDF / static TF | wheels, laser, IMU, OV9282 |

```
RPLIDAR C1     --/scan------------> slam_toolbox --(map→odom)--> TF
ORB-SLAM3 VIO  --/vio/odometry----\
RB3 IMU        --/imu/data---------> EKF --(odom→base_footprint)--> TF
bridge         --/odom (vx only)--/
Nav2           --/cmd_vel----------> wave_rover_bridge --> motors
```

VIO is ORB-SLAM3 mono-inertial on the OV9282 tracking camera + RB3 IMU
(`rb3_vio.launch.py` / `perception.launch.py`). The relay
`scripts/orbslam3_pose_to_odom.py` publishes `/vio/odometry` only after the
health gate (gravity-aligned + metric convergence); see
[orbslam3_calibration.md](orbslam3_calibration.md).

**Critical:** the bridge runs with `publish_tf: false` so it does **not** also
publish `odom→base_link`. The EKF alone owns that transform.

## Platform Nav2 choices

The base **cannot pivot in place**. Shared policy in both Nav2 configs:

- RPP controller: `use_rotate_to_heading: false`, `allow_reversing: true`
- Loose `yaw_goal_tolerance` (0.40)
- No Spin recovery (`behavior_trees/navigate_*_no_spin.xml`)
- Rectangular footprint (not a single inflated circle)

| Config | Where | Planner |
|--------|--------|---------|
| `config/nav2.yaml` | Onboard (`full_stack` / `nav.launch`) | **NavFn** (2D) |
| `laptop/nav2_laptop.yaml` | Laptop remote Nav2 | **SmacPlannerHybrid**, `REEDS_SHEPP`, `minimum_turning_radius: 0.35` |

Prefer the laptop path for demos and teaching — lighter on the RB3 CPU and
better kinematic paths. Onboard NavFn is fine for quick local tests; if corners
demand pivots the robot cannot execute, switch onboard to Smac or offload Nav2.

Drive calibration (open-loop boost, trim, deadband):
[odometry_calibration.md](odometry_calibration.md).

## Bring-up

### Recommended: manual two-step

```bash
# 1) base + VIO + EKF (no SLAM, no Nav2)
ros2 launch QB3rt odometry_bringup.launch.py
#    figure-8, then STAND STILL until "VIO metric convergence confirmed"
#    (/vio/ready=true); parked twist ~0 on /odometry/filtered

# 2) RPLIDAR + slam_toolbox
ros2 launch QB3rt slam_toolbox.launch.py
```

Then either run onboard Nav2 (`ros2 launch QB3rt nav.launch.py`) or follow
[laptop/README.md](../laptop/README.md).

Why gate SLAM: lateral `y`/`vy` is VIO-only-observable. An under-converged VIO
can push the EKF sideways while parked (seen 2026-07-19). Waiting until
`/odometry/filtered` is quiet keeps the map honest.

### Automatic VIO gate

```bash
# sensing + SLAM only (Nav2 on laptop)
ros2 launch QB3rt full_stack.launch.py enable_nav:=false

# everything onboard including Nav2
ros2 launch QB3rt full_stack.launch.py

# camera-free lidar SLAM path
ros2 launch QB3rt lidar_slam.launch.py
```

`full_stack` / `lidar_slam` hold slam_toolbox until `/vio/ready` when perception
is enabled (`gate_slam_on_vio`).

### Verify

```bash
ros2 topic hz /scan                 # ~10 Hz
ros2 topic hz /odometry/filtered
ros2 topic hz /map                  # after slam_toolbox is up
ros2 run tf2_tools view_frames      # map→odom→base_footprint→base_link→laser
```

### Laptop-side demo (verified 2026-10-17, unit 42 on ROS_NET_5)

Drive SLAM/Nav2 from the laptop over Wi-Fi. Everything below works on any
provisioned unit on ROS_NET_5 — the only per-unit value is the domain ID.

```bash
# on the unit: full onboard stack (base + odom + slam_toolbox + Nav2 +
# built-in compressed camera republish — see below)
ssh ubuntu@192.168.2.<N> qb3rt-up full_stack

# on the laptop: env is already Cyclone + matching domain; multicast works
# on ROS_NET_5 so no peer file is needed laptop-side
rviz2 -d rviz/fleet_demo.rviz
```

`/image_raw` is 720p mono8 raw (~200 Mbps) — too heavy for Wi-Fi. Since
2026-10-17 `perception.launch.py` runs an `image_transport republish` node
(`image_raw_compressed_repub`) that exposes `/image_raw/compressed`
automatically in every camera profile; disable with `enable_compressed:=false`.
On units running an older payload, the same republish can be started manually:

```bash
ssh ubuntu@192.168.2.<N> 'sudo systemd-run --unit=qb3rt-imgrepub \
  --setenv=HOME=/home/ubuntu --setenv=ROS_DOMAIN_ID=<N> \
  --setenv=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp bash -lc \
  "source /opt/qb3rt/install/setup.bash && \
   ros2 run image_transport republish raw compressed --ros-args \
   -r in:=image_raw -r out/compressed:=image_raw/compressed"'
```

`rviz/fleet_demo.rviz` ships Grid/TF/Map/LaserScan/RobotModel/Odometry-trail,
a **Camera** display on `/image_raw` with `Transport Hint: compressed`, the
Nav2 panel (class `nav2_rviz_plugins/Navigation 2` — `Nav2Panel` does not
exist) and the 2D Pose Estimate / Nav Goal / Publish Point tools.

Explore by clicking **2D Goal Pose** toward unmapped (gray) space; the map
extends as the robot drives. Save the finished map on the unit:

```bash
ssh ubuntu@192.168.2.<N> 'mkdir -p ~/maps && \
  ros2 run nav2_map_server map_saver_cli -f ~/maps/lab \
  --ros-args -p map_subscribe_transient_local:=true'
```

Teardown: `ssh ubuntu@192.168.2.<N> qb3rt-down` (also stop `qb3rt-imgrepub`
if the manual republisher was used).

Gotchas seen:

- `systemd-run` without `HOME=` kills rcl nodes at startup
  (`rcutils_expand_user failed`) — always pass `--setenv=HOME=/home/ubuntu`.
- Unit clocks run ~80 days behind the laptop (no WAN/time-sync) — harmless
  for RViz since `/tf`, `/scan`, `/map` share the unit's own clock domain;
  transient "extrapolation into the future"/queue-full messages at startup
  are noise.
- There is no autonomous frontier explorer in the image — exploration is
  goal-driven. `explore_lite` could be added if hands-free coverage is wanted.

For Nav2 goals: lifecycle nodes active, then set a **Nav2 Goal** in RViz. Confirm
`/cmd_vel` is non-zero while driving (`enable_base_driver` defaults true).

## Tuning notes

- Open-loop turn scrub is imperfect; Nav2 closes the loop via the EKF — expect
  arcs, not crisp corners.
- RPP overshoot: tune `lookahead_dist` / lower `desired_linear_vel`.
- `cmd_vel` not reaching motors: collision_monitor `cmd_vel_out_topic` must be
  `/cmd_vel` and match the bridge subscription.
- CPU: prefer laptop Nav2 + RViz; on the robot, `enable_vio:=false` drops VIO if
  you only need lidar SLAM with wheel+gyro prior.

## Key files

| Path | Role |
|------|------|
| `launch/odometry_bringup.launch.py` | Base + VIO + EKF (manual step 1) |
| `launch/slam_toolbox.launch.py` | Lidar + slam_toolbox (manual step 2) |
| `launch/lidar_slam.launch.py` | Auto VIO-gated lidar SLAM |
| `launch/full_stack.launch.py` | Full onboard stack |
| `launch/nav.launch.py` | Onboard Nav2 only |
| `config/slam_toolbox.yaml` | Mapping params |
| `config/nav2.yaml` | Onboard Nav2 |
| `laptop/nav2_laptop.yaml` | Remote Nav2 |
| `behavior_trees/navigate_*_no_spin.xml` | Recoveries without Spin |
| `vendor_overrides/.../wave_rover_bridge.yaml` | `publish_tf: false`, drive model |
