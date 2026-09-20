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
