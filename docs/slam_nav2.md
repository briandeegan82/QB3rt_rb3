# Live SLAM + Nav2 bring-up (QB3rt)

Live 2D SLAM (`slam_toolbox` mapping) and Nav2 navigation for the QB3rt
skid-steer AGV, driving through the **calibrated HTTP bridge**
(`wave_rover_bridge.py`, `spin_boost 4.0`). No pre-saved map: the robot maps
*while* it navigates.

## RB3 preflight (before any file edits under `/usr/share`)

`/usr/share` is mounted read-only by default on RB3. If your workflow includes
editing deployed configs or scripts under `/usr/share`, run:

```bash
source /root/rover_env.sh
mount | grep ' on /usr '
```

Expected: `/usr` is remounted `rw`. If your task is launch-only and does not
edit `/usr/share`, this step is not required.

## Architecture / TF ownership

| Transform | Owner | Source |
| --- | --- | --- |
| `map -> odom` | `slam_toolbox` | `lidar_slam.launch.py` (`/scan`) |
| `odom -> base_footprint` | `robot_localization` EKF | `odometry.launch.py` (`ekf.yaml`) |
| `base_footprint -> base_link -> laser_frame` | `robot_state_publisher` | URDF (`static_transforms.launch.py`) |
| `main_camera_frame -> oak_parent_frame -> oak -> ...` | static TF + depthai URDF | `perception.launch.py` |

Data flow:

```
RPLIDAR C1  --/scan-->  slam_toolbox --(map->odom)-->  TF
OAK-D VIO   --/oak/vio/odometry--\
RB3 IMU     --/imu/data----------->  EKF --(odom->base_footprint)--> TF
bridge      --/odom (vx only)-----/
Nav2 (navigation_launch) --/cmd_vel--> wave_rover_bridge.py --HTTP--> motors
```

**Critical:** the bridge runs with `publish_tf: false`
(`wave_rover_bridge.yaml`) so it does *not* also publish `odom->base_link`. The
EKF is the sole owner of the `odom` transform; if both published, `base_link`
would have two TF parents and SLAM/Nav2 would break. The bridge still publishes
`/odom` (consumed by the EKF as `odom1`, vx only).

**OAK-D camera TF:** in VIO mode the depthai driver would otherwise broadcast its
own `odom -> oak_parent_frame` transform, floating the whole camera subtree off
`odom` as a second branch parallel to the robot body (and a second mover under
`odom` alongside the EKF). We disable that broadcast (`vio.i_publish_tf: false`
in `config/vio.yaml`) and instead publish a static
`main_camera_frame -> oak_parent_frame` transform (`perception.launch.py`,
`cam_mount_frame`/`cam_parent_frame` args), so the camera is rigidly mounted under
`base_link` like any other sensor. VIO still feeds the EKF via the
`/oak/vio/odometry` *topic* (`odom0`); `i_publish_tf` gates only the TF, not the
topic. The EKF can still resolve the VIO message's `child_frame_id`
(`oak_parent_frame`) to `base_footprint` through this static mount.

## Platform-specific Nav2 choices (`config/nav2.yaml`)

This robot **cannot pivot in place** (4-wheel scrub stalls) and has a minimum
powered turn radius of roughly 0.3-0.4 m. The config reflects that:

- **Controller:** Regulated Pure Pursuit (`FollowPath`) with
  `use_rotate_to_heading: false` and `allow_reversing: false` — pure arc-based
  path following, no in-place rotation. `desired_linear_vel: 0.22` (calibrated
  max is ~0.42; velocity_smoother caps at 0.26).
- **Goal checker:** `yaw_goal_tolerance: 0.40` (loose) — the robot can't pivot
  to fine-tune the final heading, so don't demand it.
- **Recoveries:** custom behavior trees
  (`behavior_trees/navigate_to_pose_no_spin.xml`,
  `navigate_through_poses_no_spin.xml`) with the **Spin recovery removed**;
  `behavior_server` loads only `backup`, `drive_on_heading`, `wait`.
- **Footprint:** `robot_radius: 0.14` (chassis ~0.18 x 0.15 m), inflation 0.25 m.
- **Costmaps:** obstacle/voxel layers from `/scan`; global static layer from the
  live `slam_toolbox` map.
- **Planner:** `NavFn` (2D) for the first cut.
- **Output:** the cmd_vel chain ends at `collision_monitor -> /cmd_vel`, which is
  exactly what the bridge subscribes to.

## Staged bring-up + verification

### Recommended: manual two-step (odometry, then SLAM by hand)

Start odometry, confirm VIO is trusted and odom is stable while parked, *then*
start SLAM — the operator is the gate, so a warming-up VIO is never mapped:

```bash
# 1) odometry only (base + VIO + EKF; no SLAM, no Nav2)
ros2 launch QB3rt odometry_bringup.launch.py
#    figure-8 ritual, THEN STAND STILL until the relay logs
#    "VIO metric convergence confirmed" (/vio/ready=true), and:
ros2 topic echo /odometry/filtered --field twist.twist   # vx,vy,vyaw ~0 parked

# 2) SLAM by hand (RPLIDAR + slam_toolbox -> map->odom)
ros2 launch QB3rt slam_toolbox.launch.py
```

Why: the EKF's lateral `y`/`vy` channel is VIO-only-observable, so an
under-converged VIO drifting while parked runs the filter away sideways (seen
2026-07-19: ~1 m/s phantom `vy`). Waiting to start SLAM until `/odometry/filtered`
is steady guarantees the map is built on trustworthy odometry. The relay also
enforces this automatically (metric-convergence gate + a 0.6 m/s speed cap in
`scripts/orbslam3_pose_to_odom.py`); the manual split is just the simplest,
most visible gate. Unlike `lidar_slam`/`full_stack`, the RPLIDAR is started in
step 2 with no `lidar_start_delay` — the OV9282 is a CSI/ISP camera
(`qrb_ros_camera`), not the old USB OAK-D, so it does not reset the lidar's bus.

### Stage 1 (alt) — automatic gated SLAM (verify TF + map)

```bash
ros2 launch QB3rt lidar_slam.launch.py
```

Note: `lidar_slam`/`full_stack` hold slam_toolbox until VIO latches `/vio/ready`
(`gate_slam_on_vio`), and start the RPLIDAR after a ~20 s `lidar_start_delay`
(a legacy OAK-D USB-hub workaround). Verify:

1. Scan is publishing: `ros2 topic hz /scan` (expect ~10 Hz).
2. TF chain is complete and single-parented:
   ```bash
   ros2 run tf2_tools view_frames        # inspect frames.pdf
   ros2 run tf2_ros tf2_echo map base_footprint
   ```
   Expect `map -> odom -> base_footprint -> base_link -> laser_frame`, with each
   frame having exactly one parent.
3. Drive the robot by hand / teleop and watch the map grow (RViz, "Map" display
   on `/map`). If `/scan` is empty, increase `lidar_start_delay`.

### Stage 2 — Full stack + Nav2 goal

```bash
ros2 launch QB3rt full_stack.launch.py use_rviz:=true
```

`enable_base_driver` defaults `true`, so Nav2's `/cmd_vel` reaches the bridge and
the wheels move. Verify:

1. All Nav2 lifecycle nodes reach **active**
   (`controller_server`, `planner_server`, `behavior_server`, `bt_navigator`,
   `smoother_server`, `velocity_smoother`, `collision_monitor`,
   `waypoint_follower`, `docking_server`).
2. In RViz, set a **Nav2 Goal**. Watch the global/local costmaps and the planned
   path appear.
3. Confirm velocity reaches the bridge: `ros2 topic echo /cmd_vel` (non-zero
   while driving) and the wheels turn.

## Tuning / fallbacks

- **Open-loop turn scrub / asymmetry:** turning is open-loop-imperfect (see
  `odometry_calibration.md`); Nav2 closes the loop on heading via the EKF, but
  expect arcs, not crisp corners.
- **RPP cornering:** if the robot overshoots or cuts corners, tune
  `lookahead_dist` / `min_lookahead_dist` and lower `desired_linear_vel`. The
  `use_regulated_linear_velocity_scaling` already slows it on tight curves.
- **Infeasible pivots from the planner:** `NavFn` can request paths with sharp
  corners the robot can't execute. If cornering misbehaves, switch the
  `planner_server` `GridBased` plugin to
  `nav2_smac_planner::SmacPlannerHybrid` with `minimum_turning_radius: 0.4` for
  kinematically feasible (Dubins/Reeds-Shepp) paths.
- **cmd_vel not reaching the bridge:** confirm the final topic is `/cmd_vel`
  (collision_monitor `cmd_vel_out_topic`) and the bridge's `cmd_vel_topic`
  matches.

## CPU / performance (RB3 / QCS6490, 8 cores)

The full stack is CPU-bound on the RB3. The OAK-D Basalt VIO is the main hog
(Basalt VIO + stereo depth + image streaming at 60 fps), which pins ~7/8 cores and
can starve the EKF (`Failed to meet update rate!`).

**VIO runs on the stock depthai `vio.yaml` (full rate).** Detuning it via a custom
params file was attempted (15 fps, no image publishing, 400p mono) but every
variant **segfaulted the pipeline builder on this OAK-D-LITE** (exit -11 at
`Pipeline type: rgbd`) - the device/driver build does not tolerate per-sensor
overrides here. So VIO is left at the stock config; if you need the CPU back, the
cleanest option is `enable_vio:=false` (lidar SLAM then localizes from the
wheel(cmd)+gyro prior). Note the OAK-D-LITE has no guaranteed IMU, so Basalt is
marginal regardless.

Nav2/SLAM trims that ARE baked in (independent of VIO):

- **Local costmap (`config/nav2.yaml`):** 2D `ObstacleLayer` instead of the 3D
  `VoxelLayer`; `always_send_full_costmap: false`; update 3 Hz / publish 1 Hz.
- **Controller:** 15 Hz (was 20).
- **SLAM:** `map->odom` at 20 Hz (was 50).

If still saturated, next levers: offload Nav2 (and/or RViz, SLAM) to the laptop
(ROS 2 is distributed — keep drivers/EKF on the robot), run `enable_vio:=false`, or
lower `slam_toolbox` `map_update_interval`. Also kill any stale DDS peer: the log
showed writes to an unreachable `192.168.4.3`, which adds failed-write churn.

## Files

- `launch/odometry_bringup.launch.py` — base + VIO + EKF, **no SLAM** (manual step 1).
- `launch/slam_toolbox.launch.py` — RPLIDAR + slam_toolbox, started by hand (manual step 2).
- `launch/lidar_slam.launch.py` — RPLIDAR + EKF + perception + slam_toolbox (auto VIO-gated).
- `launch/nav.launch.py` — Nav2 navigation-only (no map_server/AMCL).
- `launch/full_stack.launch.py` — everything + optional RViz.
- `config/slam_toolbox.yaml` — mapping params (frames, indoor tuning).
- `config/nav2.yaml` — Nav2 params (RPP, no-spin, footprint, costmaps).
- `behavior_trees/navigate_*_no_spin.xml` — recovery trees without Spin.
- `wave_rover_controller/config/wave_rover_bridge.yaml` — `publish_tf: false`.
