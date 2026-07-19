# QB3rt — RB3 Gen 2 AGV

Skid-steer AGV on a WAVE ROVER base with a Qualcomm RB3 Gen 2 (QIRP) brain.
Perception: ORB-SLAM3 mono-inertial VIO on the OV9282 tracking camera + RB3
IMU, RPLIDAR C1 for 2D SLAM (slam_toolbox). Navigation: Nav2, onboard or
remote on the laptop.

**GitHub:** [briandeegan82/QB3rt_rb3](https://github.com/briandeegan82/QB3rt_rb3)
— versioned backup of this tree. On the robot, `/root/QB3rt` remains the
live edit/deploy source; push/pull against `origin` when you want changes
off-box or restored after a reflash.

## Where things live (and why)

| path | role |
|------|------|
| `/root/QB3rt` | **THE PROJECT.** Canonical source on the robot. Edit here. Persists across ostree reflashes (`/root` = `/var/roothome`). |
| `/usr/share/QB3rt` | Install space. Running nodes load from here. **Never edit directly** — deploy instead. |
| `/root/QB3rt/vendor_overrides/` | Files we maintain for vendor packages that have no source on the robot (the wave_rover bridge + its calibrated config). A reflash reverts the vendor package; re-deploying restores it. |
| `/root/QB3rt/laptop/` | Remote-Nav2 bundle for the laptop (see `laptop/README.md`). |
| `~/mnt/rb3` (laptop) | sshfs mount of the robot's `/`; the project is `~/mnt/rb3/root/QB3rt`. |
| GitHub `QB3rt_rb3` | Off-robot git remote for this package (launch/config/scripts/docs/…). |

`/usr` is a read-only ostree mount: before any deploy, run
`source /root/rover_env.sh` **in a device shell** (it remounts `/usr` rw and
sets up the ROS/DDS env).

## Clone / restore from GitHub

```bash
# fresh tree onto the robot (or a laptop working copy)
git clone https://github.com/briandeegan82/QB3rt_rb3.git /root/QB3rt
# then deploy into install space (on-device, after remounting /usr rw):
bash /root/QB3rt/deploy.sh
```

`.gitignore` keeps out `__pycache__/`, `results/` (calibration run dumps),
and ament install-space leftovers (`local_setup.*`, `environment/`, `hook/`,
`cmake/`). Source of truth for what gets installed is still `deploy.sh`.

## Edit → deploy → relaunch

```bash
# 1. edit files under /root/QB3rt (directly or via the laptop mount)
# 2. deploy the whole tree (works on-device or from the laptop mount):
bash /root/QB3rt/deploy.sh
# 3. relaunch the affected nodes
# 4. (optional) commit + push to GitHub when the change should leave the robot
```

`deploy.sh` syncs launch/config/urdf/scripts/behavior_trees/rviz/docs +
package.xml into `/usr/share/QB3rt`, clears stale `__pycache__`, and installs
the vendor overrides (bridge + calibration) into the wave_rover_controller
package. No hand-maintained file lists.

## Frames & fusion (REP-105)

```
map ──(slam_toolbox)──> odom ──(EKF)──> base_footprint ──(URDF)──> base_link ──> wheels/laser/imu/cams
```

- **EKF** (`config/ekf.yaml`) is the *sole* owner of `odom->base_footprint`.
  It fuses: ORB-SLAM3 VIO pose **differentially** (body-frame deltas — immune
  to the VIO-vs-gyro yaw-origin mismatch), IMU yaw *rate*, and calibrated
  wheel `vx` from the bridge.
- **VIO trust gate** (`scripts/orbslam3_pose_to_odom.py`): the lateral `y`/`vy`
  channel is *VIO-only-observable*, so a warming-up VIO that drifts while parked
  would run the filter away sideways (seen 2026-07-19: ~1 m/s phantom `vy`). The
  relay therefore only feeds the EKF once VIO is BOTH gravity-aligned (roll/pitch
  ~0) AND **metrically converged** — confirmed when, with the wheels reporting
  stationary, VIO also reads ~0 speed for a few seconds. It also drops any VIO
  frame implying >0.6 m/s (above the platform's ~0.42 m/s top speed).
  **Operator:** end the figure-8 init ritual with a brief stop so the gate can
  confirm.
- **slam_toolbox** (`config/slam_toolbox.yaml`) owns `map->odom` and `/map`.
- The wave_rover bridge must keep `publish_tf: false` (two parents for
  `base_link` otherwise — this broke Nav2 once already).

## Drive controller

`vendor_overrides/wave_rover_controller/wave_rover_bridge.py` maps `/cmd_vel`
to motor commands through the calibrated open-loop model
(`docs/odometry_calibration.md`): affine speed-scaled feedforward
(friction-floor `motor_deadband` + measured `max_speed`), speed-scheduled
skid-steer turn gain (exponential `spin_boost_max`/`spin_boost_k`), left/right
`straight_trim`, and the firmware `motor_cmd_max: 0.5` full-scale cap
(fractions above 0.5 stall this firmware). The base cannot pivot in place;
Nav2 is configured for arc-only motion everywhere (RPP without
rotate-to-heading, no Spin recovery, Smac Hybrid DUBIN planner on the laptop).

## Bring-up

### Manual two-step (recommended for hands-on mapping)

Start odometry first, confirm it is healthy, *then* start SLAM by hand — you are
the gate, so a warming-up VIO can never be baked into the map:

```bash
source /root/rover_env.sh

# 1) odometry only: base + VIO + EKF (no SLAM, no Nav2)
ros2 launch QB3rt odometry_bringup.launch.py
#    - wait for /odometry/filtered
#    - do the ORB-SLAM3 figure-8 ritual, THEN STAND STILL until the relay logs
#      "VIO metric convergence confirmed" (i.e. /vio/ready=true)
#    - sanity: ros2 topic echo /odometry/filtered --field twist.twist  # ~0 parked

# 2) only now start SLAM (RPLIDAR + slam_toolbox -> map->odom)
ros2 launch QB3rt slam_toolbox.launch.py

# 3) Nav2 remote on the laptop: laptop/README.md
```

### One-shot (automatic /vio/ready gate)

```bash
source /root/rover_env.sh

# everything onboard (SLAM + Nav2 on the robot):
ros2 launch QB3rt full_stack.launch.py

# robot side only, Nav2 remote on the laptop:
ros2 launch QB3rt full_stack.launch.py enable_nav:=false
# ... then on the laptop: laptop/README.md
```

Here `full_stack`/`lidar_slam` hold slam_toolbox automatically until VIO latches
`/vio/ready` (`gate_slam_on_vio`), instead of the manual step 2 above.

Other entry points: `odometry_bringup.launch.py` (base+VIO+EKF, no SLAM),
`slam_toolbox.launch.py` (RPLIDAR + slam_toolbox, manual start),
`lidar_slam.launch.py` (camera-free lidar SLAM), `perception.launch.py`
(camera/VIO), `odom_square_test.launch.py` (calibration; see
`docs/odometry_calibration.md`).

> **Clock:** the RB3 has no working RTC (boots at 1970) and depends on
> `systemd-timesyncd` against the laptop. Its first sync **steps** the clock ~1
> min after boot; if that lands while the stack is running, robot_localization
> sees a time discontinuity and drifts. Let the clock sync (or freeze it with
> `sudo timedatectl set-ntp false`) **before** launching. See `laptop/README.md`.
