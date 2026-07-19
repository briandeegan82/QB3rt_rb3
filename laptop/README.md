# QB3rt remote Nav2 (laptop) runbook

Nav2 runs on the laptop; the robot runs everything else onboard:

| where  | what                                                                 | TF it owns                |
|--------|----------------------------------------------------------------------|---------------------------|
| robot  | base driver + IMU + ORB-SLAM3 VIO + EKF + RPLIDAR + slam_toolbox     | `odom->base_footprint` (EKF), `map->odom` (slam_toolbox), URDF statics |
| laptop | Nav2 (planner/controller/behaviors/BT, velocity smoother, collision monitor) + RViz | none                      |

The laptop's final `/cmd_vel` travels over WiFi to the wave_rover bridge on the
robot. The bridge's `cmd_timeout: 0.5` watchdog stops the wheels if the link
drops mid-drive.

## One-time laptop setup

1. Install Nav2 (not present on this laptop yet):

   ```bash
   sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
   ```

2. Install the bundle (robot mounted at `~/mnt/rb3`):

   ```bash
   bash ~/mnt/rb3/root/QB3rt/laptop/install_on_laptop.sh
   ```

   This copies configs + the no-spin behavior trees to `~/qb3rt_laptop`.
   Re-run it after editing the canonical copies in `/root/QB3rt/laptop/`.

3. `~/cyclonedds.xml` must exist on the laptop (it does) and peer with the
   robot's IP (192.168.0.100). The robot side is `/opt/cyclonedds.xml` via
   `rover_env.sh`.

## Clock sync (do not skip)

TF is stamped by the robot and consumed on the laptop. If the clocks disagree
by more than the transform tolerances (~0.3-0.5 s), every costmap update and
controller cycle fails with extrapolation errors. Check:

```bash
# on the laptop
date +%s.%N; ssh/console on robot: date +%s.%N   # or compare `ros2 topic echo /scan --field header.stamp`
```

If skewed, sync the RB3 (NTP against the router or the laptop) before launching.
Symptoms of skew: "Lookup would require extrapolation into the future/past" spam
from costmap_2d / RPP.

### CRITICAL: sync the RB3 clock BEFORE launching, never during (2026-07-19)

The RB3 has **no working battery-backed RTC** (`timedatectl` shows `RTC time`
stuck at 1970). Every boot it comes up with a wrong clock and `systemd-timesyncd`
corrects it by **stepping** the system clock ~1 minute after boot, once it can
reach a time source. If the robot stack (and its EKF) is already running when
that step lands, robot_localization sees one enormous `dt`, its covariance blows
up, and `/odometry/filtered` **drifts wildly** — this exact failure cost hours on
2026-07-19. A Wi-Fi drop + re-sync can re-trigger it mid-run.

The laptop must run an actual NTP **server** (`systemd-timesyncd` is client-only;
the RB3 pointed at it as `NTP=<laptop-ip>`). Then, on the RB3, either:

```bash
# A) let the boot-time step happen, THEN freeze for the session before launching
timeout 120 sh -c 'until [ "$(timedatectl show -p NTPSynchronized --value)" = yes ]; do sleep 1; done'
sudo timedatectl set-ntp false
# ...now launch the robot stack

# or B) run chrony on the RB3 with boot-only stepping (makestep 1.0 3) so it
#       steps early (before launch) and only slews afterward.
```

Consider `fake-hwclock` (or fixing the RTC battery) to shrink the boot-time step.

## Run order

1. **Robot** (in your device shell). Make sure the clock is already synced
   (above). Two options:

   **Manual two-step (recommended):** start odometry, confirm it, then SLAM.

   ```bash
   source /root/rover_env.sh
   ros2 launch QB3rt odometry_bringup.launch.py
   #   do the ORB-SLAM3 figure-8 ritual, then STAND STILL until the relay logs
   #   "VIO metric convergence confirmed" (/vio/ready=true), and
   #   ros2 topic echo /odometry/filtered --field twist.twist   # ~0 while parked
   ros2 launch QB3rt slam_toolbox.launch.py        # only now: RPLIDAR + SLAM
   ```

   **One-shot (automatic VIO gate):**

   ```bash
   source /root/rover_env.sh
   ros2 launch QB3rt full_stack.launch.py enable_nav:=false
   ```

   Wait until: `/scan` streaming, `/map` publishing (slam_toolbox up),
   `/odometry/filtered` streaming (EKF up). ORB-SLAM3 needs the figure-8 ritual
   (end with a stop) to initialize VIO; the EKF runs on IMU+wheel vx until VIO is
   trusted, so odom is available before VIO converges.

2. **Laptop**:

   ```bash
   source ~/qb3rt_laptop/qb3rt_env.sh
   ros2 launch ~/qb3rt_laptop/nav2_laptop.launch.py
   ```

   RViz opens with the Nav2 default view. Check the map and TF arrive, then
   send a goal with "Nav2 Goal".

## Sanity checks when something is off

```bash
source ~/qb3rt_laptop/qb3rt_env.sh
ros2 topic hz /scan                # lidar arriving over WiFi?
ros2 topic hz /odometry/filtered   # EKF arriving?
ros2 run tf2_tools view_frames     # map->odom->base_footprint->base_link chain complete?
ros2 topic echo /cmd_vel --once    # controller output reaching DDS?
```

- No topics at all -> env not sourced / wrong `CYCLONEDDS_URI` (must be
  `~/cyclonedds.xml`, **not** `/opt/cyclonedds.xml` — that path only exists on
  the robot) / robot not up.
- Topics but TF errors -> clock skew (above).
- Goal accepted but robot doesn't move -> is the bridge running with
  `enable_base_driver:=true` (full_stack default)? `ros2 topic hz /cmd_vel` on
  the robot side.

## Notes on this config

- Planner is **SmacPlannerHybrid, REEDS_SHEPP model, minimum_turning_radius
  0.35** — the skid-steer cannot pivot, but REEDS_SHEPP permits a short
  reverse instead of the wide forward-only loop DUBIN forces when a goal's
  heading mismatch has no forward-only solution (changed 2026-07-14; DUBIN
  caused 13 consecutive circling replans on one live goal). Paths can include
  brief reversing; that is intentional.
- Controller is Regulated Pure Pursuit with `use_rotate_to_heading: false`,
  loose `yaw_goal_tolerance` (base can't fine-tune heading in place), and
  `allow_reversing: true` to match the planner. `reverse_penalty: 4.0`
  reserves reversing for clearly-shorter cases (raised from 2.0 after one
  goal oscillated 28s at a Reeds-Shepp cusp) and `movement_time_allowance:
  8.0` (down from 15.0) aborts a stuck/oscillating robot roughly 2x faster.
  `regulated_linear_scaling_min_radius: 0.5` — a soft speed-reduction
  threshold, kept slightly above `minimum_turning_radius` as tracking
  margin; do not set planner/controller turn-radius values from open-loop
  drive calibration numbers, they measure feedforward accuracy, not a
  kinematic limit (bit us 2026-07-14, see qb3rt-restructure memory).
- Reversing is tuned and works on most goals, but is not exhaustively
  road-tested — watch new reversing maneuvers, especially near cusps.
- Recovery behaviors exclude Spin (the custom `*_no_spin.xml` BTs).
- `transform_tolerance` is raised vs the onboard config to absorb WiFi latency.
- VIO **is** fused into the EKF again (re-fused 2026-07-18 as health-gated
  `odom1`: differential x/y/yaw), on top of wheel vx + gyro yaw rate. It is only
  trusted after gravity alignment AND metric convergence (see the robot
  `README.md` "VIO trust gate"); until then the EKF coasts on wheel+gyro, so you
  can launch Nav2 without waiting on VIO. A badly-initialized VIO CAN destabilize
  odometry (1 m/s phantom `vy` on 2026-07-19) — if odom drifts while parked,
  redo the figure-8 ritual and end with a stop. (This supersedes the earlier
  "VIO removed 2026-07-13" note.)
