# Part 2 — Autonomous navigation with slam_toolbox and Nav2

In Part 1 you drove qb3rt by keyboard while slam_toolbox built the map. In this part the robot drives itself. You give it a goal pose on the map, Nav2 plans a path through the costmap, and the controller follows it. slam_toolbox keeps mapping the whole time, so you can send goals into space the robot has not seen yet and watch the map grow.

The laptop side is the same as Part 1: the `ros2_ws` pixi workspace, RoboStack Jazzy, your domain, CycloneDDS unicast. Do not source `/opt/ros/jazzy`. Nav2 runs on the robot. The laptop only runs RViz.

## How the pieces fit

```
RPLIDAR C1 --/scan--> slam_toolbox ------------> /map, TF map -> odom
                 \--> rf2o --/odom_rf2o--\
bridge ------/odom (wheel vx)-------------> EKF -> TF odom -> base_footprint
RB3 IMU -----/imu/data-------------------/

/goal_pose (RViz) or /navigate_to_pose (action)
      |
  bt_navigator  (behaviour tree: plan, follow, recover)
      |-- planner_server     global costmap (frame map)   NavFn  -> /plan
      |-- controller_server  local costmap  (frame odom)  RotationShim + RPP -> cmd_vel_nav
      |
  velocity_smoother -> cmd_vel_smoothed -> collision_monitor -> /cmd_vel -> WAVE ROVER bridge
```

What each part does:

- **slam_toolbox** owns `map → odom`, as in Part 1. Nav2 does not localise in this setup: no AMCL, no saved map.
- **EKF** owns `odom → base_footprint`. Use the full filter (`ekf.yaml`: wheel vx, rf2o, IMU yaw rate). The IMU-only filter barely translates, and Nav2 then thinks the robot never reaches the goal.
- **Global costmap** is the slam_toolbox map plus live scan obstacles, inflated. The planner searches it for a path from the robot to the goal.
- **Local costmap** is a 3 m × 3 m window around the robot, built from `/scan` only. The controller uses it to follow the path without hitting anything.
- **RotationShimController** turns the robot to face the path first if it points more than about 45° away. It then hands over to **Regulated Pure Pursuit (RPP)**, which steers toward a point a lookahead distance along the path and slows down on tight curves and near the goal.
- **velocity_smoother** caps speed and acceleration. **collision_monitor** slows the robot when the footprint is about to hit a scan point. It publishes the final `/cmd_vel`.
- **The behaviour tree** (`behavior_trees/navigate_to_pose_no_spin.xml`) replans once a second and, on failure, clears the costmaps and waits. Spin and BackUp recoveries are removed because this skid-steer scrubs badly when it spins in place.

The parameters are in `config/nav2.yaml` on the robot (`/opt/qb3rt/install/share/QB3rt/config/nav2.yaml`). Open it alongside this guide. The comments explain why each value was chosen for this robot.

## 1. Start the stack on the robot

Connect the laptop to the robot's network and check `pixi run topics` as in Part 1. Then:

```bash
ssh -i ~/.ssh/qb3rt_fleet -o IdentitiesOnly=yes ubuntu@<robot-ip>
qb3rt-up lidar_nav ekf_config:=/opt/qb3rt/install/share/QB3rt/config/ekf.yaml
```

`lidar_nav` is `lidar_slam` from Part 1 plus Nav2. It stops any running stack first. Place the robot where you want the map origin before you run it. The `map` frame starts at the robot's pose when slam_toolbox takes its first scan, with +x pointing out of the robot's nose.

Follow the log in a second SSH session:

```bash
journalctl -u qb3rt-stack@lidar_nav -f
```

Nav2 is ready when `lifecycle_manager_navigation` prints `Managed nodes are active`. That takes up to about 30 s on the RB3. If it prints `Failed to bring up all requested nodes`, scroll up to the first error. It is usually a TF problem: check the chain with the `tf2_echo` commands from Part 1.

**Nav2 now publishes `/cmd_vel`. Do not run teleop at the same time.** The two will fight. Keep that second SSH session ready to type `qb3rt-down`, which stops everything. The bridge stops the wheels 0.5 s after `/cmd_vel` stops arriving.

## 2. RViz

On the laptop, from `ros2_ws`, run `pixi run rviz2`. Set up the Part 1 displays first (Fixed Frame `map`, TF, Map `/map`, LaserScan `/scan`, RobotModel), then add:

| Display | Topic | Settings |
|---|---|---|
| Map | `/global_costmap/costmap` | Color Scheme `costmap`, Alpha 0.5 |
| Map | `/local_costmap/costmap` | Color Scheme `costmap`, Alpha 0.5 |
| Path | `/plan` | the global plan, green |
| Path | `/received_global_plan` | the part of the plan the controller is following |
| Polygon | `/local_costmap/published_footprint` | the footprint Nav2 checks for collisions |

If a costmap stays blank, set that display's Durability Policy to Transient Local. If you have the Nav2 RViz plugins, you can also add the `Navigation 2` panel to see goal status and cancel goals. It is optional.

How to read the costmap colours: pink/magenta cells are lethal (an obstacle, or closer to one than the robot's inscribed radius). The blue-to-cyan band around them is inflation, where cost falls with distance. The planner avoids that band where it can. Unknown space has its own colour. `allow_unknown: true` lets the planner go through it.

## 3. First goal

Start in an area you have already mapped. Pick a spot about 1 m away in clear floor.

In RViz, choose **2D Goal Pose**, click the goal position, drag toward the heading you want and release. This publishes `/goal_pose`, and bt_navigator turns it into a `navigate_to_pose` goal.

What should happen:

1. A path appears on `/plan` within a second.
2. If the goal is behind the robot, it first rotates on the spot (rotation shim) until it roughly faces the path.
3. It drives along the path, slowing down on curves and for the last 0.6 m.
4. It stops within 0.30 m of the goal and 0.6 rad of the goal heading. The log prints `Goal succeeded`.

The tolerances are loose on purpose. The robot cannot make fine heading adjustments in place.

You can also send goals from a terminal on the robot. This is useful for repeatable tests:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

The feedback shows the distance remaining and the recovery count. Ctrl-C cancels the goal.

## 4. Navigate into unmapped space

Send a goal into grey (unknown) space just beyond the mapped edge, or just past a doorway. The planner treats unknown cells as passable. As the robot drives, new scans fill in the map and the global costmap, and the planner replans every second. If a wall appears across the planned route, the path moves to go around it.

This is basic exploration by hand: send a goal at the frontier, let the robot reach it, look at the new map, then pick the next goal. Keep goals short (1–1.5 m) at first. Long goals into unknown space often end against a wall the robot had not seen yet.

## 5. Doorways and inflation

Narrow gaps are where Nav2 tuning matters most. The footprint in `nav2.yaml` is the rectangle `[[0.10, 0.095], ..., [-0.10, 0.095]]`, so:

- inscribed radius ≈ 0.095 m (closer than this to an obstacle is a collision in any orientation)
- circumscribed radius ≈ 0.138 m (the corners)

Inflation sets the cost of cells near obstacles:

```math
\text{cost}(d) = 252 \, e^{-k (d - r_\text{inscribed})}, \quad r_\text{inscribed} < d < \text{inflation\_radius}
```

where `k` is `cost_scaling_factor`. Cells beyond `inflation_radius` have zero cost.

Example: a 0.7 m doorway. At the centre the robot is 0.35 m from each side.

- `inflation_radius: 0.25` (the default here): the centre line has zero cost. The planner goes through the doorway happily, but cuts corners close to the frame.
- `inflation_radius: 0.45`, `cost_scaling_factor: 3.0`: centre cost ≈ 252 · e^(−3 × 0.255) ≈ 117. The doorway is still passable, but now it is expensive, so the planner prefers any wider route and keeps the robot centred.
- Make `inflation_radius` large enough with `k` high enough and the whole doorway becomes high-cost or lethal. The doorway then disappears from the plan.

Look at the global costmap at the doorway in RViz while you change these values. You can change them live:

```bash
ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.35
ros2 param set /global_costmap/global_costmap inflation_layer.cost_scaling_factor 4.0
ros2 param set /local_costmap/local_costmap  inflation_layer.inflation_radius 0.35
```

Keep the global and local costmaps consistent. If the global planner squeezes through a gap that the local controller thinks is blocked, the robot stalls at the doorway.

## 6. Other parameters worth knowing

Most controller and smoother parameters can also be set live with `ros2 param set`, so you can try a value without restarting the stack (and losing the map).

| Parameter | Value | What it does on this robot |
|---|---|---|
| `velocity_smoother.max_velocity` | `[0.26, 0.0, 1.5]` | Hard caps on vx, vy, ωz. The angular cap matters: turn faster than rf2o and slam_toolbox can follow and the map smears. Use the turn rate you found to be safe in Part 1. |
| `FollowPath.desired_linear_vel` | 0.22 | RPP's cruise speed. |
| `FollowPath.lookahead_dist` (min/max) | 0.8 (0.6–1.2) | Longer lookahead gives smoother, wider arcs but cuts corners. Shorter follows the path closely but oscillates. |
| `FollowPath.angular_dist_threshold` | 0.785 | Heading error (rad) above which the shim rotates in place before driving. |
| `FollowPath.rotate_to_heading_angular_vel` | 1.2 | Spin rate during shim rotations. Spinning on the spot works on vinyl but barely on carpet. On carpet, lower this or remove the shim (see the comment in `nav2.yaml`). |
| `general_goal_checker.xy_goal_tolerance` / `yaw_goal_tolerance` | 0.30 / 0.60 | When the goal counts as reached. Too tight and the robot hunts back and forth at the goal. |
| `progress_checker.required_movement_radius` / `movement_time_allowance` | 0.1 m / 8 s | If the robot moves less than this within that time, the goal fails with `Failed to make progress` and recovery starts. |
| `collision_monitor.FootprintApproach.time_before_collision` | 1.2 s | Slows the robot when it is about to touch a scan point. This is why it creeps through tight gaps. |
| `GridBased.tolerance` | 0.5 | How far from an unreachable goal the planner may stop instead. |

To keep your changes across restarts, copy the file into your own workspace and point the stack at it. Do not edit the system copy:

```bash
mkdir -p ~/nav2_params
cp /opt/qb3rt/install/share/QB3rt/config/nav2.yaml ~/nav2_params/my_nav2.yaml
# edit ~/nav2_params/my_nav2.yaml, then
qb3rt-up lidar_nav ekf_config:=/opt/qb3rt/install/share/QB3rt/config/ekf.yaml \
  params_file:=/home/ubuntu/nav2_params/my_nav2.yaml
```

`qb3rt-up` restarts the whole stack, so the map starts again from scratch.

## 7. Goals from a script

Clicking goals in RViz does not scale. `nav2_simple_commander` wraps the Nav2 actions in a small Python API. Save this on the robot as `~/waypoints.py` and run it with `python3 ~/waypoints.py` while `lidar_nav` is up:

```python
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# (x, y, yaw) in the map frame. Edit for your space.
WAYPOINTS = [(1.0, 0.0, 0.0), (1.5, 0.8, 1.57), (0.0, 0.0, 3.14)]


def pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def main():
    rclpy.init()
    nav = BasicNavigator()
    # slam_toolbox localises here, not AMCL; don't wait for AMCL or an initial pose.
    nav.waitUntilNav2Active(localizer="robot_localization")
    for x, y, yaw in WAYPOINTS:
        nav.goToPose(pose(nav, x, y, yaw))
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb:
                print(f"  {fb.distance_remaining:.2f} m to go", end="\r")
        result = nav.getResult()
        print(f"({x}, {y}): {result.name}")
        if result != TaskResult.SUCCEEDED:
            break
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

`goToPose` sends one goal at a time, so you can check each result before going on. `goThroughPoses` and `followWaypoints` send a whole list in one call. Note that `nav.cancelTask()` stops the current goal.

## 8. Save the map

When the space is mapped, save it on the robot:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab \
  --ros-args -p map_subscribe_transient_local:=true
```

This writes `lab.pgm` and `lab.yaml`. Stop the stack with `qb3rt-down` when you are done.

## Debug

**Goal accepted, robot doesn't move.** Check `ros2 topic hz /cmd_vel` on the robot. Nothing: look in the journal for controller errors. Non-zero but no motion: the bridge is not running or not getting the message. `ros2 topic info /cmd_vel` should show exactly one publisher (collision_monitor). Two publishers means teleop is still running.

**Nothing happens on 2D Goal Pose.** Nav2 is not active yet (wait for `Managed nodes are active`), or the goal is in a frame other than `map`. Check `ros2 lifecycle get /bt_navigator`. It should print `active`.

**`Failed to create plan` / no path.** The goal or the robot is in lethal space, or inflation has closed off a gap. Look at the global costmap around both ends. A robot that starts against a wall sometimes thinks it is inside an obstacle. Drive it clear first.

**`Failed to make progress`, then recoveries.** The robot is scrubbing (carpet, shim rotations), stuck on something below the scan plane, or creeping because collision_monitor is slowing it. Watch which happens. Lower the shim spin rate on carpet.

**Robot oscillates at the goal or circles it.** Goal tolerance too tight for the robot, or the lookahead too short. Loosen the yaw tolerance first.

**Map smears or doubles while Nav2 drives.** The turns are faster than the odometry can track. Lower `rotate_to_heading_angular_vel` and the angular entry of `velocity_smoother.max_velocity`. Teleop at the same turn rate should give a clean map.

**Costmap shows obstacles that aren't there, rotated or mirrored.** That is `lidar_yaw`, not Nav2. Redo the box-at-the-nose check from Part 1.

**`Lookup would require extrapolation` errors.** A TF publisher has stopped or is late. With Nav2 on the robot this is almost always the EKF or slam_toolbox. Check `ros2 topic hz /odometry/filtered` and `ros2 run tf2_ros tf2_echo map base_footprint`.

## Further reading

- Nav2 concepts, costmaps and tuning guide: https://docs.nav2.org (see *Tuning Guide* and *Configuration Guide → Regulated Pure Pursuit / Rotation Shim*)
- slam_toolbox: https://github.com/SteveMacenski/slam_toolbox
- nav2_simple_commander API: https://docs.nav2.org/commander_api/index.html
- Background on this robot's Nav2 choices: [slam_nav2.md](slam_nav2.md)
