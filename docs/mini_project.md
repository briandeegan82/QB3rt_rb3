# EE5110 Mini-Project — Visual Mapping, Perception & Sensor Fusion

**Platform:** Qualcomm RB3 Gen 2 + WAVE ROVER chassis, ROS 2 Jazzy
**Duration:** weeks 6–12 (demo + report due end of week 12) | **Solo project** | **Weight:** see rubric

---

## 1. Objective

Build, step by step, a robot that **autonomously finds a designated object
and navigates to it**. The final demonstration must show, running together
on the RB3:

1. The robot localizes itself in a saved map (`slam_toolbox` map + AMCL).
2. YOLOE object detection running on the Hexagon NPU identifies the target
   class and reports its bearing relative to the camera.
3. The system converts that detection into a goal in the map frame and
   sends it to Nav2.
4. Nav2 drives the robot to a safe stand-off distance from the object —
   without collisions, without human joystick input.

The graded product is the *chain*, not any single component: perception →
estimation → planning must close the loop.

## 2. What the platform gives you (and what it does not)

Each unit is provisioned by Ansible with:

- Ubuntu 24.04 + ROS 2 Jazzy, `rmw_cyclonedds_cpp`, a fleet `ROS_DOMAIN_ID`
- Drive bridge (`cmd_vel` → WAVE ROVER) publishing open-loop `/odom`
- Onboard IMU (`/imu/data` ~200 Hz), RPLIDAR (`/scan` ~10 Hz)
- URDF + `static_transforms.launch.py` (a **parameterized** frame tree —
  you must verify and set the parameters correctly for *your* unit)
- `robot_localization` EKF configs, `slam_toolbox`, Nav2 configs
- YOLOE NPU pipeline (`qrb_ros` camera → preprocess → QNN inference →
  postprocess → `/yoloe/detections`) with a COCO-80 model
- ORB-SLAM3 mono-inertial build on the OV9282 tracking camera

**Deliberately not provided:** a correct per-unit configuration, tuned
navigation, detection→goal conversion, or any integration code. Two units
can differ in drivetrain polarity and lidar mounting orientation — assuming
the reference config fits your robot is a bug, and discovering that is part
of the exercise.

## 3. Safety rules (mandatory — violations void the demo)

- **Never command long blind motions.** Check `/scan` in the correct
  base-frame sector before *every* drive leg; keep legs ≤ 0.5 m until you
  trust your transforms.
- `cmd_vel` commands must always be time-bounded (`timeout`), with a
  stop verified between pulses.
- Have an estop plan whenever the wheels are live — keyboard focus on a
  stop command, or a second person holding the e-stop.
- Verify which physical direction is your robot's "front" before first
  motion — do not assume.

## 4. Staged milestones

Work through the stages in order. Each stage has an *exit check* you
demonstrate before proceeding — this enforces the build-up and gives you
marks incrementally even if the capstone isn't reached.

### Stage 0 — Bring-up & conventions
Connect (SSH, domain, DDS), list nodes/topics, verify `/odom`, `/imu/data`,
`/scan` rates. **Probe your unit's conventions:** command +0.15 m/s `linear.x`
— which way does it roll? Command +0.5 rad/s `angular.z` — which way does it
turn vs. REP-103 (CCW viewed from above)? Record findings; set
`drive_inverted`/`lidar_yaw` accordingly and justify.

*Exit check:* one-sentence per-convention report + live `cmd_vel` demo where
each motion matches the command's intent.

### Stage 1 — TF tree
Visualize the transform tree (`ros2 run tf2_tools view_frames`, RViz).
Verify `map`/`odom`/`base_footprint`/`laser`/camera frames exist and agree
with REP-105. Prove the lidar frame is right: put an obstacle at the nose,
show it projects into the **front** sector in `base_footprint`, not the rear.

*Exit check:* TF tree screenshot + a scan-sector demonstration with a known
object at a known direction.

### Stage 2 — Odometry & the open-loop problem
Drive a 1 m square; plot commanded vs. `/odom`-believed path. Measure the
error on your demo surface (note: skid-steer slip is *surface-dependent* —
measure carpet AND hard floor if available). Quantify yaw drift over a 360°
turn using `/imu/data`.

*Exit check:* one figure + measured error numbers; explain *why* wheel
odometry alone is insufficient (this motivates the EKF and AMCL later).

### Stage 3 — EKF sensor fusion
Bring up `robot_localization`: fuse wheel forward-velocity + IMU yaw-rate.
Verify `/odometry/filtered` tracks commanded motion and that
`odom → base_footprint` publishes. Compare filtered vs. raw odometry over
the same square.

*Exit check:* live `/odometry/filtered` echo during a drive + explanation of
what each fused input contributes and what is *not* observable.

### Stage 4 — Mapping
Run `slam_toolbox`; build a map of the assigned area. Save it (`yaml`+`pgm`).
Common failure modes worth experiencing: ghost walls from a heading that
never physically changed, mirrored geometry from a wrong drive convention,
scan drops while TF is incomplete.

*Exit check:* a `map → base_footprint` live transform + a saved map your
instructor can reload.

### Stage 5 — Localization + Nav2
AMCL on your saved map; seed, drive, watch particles converge. Send a
`navigate_to_pose` goal and iterate until the robot reaches it and *stops*.
You will need to reason about goal tolerance vs. a skid-steer's yaw
authority, and why a controller that pivots first (rotation shim) helps.

*Exit check:* a Nav2 goal that physically succeeds, with your `nav2.yaml`
diff and a sentence per changed parameter saying why.

### Stage 6 — Perception
Run the YOLOE pipeline; echo `/yoloe/detections`
(`qrb_ros_vision_msgs/msg/Detection2DWithMaskArray`). For a chosen COCO
class, extract class/score/bbox, convert horizontal image offset to a
bearing using the camera FOV, and verify against a real object at a known
angle. Characterize limits: minimum detectable size, frame edges, and the
floor blind spot near the nose.

*Exit check:* printed bearing estimate matching a measured physical angle
within ~10°.

### Stage 7 — Capstone: find & navigate
Integrate: sweep (or use a cued direction) until the target class is
detected; estimate bearing; combine with the AMCL pose to compute a
map-frame goal at a safe stand-off; send it via the Nav2 action; handle
re-detection/replanning if the first goal aborts. The robot must stop
clear of the object using its costmaps.

*Exit check:* live end-to-end run — target placed by the instructor, robot
drives to it unassisted.

### Stage 8 — Stretch (optional, bonus)
Fuse ORB-SLAM3 VIO into the EKF through the provided health-gated bridge.
Initialize with smooth translational motion (rotation-only motion cannot
converge a mono-inertial estimate). Compare `/odometry/filtered` drift with
and without VIO.

## 5. Suggested schedule

| Week | Milestone |
|---|---|
| 6 | Stages 0–1: platform access, convention probe, TF tree verified |
| 7 | Stage 2: odometry characterization (square test, surface comparison) |
| 8 | Stages 3–4: EKF live; map built, saved, reloadable |
| 9 | Stage 5: AMCL + first successful Nav2 goal |
| 10 | Stage 6: YOLOE detections → bearing estimate verified |
| 11 | Stage 7: capstone integration, dry-run demos; report drafted |
| 12 | Buffer / polish; **demo + report due end of week** |

The schedule front-loads the risky hardware stages — if Stage 5 slips,
weeks 11–12 still absorb it without losing demo marks for Stages 0–4.

## 6. Demonstration format

15 minutes per student: ~10 min live staged demo (each exit check you can
show counts), ~5 min questions. The instructor chooses the target class for
the capstone. **Backup rule:** if live hardware fails, a recorded video of
the same staged runs earns demo marks at 60%.

## 7. Report

**4–6 pages, IEEE conference format** (incl. figures; refs don't count
against the limit). Structure:

1. *System overview* — block diagram of your final data flow (sensors →
   estimation → planning → actuation)
2. *Per-stage results* — the exit-check evidence (figures/measurements),
   what broke and how you diagnosed it
3. *Analysis* — odometry/AMCL error characterization, detector limits,
   Nav2 tuning rationale
4. *Reflection* — the failure that taught you most; what you'd harden for
   real deployment

Quality > coverage: a correct, measured Stage-5 write-up beats a hand-wavy
Stage-8 one.

## 8. Rubric (100 pts)

| Component | Pts |
|---|---|
| **Live demonstration** | **60** |
| — Stages 0–2: conventions, TF, odometry characterization | 12 |
| — Stages 3–4: EKF + saved, coherent map | 12 |
| — Stage 5: successful Nav2 goal + tuning rationale | 12 |
| — Stage 6: detection → verified bearing estimate | 10 |
| — Stage 7 capstone: find & navigate to instructor target | 14 |
| **Report** (4–6 pp) | **30** |
| — System overview & data flow | 6 |
| — Evidence per stage (figures, measurements) | 12 |
| — Analysis & failure diagnosis | 8 |
| — Reflection | 4 |
| **Process** (stage exits signed off on time, safety observed) | **10** |
| *Bonus: Stage 8 VIO fusion with quantified improvement* | *+8* |

Partial credit: exit checks are independent — later stages not reached do
not zero earlier marks. The capstone awards partial credit for a correct
detection→goal chain even if Nav2 aborts (with diagnosis).

## 9. References

- Nav2 documentation: https://docs.nav2.org/ (concepts → controller server,
  rotation shim, goal checker)
- `slam_toolbox`: https://github.com/SteveMacenski/slam_toolbox
- `robot_localization` docs & state-estimation tutorial:
  https://docs.ros.org/en/jazzy/p/robot_localization/
- REP-103 (units & conventions), REP-105 (frame conventions):
  https://www.ros.org/reps/rep-0103.html · https://www.ros.org/reps/rep-0105.html
- tf2 tutorials: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/
- YOLOE ("Seeing Anything"): https://arxiv.org/abs/2503.07465 ·
  Qualcomm AI Hub model export: https://aihub.qualcomm.com/
- Cyclone DDS config (ROS_DOMAIN_ID / multicast):
  https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html
- RPLIDAR C1 datasheet (range/FOV sanity): https://www.slamtec.com/en/C1
- Macenski & Jambrecic, "SLAM Toolbox: SLAM for the dynamic world",
  *J. Open Source Software* (2021) — for the report's related-work section
