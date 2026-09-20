# ORB-SLAM3 Calibration for QB3rt (OV9282 + RB3 IMU)

`config/orbslam3_ov9282_imu.yaml` was **calibrated and deployed on 2026-07-10** — the
placeholders are gone. This doc records the deployed values and the procedure to
re-calibrate. Do the numbered steps in order **only when re-calibrating**.

## Deployed calibration (2026-07-10, Kalibr `ov9282_calibration`)

- **Model: `KannalaBrandt8` (fisheye / equidistant), NOT PinHole.** Kalibr calibrated
  the OV9282 with the `equidistant` model, so the four distortion values are
  Kannala-Brandt `k1..k4` — do **not** paste them into pinhole `p1/p2`.
- **Resolution: 1280×720** (Kalibr was run at 720). Native sensor is 800; capture
  height in `rb3_vio.launch.py` **and** `perception.launch.py` was dropped 800→720 to
  match, and `qrb_ros_camera/config/camera_info_ov9282.yaml` was set to 720 + the same
  fisheye intrinsics. `nv12_to_mono8` preserves height (no vertical crop), so what
  ORB-SLAM3 sees is the raw capture resolution.
- **`IMU.T_b_c1 = inv(T_cam_imu)`** from `ov9282_calibration-camchain-imucam.yaml`
  (lever arm ≈ 0.117 m).
- **IMU noise:** Allan-variance values, inflated for robust VIO (noise ×2, random
  walks ×10 over raw allan — see `ov9282_calibration/imu.yaml`).
- **Not applied:** Kalibr `timeshift_cam_imu = −0.0195 s`. Stock ORB-SLAM3 has no IMU
  time-offset field; suspect it if initialization is sluggish.

> **Deploy note:** edit `config/orbslam3_ov9282_imu.yaml` in the git clone on
> your laptop, then push it over SSH:
> `deploy/update_project.sh --unit <id>` (or `--host ubuntu@<ip>`). Do not
> hand-edit the installed copy at `/opt/qb3rt/install/share/QB3rt` on-device —
> it's overwritten by the next deploy.

## RB3 preflight

SSH in as `ubuntu@<unit-ip>` — the ROS/DDS environment auto-loads at login via
`/etc/profile.d/qb3rt-ros-env.sh`, nothing to source by hand. The rootfs is a
normal writable filesystem (no ostree remount needed).

---

## Step 1 — Camera intrinsics (Kalibr or camera_calibration)

Calibrate the OV9282 at the **exact resolution used in flight** — currently **1280×720**
(set in `rb3_vio.launch.py` and `perception.launch.py`). If you change it, update both
launch files and `camera_info_ov9282.yaml` to match. The OV9282 tracking lens is wide-FOV
and is best fit by the **equidistant / fisheye** model → ORB-SLAM3 `Camera.type: KannalaBrandt8`.

```bash
# Record a bag while moving the checkerboard slowly in front of the camera
ros2 bag record /image_raw /camera_info -o ov9282_cal

# Run camera_calibration (ros2 version)
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  --ros-args -r image:=/image_raw -r camera_info:=/camera_info
```

For the fisheye/equidistant model, extract `fx`, `fy`, `cx`, `cy` and the four
`distortion_coeffs` → `Camera1.k1..k4` (Kannala-Brandt), and set
`Camera.type: KannalaBrandt8` in `orbslam3_ov9282_imu.yaml`. (Only use pinhole
`k1,k2,p1,p2` if you deliberately calibrate with a radtan/plumb_bob model instead.)

Also update `qrb_ros_camera/config/camera_info_ov9282.yaml` to keep the two in sync
(ORB-SLAM3 and the camera driver use separate copies).

---

## Step 2 — IMU–camera extrinsics: `IMU.T_b_c1` (Kalibr)

This is the **most important calibration** for mono-inertial VIO. The identity
placeholder will give wrong scale and heading.

`T_b_c1` is the transform from the IMU body frame (`/imu/data` frame) to the camera
frame — a 4×4 matrix encoding rotation + translation.

### Record a calibration bag

The robot must be **stationary on a flat surface** while you move the checkerboard
in front of it, exciting all IMU axes:

```bash
# 200 Hz IMU, 30 Hz camera — record at least 60 s of motion
ros2 bag record /imu/data /image_raw -o kalibr_imu_cam
```

### Convert to Kalibr format and run

```bash
# Convert ROS2 bag to ROS1 bag (Kalibr is ROS1-based)
pip install rosbags
rosbags-convert kalibr_imu_cam --dst kalibr_imu_cam.bag

# Run Kalibr (requires Docker or a ROS1 workspace with Kalibr installed)
kalibr_calibrate_imu_camera \
  --bag kalibr_imu_cam.bag \
  --cam camchain.yaml \        # output of Step 1 in Kalibr camchain format
  --imu imu.yaml \             # see IMU noise model section below
  --target checkerboard.yaml   # matches the board used in Step 1
```

Kalibr outputs `results-imucam-*.yaml`. The `T_cam_imu` matrix there is the inverse
of `T_b_c1`; invert it before pasting into `orbslam3_ov9282_imu.yaml`:

```python
import numpy as np
T_cam_imu = np.array([...])   # from Kalibr output
T_b_c1 = np.linalg.inv(T_cam_imu)
```

---

## Step 3 — IMU noise model

The values below affect ORB-SLAM3's IMU pre-integration weighting. Use the ICM-42688
datasheet for a starting point, then refine with Allan variance if needed.

| Parameter | Deployed (2026-07-10) | Unit |
|---|---|---|
| `IMU.NoiseGyro` | 4.903e-4 | rad/s/√Hz |
| `IMU.NoiseAcc` | 3.687e-3 | m/s²/√Hz |
| `IMU.GyroWalk` | 4.785e-5 | rad/s²/√Hz |
| `IMU.AccWalk` | 6.278e-4 | m/s³/√Hz |
| `IMU.Frequency` | 200.0 | Hz (verify with `ros2 topic hz /imu/data`) |

The deployed values are the raw Allan-variance results **inflated** for robust VIO
(noise ×2, random walks ×10) — the joint imu-cam calibration showed the raw model was
optimistic. Raw values are preserved in comments in `ov9282_calibration/imu.yaml`.

For Allan variance (more accurate):

```bash
# Record a long stationary bag (> 2 hours ideally, minimum 30 min)
ros2 bag record /imu/data -o imu_static

# Run imu_utils or allan_variance_ros
ros2 run allan_variance_ros allan_variance /path/to/imu_static
```

---

## Step 4 — Verify initialization

After updating all placeholders, launch the VIO chain standalone and confirm
ORB-SLAM3 initializes within ~5 seconds of motion:

```bash
ros2 launch QB3rt rb3_vio.launch.py show_viewer:=false
ros2 topic hz /vio/odometry   # should publish at ~30 Hz once initialized
ros2 topic echo /vio/odometry --once  # check that x,y values change with motion
```

If initialization takes > 15 s or fails repeatedly, the most likely cause is a wrong
`IMU.T_b_c1` — re-run Kalibr. Also confirm `/image_raw` is actually 720p
(`ros2 topic echo /image_raw --field height --once`); if `qrb_ros_camera` can't deliver
1280×720 the intrinsics won't match the image. The un-applied `timeshift_cam_imu`
(≈ −20 ms) is a secondary suspect.

---

## Step 4b — The init ritual and the VIO trust gate (IMPORTANT)

ORB-SLAM3's mono-inertial estimate is not metric until VIBA converges, which
needs *translational* excitation: after launch, drive a smooth **figure-8** for
~15-20 s, **then STAND STILL for a few seconds.**

The stop matters. The relay (`scripts/orbslam3_pose_to_odom.py`) will not let the
EKF trust VIO — nor latch `/vio/ready` — until VIO is BOTH:

1. gravity-aligned (corrected roll/pitch ~0), and
2. **metrically converged**: with the wheels reporting stationary, VIO also reads
   ~0 speed for `metric_window_s` (3 s). Ending the ritual with a stop is what
   lets this confirm; watch for the log `VIO metric convergence confirmed`.

Why this gate exists (2026-07-19): a *short/weak* figure-8 left VIO upright but
scale-unconverged, so it reported a steady ~1 m/s of phantom translation while
the robot was parked. Because the EKF's lateral `y`/`vy` channel is
VIO-only-observable, it integrated that into a runaway sideways drift of
`/odometry/filtered`. The fix was a longer figure-8 (real convergence) plus the
metric gate and a 0.6 m/s speed cap in the relay. If you ever see `vy` growing
while parked, VIO is under-converged — **redo the ritual, longer, and stop.**

## Step 5 — EKF sanity check

After VIO is publishing reliably AND the trust gate has confirmed (Step 4b),
confirm the EKF is fusing it and is quiet when parked:

```bash
ros2 topic echo /odometry/filtered --field twist.twist   # vx, vy, vyaw all ~0 parked
ros2 topic echo /odometry/filtered --once
# x, y should track actual robot position; yaw should match motion direction
```

Drive a straight 1 m line and check that `/odometry/filtered` reports ~1 m in x.
Then drive a 1 m square and check closure error (< 10 cm is good for this setup
without loop closure; slam_toolbox corrects residual drift via map→odom).
