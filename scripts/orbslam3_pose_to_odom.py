#!/usr/bin/env python3
"""Bridge ORB-SLAM3 camera_pose (PoseStamped) -> nav_msgs/Odometry for the EKF.

orb_slam3_ros_mono_imu publishes only `camera_pose` as a geometry_msgs/PoseStamped:
the pose of the camera in the SLAM "world" frame, expressed in CAMERA-OPTICAL axes
(x-right, y-down, z-forward) - and with no covariance, no child_frame_id, and no
twist. robot_localization's EKF (config/ekf.yaml, odom0) consumes nav_msgs/Odometry
in REP-103 axes (x-forward, y-left, z-up). This node bridges the two:

  in  : geometry_msgs/PoseStamped  (default topic: camera_pose)
  out : nav_msgs/Odometry          (default topic: /vio/odometry)

What it does:
  1. Re-bases the whole trajectory from camera-optical axes into a REP-103 / ROS
     odom frame via a fixed change-of-basis R_REBASE, applied as p_ros = R p_opt
     and M_ros = R M_opt R^T so both translation and orientation end up
     consistent (see R_REBASE below for what it actually contains and why - it
     is NOT the textbook optical->ROS formula alone).
  2. Applies a second, fixed correction (R_MOUNT) removing the camera's own
     physical mount tilt from the ORIENTATION, turning the camera attitude into
     the BASE attitude. R_MOUNT is DERIVED from the Kalibr IMU.T_b_c1 extrinsic
     (see below), not fitted from session data.
  3. Removes the camera->base lever arm from the POSITION (the pose is the
     CAMERA's, but child_frame_id claims base_footprint; the camera sits
     ~7.3 cm off the base origin horizontally, so pure yaw would otherwise
     show up as ~15 cm of phantom XY translation).
  4. Stamps a diagonal covariance (the EKF needs one; ORB-SLAM3 provides none).
  5. Sets header.frame_id (odom) and child_frame_id (base_footprint) so the EKF
     accepts it as odom1.
  6. HEALTH-GATES the output (2026-07-18): publishes only while ORB-SLAM3's
     world is gravity-aligned (corrected base roll/pitch ~0). Before VIBA
     converges (init ritual) or when tracking degrades, output is suppressed so
     the EKF (odom1, differential) coasts on wheel+gyro instead of ingesting
     wandering VIO poses. This is what makes re-fusing VIO safe after it was
     removed 2026-07-13 for out-voting wheel+gyro while driving; see
     config/ekf.yaml's odom1 block. Gating uses the base attitude, so it is
     active only when both optical_to_ros and apply_mount_correction are on.
  6b. METRIC-CONVERGENCE gate (2026-07-19): gravity alignment is necessary but
     NOT sufficient. A short/weak figure-8 can leave VIO upright (passing 6) yet
     scale-unconverged, reporting ~1 m/s of phantom translation while PARKED,
     which the EKF integrates as a lateral runaway (y/vy is VIO-only-observable).
     So output is also withheld (and /vio/ready not latched) until, with wheel
     odom reporting the robot stationary, VIO ALSO reads ~0 speed for a sustained
     window - proof it is metrically initialized, not just level. Additionally
     max_speed_m_s was tightened to 0.6 (~platform top speed) so any VIO frame
     implying impossible speed is dropped. Needs wheel odom (wheel_odom_topic);
     camera-only bringups fall back to the roll/pitch-only gate (6). OPERATOR:
     end the figure-8 ritual with a brief stop so this gate can confirm.

CALIBRATION HISTORY (2026-07-14) - R_REBASE was established EMPIRICALLY from
real hardware data after two theoretical attempts failed (R_MOUNT was later
RE-derived from Kalibr once the frame conventions were pinned down - see the
DEEP DIVE addendum at the end of this section):
  - Attempt 1 assumed the textbook optical->ROS formula
    (x_ros=z_opt, y_ros=-x_opt, z_ros=-y_opt) was correct on its own, and added
    only a camera-mount-tilt correction on top. WRONG: a live orientation
    sample on a stationary, level robot decoded to roll=-55/pitch=+38 deg.
  - Attempt 2 "fixed" that tilt correction's sign via a from-scratch physical
    re-derivation, validated by a parametric sweep of synthetic yaw angles.
    ALSO WRONG despite passing its own sweep test: physically rotating the
    robot about true Z showed up in RViz as a rotation about X - the sweep
    only checked internal self-consistency, not agreement with reality. Worse,
    this class of fix (composing a correction via RIGHT-multiplication,
    m_final = m_ros @ R_extra) is PROVABLY INCAPABLE of changing which axis
    captures rotation as the robot moves: for any fixed R_extra,
    (m(b) @ R_extra) @ (m(a) @ R_extra)^T = m(b) @ m(a)^T identically, since
    R_extra @ R_extra^T = I cancels out. So the true bug (rotation appearing on
    the wrong axis) could never have been fixed by right-multiplication alone,
    no matter which angle/sign was chosen.
  - The actual fix needed a CONJUGATION (m -> T @ m @ T^T), which - unlike
    right-multiplication - DOES change the apparent axis of relative rotation.
    Diagnosed by physically yawing the robot ~90 deg at a time (4 samples,
    optical_to_ros rebase active, mount correction disabled via
    camera_mount_pitch_deg:=0.0 - since removed) and measuring the empirical
    relative-rotation axis between consecutive samples directly: it came out
    as (-1.000, ~0, ~0) - i.e. true world-Z yaw was appearing entirely on the
    rebased frame's OWN X axis. T = Ry(90 deg) is the (clean, exact) rotation
    that maps that axis to Z; composing it into the rebase conjugation
    (R_REBASE = T @ [textbook optical->ROS matrix]) collapsed yaw onto the
    correct channel, leaving a residual near-constant roll/pitch offset - the
    genuine, but not cleanly-parametrizable, camera mount tilt - fit as
    R_MOUNT from the same 4 samples. Validated: <0.7 deg roll/pitch residual
    across all 4 independent real samples, yaw tracking the ~90 deg turns to
    within a few degrees.

LESSON: validate ANY orientation/frame fix against real, physically-executed
motion (ideally a multi-sample sweep), never against synthetic data alone and
never against a single live sample - both can pass while still being wrong.

DEEP DIVE ADDENDUM (2026-07-14, later the same day) - the R_MOUNT mystery
resolved. Decomposing Kalibr's IMU.T_b_c1 established the actual frame
geometry: the /imu/data frame is x=robot-RIGHT, y=robot-FORWARD, z=UP (yawed
-90 deg vs base FLU; corroborated by the Kalibr lever arm matching the URDF
offsets only under that mapping, image-right = robot-right, and the EKF's
working yaw-rate-on-z), and the OV9282 optical axis is elevated 35.3 deg
above horizontal - NOT ~55 deg; 54.7 deg is the raw rotation ANGLE of the
Kalibr matrix (the complement), an artifact of the stacked IMU-yaw + optical
conventions. With that pinned down, the mount correction is DERIVABLE in
closed form (R_MOUNT below) and the model predicts a stationary level robot
reads roll~180/pitch~-54.7 on the RAW rebased orientation - exactly what
session 2 measured, i.e. session 2 was a HEALTHY gravity-aligned session.
The old fitted R_MOUNT differed from the derived one by 180 deg yaw
(arbitrary per-session world yaw, harmless) PLUS ~16 deg of non-yaw error:
session 1 (the fit source) had a ~16 deg gravity-alignment (VIBA) error
baked in. So "no fixed constant transfers" was true only of FITTED
constants; the derived one is hardware truth. After R_MOUNT, stationary
roll/pitch should be ~0 - a nonzero reading is a per-session VIBA health
indicator, not a correction to re-fit.

LIVE VALIDATION (2026-07-14, after deploying the derived R_MOUNT): a fresh
4-sample ~90 deg yaw test read pitch ~0 at every yaw (R_MOUNT confirmed) but
roll = 180 constant - the world frame is gravity-aligned z-UP (ORB-SLAM3's
standard convention), so the z-down interpretation behind the original
_T_AXIS_FIX choice had the vertical sign flipped (see _R_WORLD_FLIP above
for why session 1's measurement couldn't distinguish the two). With
_R_WORLD_FLIP composed in, expected stationary reading on a healthy session:
roll ~0, pitch ~0, yaw = physical yaw + arbitrary per-session offset, with
CCW positive; position free of yaw-dependent wobble.
"""

import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


# Textbook optical -> ROS-body axis-convention swap (x_ros=z_opt, y_ros=-x_opt,
# z_ros=-y_opt). Kept for reference/documentation - NOT used alone; see
# R_REBASE below, which composes an additional empirically-required correction
# on top of this (see the module docstring's CALIBRATION HISTORY).
_R_TEXTBOOK_OPT_ROS = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ]
)

# T: additional axis-remapping conjugation, empirically required on top of the
# textbook formula (2026-07-14, see docstring). Exactly Ry(90 deg) - a clean,
# exact value (the empirical relative-rotation axis measured (-1.000, ~0, ~0)
# across 3 independent sample pairs, not an arbitrary angle needing more
# precision), consistent with a genuine axis-labeling mismatch rather than a
# continuous physical tilt.
_T_AXIS_FIX = np.array(
    [
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ]
)

# World vertical flip (2026-07-14, LIVE 4-sample yaw test after the derived
# R_MOUNT was deployed): the corrected output read roll=180/pitch~0 at every
# yaw - the exact signature of the ORB-SLAM3 world being gravity-aligned
# z-UP (its standard gI=(0,0,-1) convention), not z-down as session 1's
# relative-axis measurement implied. That measurement was sign-ambiguous
# (CW physical turns + z-up world produce the same (-1,0,0) axis as CCW
# turns + z-down; the turn direction was never recorded). Without this flip
# the output frame was upside-down: constant 180 roll, inverted yaw sign,
# y-mirrored trajectory, and a partially-wrong lever-arm removal (~13 cm
# stationary position spread across yaws, observed live).
_R_WORLD_FLIP = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
)

# R_REBASE: the actual change-of-basis used for BOTH position and orientation
# (p_ros = R_REBASE @ p_opt ; M_ros = R_REBASE @ M_opt @ R_REBASE^T). Position
# uses this too, not just orientation, because R_REBASE corrects how the WORLD
# frame's own optical-labeled axes get relabeled into ROS axes - a property of
# the rebase itself, independent of the camera's own physical mount tilt. This
# is also the most likely explanation for the earlier-observed "x/z position
# coupling while driving" symptom, previously (and only partly correctly)
# attributed to VIO tracking noise alone.
#
# The full composition collapses to exactly Rz(+90 deg) - i.e. the ORB-SLAM3
# world frame is ALREADY a z-up, right-handed frame and only differs from the
# output odom frame by an (arbitrary anyway) yaw. It is kept as the explicit
# product so the calibration history in the module docstring stays traceable.
R_ROS_OPT = _R_WORLD_FLIP @ _T_AXIS_FIX @ _R_TEXTBOOK_OPT_ROS

# Rotation block of Kalibr IMU.T_b_c1 (config/orbslam3_ov9282_imu.yaml,
# CALIB 2026-07-10): columns = camera-optical axes expressed in the /imu/data
# body frame. Keep in sync with the ORB-SLAM3 config if re-calibrated.
_R_BC_KALIBR = np.array(
    [
        [0.9999878475, 0.0029333643, -0.0039623511],
        [0.0015378460, 0.5780185589, 0.8160221692],
        [0.0046840028, -0.8160183460, 0.5780070234],
    ]
)

# /imu/data axes expressed in base FLU: x_imu=robot-right, y_imu=robot-forward,
# z_imu=up (the ICM-42688 is mounted yawed -90 deg vs the chassis; see the
# DEEP DIVE addendum in the module docstring for the corroborating evidence).
_R_BASE_IMU = np.array(
    [
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ]
)

# R_MOUNT: fixed camera-mount correction, applied by right-multiplication AFTER
# the R_ROS_OPT conjugation (m_final = m_ros @ R_MOUNT). It converts the
# (rebased) CAMERA attitude into the BASE attitude, removing the OV9282's
# 35.3 deg up-tilt and the optical-axes labeling in one step. DERIVED from the
# Kalibr extrinsic - do NOT replace it with a matrix fitted from session data:
# a fit absorbs that session's arbitrary world yaw AND its VIBA gravity error
# (that mistake is documented in the module docstring). Derivation: base axes
# in the rebased-world labeling = R_ROS_OPT @ (base axes in optical labels)
# = R_ROS_OPT @ (R_base_imu @ R_bc)^T.
R_MOUNT = R_ROS_OPT @ (_R_BASE_IMU @ _R_BC_KALIBR).T

# Camera position in the base_footprint frame (m), from the Kalibr lever arm
# mapped through _R_BASE_IMU plus the URDF imu mount (base_link + (0, 0.030,
# 0.070)) and the base_footprint->base_link height (0.0847). Used to remove
# the camera->base lever arm from the published position.
_CAM_POS_BASE = np.array([0.0849099141, 0.0159180436, 0.2338549663])


def quat_to_matrix(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ]
    )


def matrix_to_quat(m):
    # Numerically stable matrix -> quaternion (x, y, z, w).
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    nrm = np.linalg.norm(q)
    return q / nrm if nrm > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0])


class PoseToOdom(Node):
    def __init__(self):
        super().__init__("orbslam3_pose_to_odom")

        self.declare_parameter("input_topic", "camera_pose")
        self.declare_parameter("output_topic", "/vio/odometry")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_footprint")
        # Re-base optical -> ROS axes (R_ROS_OPT, which includes the empirically
        # -required axis fix - see module docstring). Disable only if
        # camera_pose is already REP-103.
        self.declare_parameter("optical_to_ros", True)
        # Apply R_MOUNT (camera attitude -> base attitude). DEFAULT TRUE
        # (re-enabled 2026-07-14 after R_MOUNT was re-derived from the Kalibr
        # extrinsic instead of fitted from session data - see R_MOUNT's own
        # comment). With a healthy gravity-aligned VIO session, stationary
        # roll/pitch should now read ~0; a large stationary roll/pitch means
        # THAT SESSION's VIBA gravity alignment is bad (re-init the VIO), not
        # that this constant needs re-fitting.
        self.declare_parameter("apply_mount_correction", True)
        # Subtract the camera->base lever arm from the position (requires the
        # mount correction for the base attitude). Without it, pure yaw shows
        # up as up-to-~15 cm of phantom XY translation (7.3 cm horizontal
        # camera offset from base_footprint).
        self.declare_parameter("apply_lever_arm", True)
        # Diagonal covariance the EKF will see (position is the trustworthy channel).
        self.declare_parameter("position_variance", 0.01)
        self.declare_parameter("orientation_variance", 0.05)
        # Jump detection: if the position moves more than this between frames,
        # treat it as an ORB-SLAM3 world-frame reset and suppress output for
        # cooldown_s seconds so rtabmap doesn't ingest discontinuous poses.
        self.declare_parameter("max_jump_m", 1.0)
        self.declare_parameter("cooldown_s", 2.0)
        # Physical-plausibility gate: max believable robot speed (m/s). Any
        # consecutive-sample displacement implying more than this is a VIO
        # artifact (map re-init origin shift, scale glitch), not motion - the
        # platform tops out at ~0.42 m/s. Without this gate the EKF's
        # differential fusion turns such a jump into a huge instantaneous
        # velocity and integrates it (2026-07-13: 344 m of phantom path in a
        # 7 s straight run). Same suppress+re-anchor handling as max_jump_m.
        # TIGHTENED to 0.6 (2026-07-19, was 1.5): an under-converged VIO reported
        # a steady ~1 m/s of phantom translation while the robot was PARKED,
        # which the EKF integrated as a lateral runaway (the y/vy channel is
        # VIO-only-observable). 0.6 sits just above the platform's ~0.42 m/s top
        # speed, so it drops physically-impossible VIO motion frame-by-frame
        # while leaving real driving intact. Raise toward 0.8 if genuine
        # fast-drive frames get dropped (VIO stamp jitter can briefly inflate
        # jump/dt); lower toward 0.5 if phantom drift still leaks.
        self.declare_parameter("max_speed_m_s", 0.6)
        # Metric-convergence gate (2026-07-19): the roll/pitch health gate proves
        # ORB-SLAM3's world is gravity-aligned but NOT that VIO is metrically
        # converged. A short/weak figure-8 init can leave VIO upright (passes
        # roll/pitch) yet not scale-converged, so it reports large phantom
        # translation while the robot is stopped - the exact 1 m/s vy runaway
        # seen on 2026-07-19. So do NOT trust VIO (nor latch /vio/ready) until,
        # with the WHEELS reporting stationary, VIO ALSO reads ~0 speed for a
        # sustained window. Needs wheel odom (wheel_odom_topic); a camera-only
        # bringup without /odom leaves this gate inactive and falls back to the
        # roll/pitch-only behavior. Operator note: end the figure-8 ritual with a
        # brief (~metric_window_s) stop so this can confirm and latch ready.
        self.declare_parameter("require_metric_convergence", True)
        self.declare_parameter("wheel_odom_topic", "/odom")
        # Wheel |vx| below this (m/s) = robot considered physically stationary.
        self.declare_parameter("metric_stationary_speed", 0.05)
        # Max VIO speed (m/s) allowed while stationary to count as converged.
        self.declare_parameter("metric_vio_speed", 0.05)
        # Sustained stationary duration (s) VIO must stay quiet to be trusted.
        self.declare_parameter("metric_window_s", 3.0)
        # Health gate: suppress output while ORB-SLAM3's world is NOT
        # gravity-aligned (VIBA not converged / tracking lost), so the EKF
        # (odom1) never ingests wandering, un-initialized VIO poses and simply
        # coasts on wheel+gyro instead. Gating uses the corrected base
        # roll/pitch, so it is only meaningful when the optical->ROS rebase AND
        # the mount correction are active; with either off, output is never
        # health-suppressed (the pre-2026-07-18 behavior). misalignment_deg is
        # the median |roll/pitch| over ~5 s above which VIO is judged unhealthy.
        self.declare_parameter("suppress_when_misaligned", True)
        self.declare_parameter("misalignment_deg", 15.0)
        # Readiness latch for launch sequencing: publish a latched (transient_local)
        # std_msgs/Bool True the first time VIO becomes gravity-aligned, so
        # slam_toolbox can be held until ORB-SLAM3 has finished its VIBA
        # calibration (see launch/lidar_slam.launch.py gate_slam_on_vio). Latched
        # once True and left there: the gate is a one-shot START condition, not a
        # continuous health signal (a later VIO hiccup must not tear SLAM down).
        # Only meaningful when the health gate is active (rebase + mount on).
        self.declare_parameter("publish_ready", True)
        self.declare_parameter("ready_topic", "/vio/ready")

        self._out_frame = self.get_parameter("odom_frame_id").value
        self._child_frame = self.get_parameter("child_frame_id").value
        self._rebase = bool(self.get_parameter("optical_to_ros").value)
        self._apply_mount = bool(self.get_parameter("apply_mount_correction").value)
        self._apply_lever_arm = bool(self.get_parameter("apply_lever_arm").value)
        pvar = float(self.get_parameter("position_variance").value)
        ovar = float(self.get_parameter("orientation_variance").value)
        self._max_jump = float(self.get_parameter("max_jump_m").value)
        self._cooldown = float(self.get_parameter("cooldown_s").value)
        self._max_speed = float(self.get_parameter("max_speed_m_s").value)
        self._require_metric_convergence = bool(
            self.get_parameter("require_metric_convergence").value
        )
        wheel_topic = self.get_parameter("wheel_odom_topic").value
        self._metric_stationary_speed = float(
            self.get_parameter("metric_stationary_speed").value
        )
        self._metric_vio_speed = float(self.get_parameter("metric_vio_speed").value)
        self._metric_window_s = float(self.get_parameter("metric_window_s").value)
        self._suppress_misaligned = bool(
            self.get_parameter("suppress_when_misaligned").value
        )
        self._misalignment_deg = float(self.get_parameter("misalignment_deg").value)
        self._publish_ready = bool(self.get_parameter("publish_ready").value)
        ready_topic = self.get_parameter("ready_topic").value
        # Whether health-gating can apply at all: roll/pitch is only the BASE
        # attitude (hence a valid VIBA-alignment signal) when both the rebase
        # and the mount correction are active.
        self._health_gate_active = self._rebase and self._apply_mount
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        # Jump / reset state.
        self._last_pos: np.ndarray | None = None  # last published position (raw, pre-rebase)
        self._last_stamp: float | None = None      # header stamp (s) of last published pose
        self._reset_at: float | None = None        # monotonic time of last detected reset

        # Gravity-alignment (VIBA) health check. On this planar robot the
        # corrected roll/pitch should sit near 0 on a healthy session; a large
        # persistent value means ORB-SLAM3's world never gravity-aligned
        # (classic un-initialized signature: stationary pitch ~ -55 deg, the
        # raw camera tilt showing through) and orientation + lever-arm outputs
        # are untrustworthy. Median over ~5 s so handheld wobble doesn't trip it.
        self._rp_window: deque[float] = deque(maxlen=75)
        self._last_health_warn = 0.0
        # VIO health for the EKF gate. Start UNHEALTHY: never feed the filter
        # until VIO has proven a gravity-aligned world (survives the init
        # ritual). Flipped by _check_gravity_alignment once the window fills.
        self._healthy = False
        # Set once, when /vio/ready is first latched True, so we don't re-publish.
        self._ready_latched = False

        # Metric-convergence gate state (see _update_metric_convergence). Wheel
        # odom is the independent "is the robot ACTUALLY stopped" reference (VIO
        # cannot self-report that when VIO is the thing drifting). _metric_ok
        # latches True once VIO has proven, over a stationary window, that it
        # agrees the robot is stopped; thereafter the max_speed gate handles any
        # later per-frame drift.
        self._have_wheel_ref = False
        self._wheel_stationary = False
        self._wheel_last_time: float | None = None
        self._metric_ok = False
        self._metric_since: float | None = None

        self._cov = self._build_cov(pvar, ovar)

        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(Odometry, out_topic, qos)
        self._sub = self.create_subscription(PoseStamped, in_topic, self._on_pose, qos)

        # Wheel-odometry reference for the metric-convergence gate: it tells us
        # when the robot is truly stationary so we can check whether VIO agrees.
        # A camera-only bringup without /odom simply never sets _have_wheel_ref,
        # leaving the metric gate inactive (roll/pitch-only fallback).
        self._wheel_sub = self.create_subscription(
            Odometry, wheel_topic, self._on_wheel, qos
        )

        # Latched readiness publisher (transient_local so a gate node starting
        # AFTER we go ready still receives the retained True).
        self._ready_pub = None
        if self._publish_ready:
            ready_qos = QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._ready_pub = self.create_publisher(Bool, ready_topic, ready_qos)
        health_gate = (
            f"health_gate=on (>{self._misalignment_deg:.0f} deg suppresses)"
            if (self._suppress_misaligned and self._health_gate_active)
            else "health_gate=off"
        )
        if self._ready_pub is not None:
            health_gate += f", latches {ready_topic} on first-trusted"
        if self._require_metric_convergence:
            health_gate += (
                f", metric_gate=on (needs wheels<{self._metric_stationary_speed:.2f}"
                f" + VIO<{self._metric_vio_speed:.2f} m/s for {self._metric_window_s:.0f}s)"
            )
        self.get_logger().info(
            f"orbslam3_pose_to_odom: {in_topic} (PoseStamped) -> {out_topic} "
            f"(Odometry, frame={self._out_frame}, child={self._child_frame}, "
            f"optical_to_ros={self._rebase}, mount_correction={self._apply_mount}, "
            f"lever_arm={self._apply_lever_arm}, {health_gate})"
        )

    def _check_gravity_alignment(self, m, now):
        """Update VIO health from corrected roll/pitch; warn (throttled) if bad.

        m is the fully-corrected base attitude; on a planar robot its
        roll/pitch should be ~0 whenever ORB-SLAM3's VIBA gravity alignment
        has converged. Sets self._healthy (consumed by the EKF output gate in
        _on_pose) from the median |roll/pitch| over the ~5 s window, so the
        median smooths handheld/driving wobble instead of tripping on one
        frame. The characteristic failure reading is pitch ~ -55 deg (world
        stuck at the initial camera frame, camera tilt showing through) - the
        fix is the init ritual (smooth figure-8 translations 15-20 s after
        launch), never re-fitting R_MOUNT.
        """
        roll = np.degrees(np.arctan2(m[2, 1], m[2, 2]))
        pitch = np.degrees(np.arcsin(np.clip(-m[2, 0], -1.0, 1.0)))
        self._rp_window.append(max(abs(roll), abs(pitch)))
        # Not enough samples yet to trust the median: stay in the startup
        # UNHEALTHY state rather than flip health on one noisy frame.
        if len(self._rp_window) < 30:
            return
        med = float(np.median(self._rp_window))
        was_healthy = self._healthy
        self._healthy = med <= self._misalignment_deg
        # NOTE: the /vio/ready latch is NOT done here anymore. Gravity alignment
        # (roll/pitch) is necessary but not sufficient - a scale-unconverged VIO
        # can be upright yet drifting. Latching now also requires metric
        # convergence and is handled in _on_pose (see _update_metric_convergence).
        # Warn on the healthy->unhealthy transition, then throttle to ~10 s
        # while it stays unhealthy.
        if not self._healthy and (was_healthy or now - self._last_health_warn >= 10.0):
            self._last_health_warn = now
            tail = (
                "Output SUPPRESSED (EKF coasts on wheel+gyro)"
                if self._suppress_misaligned
                else "Output still published (suppress_when_misaligned=false)"
            )
            self.get_logger().warn(
                f"VIO world NOT gravity-aligned: median |roll/pitch| = "
                f"{med:.0f} deg over the last {len(self._rp_window)} samples "
                f"(healthy sessions read ~0; ~55 = VIBA never converged). "
                f"{tail} - redo the init ritual (smooth figure-8 "
                f"translations, 15-20 s)."
            )

    def _on_wheel(self, msg: Odometry):
        """Track wheel-odometry stationarity as the metric-convergence reference.

        Only the commanded/estimated forward speed (twist vx) is needed: it is
        the independent witness that the robot is physically stopped, which VIO
        cannot provide when VIO itself is the thing drifting.
        """
        self._have_wheel_ref = True
        self._wheel_last_time = time.monotonic()
        self._wheel_stationary = (
            abs(msg.twist.twist.linear.x) < self._metric_stationary_speed
        )

    def _update_metric_convergence(self, now, vio_speed):
        """Latch VIO as metrically converged, not merely gravity-aligned.

        A short/weak figure-8 can leave ORB-SLAM3 upright (passing the roll/pitch
        health gate) yet not scale-converged, so it reports large phantom
        translation while the robot is stopped - which the EKF, whose lateral
        (y/vy) channel is VIO-only-observable, integrates into a sideways runaway
        (observed 2026-07-19: ~1 m/s vy while parked). We therefore only TRUST
        VIO once, with the WHEELS reporting stationary, VIO ALSO reads ~0 speed
        for a sustained window. One-shot latch: after that, per-frame drift is
        caught by the max_speed gate. Needs a fresh wheel reference; without one
        (camera-only bringup) the gate is inactive and the caller falls back to
        the roll/pitch-only behavior.
        """
        if self._metric_ok or not self._require_metric_convergence:
            return
        # A stale (or never-seen) wheel reference means we cannot assert the
        # robot is stationary, so we cannot judge convergence: leave it to the
        # fallback (metric gate inactive) rather than latch on VIO alone.
        wheel_fresh = (
            self._wheel_last_time is not None
            and (now - self._wheel_last_time) < 1.0
        )
        if not wheel_fresh:
            return
        if (
            self._wheel_stationary
            and vio_speed is not None
            and vio_speed < self._metric_vio_speed
        ):
            if self._metric_since is None:
                self._metric_since = now
            elif now - self._metric_since >= self._metric_window_s:
                self._metric_ok = True
                self.get_logger().info(
                    "VIO metric convergence confirmed: wheels stationary and "
                    f"VIO speed < {self._metric_vio_speed:.2f} m/s sustained "
                    f"{self._metric_window_s:.0f}s - VIO now trusted by the EKF."
                )
        else:
            # Robot moving, or VIO still reports motion while parked: not yet
            # converged (a drifting VIO can never satisfy this, so it stays
            # untrusted and the EKF coasts on wheel+gyro).
            self._metric_since = None

    @staticmethod
    def _build_cov(pvar, ovar):
        cov = [0.0] * 36
        cov[0] = pvar       # x
        cov[7] = pvar       # y
        cov[14] = pvar      # z
        cov[21] = ovar      # roll
        cov[28] = ovar      # pitch
        cov[35] = ovar      # yaw
        return cov

    def _on_pose(self, msg: PoseStamped):
        p = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )

        # Jump detection: guard against ORB-SLAM3 world-frame resets.
        # Distances are preserved under R_ROS_OPT (orthogonal), so check on raw p.
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._reset_at is not None:
            if now - self._reset_at < self._cooldown:
                return                                  # still in cooldown; drop
            self._reset_at = None
            self._last_pos = None                       # accept next pose as new origin
            self._last_stamp = None
        if self._last_pos is not None:
            jump = float(np.linalg.norm(p - self._last_pos))
            dt = stamp - self._last_stamp if self._last_stamp is not None else 0.0
            vio_speed = jump / dt if dt > 0.0 else None
            # Feed the metric-convergence gate BEFORE any suppression: a VIO
            # drifting while the robot is parked must PREVENT trust (reset the
            # convergence window), not merely be dropped frame-by-frame.
            self._update_metric_convergence(now, vio_speed)
            # Absolute jump (origin shift) OR implied speed beyond what the
            # platform can physically do (fast reset/scale artifact): both are
            # discontinuities the EKF must never see.
            implausible = (
                jump > self._max_jump
                or dt <= 0.0
                or (self._max_speed > 0.0 and vio_speed is not None
                    and vio_speed > self._max_speed)
            )
            if implausible:
                speed_txt = f"{jump / dt:.1f} m/s" if dt > 0.0 else "dt<=0"
                self.get_logger().warn(
                    f"ORB-SLAM3 discontinuity: jumped {jump:.2f} m in "
                    f"{max(dt, 0.0) * 1000:.0f} ms ({speed_txt}; limits "
                    f"{self._max_jump} m / {self._max_speed} m/s). "
                    f"Suppressing output for {self._cooldown:.1f}s."
                )
                self._reset_at = now
                self._last_pos = None
                self._last_stamp = None
                return
        self._last_pos = p.copy()
        self._last_stamp = stamp

        m = quat_to_matrix(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

        if self._rebase:
            p = R_ROS_OPT @ p
            m = R_ROS_OPT @ m @ R_ROS_OPT.T
            if self._apply_mount:
                # Camera attitude -> base attitude (see R_MOUNT above).
                m = m @ R_MOUNT
                if self._apply_lever_arm:
                    # p is the CAMERA position; move it to the base origin.
                    # Shifts the odometry origin by a constant (odometry is
                    # relative, so that is harmless) and removes the yaw-
                    # dependent camera-offset wobble.
                    p = p - m @ _CAM_POS_BASE
                self._check_gravity_alignment(m, now)
                # Trust requires BOTH gravity alignment (roll/pitch) AND metric
                # convergence (VIO agreed the parked robot was stopped). The
                # metric gate is only enforced when a wheel reference exists;
                # otherwise (camera-only) it falls back to roll/pitch alone.
                metric_gate = self._require_metric_convergence and self._have_wheel_ref
                trusted = self._healthy and (self._metric_ok or not metric_gate)
                # Latch /vio/ready the first time VIO is fully trusted (gravity-
                # aligned + metric-converged). One-shot start gate for slam_toolbox.
                if trusted and not self._ready_latched:
                    self._ready_latched = True
                    if self._ready_pub is not None:
                        self._ready_pub.publish(Bool(data=True))
                        self.get_logger().info(
                            "VIO gravity-aligned"
                            + (" + metric-converged" if metric_gate else "")
                            + " - latched /vio/ready=true (slam_toolbox may now start)."
                        )
                # Health gate: while VIO is not trusted (VIBA not converged /
                # tracking lost / scale not yet proven) drop the message entirely
                # so the EKF (odom1) sees a dropout and coasts on wheel+gyro,
                # rather than fusing wandering, un-initialized poses.
                if self._suppress_misaligned and not trusted:
                    return

        q = matrix_to_quat(m)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self._out_frame
        odom.child_frame_id = self._child_frame
        odom.pose.pose.position.x = float(p[0])
        odom.pose.pose.position.y = float(p[1])
        odom.pose.pose.position.z = float(p[2])
        odom.pose.pose.orientation.x = float(q[0])
        odom.pose.pose.orientation.y = float(q[1])
        odom.pose.pose.orientation.z = float(q[2])
        odom.pose.pose.orientation.w = float(q[3])
        odom.pose.covariance = self._cov
        # No twist from ORB-SLAM3; leave it zero with large covariance so the EKF
        # ignores it (ekf.yaml fuses only absolute x,y from this source anyway).
        twist_cov = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            twist_cov[i] = 1e6
        odom.twist.covariance = twist_cov
        self._pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
