#!/usr/bin/env python3
"""Open-loop odometry test / drive calibration for the QB3rt AGV.

Streams cmd_vel in an open loop and records every odometry source so they can be
compared against the commanded motion. Several modes share the same recording
and reporting machinery:

  mode=square     drive an N-sided square; report closing error / heading drift /
                  path length per odom source (the headline accuracy test).
  mode=straight   drive one straight leg; report VIO-measured distance + actual
                  speed and the max_speed that makes commanded == actual.
  mode=deadband   ramp the raw motor fraction until the wheels break static
                  friction; recommend motor_deadband (bridge floor auto-disabled).
  mode=speed      sweep motor fractions, measure steady-state speed via VIO, fit a
                  line and recommend max_speed (bridge floor auto-disabled).
  mode=turn       drive an arc of a known angle (roll forward while steering);
                  measure actual rotation via the IMU-backed EKF and recommend
                  the spin_boost correction.
  mode=trim       drive one straight leg; measure heading drift via the
                  IMU-backed EKF and recommend the straight_trim correction (run
                  at two speeds for the straight_trim_slope schedule).
  mode=vio_check  command no motion; print VIO net displacement so a known hand
                  push can verify VIO's scale (is the ruler trustworthy?).
  mode=turn_check command no motion; print cumulative EKF yaw so a known hand
                  rotation can verify the gyro/EKF rotational scale (is the turn
                  ruler trustworthy?).
  mode=oneside    drive one side at full and the other static for a fixed time;
                  measure the resulting rotation, displacement and implied turn
                  radius via the IMU-backed EKF (sanity check; expect scrub).

Odometry sources (configurable):
  /odometry/filtered  robot_localization EKF (VIO + IMU + wheel)   [fused]
  /vio/odometry       RB3 ORB-SLAM3 mono-inertial VIO              [ground truth]
  /odom               wave_rover_bridge dead reckoning             [command echo]
"""

import csv
import json
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

try:
    from rclpy.parameter_client import AsyncParameterClient
except Exception:  # pragma: no cover - older rclpy
    AsyncParameterClient = None

PHASE_STRAIGHT = "straight"
PHASE_TURN = "turn"
PHASE_SETTLE = "settle"


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class OdomSource:
    """Tracks the latest pose and full trajectory of one odometry topic."""

    def __init__(self, topic):
        self.topic = topic
        self.last = None  # (t, x, y, yaw)
        self.first = None  # first sample seen (used as the source's local origin)
        self.trajectory = []  # list of (t, x, y, yaw)
        self.corners = []  # poses captured at each corner / event
        self.vel = []  # list of (t, speed) from the message twist (body frame)

    def update(self, msg):
        now = time.time()
        p = msg.pose.pose
        yaw = yaw_from_quaternion(p.orientation)
        sample = (now, p.position.x, p.position.y, yaw)
        self.last = sample
        if self.first is None:
            self.first = sample
        self.trajectory.append(sample)
        v = msg.twist.twist.linear
        self.vel.append((now, math.hypot(v.x, v.y)))

    def avg_speed(self, t0, t1):
        """Mean reported speed over [t0, t1]; None if no samples in range."""
        vals = [s for (t, s) in self.vel if t0 <= t <= t1]
        return sum(vals) / len(vals) if vals else None

    def displacement(self, t0, t1):
        """Straight-line distance between the poses nearest t0 and t1."""
        pts = [p for p in self.trajectory if t0 <= p[0] <= t1]
        if len(pts) < 2:
            return None, None
        dist = math.hypot(pts[-1][1] - pts[0][1], pts[-1][2] - pts[0][2])
        return dist, pts[-1][0] - pts[0][0]

    def yaw_change(self, t0, t1):
        """Signed cumulative yaw over [t0, t1], unwrapped across +-pi so it
        survives multiple revolutions. None if too few samples."""
        pts = [p for p in self.trajectory if t0 <= p[0] <= t1]
        if len(pts) < 2:
            return None
        total = 0.0
        for a, b in zip(pts, pts[1:]):
            total += wrap_angle(b[3] - a[3])
        return total

    def snapshot(self, label):
        """Record the current pose against a labelled event (e.g. 'start')."""
        self.corners.append((label, self.last))

    def has_data(self):
        return self.last is not None

    def path_length(self):
        total = 0.0
        for a, b in zip(self.trajectory, self.trajectory[1:]):
            total += math.hypot(b[1] - a[1], b[2] - a[2])
        return total


class SquareTest(Node):
    def __init__(self):
        super().__init__("odom_square_test")

        # --- Parameters ---
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.side_length = float(self.declare_parameter("side_length", 1.0).value)
        self.num_sides = int(self.declare_parameter("num_sides", 4).value)
        self.linear_speed = float(self.declare_parameter("linear_speed", 0.15).value)
        self.angular_speed = float(self.declare_parameter("angular_speed", 0.8).value)
        self.turn_angle = float(self.declare_parameter("turn_angle", math.pi / 2.0).value)
        # Forward speed held DURING a turn. A skid-steer can't reliably spin in
        # place (4-wheel scrub), so corners are driven as arcs: the wheels keep
        # rolling forward while steering, radius = turn_linear_speed/angular_speed.
        # Set 0.0 only if your platform can truly pivot in place.
        self.turn_linear_speed = float(self.declare_parameter("turn_linear_speed", 0.20).value)
        self.publish_rate = float(self.declare_parameter("publish_rate", 20.0).value)
        self.settle_time = float(self.declare_parameter("settle_time", 1.5).value)
        self.direction = str(self.declare_parameter("direction", "ccw").value).lower()

        self.mode = str(self.declare_parameter("mode", "square").value).lower()
        # Must match the bridge max_speed: the sweep commands
        # linear_x = fraction * driver_max_speed (so fraction == motor fraction).
        # (These in-code defaults are normally overridden by the yaml/launch
        # mirrors; kept current so the script is self-consistent run bare.)
        self.driver_max_speed = float(self.declare_parameter("driver_max_speed", 0.56).value)
        # Bridge straight_trim in effect during the run (mode=trim scales it).
        self.driver_straight_trim = float(
            self.declare_parameter("driver_straight_trim", 0.12).value
        )
        self.driver_node = str(self.declare_parameter("driver_node", "waverover_bridge").value)

        # Deadband sweep.
        self.db_start = float(self.declare_parameter("deadband_start", 0.0).value)
        self.db_step = float(self.declare_parameter("deadband_step", 0.02).value)
        self.db_max = float(self.declare_parameter("deadband_max", 0.6).value)
        self.step_hold = float(self.declare_parameter("step_hold", 2.5).value)
        self.move_threshold = float(self.declare_parameter("move_threshold", 0.02).value)

        # Speed sweep. Defaults stay in VIO's trackable range and use a long,
        # single-direction baseline (alternate off) for reliable measurement.
        self.sp_start = float(self.declare_parameter("speed_start", 0.25).value)
        self.sp_step = float(self.declare_parameter("speed_step", 0.25).value)
        self.sp_max = float(self.declare_parameter("speed_max", 1.0).value)
        self.speed_hold = float(self.declare_parameter("speed_hold", 3.5).value)
        self.accel_skip = float(self.declare_parameter("accel_skip", 0.8).value)
        self.speed_alternate = bool(self.declare_parameter("speed_alternate", False).value)

        # Arc turn (mode=turn). Sweep a (large) known angle for good SNR.
        self.turn_target = float(self.declare_parameter("turn_target", 2.0 * math.pi).value)
        # MUST match wave_rover_bridge.yaml exponential turn gain:
        #   boost(|v|) = 1 + (spin_boost_max - 1) * exp(-spin_boost_k * |v|)
        self.driver_spin_boost_max = float(
            self.declare_parameter("driver_spin_boost_max", 10.0).value
        )
        self.driver_spin_boost_k = float(
            self.declare_parameter("driver_spin_boost_k", 1.4).value
        )
        # Settle held AFTER the turn drive ends, before reading the final heading.
        # The rover coasts a little once the command stops and the EKF orientation
        # lags slightly; measuring immediately reads the rotation low. This window
        # lets both finish so the measured total matches the real turn.
        self.turn_settle = float(self.declare_parameter("turn_settle", 1.0).value)

        # One-side drive test (mode=oneside): drive one side at full and the other
        # static, hold for a fixed time, and measure the resulting rotation. The
        # static side scrubs - this characterises the sharpest powered turn.
        self.oneside_side = str(self.declare_parameter("oneside_side", "left").value).lower()
        self.oneside_duration = float(self.declare_parameter("oneside_duration", 2.0).value)
        # MUST match wave_rover_bridge.yaml track_width (folded into the twist).
        self.driver_track_width = float(self.declare_parameter("driver_track_width", 0.15).value)
        # Solve the bridge kinematics for one wheel side = full, the other = 0:
        #   left/right = (vx -/+ turn)/max_speed,  turn = wz*boost(vx)*track/2
        #   => vx = max_speed/2,  |wz| = max_speed/(boost(vx)*track_width)
        # left side full -> turns cw (-z); right side full -> turns ccw (+z).
        self.oneside_vx = 0.5 * self.driver_max_speed
        _boost = self._boost_at(self.oneside_vx)
        _denom = _boost * self.driver_track_width
        self.oneside_wz = (self.driver_max_speed / _denom) if _denom > 1e-9 else 0.0
        self.oneside_sign = -1.0 if self.oneside_side == "left" else 1.0

        self.odom_topics = list(
            self.declare_parameter(
                "odom_topics",
                ["/odometry/filtered", "/vio/odometry", "/odom"],
            ).value
        )

        out = str(self.declare_parameter("output_dir", "").value)
        if not out:
            try:
                from ament_index_python.packages import get_package_share_directory
                out = os.path.join(get_package_share_directory("QB3rt"), "results")
            except Exception:
                out = os.path.join(os.getcwd(), "results")
        self.output_dir = out

        # --- Sources ---
        self.sources = {}
        for topic in self.odom_topics:
            src = OdomSource(topic)
            self.create_subscription(
                Odometry, topic, lambda m, s=src: s.update(m), 10
            )
            self.sources[topic] = src

        # --- State ---
        self.started = False
        self.finished = False
        self.shutdown_requested = False
        self.phase = PHASE_STRAIGHT
        self.phase_start = 0.0
        self.side_index = 0
        self._after_settle = PHASE_STRAIGHT

        # Deadband sweep state.
        self._db_levels = []
        self._db_i = 0
        self._db_state = "settle"
        self._db_ref = None
        self._db_start_xy = None
        self._db_results = []
        self._db_breakaway = None
        if self.mode == "deadband":
            f = self.db_start
            while f <= self.db_max + 1e-9:
                self._db_levels.append(round(f, 4))
                f += self.db_step

        # Speed sweep state.
        self._sp_levels = []
        self._sp_i = 0
        self._sp_state = "settle"
        self._sp_ref = None
        self._sp_t_mid = None  # wall time when the steady-state window begins
        self._sp_results = []  # (fraction, actual_speed_mps)

        # vio_check state.
        self._vc_last_log = 0.0

        # turn_check state.
        self._tc_last_log = 0.0
        self._tc_ref = None

        # Turn calibration state.
        self._tn_state = "settle"
        self._tn_ref = None
        self._tn_t0 = None
        self._tn_actual = None

        # Trim calibration state (straight-line drift -> straight_trim).
        self._tr_state = "settle"
        self._tr_ref = None
        self._tr_t0 = None
        self._tr_dtheta = None
        self._tr_dist = None

        # One-side test state.
        self._os_state = "settle"
        self._os_ref = None
        self._os_t0 = None
        self._os_actual = None
        self._os_disp = None
        self.turn_cal_duration = (
            abs(self.turn_target) / self.angular_speed if self.angular_speed else 0.0
        )

        if self.mode == "speed":
            f = self.sp_start
            while f <= self.sp_max + 1e-9:
                self._sp_levels.append(round(f, 4))
                f += self.sp_step

        # ccw (default) turns left (+z); cw turns right (-z).
        self.turn_sign = 1.0 if self.direction != "cw" else -1.0

        # --- Derived timings ---
        self.straight_duration = self.side_length / self.linear_speed if self.linear_speed else 0.0
        self.turn_duration = self.turn_angle / self.angular_speed if self.angular_speed else 0.0
        self.dt = 1.0 / self.publish_rate

        # --- I/O ---
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.get_logger().info(
            f"odom_square_test configured: mode={self.mode} sides={self.num_sides} "
            f"side_length={self.side_length} m v={self.linear_speed} m/s "
            f"w={self.angular_speed} rad/s turn_v={self.turn_linear_speed} m/s "
            f"straight={self.straight_duration:.2f}s "
            f"turn={self.turn_duration:.2f}s dir={self.direction} "
            f"cmd_vel={self.cmd_vel_topic}"
        )
        self.get_logger().info(f"recording odom sources: {self.odom_topics}")

        if self.mode in ("deadband", "speed"):
            self._disable_bridge_floor()

        if self.mode == "oneside":
            self._neutralize_bridge_for_oneside()

        if self.mode == "deadband":
            self.get_logger().info(
                f"DEADBAND SWEEP: fractions {self.db_start:.2f}..{self.db_max:.2f} "
                f"step {self.db_step:.2f}, {self.step_hold:.1f}s/step, "
                f"move_threshold {self.move_threshold:.3f} m."
            )
        if self.mode == "speed":
            est_leg = self.speed_hold * self.driver_max_speed * self.sp_max
            self.get_logger().info(
                f"SPEED SWEEP: fractions {self.sp_start:.2f}..{self.sp_max:.2f} "
                f"step {self.sp_step:.2f}, {self.speed_hold:.1f}s/step "
                f"(skip {self.accel_skip:.1f}s accel), alternate={self.speed_alternate}. "
                f"Needs ~{est_leg:.1f} m of clear space"
                f"{' (alternating)' if self.speed_alternate else ' (one direction)'}."
            )
        if self.mode == "turn":
            radius = (self.turn_linear_speed / self.angular_speed
                      if self.angular_speed else 0.0)
            self.get_logger().info(
                f"ARC TURN: rotate {self.turn_target:.3f} rad "
                f"({math.degrees(self.turn_target):.0f} deg) at "
                f"{self.angular_speed:.2f} rad/s while rolling "
                f"{self.turn_linear_speed:.2f} m/s {self.direction} "
                f"({self.turn_cal_duration:.1f}s). Drives a circle of radius "
                f"~{radius:.2f} m (clear ~{2 * radius + 0.5:.1f} m); "
                f"measured via IMU-backed EKF."
            )
        if self.mode == "oneside":
            self.get_logger().info(
                f"ONE-SIDE TEST: drive {self.oneside_side} side FULL, other STATIC "
                f"for {self.oneside_duration:.1f}s "
                f"(cmd vx={self.oneside_vx:.2f} m/s, "
                f"wz={self.oneside_sign * self.oneside_wz:.2f} rad/s); rotation via "
                f"IMU-backed EKF. Expect scrub on the static side; clear ~1 m around."
            )

        # Give subscriptions a moment to connect and receive a first sample.
        self._warmup_until = time.time() + 2.0
        # Sweeps/tests measure against the camera/EKF, which can take several
        # seconds to come up (OAK-D connect + VIO init). Don't start until live.
        self._detect_deadline = time.time() + 25.0
        self._detect_warned = False
        self.timer = self.create_timer(self.dt, self._tick)

    # ------------------------------------------------------------------ control
    def _boost_at(self, v):
        """Mirror of wave_rover_bridge._boost_at (exponential turn gain)."""
        peak = max(1.0, float(self.driver_spin_boost_max))
        k = float(self.driver_spin_boost_k)
        if k <= 0.0:
            return peak
        return 1.0 + (peak - 1.0) * math.exp(-k * abs(v))

    def _publish_cmd(self, vx, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def _snapshot_all(self, label):
        for src in self.sources.values():
            src.snapshot(label)

    def _tick(self):
        now = time.time()

        # Warm-up: keep publishing zero so the watchdog stays happy, wait for data.
        if not self.started:
            self._publish_cmd(0.0, 0.0)
            if now < self._warmup_until:
                return
            # Don't move until the ground-truth sensor is live. Deadband can use
            # VIO or EKF; every other mode that records VIO needs VIO specifically.
            if self.mode == "deadband":
                gate_missing = self._pick_detection_source() is None
            elif self.mode in ("turn", "turn_check", "oneside", "trim"):
                gate_missing = self._pick_turn_source() is None
            elif "/vio/odometry" in self.odom_topics:
                vio = self.sources.get("/vio/odometry")
                gate_missing = not (vio and vio.has_data())
            else:
                gate_missing = False
            if gate_missing:
                if now < self._detect_deadline:
                    if not self._detect_warned:
                        self.get_logger().warn(
                            "waiting for camera/EKF odometry before starting "
                            "(ORB-SLAM3 VIO still coming up)..."
                        )
                        self._detect_warned = True
                    return
                self.get_logger().error(
                    "no camera/EKF odometry after warmup; results will be "
                    "unreliable. Check that bringup:=true and the tracking camera works."
                )
            self.started = True
            self.phase_start = now
            ready = [t for t, s in self.sources.items() if s.has_data()]
            missing = [t for t, s in self.sources.items() if not s.has_data()]
            if ready:
                self.get_logger().info(f"receiving odom from: {ready}")
            if missing:
                self.get_logger().warn(
                    f"no odom yet from: {missing} (will still record if it appears)"
                )
            self._snapshot_all("start")
            self.get_logger().info(f"=== {self.mode} started ===")
            return

        if self.finished:
            return

        if self.mode == "deadband":
            self._deadband_tick(now)
            return

        if self.mode == "speed":
            self._speed_tick(now)
            return

        if self.mode == "vio_check":
            self._vio_check_tick(now)
            return

        if self.mode == "turn_check":
            self._turn_check_tick(now)
            return

        if self.mode == "oneside":
            self._oneside_tick(now)
            return

        if self.mode == "turn":
            self._turn_tick(now)
            return

        if self.mode == "trim":
            self._trim_tick(now)
            return

        elapsed = now - self.phase_start

        if self.phase == PHASE_STRAIGHT:
            if elapsed < self.straight_duration:
                self._publish_cmd(self.linear_speed, 0.0)
            else:
                self._publish_cmd(0.0, 0.0)
                self._snapshot_all(f"side{self.side_index + 1}_end")
                self.get_logger().info(
                    f"side {self.side_index + 1}/{self.num_sides} straight done"
                )
                if self.mode == "straight":
                    self._finish()
                    return
                self._enter(PHASE_SETTLE, after=PHASE_TURN, now=now)
        elif self.phase == PHASE_TURN:
            if elapsed < self.turn_duration:
                # Arc turn: roll forward while steering (skid-steer can't pivot).
                self._publish_cmd(self.turn_linear_speed, self.turn_sign * self.angular_speed)
            else:
                self._publish_cmd(0.0, 0.0)
                self._snapshot_all(f"corner{self.side_index + 1}")
                self.side_index += 1
                if self.side_index >= self.num_sides:
                    self._finish()
                    return
                self._enter(PHASE_SETTLE, after=PHASE_STRAIGHT, now=now)
        elif self.phase == PHASE_SETTLE:
            self._publish_cmd(0.0, 0.0)
            if elapsed >= self.settle_time:
                self.phase = self._after_settle
                self.phase_start = now

    def _enter(self, phase, after, now):
        self.phase = phase
        self.phase_start = now
        if phase == PHASE_SETTLE:
            self._after_settle = after

    # ------------------------------------------------------------- calibration
    def _set_driver_params(self, params, what):
        """Best-effort live param set on the bridge. params: list of (name, value)."""
        if AsyncParameterClient is None:
            self.get_logger().warn(
                f"AsyncParameterClient unavailable; set {self.driver_node} {what} manually"
            )
            return
        try:
            client = AsyncParameterClient(self, self.driver_node)
            if not client.wait_for_services(timeout_sec=5.0):
                self.get_logger().warn(
                    f"could not reach {self.driver_node} param services; set {what} manually"
                )
                return
            client.set_parameters(
                [Parameter(n, Parameter.Type.DOUBLE, float(v)) for n, v in params]
            )
            self.get_logger().info(f"set {self.driver_node} {what} for the test")
        except Exception as exc:
            self.get_logger().warn(
                f"could not set {self.driver_node} {what} ({exc}); do it manually"
            )

    def _disable_bridge_floor(self):
        self._set_driver_params([("motor_deadband", 0.0)], "motor_deadband=0.0")

    def _neutralize_bridge_for_oneside(self):
        # One side must be exactly static and the other exactly full, so remove the
        # friction floor (which would otherwise lift the 'static' side off zero) and
        # the straight trim (which skews L/R). spin_boost is left as-is and folded
        # into the commanded twist instead. Relaunch the bridge to restore.
        self._set_driver_params(
            [("motor_deadband", 0.0), ("straight_trim", 0.0)],
            "motor_deadband=0.0 straight_trim=0.0",
        )

    def _pick_detection_source(self):
        # Motion must be measured by something that observes the real world.
        # /odom is dead-reckoned from the commands, so it would "move" even if the
        # wheels never turn - never use it for breakaway detection.
        for t in ("/odometry/filtered", "/vio/odometry"):
            s = self.sources.get(t)
            if s and s.has_data():
                return t
        for t, s in self.sources.items():
            if t != "/odom" and s.has_data():
                return t
        return None

    def _pick_speed_source(self):
        # For SPEED, prefer raw VIO over the EKF: the EKF fuses the wheel /odom
        # velocity, so its twist partly echoes the *commanded* speed and would
        # report a falsely perfect result. VIO is independent ground truth (its
        # only weakness is dropping out at high speed - real information we want).
        for t in ("/vio/odometry", "/odometry/filtered"):
            s = self.sources.get(t)
            if s and s.has_data():
                return t
        return self._pick_detection_source()

    def _pick_turn_source(self):
        # For TURN, prefer the EKF: it fuses the IMU gyro, which integrates yaw
        # far more accurately than VIO (VIO is weaker on rotation, where features
        # blur). Fall back to VIO, never /odom.
        for t in ("/odometry/filtered", "/vio/odometry"):
            s = self.sources.get(t)
            if s and s.has_data():
                return t
        return self._pick_detection_source()

    def _xy_of(self, topic):
        s = self.sources.get(topic) if topic else None
        if s and s.last:
            return (s.last[1], s.last[2])
        return None

    def _detection_xy(self):
        return self._xy_of(self._db_ref)

    # ------------------------------------------------------------ deadband mode
    def _deadband_tick(self, now):
        elapsed = now - self.phase_start

        if self._db_state == "settle":
            self._publish_cmd(0.0, 0.0)
            if elapsed >= self.settle_time:
                if self._db_ref is None:
                    self._db_ref = self._pick_detection_source()
                    if self._db_ref is None:
                        self.get_logger().warn(
                            "no camera/EKF odometry to detect motion; /odom excluded."
                        )
                self._db_start_xy = self._detection_xy()
                self._db_state = "drive"
                self.phase_start = now
            return

        f = self._db_levels[self._db_i]
        self._publish_cmd(f * self.driver_max_speed, 0.0)
        if elapsed < self.step_hold:
            return

        self._publish_cmd(0.0, 0.0)
        end_xy = self._detection_xy()
        moved = 0.0
        if end_xy and self._db_start_xy:
            moved = math.hypot(
                end_xy[0] - self._db_start_xy[0], end_xy[1] - self._db_start_xy[1]
            )
        self._db_results.append((f, moved))
        self.get_logger().info(f"deadband sweep: fraction={f:.3f} -> moved {moved:.3f} m")

        if moved >= self.move_threshold:
            self._db_breakaway = f
            self._finish()
            return

        self._db_i += 1
        if self._db_i >= len(self._db_levels):
            self._finish()
            return
        self._db_state = "settle"
        self.phase_start = now

    # --------------------------------------------------------------- speed mode
    def _speed_tick(self, now):
        elapsed = now - self.phase_start

        if self._sp_state == "settle":
            self._publish_cmd(0.0, 0.0)
            if elapsed >= self.settle_time:
                if self._sp_ref is None:
                    self._sp_ref = self._pick_speed_source()
                    if self._sp_ref is None:
                        self.get_logger().warn(
                            "no camera/EKF odometry to measure speed; /odom excluded."
                        )
                self._sp_t_mid = None
                self._sp_state = "drive"
                self.phase_start = now
            return

        # drive (alternate direction each step to stay within a short runway)
        f = self._sp_levels[self._sp_i]
        sign = -1.0 if (self.speed_alternate and self._sp_i % 2 == 1) else 1.0
        self._publish_cmd(sign * f * self.driver_max_speed, 0.0)

        # Steady-state window begins once the accel transient has passed.
        if self._sp_t_mid is None and elapsed >= self.accel_skip:
            self._sp_t_mid = now

        if elapsed < self.speed_hold:
            return

        self._publish_cmd(0.0, 0.0)
        t0 = self._sp_t_mid if self._sp_t_mid is not None else self.phase_start
        v = None
        src = self.sources.get(self._sp_ref)
        if src is not None:
            # Primary: net displacement / time over the steady window. A long,
            # single-direction baseline swamps VIO position noise far better than
            # the (noisy, often empty) VIO twist estimate.
            dist, dt = src.displacement(t0, now)
            if dist is not None and dt and dt > 0:
                v = dist / dt
            else:
                v = src.avg_speed(t0, now)
        v = v or 0.0
        self._sp_results.append((f, v))
        self.get_logger().info(f"speed sweep: fraction={f:.2f} -> {v:.3f} m/s")

        self._sp_i += 1
        if self._sp_i >= len(self._sp_levels):
            self._finish()
            return
        self._sp_state = "settle"
        self.phase_start = now

    # ----------------------------------------------------------- vio_check mode
    def _vio_check_tick(self, now):
        # Command no motion - the operator pushes the rover by hand. Periodically
        # report VIO net displacement + path length so a known push (e.g. exactly
        # 1.0 m) can be compared against what VIO reports. This isolates whether
        # VIO's scale is trustworthy before believing any speed calibration.
        self._publish_cmd(0.0, 0.0)
        if now - self._vc_last_log < 0.5:
            return
        self._vc_last_log = now
        s = self.sources.get("/vio/odometry")
        if not (s and s.first and s.last):
            self.get_logger().warn("vio_check: no VIO data yet")
            return
        net = math.hypot(s.last[1] - s.first[1], s.last[2] - s.first[2])
        self.get_logger().info(
            f"vio_check: net displacement = {net:.3f} m | path = {s.path_length():.3f} m "
            f"| samples = {len(s.trajectory)}  (push the rover a known distance and compare)"
        )

    # ---------------------------------------------------------- turn_check mode
    def _turn_check_tick(self, now):
        # Command no motion - the operator rotates the rover by hand. Periodically
        # report cumulative EKF (gyro) yaw so a known rotation (e.g. exactly 360
        # deg against a floor mark) can be compared against what the EKF reports.
        # This isolates the rotational scale of the ruler from the drive, before
        # believing any spin_boost number. Prefers the IMU-backed EKF, then VIO.
        self._publish_cmd(0.0, 0.0)
        if self._tc_ref is None:
            self._tc_ref = self._pick_turn_source()
        if now - self._tc_last_log < 0.5:
            return
        self._tc_last_log = now
        s = self.sources.get(self._tc_ref) if self._tc_ref else None
        if not (s and s.first and s.last):
            self.get_logger().warn("turn_check: no EKF/VIO yaw data yet")
            return
        # Cumulative (unwrapped) yaw since the first sample - survives >360 deg.
        cum = s.yaw_change(s.first[0], now)
        self.get_logger().info(
            f"turn_check [{self._tc_ref}]: cumulative yaw = {math.degrees(cum):.1f} deg "
            f"({cum:.3f} rad) | samples = {len(s.trajectory)}  "
            f"(rotate the rover a known angle by hand and compare)"
        )

    # ---------------------------------------------------------------- turn mode
    def _turn_tick(self, now):
        elapsed = now - self.phase_start

        if self._tn_state == "settle":
            self._publish_cmd(0.0, 0.0)
            if elapsed >= self.settle_time:
                if self._tn_ref is None:
                    self._tn_ref = self._pick_turn_source()
                    if self._tn_ref is None:
                        self.get_logger().warn(
                            "no EKF/VIO odometry to measure rotation; /odom excluded."
                        )
                self._tn_t0 = now
                self._tn_state = "drive"
                self.phase_start = now
            return

        if self._tn_state == "drive":
            if elapsed < self.turn_cal_duration:
                # Arc turn: roll forward while steering (skid-steer can't pivot).
                self._publish_cmd(self.turn_linear_speed, self.turn_sign * self.angular_speed)
                return
            # Stop commanding, then let the rover coast to rest and the EKF
            # orientation converge before reading the heading (otherwise the
            # post-command coast + filter lag are missed and the turn reads low).
            self._publish_cmd(0.0, 0.0)
            self._tn_state = "post"
            self.phase_start = now
            return

        # post: settle at rest, then measure total rotation start -> rest.
        self._publish_cmd(0.0, 0.0)
        if elapsed < self.turn_settle:
            return
        src = self.sources.get(self._tn_ref)
        self._tn_actual = src.yaw_change(self._tn_t0, now) if src is not None else None
        self._finish()

    # ---------------------------------------------------------------- trim mode
    def _trim_tick(self, now):
        # Drive one straight leg, then measure the heading drift with the
        # IMU-backed EKF (gyro) - the true rotation reference - and recommend a
        # straight_trim that would zero it. Same settle -> drive -> post pattern
        # as the turn calibration.
        elapsed = now - self.phase_start

        if self._tr_state == "settle":
            self._publish_cmd(0.0, 0.0)
            if elapsed >= self.settle_time:
                if self._tr_ref is None:
                    self._tr_ref = self._pick_turn_source()
                    if self._tr_ref is None:
                        self.get_logger().warn(
                            "no EKF/VIO odometry to measure drift; /odom excluded."
                        )
                self._tr_t0 = now
                self._tr_state = "drive"
                self.phase_start = now
            return

        if self._tr_state == "drive":
            if elapsed < self.straight_duration:
                self._publish_cmd(self.linear_speed, 0.0)
                return
            # Stop, then let the rover coast to rest and the EKF orientation
            # converge before reading the final heading (as in the turn mode).
            self._publish_cmd(0.0, 0.0)
            self._tr_state = "post"
            self.phase_start = now
            return

        # post: settle at rest, then measure heading drift + distance start->rest.
        self._publish_cmd(0.0, 0.0)
        if elapsed < self.turn_settle:
            return
        src = self.sources.get(self._tr_ref)
        if src is not None:
            self._tr_dtheta = src.yaw_change(self._tr_t0, now)
            self._tr_dist, _ = src.displacement(self._tr_t0, now)
        self._finish()

    # ----------------------------------------------------------- one-side mode
    def _oneside_tick(self, now):
        elapsed = now - self.phase_start

        if self._os_state == "settle":
            self._publish_cmd(0.0, 0.0)
            if elapsed >= self.settle_time:
                if self._os_ref is None:
                    self._os_ref = self._pick_turn_source()
                    if self._os_ref is None:
                        self.get_logger().warn(
                            "no EKF/VIO odometry to measure rotation; /odom excluded."
                        )
                self._os_t0 = now
                self._os_state = "drive"
                self.phase_start = now
            return

        if self._os_state == "drive":
            if elapsed < self.oneside_duration:
                # One side full, other static (folded into a single twist).
                self._publish_cmd(self.oneside_vx, self.oneside_sign * self.oneside_wz)
                return
            self._publish_cmd(0.0, 0.0)
            self._os_state = "post"
            self.phase_start = now
            return

        # post: coast/settle at rest, then measure total rotation + displacement.
        self._publish_cmd(0.0, 0.0)
        if elapsed < self.turn_settle:
            return
        src = self.sources.get(self._os_ref)
        if src is not None:
            self._os_actual = src.yaw_change(self._os_t0, now)
            self._os_disp, _ = src.displacement(self._os_t0, now)
        else:
            self._os_actual = None
            self._os_disp = None
        self._finish()

    # ----------------------------------------------------------------- finish
    def _finish(self):
        if self.finished:
            return
        self.finished = True
        self._publish_cmd(0.0, 0.0)
        self.get_logger().info(f"=== {self.mode} complete, stopping rover ===")
        if self.mode == "deadband":
            self._report_deadband()
        elif self.mode == "speed":
            self._report_speed()
        elif self.mode == "turn":
            self._report_turn()
        elif self.mode == "trim":
            self._report_trim()
        elif self.mode == "oneside":
            self._report_oneside()
        else:
            self._report_and_save()
        self.shutdown_requested = True

    # ------------------------------------------------------------- reporting
    def _report_deadband(self):
        lines = ["", "=" * 72, "MOTOR DEADBAND CALIBRATION (static-friction floor)"]
        lines.append(f"  step {self.db_step:.3f}, hold {self.step_hold:.1f}s, "
                     f"detect via {self._db_ref or 'NONE'}, "
                     f"move_threshold {self.move_threshold:.3f} m")
        lines.append("=" * 72)
        lines.append(f"{'motor_fraction':>15}{'moved_m':>12}")
        lines.append("-" * 72)
        for f, moved in self._db_results:
            lines.append(f"{f:>15.3f}{moved:>12.3f}")
        lines.append("-" * 72)
        if self._db_ref is None:
            lines.append("RESULT: no camera/EKF odometry available to detect motion.")
        elif self._db_breakaway is None:
            lines.append("RESULT: no breakaway within the swept range; raise deadband_max.")
        else:
            db = self._db_breakaway
            lines.append(f"RESULT: wheels break free at motor fraction ~ {db:.3f}")
            lines.append(f"        recommended motor_deadband: {db:.3f}")
            lines.append(f"        ros2 param set /{self.driver_node} motor_deadband {db:.3f}")
            lines.append("        (then set motor_deadband in wave_rover_bridge.yaml).")
        lines.append("=" * 72)
        self.get_logger().info("\n".join(lines))
        self._save_json("deadband_sweep", {
            "mode": "deadband",
            "detection_source": self._db_ref,
            "parameters": {
                "deadband_start": self.db_start,
                "deadband_step": self.db_step,
                "deadband_max": self.db_max,
                "step_hold": self.step_hold,
                "move_threshold": self.move_threshold,
                "driver_max_speed": self.driver_max_speed,
            },
            "samples": [{"fraction": f, "moved_m": m} for f, m in self._db_results],
            "breakaway_fraction": self._db_breakaway,
        })

    def _report_speed(self):
        # Speed-vs-fraction is AFFINE, not linear-through-origin: with the floor
        # off the wheels don't move until the command clears static friction, so
        #     v ~= slope * fraction + intercept = V_full * (fraction - f0)
        # The speed at full output is the line extrapolated to fraction = 1.0.
        pts = [(f, v) for f, v in self._sp_results if v > 1e-6 and f > 1e-9]
        slope = intercept = vfull = f0 = None
        if len(pts) >= 2:
            n = len(pts)
            mf = sum(f for f, _ in pts) / n
            mv = sum(v for _, v in pts) / n
            sxx = sum((f - mf) ** 2 for f, _ in pts)
            sxy = sum((f - mf) * (v - mv) for f, v in pts)
            if sxx > 1e-12:
                slope = sxy / sxx
                intercept = mv - slope * mf
                vfull = max(slope + intercept, 0.0)  # extrapolate to fraction=1.0
                if slope > 1e-9:
                    f0 = -intercept / slope  # breakaway fraction
        elif len(pts) == 1:
            vfull = pts[0][1] / pts[0][0]

        implied = [v / f for f, v in pts]
        rising = len(implied) >= 2 and implied[-1] > 1.3 * implied[0]
        falling = len(implied) >= 2 and implied[-1] < 0.77 * max(implied)

        lines = ["", "=" * 72, "STRAIGHT-LINE SPEED CALIBRATION (max_speed)"]
        lines.append(
            f"  fractions {self.sp_start:.2f}..{self.sp_max:.2f} step {self.sp_step:.2f}, "
            f"hold {self.speed_hold:.1f}s (skip {self.accel_skip:.1f}s), "
            f"measured via {self._sp_ref or 'NONE'}"
        )
        lines.append("=" * 72)
        lines.append(f"{'motor_fraction':>15}{'actual_mps':>13}{'implied_Vfull':>16}")
        lines.append("-" * 72)
        for f, v in self._sp_results:
            imp = (v / f) if f > 1e-9 else float("nan")
            lines.append(f"{f:>15.2f}{v:>13.3f}{imp:>16.3f}")
        lines.append("-" * 72)
        if self._sp_ref is None:
            lines.append("RESULT: no VIO/camera odometry available to measure speed.")
            lines.append("        (/odom is excluded - it only echoes the command.)")
        elif vfull is None:
            lines.append("RESULT: no motion measured; check the rover/floor and runway.")
        else:
            if slope is not None:
                lines.append(f"RESULT: linear fit v = {slope:.3f}*fraction + ({intercept:+.3f})")
                if f0 is not None:
                    lines.append(f"        breakaway fraction f0 ~ {f0:.2f}")
            lines.append(f"        V_full (speed at full output) ~ {vfull:.3f} m/s")
            lines.append(f"        recommended max_speed: {vfull:.3f}")
            lines.append(f"        ros2 param set /{self.driver_node} max_speed {vfull:.3f}")
            lines.append("        (then set max_speed in wave_rover_bridge.yaml).")
            if rising:
                lines.append("")
                lines.append("NOTE: implied_Vfull is still RISING across the sweep - you are")
                lines.append("      measuring near the static-friction breakaway, not the linear")
                lines.append("      region. V_full above is the line extrapolated to full output;")
                lines.append("      raise speed_max to capture the linear part and tighten it.")
            elif falling:
                lines.append("")
                lines.append("WARNING: implied_Vfull COLLAPSES on the top rows - VIO is losing")
                lines.append("         tracking at speed. Lower speed_max so every step stays")
                lines.append("         VIO-trackable, and trust the lower, stable rows.")
        lines.append("=" * 72)
        self.get_logger().info("\n".join(lines))
        self._save_json("speed_sweep", {
            "mode": "speed",
            "detection_source": self._sp_ref,
            "parameters": {
                "speed_start": self.sp_start,
                "speed_step": self.sp_step,
                "speed_max": self.sp_max,
                "speed_hold": self.speed_hold,
                "accel_skip": self.accel_skip,
                "speed_alternate": self.speed_alternate,
                "driver_max_speed": self.driver_max_speed,
            },
            "samples": [{"fraction": f, "actual_mps": v} for f, v in self._sp_results],
            "fit": {"slope": slope, "intercept": intercept, "v_full": vfull,
                    "breakaway_fraction": f0},
        })

    def _report_turn(self):
        commanded = self.turn_sign * abs(self.turn_target)  # signed target
        actual = self._tn_actual
        lines = ["", "=" * 72, "ARC TURN CALIBRATION (spin_boost)"]
        lines.append(
            f"  commanded: rotate {abs(self.turn_target):.3f} rad "
            f"({math.degrees(abs(self.turn_target)):.1f} deg) at "
            f"{self.angular_speed:.2f} rad/s while rolling "
            f"{self.turn_linear_speed:.2f} m/s {self.direction}, "
            f"measured via {self._tn_ref or 'NONE'}"
        )
        lines.append("=" * 72)

        suggested = None
        ratio = None
        if self._tn_ref is None or actual is None:
            lines.append("RESULT: no EKF/VIO rotation measured.")
            lines.append("        (/odom is excluded - it only echoes the command.)")
        else:
            lines.append(f"  actual: rotated {abs(actual):.3f} rad "
                         f"({math.degrees(abs(actual)):.1f} deg)")
            # Same-sign check: a negative ratio means it spun the wrong way.
            signed_ratio = (actual / commanded) if abs(commanded) > 1e-9 else float("nan")
            ratio = abs(actual) / abs(commanded) if abs(commanded) > 1e-9 else float("nan")
            lines.append(f"  ratio actual/commanded = {signed_ratio:.3f}")
            if signed_ratio < 0:
                lines.append("  WARNING: rotated the WRONG way - check 'direction' / wiring.")
            if ratio > 1e-3:
                # Effective boost at this arc's forward speed; scale it so
                # actual == commanded, then solve the exponential for a new k
                # (keep spin_boost_max fixed) — or for a new max if k would go
                # non-positive / the needed peak exceeds max.
                v = abs(self.turn_linear_speed)
                boost_now = self._boost_at(v)
                needed = boost_now / ratio
                lines.append("")
                lines.append(
                    f"  effective boost at v={v:.2f} m/s: {boost_now:.3f} "
                    f"(= 1+({self.driver_spin_boost_max:.1f}-1)"
                    f"*exp(-{self.driver_spin_boost_k:.2f}*{v:.2f}))"
                )
                lines.append(f"  needed boost at this speed: {needed:.3f}")
                if ratio < 1.0:
                    lines.append(f"  UNDER-rotating ({ratio:.2f}x): need more boost "
                                 f"(lower k and/or raise spin_boost_max).")
                else:
                    lines.append(f"  OVER-rotating ({ratio:.2f}x): need less boost "
                                 f"(raise k and/or lower spin_boost_max).")
                suggested_k = None
                suggested_max = None
                if needed > 1.0 + 1e-6 and v > 1e-6:
                    # Solve needed = 1 + (max-1)*exp(-k*v) for k (max fixed).
                    peak = max(1.0, self.driver_spin_boost_max)
                    frac = (needed - 1.0) / (peak - 1.0) if peak > 1.0 else 0.0
                    if 0.0 < frac < 1.0:
                        suggested_k = -math.log(frac) / v
                        suggested = suggested_k
                        lines.append(
                            f"  recommended spin_boost_k: {suggested_k:.3f} "
                            f"(keep spin_boost_max={peak:.1f})"
                        )
                        lines.append(
                            f"    ros2 param set /{self.driver_node} "
                            f"spin_boost_k {suggested_k:.3f}"
                        )
                    else:
                        # Needed peak is above/below what max can deliver at
                        # this v with any k — recommend a new max at current k.
                        k = max(0.0, self.driver_spin_boost_k)
                        denom = math.exp(-k * v) if k > 0.0 else 1.0
                        suggested_max = 1.0 + (needed - 1.0) / denom
                        suggested = suggested_max
                        lines.append(
                            f"  recommended spin_boost_max: {suggested_max:.3f} "
                            f"(keep spin_boost_k={k:.2f}; needed peak out of "
                            f"range for k-only solve)"
                        )
                        lines.append(
                            f"    ros2 param set /{self.driver_node} "
                            f"spin_boost_max {suggested_max:.3f}"
                        )
                lines.append(
                    "  Persist in wave_rover_bridge.yaml and re-run to verify "
                    "ratio ~ 1.0. Prefer two-speed fits for k."
                )
        lines.append("=" * 72)
        self.get_logger().info("\n".join(lines))
        self._save_json("turn_cal", {
            "mode": "turn",
            "measurement_source": self._tn_ref,
            "parameters": {
                "turn_target_rad": self.turn_target,
                "angular_speed": self.angular_speed,
                "turn_linear_speed": self.turn_linear_speed,
                "direction": self.direction,
                "driver_spin_boost_max": self.driver_spin_boost_max,
                "driver_spin_boost_k": self.driver_spin_boost_k,
                "turn_settle": self.turn_settle,
            },
            "commanded_rad": commanded,
            "actual_rad": actual,
            "ratio": ratio,
            "suggested_spin_boost": suggested,
        })

    def _report_trim(self):
        # Straight-line drift model (constant curvature over the leg):
        #   dtheta = 2 * (T_applied + B_phys) * D / track_width
        # where T_applied is the trim that was in effect (driver_straight_trim)
        # and B_phys is the drivetrain's own imbalance (same trim units). To make
        # the drift zero we want (T_new + B_phys) = 0, i.e.
        #   T_new = T_applied - dtheta * track_width / (2 * D).
        # straight_trim '+ curves left' (+dtheta CCW), matching the bridge sign.
        dtheta = self._tr_dtheta
        dist = self._tr_dist
        track = self.driver_track_width
        lines = ["", "=" * 72, "STRAIGHT-LINE TRIM CALIBRATION (straight_trim)"]
        lines.append(
            f"  commanded: drive {self.side_length:.2f} m straight at "
            f"{self.linear_speed:.2f} m/s (trim {self.driver_straight_trim:+.3f} "
            f"in effect), measured via {self._tr_ref or 'NONE'}"
        )
        lines.append("=" * 72)

        suggested = None
        t_res = None
        if self._tr_ref is None or dtheta is None or dist is None:
            lines.append("RESULT: no EKF/VIO drift measured.")
            lines.append("        (/odom is excluded - it only echoes the command.)")
        elif dist < 0.05:
            lines.append(f"RESULT: measured distance {dist:.3f} m too small; "
                         "check the rover/floor/runway.")
        else:
            drift_dir = "left/CCW" if dtheta >= 0 else "right/CW"
            lines.append(f"  drift: {math.degrees(dtheta):+.1f} deg ({dtheta:+.3f} rad) "
                         f"{drift_dir} over {dist:.2f} m")
            t_res = dtheta * track / (2.0 * dist)
            suggested = self.driver_straight_trim - t_res
            lines.append(f"  residual imbalance ~ {t_res:+.3f} trim units")
            lines.append("")
            lines.append(f"  recommended straight_trim: {suggested:+.3f} "
                         f"(= {self.driver_straight_trim:+.3f} - {t_res:+.3f})")
            lines.append(f"    ros2 param set /{self.driver_node} straight_trim {suggested:.3f}")
            lines.append(f"  (and set straight_trim: {suggested:.3f} in "
                         "wave_rover_bridge.yaml). Re-run to verify drift ~ 0.")
            lines.append("")
            lines.append("  For the SPEED SCHEDULE, run this at two linear_speed values")
            lines.append("  v1 < v2 and set in wave_rover_bridge.yaml:")
            lines.append("    straight_trim_ref_speed = v1")
            lines.append("    straight_trim           = trim_at_v1")
            lines.append("    straight_trim_slope     = (trim_at_v2 - trim_at_v1)/(v2 - v1)")
        lines.append("=" * 72)
        self.get_logger().info("\n".join(lines))
        self._save_json("trim_cal", {
            "mode": "trim",
            "measurement_source": self._tr_ref,
            "parameters": {
                "side_length_m": self.side_length,
                "linear_speed_mps": self.linear_speed,
                "driver_straight_trim": self.driver_straight_trim,
                "driver_track_width": track,
                "turn_settle": self.turn_settle,
            },
            "drift_rad": dtheta,
            "distance_m": dist,
            "residual_trim": t_res,
            "suggested_straight_trim": suggested,
        })

    def _report_oneside(self):
        actual = self._os_actual
        disp = self._os_disp
        lines = ["", "=" * 72, "ONE-SIDE DRIVE TEST (one side full, other static)"]
        lines.append(
            f"  commanded: {self.oneside_side} side FULL, other STATIC for "
            f"{self.oneside_duration:.1f}s "
            f"(vx={self.oneside_vx:.3f} m/s, "
            f"wz={self.oneside_sign * self.oneside_wz:.3f} rad/s), "
            f"measured via {self._os_ref or 'NONE'}"
        )
        lines.append("=" * 72)

        rate = None
        radius = None
        if self._os_ref is None or actual is None:
            lines.append("RESULT: no EKF/VIO rotation measured.")
            lines.append("        (/odom is excluded - it only echoes the command.)")
        else:
            deg = math.degrees(actual)
            rate = deg / self.oneside_duration if self.oneside_duration > 0 else float("nan")
            spin = "ccw" if actual >= 0 else "cw"
            lines.append(
                f"  rotated {abs(deg):.1f} deg ({abs(actual):.3f} rad) {spin} "
                f"in {self.oneside_duration:.1f}s"
            )
            lines.append(f"  mean yaw rate ~ {abs(rate):.1f} deg/s "
                         f"({abs(math.radians(rate)):.3f} rad/s)")
            if disp is not None:
                lines.append(f"  net base displacement ~ {disp:.3f} m")
                half = abs(actual) / 2.0
                if half > 1e-3:
                    # Constant-curvature arc: chord = 2 R sin(theta/2).
                    radius = disp / (2.0 * math.sin(half))
                    lines.append(
                        f"  implied turn radius ~ {radius:.3f} m  "
                        f"(0 = pure pivot; ~track_width/2 = "
                        f"{self.driver_track_width / 2:.3f} m if pivoting about the "
                        f"static side, larger = scrubbing outward)"
                    )
            lines.append("")
            lines.append("  NOTE: one side static = maximum-asymmetry powered turn; the static")
            lines.append("        side scrubs. Use this to judge how sharply the rover turns")
            lines.append("        under power and how much it slips.")
        lines.append("=" * 72)
        self.get_logger().info("\n".join(lines))
        self._save_json("oneside_test", {
            "mode": "oneside",
            "measurement_source": self._os_ref,
            "parameters": {
                "oneside_side": self.oneside_side,
                "oneside_duration": self.oneside_duration,
                "cmd_vx": self.oneside_vx,
                "cmd_wz": self.oneside_sign * self.oneside_wz,
                "driver_max_speed": self.driver_max_speed,
                "driver_spin_boost_max": self.driver_spin_boost_max,
                "driver_spin_boost_k": self.driver_spin_boost_k,
                "driver_track_width": self.driver_track_width,
                "turn_settle": self.turn_settle,
            },
            "actual_rad": actual,
            "mean_yaw_rate_degps": rate,
            "net_displacement_m": disp,
            "implied_radius_m": radius,
        })

    def _report_and_save(self):
        summary = {
            "timestamp": datetime.now().isoformat(),
            "mode": self.mode,
            "commanded": {
                "side_length_m": self.side_length,
                "num_sides": self.num_sides,
                "linear_speed_mps": self.linear_speed,
                "angular_speed_radps": self.angular_speed,
                "straight_duration_s": self.straight_duration,
                "direction": self.direction,
                "driver_max_speed": self.driver_max_speed,
            },
            "sources": {},
        }

        ideal = (self.num_sides * self.side_length) if self.mode == "square" else self.side_length

        lines = ["", "=" * 72]
        if self.mode == "straight":
            lines.append("ODOMETRY STRAIGHT-LINE CALIBRATION RESULTS")
            lines.append(f"  commanded: {self.side_length} m at {self.linear_speed} m/s "
                         f"for {self.straight_duration:.2f} s")
        else:
            lines.append("ODOMETRY SQUARE TEST RESULTS")
            lines.append(f"  commanded: {self.num_sides}x{self.side_length} m sides "
                         f"(perimeter {ideal} m)")
        lines.append("=" * 72)
        lines.append(f"{'source':<24}{'close_err_m':>13}{'head_err_deg':>14}"
                     f"{'path_m':>10}{'samples':>9}")
        lines.append("-" * 72)

        for topic, src in self.sources.items():
            if not src.has_data():
                lines.append(f"{topic:<24}{'NO DATA':>13}")
                summary["sources"][topic] = {"no_data": True}
                continue
            start = None
            for label, pose in src.corners:
                if label == "start" and pose is not None:
                    start = pose
                    break
            if start is None:
                start = src.first
            end = src.last
            close_err = math.hypot(end[1] - start[1], end[2] - start[2])
            head_err = math.degrees(wrap_angle(end[3] - start[3]))
            path = src.path_length()
            summary["sources"][topic] = {
                "closing_error_m": close_err,
                "heading_error_deg": head_err,
                "path_length_m": path,
                "samples": len(src.trajectory),
            }
            lines.append(f"{topic:<24}{close_err:>13.3f}{head_err:>14.2f}"
                         f"{path:>10.2f}{len(src.trajectory):>9}")

        lines.append("-" * 72)
        lines.append("close_err_m  = distance between start and end pose (ideal 0)")
        lines.append("head_err_deg = heading drift start->end (ideal 0)")
        lines.append("path_m       = total measured path length (ideal perimeter above)")
        lines.append("-" * 72)

        if self.mode == "straight":
            self._append_calibration(lines, summary)

        lines.append("=" * 72)
        self.get_logger().info("\n".join(lines))

        prefix = "straight_test" if self.mode == "straight" else "square_test"
        self._save_json(prefix, summary, also_csv=True)

    def _append_calibration(self, lines, summary):
        # Reference must be an INDEPENDENT, real-world sensor: prefer VIO, then the
        # EKF (its position is VIO-driven). NEVER /odom - it dead-reckons the
        # command, so it just echoes commanded speed and tells us nothing.
        order = ["/vio/odometry", "/odometry/filtered"]
        ref_topic = next(
            (t for t in order if t in self.sources and self.sources[t].has_data()),
            None,
        )
        lines.append("-" * 72)
        if ref_topic is None or self.straight_duration <= 0.0:
            lines.append(
                "CALIBRATION: no VIO/EKF data to measure actual speed "
                "(/odom is excluded - it only echoes the command)."
            )
            return

        entry = summary["sources"][ref_topic]
        measured = entry["closing_error_m"]  # net displacement of the straight run
        actual_speed = measured / self.straight_duration
        ratio = measured / self.side_length if self.side_length > 0 else float("nan")
        suggested = self.driver_max_speed * ratio

        summary["calibration"] = {
            "reference_source": ref_topic,
            "commanded_distance_m": self.side_length,
            "commanded_speed_mps": self.linear_speed,
            "measured_distance_m": measured,
            "actual_speed_mps": actual_speed,
            "distance_ratio": ratio,
            "driver_max_speed": self.driver_max_speed,
            "suggested_max_speed": suggested,
        }

        lines.append(f"CALIBRATION (using {ref_topic}):")
        lines.append(f"  commanded {self.side_length:.3f} m @ {self.linear_speed:.3f} m/s, "
                     f"measured {measured:.3f} m -> actual {actual_speed:.3f} m/s "
                     f"({ratio:.2f}x commanded)")
        lines.append(f"  to make actual match commanded, scale driver max_speed by {ratio:.2f}:")
        lines.append(f"    ros2 param set /{self.driver_node} max_speed {suggested:.3f}")
        lines.append(f"  (and set max_speed: {suggested:.3f} in wave_rover_bridge.yaml). "
                     "Re-run to verify.")

    # ----------------------------------------------------------------- output
    def _save_json(self, prefix, data, also_csv=False):
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            data.setdefault("timestamp", datetime.now().isoformat())
            json_path = os.path.join(self.output_dir, f"{prefix}_{stamp}.json")
            with open(json_path, "w") as f:
                json.dump(data, f, indent=2)
            if also_csv:
                for topic, src in self.sources.items():
                    if not src.trajectory:
                        continue
                    safe = topic.strip("/").replace("/", "_")
                    csv_path = os.path.join(self.output_dir, f"{prefix}_{stamp}_{safe}.csv")
                    with open(csv_path, "w", newline="") as f:
                        w = csv.writer(f)
                        w.writerow(["t", "x", "y", "yaw"])
                        w.writerows(src.trajectory)
            self.get_logger().info(f"results saved to {self.output_dir} (prefix {prefix}_{stamp})")
        except OSError as exc:
            self.get_logger().error(f"failed to write results to {self.output_dir}: {exc}")

    def stop_rover(self):
        try:
            for _ in range(3):
                self._publish_cmd(0.0, 0.0)
                time.sleep(0.02)
        except Exception:
            pass


def main():
    rclpy.init()
    node = SquareTest()
    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_rover()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
