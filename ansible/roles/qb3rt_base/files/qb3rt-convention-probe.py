#!/usr/bin/env python3
"""qb3rt-convention-probe — measure this unit's hardware conventions and
write /etc/qb3rt/unit_conventions.yaml.

WHY: WAVE ROVER chassis are hand-assembled. On qb3rt-7 the drivetrain drove
toward the physical REAR for +linear.x while +angular.z correctly yawed CCW
(L/R swapped + polarity-reversed), and the lidar scan-zero faces the rear on
some units. Students should verify, not discover — this probe automates the
discovery so provisioning can stamp a known-good baseline.

Probes (each is a short, bounded cmd_vel pulse):
  1. linear:   +x pulse; the near obstacle's scan range shrinking means the
               chassis moved toward its nose (forward) — the discriminator
               is the lidar, not IMU accel, because IMU-x vs body-x sign is
               itself a per-unit convention (qb3rt-17). IMU launch accel is
               kept as a fallback only.
  2. angular:  +z pulse; mean gyro_z must be positive (REP-103). A negative
               mean is flagged for MANUAL review (no auto-fix exists).
  3. lidar:    the same front obstacle's raw scan angle reveals where
               scan-zero physically points -> lidar_yaw.

Prereqs on the unit: the base stack must be RUNNING (waverover_bridge for
cmd_vel, IMU publisher, RPLIDAR):
    ros2 launch QB3rt odometry.launch.py enable_base_driver:=true
    ros2 launch QB3rt rplidar_c1.launch.py        # if not already included

Run:  sudo qb3rt-convention-probe          (writes /etc/qb3rt/...)
Output: /etc/qb3rt/unit_conventions.yaml   {drive_inverted, lidar_yaw}
"""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan

CONV_FILE = "/etc/qb3rt/unit_conventions.yaml"


class Probe(Node):
    def __init__(self):
        super().__init__("qb3rt_convention_probe")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.imu = []
        self.scan = None
        self.create_subscription(Imu, "/imu/data", lambda m: self.imu.append(m), 50)
        self.create_subscription(
            LaserScan, "/scan", lambda m: setattr(self, "scan", m), 10
        )

    def spin(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, what, get, timeout=8.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if get():
                return True
        print(f"ERROR: no {what} after {timeout:.0f}s — is the base stack up?")
        return False

    def bridge_drive_inverted(self):
        """Read the waverover_bridge's CURRENTLY EFFECTIVE drive_inverted
        (yaml default folded with any existing conventions override). The
        linear probe's +x pulse runs under that active setting, so the
        correct write is `current` when motion is forward and `not current`
        when backward — NOT a hard-coded true. Without this the yaml's
        fleet default (true, qb3rt-7's quirk) makes every normal-polarity
        unit drive backward during the probe and get misclassified."""
        cli = self.create_client(GetParameters, "/waverover_bridge/get_parameters")
        if not cli.wait_for_service(timeout_sec=3.0):
            print("  (could not read bridge drive_inverted param — "
                  "assuming the active setting is false)")
            return False
        req = GetParameters.Request()
        req.names = ["drive_inverted"]
        fut = cli.call_async(req)
        t0 = time.time()
        while time.time() - t0 < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if fut.done():
                vals = fut.result().values
                if vals:
                    return bool(vals[0].bool_value)
                break
        return False

    def pulse(self, linear=0.0, angular=0.0, secs=1.0):
        """Command a bounded twist; collect IMU samples during the pulse."""
        self.imu.clear()
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        t0 = time.time()
        while time.time() - t0 < secs:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.pub.publish(Twist())
        self.spin(0.5)
        return list(self.imu)


def nearest_obstacle_range(scan):
    """Nearest valid return in 0.2..1.0 m — the calibration object placed at
    the nose. Frame-independent: physical distance, no yaw convention needed."""
    if scan is None:
        return None
    best = None
    for r in scan.ranges:
        if 0.2 < r < 1.0 and math.isfinite(r):
            if best is None or r < best:
                best = r
    return best


def probe_linear(p):
    print("linear probe: +x 0.15 m/s for 1.2 s (watch the robot) ...")
    r0 = nearest_obstacle_range(p.scan)
    samples = p.pulse(linear=0.15, secs=1.2)
    p.spin(0.3)  # let a post-pulse scan arrive
    r1 = nearest_obstacle_range(p.scan)
    if len(samples) < 20:
        return None, f"insufficient IMU samples ({len(samples)})"
    # Integrate accel_x over the pulse -> delta-v. Sign says which way the
    # chassis physically accelerated. Bias cancels poorly over a short window,
    # so also use the early-window mean (accel spike dominates first ~0.5 s).
    t0 = samples[0].header.stamp.sec + samples[0].header.stamp.nanosec * 1e-9
    dv, last = 0.0, None
    for m in samples:
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if last is not None:
            dv += m.linear_acceleration.x * (t - last)
        last = t
    early = [
        m.linear_acceleration.x
        for m in samples
        if (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - t0) < 0.5
    ]
    mean_early = sum(early) / len(early) if early else 0.0
    dr = (r0 - r1) if (r0 is not None and r1 is not None) else None
    return (dv, mean_early, dr), None


def probe_angular(p):
    print("angular probe: +z 0.5 rad/s for 1.5 s ...")
    samples = p.pulse(angular=0.5, secs=1.5)
    if len(samples) < 20:
        return None, f"insufficient IMU samples ({len(samples)})"
    mean_wz = sum(m.angular_velocity.z for m in samples) / len(samples)
    return mean_wz, None


def probe_lidar(p):
    if p.scan is None:
        return None, "no scan received"
    m = p.scan
    # Find the nearest valid cluster inside 0.2..1.0 m.
    best = None
    for i, r in enumerate(m.ranges):
        if 0.2 < r < 1.0 and math.isfinite(r):
            a = m.angle_min + i * m.angle_increment
            a = (a + math.pi) % (2 * math.pi) - math.pi
            if best is None or r < best[0]:
                best = (r, a)
    if best is None:
        return None, "no obstacle within 0.2-1.0 m — place one at the nose"
    return best, None


def main():
    rclpy.init()
    p = Probe()
    print("waiting for /imu/data and /scan ...")
    if not p.wait_for("IMU", lambda: len(p.imu) > 5):
        return 2
    if not p.wait_for("scan", lambda: p.scan is not None):
        return 2

    # The calibration object must be in place BEFORE the linear pulse — the
    # range-shrink during the pulse is the direction discriminator. The --yes
    # flag skips the prompt: isatty() alone is unreliable because ansible's
    # command channel presents a pty, which blocks on input() forever.
    if sys.stdin.isatty() and "--yes" not in sys.argv:
        input("Place a tall object ~0.5 m in front of the robot's NOSE "
              "(camera-facing end), then press ENTER ...")
    else:
        print("Non-interactive run — assuming the calibration object is "
              "already at the nose; settling 2 s ...")
        time.sleep(2.0)
    p.spin(1.0)

    results = {}
    ok = True

    # --- linear -----------------------------------------------------------
    r, err = probe_linear(p)
    if err:
        print(f"linear probe FAILED: {err} — set drive_inverted manually")
        ok = False
    else:
        dv, mean_early, dr = r
        print(f"  integrated dv = {dv:+.3f} m/s, launch mean accel = "
              f"{mean_early:+.2f} m/s^2, obstacle range delta = "
              f"{'n/a' if dr is None else f'{dr:+.3f} m'}")
        # PRIMARY discriminator: obstacle range delta. The object sits at the
        # physical nose, so dr > 0 (approached) means the pulse drove FORWARD
        # and dr < 0 (receded) means BACKWARD — regardless of IMU mounting
        # convention (qb3rt-17's IMU-x disagreed with body-x). A 1.2 s pulse
        # at 0.15 m/s moves ~0.15-0.2 m, far above C1 range noise (~2 cm).
        # FALLBACK: IMU launch-spike sign, only when the lidar can't see the
        # object or the delta is ambiguous (<5 cm). A dead drivetrain gives
        # both ~0 — inconclusive, never a guess.
        if dr is not None and abs(dr) > 0.05:
            forward = dr > 0
            if (mean_early > 0) != forward and abs(mean_early) >= 0.15:
                print(f"  NOTE: IMU accel sign disagrees with lidar "
                      f"(accel {'+' if mean_early > 0 else '-'}, motion "
                      f"{'forward' if forward else 'backward'}) — this "
                      f"unit's IMU-x is flipped vs body-x; trusting lidar.")
        elif abs(mean_early) >= 0.15:
            forward = mean_early > 0
            print("  (lidar delta unusable — fell back to IMU launch accel)")
        else:
            print("  NO physical launch detected (rover unpowered? wheels "
                  "blocked? serial dead?) — drive_inverted left UNWRITTEN. "
                  "Fix power/drive, re-run, or verify by eye.")
            ok = False
            forward = None
        if forward is not None:
            current = p.bridge_drive_inverted()
            # Backward motion while `current` is active means the flip is
            # wrong -> write the opposite. Forward means keep it.
            results["drive_inverted"] = current if forward else not current
            print(f"  -> drive_inverted = {results['drive_inverted']} "
                  f"(active during pulse: {current}; motion "
                  f"{'forward' if forward else 'backward'})")

    # --- angular ----------------------------------------------------------
    wz, err = probe_angular(p)
    if err:
        print(f"angular probe FAILED: {err}")
    elif wz is not None:
        print(f"  mean gyro_z = {wz:+.3f} rad/s under +z command")
        if wz < -0.05:
            print("  WARNING: +z produced CLOCKWISE rotation — violates REP-103 "
                  "and drive_inverted cannot fix it. Inspect motor wiring "
                  "manually before releasing this unit.")
        elif wz < 0.05:
            print("  NOTE: barely any rotation measured (high-friction floor "
                  "or stalled wheels) — verify yaw by eye.")
        else:
            print("  -> yaw convention OK (CCW positive)")

    # --- lidar -------------------------------------------------------------
    r, err = probe_lidar(p)
    if err:
        print(f"lidar probe FAILED: {err} — set lidar_yaw manually")
        ok = False
    else:
        dist, raw_angle = r
        print(f"  obstacle at {dist:.2f} m, raw scan angle {math.degrees(raw_angle):+.0f} deg")
        # Object is at the physical nose. If it appears at raw ~180 deg, the
        # lidar's scan-zero faces the REAR -> URDF yaw = pi. Near 0 -> yaw = 0.
        if abs(raw_angle) > math.pi / 2:
            results["lidar_yaw"] = round(math.pi, 6)
            print("  -> lidar_yaw = pi (scan-zero faces physical rear)")
        else:
            results["lidar_yaw"] = 0.0
            print("  -> lidar_yaw = 0 (scan-zero faces physical front)")

    # --- write --------------------------------------------------------------
    if not results:
        print("\nNothing measured — no file written.")
        return 1
    body = (
        "# Per-unit hardware conventions, measured by qb3rt-convention-probe.\n"
        "# Consumed by: wave_rover_bridge.py (drive_inverted) and\n"
        "# static_transforms.launch.py (lidar_yaw default).\n"
        f"# probed: {time.strftime('%Y-%m-%d %H:%M:%S')} on {os.uname().nodename}\n"
    )
    for k, v in results.items():
        body += f"{k}: {str(v).lower()}\n"
    try:
        os.makedirs(os.path.dirname(CONV_FILE), exist_ok=True)
        with open(CONV_FILE, "w") as f:
            f.write(body)
    except PermissionError:
        print(f"\nPermission denied writing {CONV_FILE} — re-run with sudo.")
        print(body)
        return 1
    print(f"\nwrote {CONV_FILE}:\n{body}")
    if not ok:
        print("One or more probes failed — review the file before releasing.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
