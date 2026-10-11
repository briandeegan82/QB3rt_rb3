#!/usr/bin/env python3
"""qb3rt-convention-probe — measure this unit's hardware conventions and
write /etc/qb3rt/unit_conventions.yaml.

WHY: WAVE ROVER chassis are hand-assembled. On qb3rt-7 the drivetrain drove
toward the physical REAR for +linear.x while +angular.z correctly yawed CCW
(L/R swapped + polarity-reversed), and the lidar scan-zero faces the rear on
some units. Students should verify, not discover — this probe automates the
discovery so provisioning can stamp a known-good baseline.

Probes (each is a short, bounded cmd_vel pulse):
  1. linear:   +x pulse; IMU accel_x integration gives physical dv. Sign
               flips on an inverted drivetrain -> drive_inverted.
  2. angular:  +z pulse; mean gyro_z must be positive (REP-103). A negative
               mean is flagged for MANUAL review (no auto-fix exists).
  3. lidar:    operator places a tall object ~0.5 m in front of the intended
               nose (camera-facing end). The scan angle of the near cluster
               reveals where scan-zero physically points -> lidar_yaw.

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


def probe_linear(p):
    print("linear probe: +x 0.15 m/s for 1.2 s (watch the robot) ...")
    samples = p.pulse(linear=0.15, secs=1.2)
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
    return (dv, mean_early), None


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

    results = {}
    ok = True

    # --- linear -----------------------------------------------------------
    r, err = probe_linear(p)
    if err:
        print(f"linear probe FAILED: {err} — set drive_inverted manually")
        ok = False
    else:
        dv, mean_early = r
        print(f"  integrated dv = {dv:+.3f} m/s, early mean accel = {mean_early:+.2f} m/s^2")
        if abs(dv) < 0.03 and abs(mean_early) < 0.3:
            print("  response too weak to classify (blocked wheels? carpet?) — "
                  "set drive_inverted manually")
            ok = False
        else:
            inverted = dv < 0 if abs(dv) >= 0.03 else mean_early < 0
            results["drive_inverted"] = inverted
            print(f"  -> drive_inverted = {inverted} "
                  f"({'physically reversed' if inverted else 'normal'})")

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
    if sys.stdin.isatty():
        input("\nPlace a tall object ~0.5 m in front of the robot's NOSE "
              "(camera-facing end), then press ENTER ...")
    else:
        print("\nNon-interactive run — assuming the calibration object is "
              "already at the nose; settling 2 s ...")
        time.sleep(2.0)
    p.spin(1.0)
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
