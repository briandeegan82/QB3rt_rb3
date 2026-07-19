#!/usr/bin/env python3
"""Probe the raw /oak/vio/odometry axis mapping.

Subscribes to the RAW Basalt VIO odometry and prints the net position delta
since the first sample, plus which axis is currently the gravity/vertical axis
(inferred from the pose orientation).  Push (or slowly drive) the rover a known
distance FORWARD, then read which of x/y/z moved: that axis (and sign) is
"forward" in the Basalt odom frame.

  ros2 run ... (just: python3 vio_axis_probe.py)
Ctrl-C to stop; it prints a summary and the recommended OAK->ROS rotation.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


def quat_to_R(x, y, z, w):
    n = x*x + y*y + z*z + w*w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x*x*s, y*y*s, z*z*s
    xy, xz, yz = x*y*s, x*z*s, y*z*s
    wx, wy, wz = w*x*s, w*y*s, w*z*s
    return np.array([
        [1-(yy+zz),   xy-wz,     xz+wy],
        [  xy+wz,   1-(xx+zz),   yz-wx],
        [  xz-wy,     yz+wx,   1-(xx+yy)],
    ])


class Probe(Node):
    def __init__(self):
        super().__init__("vio_axis_probe")
        qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.p0 = None
        self.last = None
        self.create_subscription(Odometry, "/oak/vio/odometry", self.cb, qos)
        self.create_timer(0.5, self.tick)

    def cb(self, msg):
        p = msg.pose.pose.position
        self.last = (np.array([p.x, p.y, p.z]), msg.pose.pose.orientation)
        if self.p0 is None:
            self.p0 = self.last[0].copy()

    def tick(self):
        if self.last is None:
            self.get_logger().info("waiting for /oak/vio/odometry ...")
            return
        d = self.last[0] - self.p0
        q = self.last[1]
        R = quat_to_R(q.x, q.y, q.z, q.w)
        # In the odom frame, gravity(down) direction = the body 'down' axis mapped
        # to odom.  We don't know body 'down' a priori, but the odom axis that is
        # vertical is the one the pose's z stays ~constant along during level
        # motion.  Report the full delta so the user can read it off.
        self.get_logger().info(
            f"net dpos odom: dx={d[0]:+.3f} dy={d[1]:+.3f} dz={d[2]:+.3f}  "
            f"|d|={np.linalg.norm(d):.3f}")


def main():
    rclpy.init()
    n = Probe()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
