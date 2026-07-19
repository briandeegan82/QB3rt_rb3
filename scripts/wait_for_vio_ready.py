#!/usr/bin/env python3
"""Block until ORB-SLAM3 VIO reports ready, then exit — a launch sequencing gate.

Subscribes to the latched /vio/ready (std_msgs/Bool, published transient_local
by scripts/orbslam3_pose_to_odom.py the first time VIO becomes gravity-aligned).
When it receives True it shuts down and exits 0; if that has not happened within
`timeout` seconds it logs a warning and exits 0 anyway.

The point is the CLEAN EXIT: launch/lidar_slam.launch.py runs this as an
ExecuteProcess and starts slam_toolbox from an OnProcessExit handler, so
slam_toolbox is held until VIO has finished its VIBA init ritual (otherwise it
would map the bogus start-up hand motion and build against an odom frame that
shifts when VIO joins the EKF). The timeout is a fallback so a skipped/failed
ritual never leaves the robot with no map at all.

Run standalone:
    ros2 run QB3rt wait_for_vio_ready.py --ros-args -p timeout:=60.0
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from std_msgs.msg import Bool


class WaitForVioReady(Node):
    def __init__(self):
        super().__init__("wait_for_vio_ready")
        self.declare_parameter("topic", "/vio/ready")
        # Seconds to wait before giving up and letting the caller proceed anyway.
        self.declare_parameter("timeout", 60.0)

        self._topic = self.get_parameter("topic").value
        self._timeout = float(self.get_parameter("timeout").value)
        self.ready = False

        # Must match the relay's latched publisher (transient_local) so we still
        # receive the retained True even if VIO went ready before we started.
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, self._topic, self._on_ready, qos)
        self.get_logger().info(
            f"waiting for VIO ready on {self._topic} "
            f"(timeout {self._timeout:.0f}s)..."
        )

    def _on_ready(self, msg: Bool):
        if msg.data:
            self.ready = True


def main(args=None):
    rclpy.init(args=args)
    node = WaitForVioReady()
    deadline = time.monotonic() + node._timeout
    try:
        while rclpy.ok() and not node.ready and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ready:
            node.get_logger().info("VIO ready - proceeding.")
        else:
            node.get_logger().warn(
                f"VIO not ready after {node._timeout:.0f}s; proceeding anyway "
                f"(slam_toolbox will start without waiting for VIO)."
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    # Exit 0 either way: OnProcessExit fires regardless, and a nonzero code
    # would look like a launch failure.
    return 0


if __name__ == "__main__":
    main()
