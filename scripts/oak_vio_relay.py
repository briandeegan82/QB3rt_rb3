#!/usr/bin/env python3
"""Relay /oak/vio/odometry into a proper ROS REP-103 frame, with covariance.

WHY THIS EXISTS
---------------
The OAK-D-Lite's IMU is mounted with its **Y axis vertical**: at rest the
accelerometer reads gravity on Y (measured live: /oak/imu/data linear_acceleration
= (0.08, -9.60, 0.43), i.e. +Y points DOWN).  Basalt therefore emits
/oak/vio/odometry in a **camera-optical-style frame whose vertical axis is Y**, not
the Z-up body frame the rest of the stack assumes.  Evidence: a level, stationary
robot reports orientation quat ~ (0.007, 0.9995, -0.007, 0.030) ~ 177 deg about Y.

The OAK VIO frame is:  Y = down (vertical),  X and Z horizontal.
A push test (2026-07-02) showed robot-FORWARD lands on **X**, not Z (X/Z are
swapped vs the usual optical/RDF layout), so forward = +x_oak.
ROS REP-103 body wants: X = forward, Y = left,  Z = up      (FLU).

VERTICAL correction is certain and baked in:  z_ros = -y_oak (up = -down).
FORWARD correction is a 1-parameter choice confirmed by the push test:
`forward_axis` names the OAK axis (and sign) that increases when the robot drives
FORWARD.  Default "+x".  To confirm/flip:

    python3 scripts/vio_axis_probe.py     # then hand-push 0.5 m forward
    # read which of dx/dy/dz grows and its sign -> set forward_axis accordingly:
    #   +x / -x / +z / -z

The relay builds a proper right-handed rotation from `forward_axis` so that
driving forward yields +X, left yields +Y, up yields +Z on the output topic.

Basalt also publishes all-zero covariance (collapses robot_localization's Kalman
gain to 1) and all-zero twist.  A diagonal pose covariance is inserted so the EKF
can weight VIO; twist is passed through untouched (it is not fused).

Parameters
----------
input_topic   : source topic  (default /oak/vio/odometry)
output_topic  : output topic  (default /oak/vio/odometry_cov)
forward_axis  : OAK axis+sign that is robot-forward: +z|-z|+x|-x (default +z)
pos_variance  : diagonal variance for x, y, z position  [m^2]  (default 0.01)
rot_variance  : diagonal variance for roll, pitch, yaw  [rad^2] (default 0.1)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

# The OAK VIO vertical axis is Y-down (gravity on +Y), confirmed from the IMU.
# ROS "up" (+Z_ros) is therefore -Y_oak.  This is fixed hardware geometry.
_UP_IN_OAK = np.array([0.0, -1.0, 0.0])

_FORWARD_VECTORS = {
    "+x": np.array([1.0, 0.0, 0.0]),
    "-x": np.array([-1.0, 0.0, 0.0]),
    "+z": np.array([0.0, 0.0, 1.0]),
    "-z": np.array([0.0, 0.0, -1.0]),
    # +y/-y are the vertical axis and are NOT valid forward choices here.
}


def _build_rotation(forward_axis: str) -> np.ndarray:
    """Rotation R such that p_ros = R @ p_oak maps the OAK VIO frame to ROS FLU.

    Rows of R are the ROS basis vectors (forward, left, up) expressed in OAK
    coordinates.  forward = chosen axis, up = -Y_oak (fixed), left = up x forward
    (right-handed: x=fwd, y=left, z=up => y = z_hat x x_hat = up x forward).
    """
    f = _FORWARD_VECTORS.get(forward_axis.lower())
    if f is None:
        raise ValueError(
            f"forward_axis must be one of {list(_FORWARD_VECTORS)}; got {forward_axis!r}"
        )
    u = _UP_IN_OAK
    # Guard: forward must be horizontal (orthogonal to the vertical axis).
    if abs(float(np.dot(f, u))) > 1e-9:
        raise ValueError(f"forward_axis {forward_axis!r} is the vertical axis; invalid")
    left = np.cross(u, f)
    R = np.vstack([f, left, u])
    return R


def _quat_to_matrix(x, y, z, w):
    n = x*x + y*y + z*z + w*w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x*x*s, y*y*s, z*z*s
    xy, xz, yz = x*y*s, x*z*s, y*z*s
    wx, wy, wz = w*x*s, w*y*s, w*z*s
    return np.array([
        [1.0-(yy+zz),   xy-wz,       xz+wy],
        [  xy+wz,     1.0-(xx+zz),   yz-wx],
        [  xz-wy,       yz+wx,     1.0-(xx+yy)],
    ])


def _matrix_to_quat(m):
    tr = m[0,0] + m[1,1] + m[2,2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2.0
        return np.array([(m[2,1]-m[1,2])/s, (m[0,2]-m[2,0])/s,
                         (m[1,0]-m[0,1])/s, 0.25*s])
    if m[0,0] > m[1,1] and m[0,0] > m[2,2]:
        s = np.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2]) * 2.0
        return np.array([0.25*s, (m[0,1]+m[1,0])/s,
                         (m[0,2]+m[2,0])/s, (m[2,1]-m[1,2])/s])
    if m[1,1] > m[2,2]:
        s = np.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2]) * 2.0
        return np.array([(m[0,1]+m[1,0])/s, 0.25*s,
                         (m[1,2]+m[2,1])/s, (m[0,2]-m[2,0])/s])
    s = np.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1]) * 2.0
    return np.array([(m[0,2]+m[2,0])/s, (m[1,2]+m[2,1])/s,
                     0.25*s, (m[1,0]-m[0,1])/s])


class OakVioRelay(Node):
    def __init__(self):
        super().__init__("oak_vio_relay")

        self.declare_parameter("input_topic",  "/oak/vio/odometry")
        self.declare_parameter("output_topic", "/oak/vio/odometry_cov")
        self.declare_parameter("forward_axis", "+x")
        self.declare_parameter("pos_variance", 0.01)
        self.declare_parameter("rot_variance", 0.1)

        in_topic  = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        fwd       = str(self.get_parameter("forward_axis").value)
        pv = float(self.get_parameter("pos_variance").value)
        rv = float(self.get_parameter("rot_variance").value)

        self._R = _build_rotation(fwd)

        self._pose_cov = [0.0] * 36
        for i, v in enumerate([pv, pv, pv, rv, rv, rv]):
            self._pose_cov[i * 7] = v

        # Twist covariance: large — Basalt publishes zero twist; we don't fuse it.
        self._twist_cov = [0.0] * 36
        for i in range(6):
            self._twist_cov[i * 7] = 1e6

        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(Odometry, out_topic, qos)
        self._sub = self.create_subscription(Odometry, in_topic, self._relay, qos)
        self.get_logger().info(
            f"oak_vio_relay: {in_topic} -> {out_topic}  "
            f"(OAK optical[Y-down] -> ROS FLU, forward={fwd}, pos_variance={pv})"
        )

    def _relay(self, msg: Odometry):
        p_in = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])
        p_out = self._R @ p_in

        q = msg.pose.pose.orientation
        M_in  = _quat_to_matrix(q.x, q.y, q.z, q.w)
        M_out = self._R @ M_in @ self._R.T
        qr = _matrix_to_quat(M_out)
        nrm = np.linalg.norm(qr)
        if nrm > 1e-12:
            qr /= nrm
        else:
            qr = np.array([0.0, 0.0, 0.0, 1.0])

        out = Odometry()
        out.header         = msg.header
        out.child_frame_id = msg.child_frame_id

        out.pose.pose.position.x    = float(p_out[0])
        out.pose.pose.position.y    = float(p_out[1])
        out.pose.pose.position.z    = float(p_out[2])
        out.pose.pose.orientation.x = float(qr[0])
        out.pose.pose.orientation.y = float(qr[1])
        out.pose.pose.orientation.z = float(qr[2])
        out.pose.pose.orientation.w = float(qr[3])
        out.pose.covariance         = self._pose_cov

        out.twist.twist      = msg.twist.twist
        out.twist.covariance = self._twist_cov

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OakVioRelay()
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
