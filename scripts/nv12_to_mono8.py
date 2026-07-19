#!/usr/bin/env python3
"""NV12 -> mono8 image converter for the RB3 Gen2 OV9282 tracking camera.

qrb_ros_camera (the Qualcomm HW-ISP camera node) publishes frames in NV12. NV12 is
YUV 4:2:0 semi-planar: a full-resolution luma (Y) plane of `step * height` bytes,
followed by interleaved chroma. For a monochrome tracking camera the luma plane IS
the grayscale image, so converting to ORB-SLAM3's expected mono8 is just a (near
zero-cost) crop of the Y plane to the image width - no colorspace math, no per-pixel
work beyond copying out row padding.

  in  : sensor_msgs/Image, NV12   (default topic: image_nv12)
  out : sensor_msgs/Image, mono8  (default topic: image_raw)

The original header (stamp + frame_id) is preserved so downstream VIO timestamps and
TF frames stay correct.

This is the CPU path and is intentionally trivial. A fully HW-offloaded alternative is
a GStreamer pipeline using the QTI element `qtivtransform` (Adreno/HW) to emit GRAY8,
but the luma-plane copy here is cheap enough that it is not worth the extra moving part.
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class Nv12ToMono8(Node):
    def __init__(self):
        super().__init__("nv12_to_mono8")

        self.declare_parameter("input_topic", "image_nv12")
        self.declare_parameter("output_topic", "image_raw")
        # QoS is decoupled between input and output because the two peers disagree:
        #   input  (image_nv12, from qrb_ros_camera): sensor data, BEST_EFFORT.
        #   output (image_raw, to orb_slam3_ros_mono_imu): that node subscribes
        #          RELIABLE and hardcodes it, so a BEST_EFFORT publisher is dropped
        #          ("incompatible QoS ... RELIABILITY") and ORB-SLAM3 gets zero frames.
        # A RELIABLE publisher is compatible with both RELIABLE and BEST_EFFORT
        # subscribers, so defaulting the output RELIABLE satisfies every consumer.
        self.declare_parameter("input_reliable", False)
        self.declare_parameter("output_reliable", True)
        # Camera->IMU time offset (s), ADDED to the outgoing image stamp so the
        # image is expressed in the IMU timebase (Kalibr: t_imu = t_cam + shift).
        # Stock ORB-SLAM3 has no time-offset field, so this relay is the one
        # place the Kalibr timeshift_cam_imu (-0.0195 s for the OV9282 rig) can
        # be applied. An unapplied ~20 ms skew makes visual-inertial init
        # fragile: it succeeds under motion then immediately diverges/loses
        # tracking (seen 2026-07-13). 0.0 = passthrough.
        self.declare_parameter("stamp_offset", 0.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        input_reliable = bool(self.get_parameter("input_reliable").value)
        output_reliable = bool(self.get_parameter("output_reliable").value)
        self._stamp_offset_ns = int(
            float(self.get_parameter("stamp_offset").value) * 1e9
        )

        def make_qos(reliable):
            return QoSProfile(
                depth=5,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE
                if reliable
                else ReliabilityPolicy.BEST_EFFORT,
            )

        self._warned_encoding = False
        self._pub = self.create_publisher(Image, output_topic, make_qos(output_reliable))
        self._sub = self.create_subscription(
            Image, input_topic, self._on_image, make_qos(input_reliable)
        )

        # Heartbeat: without it this node is silent and a stalled pipeline is
        # indistinguishable from a healthy one (bit us 2026-07-13: /image_raw
        # was dead on-device and nothing said why). Logs in/out rates so the
        # launch console always shows where frames stop.
        self._in_count = 0
        self._out_count = 0
        self._report_t = time.monotonic()
        self.create_timer(5.0, self._report)

        self.get_logger().info(
            f"nv12_to_mono8: {input_topic} (NV12) -> {output_topic} (mono8), "
            f"stamp_offset={self._stamp_offset_ns / 1e9:+.4f}s"
        )

    def _report(self):
        now = time.monotonic()
        dt = now - self._report_t
        in_hz = self._in_count / dt if dt > 0 else 0.0
        out_hz = self._out_count / dt if dt > 0 else 0.0
        log = self.get_logger()
        if self._in_count == 0:
            log.warn("no NV12 frames received in the last %.0f s "
                     "(camera stalled or QoS/topic mismatch)" % dt)
        elif self._out_count == 0:
            log.warn("received %.1f Hz NV12 but published 0 mono8 frames "
                     "(conversion failing - check earlier warnings)" % in_hz)
        else:
            log.info("in %.1f Hz -> out %.1f Hz" % (in_hz, out_hz))
        self._in_count = 0
        self._out_count = 0
        self._report_t = now

    def _on_image(self, msg: Image):
        self._in_count += 1
        enc = (msg.encoding or "").lower()
        if enc and enc not in ("nv12", "yuv420sp", ""):
            # Not fatal: we still treat the first H*step bytes as the luma plane, but
            # warn once so a wrong upstream format is noticeable.
            if not self._warned_encoding:
                self.get_logger().warn(
                    f"expected NV12, got encoding '{msg.encoding}'; "
                    "treating leading bytes as luma plane"
                )
                self._warned_encoding = True

        h = msg.height
        w = msg.width
        # Row stride of the luma plane (may include hardware alignment padding).
        step = msg.step if msg.step >= w else w
        if h == 0 or w == 0:
            return

        # Zero-copy view over the message buffer (msg.data supports the buffer
        # protocol; the old bytes(msg.data) added a full ~1.4 MB copy per frame).
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        needed = step * h
        if buf.size < needed:
            self.get_logger().warn(
                f"frame too small for NV12 luma plane: have {buf.size}, "
                f"need {needed} (step={step}, h={h})"
            )
            return

        # Luma plane = first step*h bytes; crop alignment padding down to width.
        if step == w:
            # No padding: the luma plane is already the mono8 image (one copy).
            data = buf[:needed].tobytes()
        else:
            data = np.ascontiguousarray(buf[:needed].reshape(h, step)[:, :w]).tobytes()

        out = Image()
        out.header = msg.header
        if self._stamp_offset_ns:
            t = (msg.header.stamp.sec * 1_000_000_000
                 + msg.header.stamp.nanosec + self._stamp_offset_ns)
            out.header.stamp.sec = t // 1_000_000_000
            out.header.stamp.nanosec = t % 1_000_000_000
        out.height = h
        out.width = w
        out.encoding = "mono8"
        out.is_bigendian = 0
        out.step = w
        out.data = data
        self._pub.publish(out)
        self._out_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = Nv12ToMono8()
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
