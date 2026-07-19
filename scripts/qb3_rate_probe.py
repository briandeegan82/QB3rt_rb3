#!/usr/bin/env python3
"""Rate probe for QB3rt camera/VIO topics.

Prints two rates per topic:
  1) header_hz: from message header timestamp deltas (source cadence)
  2) recv_hz  : from callback arrival deltas (subscriber-observed cadence)

This helps distinguish publisher timing from host/subscriber bottlenecks.
"""

from collections import deque
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class TopicStats:
    def __init__(self, name: str, max_samples: int):
        self.name = name
        self.header_dt = deque(maxlen=max_samples)
        self.recv_dt = deque(maxlen=max_samples)
        self._last_header = None
        self._last_recv = None
        self.count = 0

    def update(self, header_time: float, recv_time: float) -> None:
        self.count += 1
        if self._last_header is not None:
            dh = header_time - self._last_header
            if dh > 0.0:
                self.header_dt.append(dh)
        if self._last_recv is not None:
            dr = recv_time - self._last_recv
            if dr > 0.0:
                self.recv_dt.append(dr)
        self._last_header = header_time
        self._last_recv = recv_time

    @staticmethod
    def _rate(samples) -> float:
        if not samples:
            return 0.0
        avg = sum(samples) / float(len(samples))
        if avg <= 0.0:
            return 0.0
        return 1.0 / avg

    @staticmethod
    def _jitter_ms(samples) -> float:
        if len(samples) < 2:
            return 0.0
        avg = sum(samples) / float(len(samples))
        var = sum((x - avg) * (x - avg) for x in samples) / float(len(samples))
        return (var ** 0.5) * 1000.0

    def snapshot(self):
        return {
            "count": self.count,
            "header_hz": self._rate(self.header_dt),
            "recv_hz": self._rate(self.recv_dt),
            "header_jitter_ms": self._jitter_ms(self.header_dt),
            "recv_jitter_ms": self._jitter_ms(self.recv_dt),
        }


class RateProbe(Node):
    def __init__(self, args):
        super().__init__("qb3_rate_probe")
        self.args = args
        self._start = time.time()
        max_samples = max(10, int(args.window_sec * 30))
        self.stats = {}

        qos = QoSProfile(
            depth=50,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.subscribed_topics = []
        if args.with_images:
            self._add_sub(Image, args.rgb_topic, qos)
            self._add_sub(Image, args.stereo_topic, qos)
        self._add_sub(CameraInfo, args.rgb_info_topic, qos)
        self._add_sub(CameraInfo, args.stereo_info_topic, qos)
        self._add_sub(Odometry, args.vio_topic, qos)

        self.create_timer(args.print_every_sec, self._tick)
        self.get_logger().info("Started qb3_rate_probe")

    def _add_sub(self, msg_type, topic: str, qos) -> None:
        self.stats[topic] = TopicStats(topic, max_samples=max(10, int(self.args.window_sec * 30)))
        self.subscribed_topics.append(topic)

        def cb(msg, t=topic):
            recv_t = time.time()
            header_t = _stamp_to_sec(msg.header.stamp)
            self.stats[t].update(header_t, recv_t)

        self.create_subscription(msg_type, topic, cb, qos)

    def _tick(self) -> None:
        elapsed = time.time() - self._start
        print("\n=== qb3_rate_probe {:.1f}s ===".format(elapsed), flush=True)
        for topic in self.subscribed_topics:
            s = self.stats[topic].snapshot()
            print(
                "{:<28} count={:<6d} header_hz={:>6.2f} recv_hz={:>6.2f} "
                "header_jitter_ms={:>6.2f} recv_jitter_ms={:>6.2f}".format(
                    topic,
                    s["count"],
                    s["header_hz"],
                    s["recv_hz"],
                    s["header_jitter_ms"],
                    s["recv_jitter_ms"],
                ),
                flush=True,
            )

        if self.args.duration_sec > 0.0 and elapsed >= self.args.duration_sec:
            self.get_logger().info("Duration reached, exiting.")
            rclpy.shutdown()


def parse_args():
    p = argparse.ArgumentParser(description="QB3rt header-based topic rate probe")
    p.add_argument("--duration-sec", type=float, default=0.0, help="Auto-exit after N seconds (0 = run until Ctrl-C)")
    p.add_argument("--print-every-sec", type=float, default=1.0, help="Print interval")
    p.add_argument("--window-sec", type=float, default=12.0, help="Approximate stats window")
    p.add_argument("--rgb-topic", default="/oak/rgb/image_raw")
    p.add_argument("--rgb-info-topic", default="/oak/rgb/camera_info")
    p.add_argument("--stereo-topic", default="/oak/stereo/image_raw")
    p.add_argument("--stereo-info-topic", default="/oak/stereo/camera_info")
    p.add_argument("--vio-topic", default="/oak/vio/odometry")
    p.add_argument(
        "--with-images",
        action="store_true",
        help="Also subscribe to heavy image_raw topics (may perturb rates on constrained hosts)",
    )
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = RateProbe(args)
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
