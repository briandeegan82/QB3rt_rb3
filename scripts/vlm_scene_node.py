#!/usr/bin/env python3
"""VLM scene node: caption frames and drive an open-vocabulary find-object
search using a llama.cpp multimodal server (SmolVLM on Adreno OpenCL).

ROS2 Jazzy, std_msgs/std_srvs only — no custom interfaces.

Interfaces
  sub  <image_topic>   sensor_msgs/CompressedImage  latest frame (jpeg bytes)
  sub  <odom_topic>    nav_msgs/Odometry            yaw for bearings (optional)
  sub  <scan_topic>    sensor_msgs/LaserScan        forward stop-range (optional)
  pub  <cmd_vel_topic> geometry_msgs/Twist          search motion
  pub  /vlm/response   std_msgs/String              JSON results
  pub  /vlm/feedback   std_msgs/String              progress lines
  sub  /vlm/command    std_msgs/String              JSON commands
  srv  ~/describe      std_srvs/Trigger             caption latest frame
  srv  ~/find_object   std_srvs/Trigger             find `target` param
  srv  ~/cancel        std_srvs/Trigger             abort running search

/vlm/command payload (JSON):
  {"cmd": "describe", "prompt": "..."}
  {"cmd": "find",     "target": "red fire extinguisher"}
  {"cmd": "cancel"}

Frame source falls back to a one-shot qtiqmmfsrc gstreamer grab when no
compressed frame has arrived — so the node works with or without
perception.launch.py running.
"""

import base64
import json
import math
import subprocess
import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

LOCATE_PROMPT = (
    "You are a robot camera. Is a '{target}' visible in this image? "
    "Answer exactly one word: LEFT, CENTER, RIGHT, or NOT-VISIBLE."
)
CLOSE_PROMPT = (
    "Is the {target} close enough to touch — filling much of the image? "
    "Answer exactly one word: CLOSE, FAR, or NOT-VISIBLE."
)
DESCRIBE_PROMPT = "Describe this image in two sentences."

DIR_OFFSET = {"LEFT": 0.35, "CENTER": 0.0, "RIGHT": -0.35}


class VlmSceneNode(Node):
    def __init__(self):
        super().__init__("vlm_scene_node")
        p = self.declare_parameter
        self.vlm_url = p("vlm_url", "http://127.0.0.1:9876").value
        self.target = p("target", "red fire extinguisher").value
        self.drive = p("drive", False).value
        self.capture_fallback = p("capture_fallback", True).value
        self.spin_speed = p("spin_speed", 0.5).value        # rad/s
        self.fwd_speed = p("fwd_speed", 0.12).value         # m/s
        self.stop_range = p("stop_range", 0.40).value       # m
        self.front_sector = math.radians(p("front_sector_deg", 25.0).value)
        self.sample_count = p("sample_count", 8).value
        self.votes = p("votes_per_sample", 2).value
        self.max_approach = p("max_approach_m", 6.0).value
        self.http_timeout = p("http_timeout", 60.0).value
        self.max_tokens = p("max_tokens", 60).value

        self._frame = None          # latest jpeg bytes
        self._frame_t = 0.0
        self._yaw = None            # odom yaw, None if no odom yet
        self._rel_yaw = 0.0         # commanded cumulative yaw (fallback)
        self._scan_ranges = []
        self._angle_min = 0.0
        self._angle_inc = 1.0
        self._cancel = threading.Event()
        self._worker = None

        image_topic = p("image_topic", "/image_raw/compressed").value
        self.create_subscription(
            CompressedImage, image_topic, self._on_frame, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, p("odom_topic", "/odometry/filtered").value,
            self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, p("scan_topic", "/scan").value,
            self._on_scan, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(
            Twist, p("cmd_vel_topic", "/cmd_vel").value, 10)
        self.resp_pub = self.create_publisher(String, "/vlm/response", 10)
        self.fb_pub = self.create_publisher(String, "/vlm/feedback", 10)
        self.create_subscription(
            String, "/vlm/command", self._on_command, 10)

        self.create_service(Trigger, "~/describe", self._srv_describe)
        self.create_service(Trigger, "~/find_object", self._srv_find)
        self.create_service(Trigger, "~/cancel", self._srv_cancel)
        self.get_logger().info(
            f"vlm_scene_node ready (server={self.vlm_url}, drive={self.drive})")

    # ---- subscriptions --------------------------------------------------
    def _on_frame(self, msg):
        self._frame = bytes(msg.data)
        self._frame_t = time.time()

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        self._yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _on_scan(self, msg):
        self._scan_ranges = list(msg.ranges)
        self._angle_min = msg.angle_min
        self._angle_inc = msg.angle_increment or 1.0

    # ---- VLM ------------------------------------------------------------
    def _query(self, jpeg, prompt, max_tokens=None):
        body = {
            "messages": [{
                "role": "user",
                "content": [
                    {"type": "image_url", "image_url": {
                        "url": "data:image/jpeg;base64," +
                               base64.b64encode(jpeg).decode()}},
                    {"type": "text", "text": prompt},
                ],
            }],
            "max_tokens": max_tokens or self.max_tokens,
            "temperature": 0.1,
        }
        req = urllib.request.Request(
            self.vlm_url + "/v1/chat/completions",
            data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=self.http_timeout) as r:
            out = json.load(r)
        return out["choices"][0]["message"]["content"].strip()

    def _vote(self, prompt, valid):
        """Query `votes` times; return majority token from `valid`."""
        tally = {}
        jpeg = self._grab_frame()
        if jpeg is None:
            return "NOT-VISIBLE"
        for _ in range(self.votes):
            try:
                ans = self._query(jpeg, prompt, 16).upper()
            except Exception as e:
                self.get_logger().warn(f"vlm query failed: {e}")
                continue
            tok = next((v for v in valid if v in ans), "NOT-VISIBLE")
            tally[tok] = tally.get(tok, 0) + 1
        return max(tally, key=tally.get) if tally else "NOT-VISIBLE"

    # ---- frames ----------------------------------------------------------
    def _grab_frame(self):
        if self._frame is not None:
            return self._frame
        if not self.capture_fallback:
            return None
        try:
            subprocess.run(
                "rm -f /tmp/vlm_cap_*.jpg; timeout --signal=SIGINT 3 "
                "gst-launch-1.0 -e qtiqmmfsrc name=qmmf qmmf.video_0 ! "
                "video/x-raw,format=NV12,width=1280,height=720,framerate=30/1 ! "
                "jpegenc ! multifilesink location=/tmp/vlm_cap_%02d.jpg",
                shell=True, check=False, capture_output=True, timeout=10)
            files = sorted(subprocess.run(
                "ls /tmp/vlm_cap_*.jpg 2>/dev/null | tail -1",
                shell=True, capture_output=True, text=True).stdout.split())
            if files:
                data = open(files[0], "rb").read()
                subprocess.run("rm -f /tmp/vlm_cap_*.jpg", shell=True)
                return data
        except Exception as e:
            self.get_logger().warn(f"gst capture failed: {e}")
        return None

    # ---- motion ----------------------------------------------------------
    def _twist(self, linear=0.0, angular=0.0):
        t = Twist()
        t.linear.x, t.angular.z = linear, angular
        if self.drive:
            self.cmd_pub.publish(t)

    def _stop(self):
        self._twist(0.0, 0.0)

    def _rotate(self, angle_rad):
        """Rotate in place; track rel_yaw for dead-reckoned bearings."""
        if abs(angle_rad) < 0.05:
            return
        sign = 1.0 if angle_rad > 0 else -1.0
        t_end = time.time() + abs(angle_rad) / self.spin_speed
        while time.time() < t_end and not self._cancel.is_set():
            self._twist(0.0, sign * self.spin_speed)
            time.sleep(0.05)
        self._stop()
        self._rel_yaw += angle_rad
        time.sleep(0.4)  # settle for a clean frame

    def _yaw_now(self):
        return self._yaw if self._yaw is not None else self._rel_yaw

    def _front_range(self):
        if not self._scan_ranges:
            return float("inf")
        vals = []
        for i, r in enumerate(self._scan_ranges):
            a = self._angle_min + i * self._angle_inc
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(a) < self.front_sector and r > 0.05:
                vals.append(r)
        return min(vals) if vals else float("inf")

    # ---- commands ---------------------------------------------------------
    def _publish_result(self, payload):
        self.resp_pub.publish(String(data=json.dumps(payload)))

    def _feedback(self, text):
        self.get_logger().info(text)
        self.fb_pub.publish(String(data=text))

    def _on_command(self, msg):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self._publish_result({"ok": False, "error": "bad json"})
            return
        c = cmd.get("cmd")
        if c == "describe":
            self._publish_result(
                {"ok": True, "caption": self._describe(
                    cmd.get("prompt") or DESCRIBE_PROMPT)})
        elif c == "find":
            if self._worker and self._worker.is_alive():
                self._publish_result(
                    {"ok": False, "error": "find already running"})
            elif self._start_find(cmd.get("target", self.target)):
                self._publish_result({"ok": True, "started": True,
                                      "target": self._target})
        elif c == "cancel":
            self._cancel.set()
            self._publish_result({"ok": True, "cancelled": True})
        else:
            self._publish_result({"ok": False, "error": f"unknown cmd {c}"})

    def _srv_describe(self, _req, resp):
        resp.success = True
        resp.message = self._describe(DESCRIBE_PROMPT)
        return resp

    def _srv_find(self, _req, resp):
        if self._worker and self._worker.is_alive():
            resp.success = False
            resp.message = "find already running"
            return resp
        resp.success = self._start_find(self.target)
        resp.message = f"searching for '{self.target}'" if resp.success \
            else "failed to start"
        return resp

    def _srv_cancel(self, _req, resp):
        self._cancel.set()
        resp.success = True
        resp.message = "cancel requested"
        return resp

    def _describe(self, prompt):
        jpeg = self._grab_frame()
        if jpeg is None:
            return "no frame available"
        try:
            return self._query(jpeg, prompt, self.max_tokens)
        except Exception as e:
            return f"vlm error: {e}"

    # ---- find loop ---------------------------------------------------------
    def _start_find(self, target):
        if self._worker and self._worker.is_alive():
            return False
        self._target = target
        self._cancel.clear()
        self._worker = threading.Thread(target=self._find_loop, daemon=True)
        self._worker.start()
        return True

    def _find_loop(self):
        result = {"found": False, "target": self._target}
        try:
            result.update(self._scan_then_approach())
        except Exception as e:
            result["error"] = str(e)
        finally:
            self._stop()
            result["ok"] = True
            self._publish_result(result)
            self._feedback(f"find finished: {result}")

    def _scan_then_approach(self):
        tgt = self._target
        step = 2.0 * math.pi / self.sample_count
        # phase 1: rotating scan
        positives = []
        for i in range(self.sample_count):
            if self._cancel.is_set():
                return {"error": "cancelled"}
            self._feedback(f"scan {i + 1}/{self.sample_count}")
            label = self._vote(
                LOCATE_PROMPT.format(target=tgt),
                ["CENTER", "LEFT", "RIGHT", "NOT-VISIBLE"])
            self._feedback(f"  heading {i}: {label}")
            if label != "NOT-VISIBLE":
                positives.append((self._yaw_now(), label))
            self._rotate(step)
        if not positives:
            return {"found": False, "reason": f"{tgt} not seen in 360 scan"}
        # prefer a CENTER sighting, else first sighting
        yaw, label = next(
            ((y, l) for y, l in positives if l == "CENTER"), positives[0])
        bearing = yaw + DIR_OFFSET[label]
        self._feedback(f"sighted at rel yaw {bearing:.2f} rad ({label})")
        # face the target
        self._rotate(math.atan2(math.sin(bearing - self._yaw_now()),
                                math.cos(bearing - self._yaw_now())))
        # phase 2: approach, re-centering on the way
        travelled = 0.0
        t_last_q = time.time()
        while not self._cancel.is_set():
            rng = self._front_range()
            if rng < self.stop_range:
                self._stop()
                ans = self._vote(
                    CLOSE_PROMPT.format(target=tgt),
                    ["CLOSE", "FAR", "NOT-VISIBLE"])
                if ans == "CLOSE":
                    return {"found": True, "stop_range_m": rng}
                return {"found": True, "stop_range_m": rng,
                        "note": f"stopped on obstacle, vlm={ans}"}
            self._twist(self.fwd_speed, 0.0)
            travelled += self.fwd_speed * 0.1
            time.sleep(0.1)
            if travelled > self.max_approach:
                self._stop()
                return {"found": False, "reason": "max approach distance"}
            if time.time() - t_last_q > 6.0:
                self._stop()
                ans = self._vote(
                    CLOSE_PROMPT.format(target=tgt),
                    ["CLOSE", "FAR", "NOT-VISIBLE"])
                t_last_q = time.time()
                if ans == "CLOSE":
                    return {"found": True, "vlm": "close"}
                if ans == "NOT-VISIBLE":
                    return {"found": False, "reason": "lost sight mid-approach"}
        return {"error": "cancelled"}


def main():
    rclpy.init()
    node = VlmSceneNode()
    try:
        rclpy.spin(node)
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
