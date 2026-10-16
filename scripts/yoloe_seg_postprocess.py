#!/usr/bin/env python3
"""YOLOE-seg postprocess for the AI Hub qnn_context_binary export.

The stock qrb_ros_yolo_process YoloSegPostProcessNode parses raw YOLOv8-seg
output ([1, 4+nc+32, anchors] + protos). The AI Hub `yoloe_seg` export does the
class-argmax on-device and emits FIVE pre-split tensors instead:

  boxes       [1, N, 4]   float32, xyxy pixels in the 640x640 model frame
  scores      [1, N]      float32, per-anchor top-class confidence
  mask_coeffs [1, N, 32]  float32, per-anchor prototype coefficients
  class_idx   [1, N]      int32/float, winning class index into prompt_text
  mask_protos [1, 32, 160, 160] float32 prototype masks

so this node does what remains: score threshold -> class-aware NMS -> mask
assembly (sigmoid(coeffs @ protos), cropped to each box) -> publish.

  in  : qrb_ros_tensor_list_msgs/TensorList   (default: yolo_segment_tensor_output)
  out : qrb_ros_vision_msgs/Detection2DWithMaskArray (default: yoloe/detections)

Labels come from an ultralytics-style yaml (`names: {0: ..., ...}`) matching the
prompt_text baked into the model at export time. The vocabulary is fixed in the
.bin — to change classes you must re-export, not just edit the yaml.
"""

import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from qrb_ros_tensor_list_msgs.msg import TensorList
from qrb_ros_vision_msgs.msg import (
    Detection2DWithMask,
    Detection2DWithMaskArray,
)
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose

# Tensor msg data_type enum (see qrb_ros_tensor_list_msgs/msg/Tensor.msg)
DTYPE_MAP = {0: np.uint8, 1: np.int8, 2: np.float32, 3: np.float64}

MODEL_RES = 640  # export height/width
PROTO_RES = 160  # mask_protos spatial size (MODEL_RES // 4)


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def _nms(boxes, scores, iou_thres):
    """Greedy NMS over xyxy boxes; returns kept indices, best first."""
    if len(boxes) == 0:
        return np.empty(0, dtype=int)
    x1, y1, x2, y2 = boxes.T
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        iou = w * h / (areas[i] + areas[order[1:]] - w * h + 1e-9)
        order = order[1:][iou <= iou_thres]
    return np.asarray(keep, dtype=int)


class YoloeSegPostprocess(Node):
    def __init__(self):
        super().__init__("yoloe_seg_postprocess")

        self.declare_parameter("input_topic", "yolo_segment_tensor_output")
        self.declare_parameter("output_topic", "yoloe/detections")
        self.declare_parameter("label_file", "")
        self.declare_parameter("quant_params_file", "")
        self.declare_parameter("score_thres", 0.25)
        self.declare_parameter("iou_thres", 0.45)
        self.declare_parameter("mask_thres", 0.5)
        self.declare_parameter("max_det", 50)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._score_thres = float(self.get_parameter("score_thres").value)
        self._iou_thres = float(self.get_parameter("iou_thres").value)
        self._mask_thres = float(self.get_parameter("mask_thres").value)
        self._max_det = int(self.get_parameter("max_det").value)

        # Per-tensor {name: (scale, offset)} for dequantizing uint8 outputs.
        # The TensorList msg carries no quantize params, so they ship in a
        # sidecar yaml produced from qnn-context-binary-utility --json_file.
        self._quant = {}
        qp_file = self.get_parameter("quant_params_file").value
        if qp_file:
            with open(qp_file) as f:
                self._quant = {
                    k: (float(v[0]), float(v[1]))
                    for k, v in (yaml.safe_load(f) or {}).items()
                }
            self.get_logger().info(
                f"quant params for {len(self._quant)} tensors from {qp_file}"
            )

        label_file = self.get_parameter("label_file").value
        self._names = {}
        if label_file:
            with open(label_file) as f:
                self._names = {
                    int(k): str(v)
                    for k, v in (yaml.safe_load(f).get("names") or {}).items()
                }
            self.get_logger().info(
                f"loaded {len(self._names)} labels from {label_file}"
            )

        qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(
            Detection2DWithMaskArray, output_topic, qos
        )
        self._sub = self.create_subscription(
            TensorList, input_topic, self._on_tensors, qos
        )

        self._in_count = 0
        self._det_count = 0
        self._report_t = time.monotonic()
        self._logged_tensors = False
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f"yoloe_seg_postprocess: {input_topic} -> {output_topic} "
            f"(score>{self._score_thres}, iou>{self._iou_thres})"
        )

    def _report(self):
        dt = time.monotonic() - self._report_t
        log = self.get_logger()
        if self._in_count == 0:
            log.warn(f"no inference tensors in the last {dt:.0f} s")
        else:
            log.info(
                f"{self._in_count / dt:.1f} Hz in, "
                f"{self._det_count} dets this window"
            )
        self._in_count = self._det_count = 0
        self._report_t = time.monotonic()

    def _decode(self, tensor):
        dt = DTYPE_MAP.get(tensor.data_type)
        if dt is None:
            raise ValueError(f"unknown tensor data_type {tensor.data_type}")
        arr = np.frombuffer(bytes(tensor.data), dtype=dt)
        if tensor.shape:
            arr = arr.reshape(tuple(tensor.shape))
        # Dequantize uint8/int8 outputs when a scale/offset is known.
        if dt in (np.uint8, np.int8) and tensor.name in self._quant:
            scale, offset = self._quant[tensor.name]
            arr = (arr.astype(np.float32) + offset) * scale
        return arr

    def _on_tensors(self, msg: TensorList):
        self._in_count += 1
        tensors = {t.name: t for t in msg.tensor_list}
        if not self._logged_tensors:
            self._logged_tensors = True
            desc = {
                k: (list(v.shape), DTYPE_MAP.get(v.data_type).__name__)
                for k, v in tensors.items()
            }
            self.get_logger().info(f"output tensors: {desc}")
        try:
            boxes = self._decode(tensors["boxes"]).reshape(-1, 4)
            scores = self._decode(tensors["scores"]).reshape(-1)
            coeffs = self._decode(tensors["mask_coeffs"]).reshape(
                boxes.shape[0], -1
            )
            classes = self._decode(tensors["class_idx"]).reshape(-1)
            protos = self._decode(tensors["mask_protos"])
            # Compiled with --force_channel_last_output: [1,H,W,32] NHWC.
            # Normalize to [32,H,W]; also accept NCHW in case a future export
            # drops the channel-last flag.
            if protos.shape[-1] == 32:
                protos = np.moveaxis(protos, -1, 0)
            protos = protos.reshape(32, PROTO_RES, PROTO_RES)
        except (KeyError, ValueError) as e:
            if self._in_count == 1:
                self.get_logger().error(
                    f"unexpected tensor layout: {e} — "
                    "expected boxes/scores/mask_coeffs/class_idx/mask_protos"
                )
            return

        keep = scores >= self._score_thres
        idx = np.nonzero(keep)[0]
        if idx.size == 0:
            return
        # Per-class NMS.
        kept = []
        for c in np.unique(classes[idx]):
            ci = idx[classes[idx] == c]
            kept.extend(ci[_nms(boxes[ci], scores[ci], self._iou_thres)])
        kept = np.asarray(
            sorted(kept, key=lambda i: -scores[i])[: self._max_det], dtype=int
        )
        if kept.size == 0:
            return

        protos_flat = protos.reshape(protos.shape[0], -1)  # [32, H*W]

        out = Detection2DWithMaskArray()
        out.header = msg.header
        for i in kept:
            x1, y1, x2, y2 = boxes[i]
            det = Detection2D()
            hyp = ObjectHypothesisWithPose()
            ci = int(classes[i])
            hyp.hypothesis.class_id = self._names.get(ci, str(ci))
            hyp.hypothesis.score = float(scores[i])
            det.results.append(hyp)
            det.bbox.center.position.x = float((x1 + x2) / 2)
            det.bbox.center.position.y = float((y1 + y2) / 2)
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            dm = Detection2DWithMask()
            dm.header = msg.header
            dm.detection = det

            # mask = sigmoid(coeffs . protos), in proto space (160x160),
            # cropped to the box mapped from the 640 frame.
            mask = _sigmoid(coeffs[i] @ protos_flat).reshape(
                PROTO_RES, PROTO_RES
            )
            s = PROTO_RES / MODEL_RES
            px1 = int(max(0, np.floor(x1 * s)))
            py1 = int(max(0, np.floor(y1 * s)))
            px2 = int(min(PROTO_RES, np.ceil(x2 * s)))
            py2 = int(min(PROTO_RES, np.ceil(y2 * s)))
            crop = mask[py1:py2, px1:px2]
            img = Image()
            img.header = msg.header
            img.encoding = "mono8"
            if crop.size:
                img.height, img.width = crop.shape
                img.step = crop.shape[1]
                img.data = (crop > self._mask_thres).astype(np.uint8).tobytes()
                # scale 0/1 -> 0/255
                img.data = (np.frombuffer(img.data, np.uint8) * 255).tobytes()
            dm.instance_mask = img
            out.array.append(dm)

        self._det_count += len(out.array)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = YoloeSegPostprocess()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
