# YOLOE open-vocabulary detection on the Hexagon NPU

Pipeline (all inside `launch/yolo_npu.launch.py`, verified 2026-10-11 on
qb3rt-7 at ~14 Hz sustained):

```
IMX577 colour camera (camera_id 0, 1920x1080@30, NV12, cam0_stream1)
  -> CvTensorCommonProcessNode  (640x640, uint8 NHWC, normalize=false)
  -> /qrb_inference_input_tensor
  -> QrbRosInferenceNode        (model .bin + /usr/lib/libQnnHtp.so)
  -> /yolo_segment_tensor_output  (TensorList, 5 uint8 tensors)
  -> scripts/yoloe_seg_postprocess.py  (dequant -> per-class NMS -> mask decode)
  -> /yoloe/detections  (Detection2DWithMaskArray)
  -> optional YoloSegOverlayNode -> /yoloe/overlay
```

Run:

```bash
ros2 launch QB3rt yolo_npu.launch.py \
    model:=/home/ubuntu/models/yoloe_lab_qa249.bin \
    label_file:=/opt/qb3rt/install/share/QB3rt/config/yoloe_lab.yaml \
    quant_params_file:=/opt/qb3rt/install/share/QB3rt/config/yoloe_lab_quant.yaml
```

`enable_camera:=false` if `cam0_stream1` is already published elsewhere.

## Model artifacts

Models are AI Hub `yoloe_seg` exports (`yoloe-v8l-seg`, Ultralytics YOLOE)
compiled as `qnn_context_binary` for `Dragonwing RB3 Gen 2 Vision Kit`
(`hexagon:v68`), quantized w8a8 with quantized I/O. The vocabulary is baked
into the weights at export via `--prompt-text` — changing classes means
re-exporting, not editing the label yaml.

Two exports exist (in `~/QB3rt_rb3/models/` on the dev laptop):

- `yoloe_lab_qa249.bin` — lab vocab: door, hallway sign, trash bin,
  fire extinguisher, person. Pair with `config/yoloe_lab.yaml` +
  `yoloe_lab_quant.yaml`.
- `yoloe_coco80_qa249.bin` — COCO-80. Pair with `yoloe_coco80.yaml` +
  `yoloe_coco80_quant.yaml`.

The quant sidecar (`*_quant.yaml`) is generated on-unit from the `.bin`:

```bash
qnn-context-binary-utility --context_binary MODEL.bin --json_file /tmp/m.json
python3 -c "import json,yaml; d=json.load(open('/tmp/m.json'));
 yaml.dump({t['name']:(t['scale'],t['offset']) for t in d['graph']['outputs']},
           open('out.yaml','w'))"
```

(The AI Hub inference job output already has these values; both models share
identical scales/offsets.)

## QAIRT version requirement — THE reason the fleet needs 2.49

Units ship with qairt 2.46 (noble apt). Two hard blockers:

1. AI Hub emits context binaries only with 2.45/2.49/2.50 toolchains, and the
   runtime refuses blobs built newer than itself ("Using newer context
   binary on old SDK"). 2.46 cannot load any AI Hub `.bin`.
2. The 2.46 V68 DSP skel **cannot create MatMul ops** — YOLOE's
   vision-language layers need it. Even composing the `.dlc` on-device fails
   ("RouterFastRPC graph prepare failed 18 / node_matmul could not create
   op"). AI Hub's hosted RB3 Gen 2 (same V68 silicon, newer runtime) runs
   the identical graph fine, so this is purely a runtime-version issue.

Fix deployed by `qb3rt_base` (ansible): install `qairt-{libs,tools,dsp-binaries}`
+ `libqnn1` 2.49 from the qcom-ppa **resolute** suite with
`dpkg --force-depends` — its declared deps `libfastrpc1`,
`hexagon-dsp-binaries`, `fastrpc-support` are functionally covered by the
image's `qcom-fastrpc1` 1.0.15 and the running adsp/cdsp daemons; pulling
the resolute fastrpc stack over it is unnecessary and riskier.

Debs are NOT in git (~210 MB). Fetch once on the control host:

```bash
ansible/roles/qb3rt_base/files/qairt/fetch-qairt-debs.sh
```

Second required step the role also does: repoint
`/usr/lib/dsp/cdsp/libQnn*.so` symlinks from unmanaged golden-image copies
under `/usr/share/qcom/...` to the packaged 2.49 files under
`/usr/share/hexagon-dsp/...`, then `systemctl restart cdsprpcd`.

## Cameras

- **IMX577 = camera_id 0** (probe log: `sensorId(0) -> cmk_imx577`). Do NOT
  trust the sensormodule filename suffix (`cmk_imx577_cam2` is the board
  slot, not the ROS camera_id).
- **OV9282 = camera_id 1** — unusable here: `qrb_ros_transport` has no mono8
  path, and its NV12 luma is grey anyway.
- A failed camera open can wedge cam-server ("unrecoverable error"); if a
  later open produces no frames, `systemctl restart cam-server` first.

## Output tensor format (why a custom postprocess exists)

The AI Hub export does class-argmax on-device and emits five pre-split
tensors, so the stock `YoloSegPostProcessNode` (which expects raw
YOLOv8-seg `[1,4+nc+32,anchors]` + protos) cannot parse it:

- `image`       in  [1,640,640,3] uint8 NHWC, scale=1/255 — feed raw pixels
- `boxes`       out [1,8400,4]  uint8, xyxy in the 640 frame
- `scores`      out [1,8400]    uint8 (scale 1/256)
- `mask_coeffs` out [1,8400,32] uint8
- `class_idx`   out [1,8400]    uint8 index into the baked vocab
- `mask_protos` out [1,160,160,32] uint8 NHWC (`--force_channel_last_output`)

`yoloe_seg_postprocess.py` dequantizes via the sidecar, runs per-class NMS,
and decodes `sigmoid(coeffs @ protos)` masks cropped to each box.

## Known fleet issue

Every `/opt/model/*.bin` on the golden image is built for **V73** — none of
them can run on these V68 units ("Request feature arch with value 73
unsupported"). The bundled ML sample stack is broken fleet-wide as shipped;
rebuilds for V68 via AI Hub are the fix if those demos are wanted.
