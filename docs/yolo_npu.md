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
`qairt:=auto` is the default: the launch uses `/opt/qairt-2.50` when it is
installed, else the system runtime (see the side-by-side section below).

## Model artifacts

Models are AI Hub `yoloe_seg` exports (`yoloe-v8l-seg`, Ultralytics YOLOE)
compiled as `qnn_context_binary` for `Dragonwing RB3 Gen 2 Vision Kit`
(`hexagon:v68`), quantized w8a8 with quantized I/O. The vocabulary is baked
into the weights at export via `--prompt-text` — changing classes means
re-exporting, not editing the label yaml.

Three exports exist (in `~/QB3rt_rb3/models/` on the dev laptop):

- `yoloe_lab_qa249.bin` — lab vocab: door, hallway sign, trash bin,
  fire extinguisher, person. Pair with `config/yoloe_lab.yaml` +
  `yoloe_lab_quant.yaml`.
- `yoloe_coco80_qa249.bin` — COCO-80. Pair with `yoloe_coco80.yaml` +
  `yoloe_coco80_quant.yaml`.
- `yoloe_mission_qa250.bin` — mission vocab: traffic cone, car,
  traffic light, pedestrian, cyclist, dog, cat. Pair with
  `config/yoloe_mission.yaml` + `yoloe_mission_quant.yaml`. QAIRT 2.50
  build, so it needs `/opt/qairt-2.50` (`qairt:=auto` handles that) — the
  system 2.49 runtime refuses it. Provenance (`export_default.log`,
  `metadata.json`) is alongside the bin in `~/QB3rt_rb3/models/yoloe_mission/`.

How the two `_qa249.bin` files were built (2026-10-09/11) and why the mission
bin can't repeat it: a default
`qai-hub-models export yoloe_seg` (compile at QAIRT 2.45, link at the AI Hub
default, then 2.50), then the compiled model re-linked on AI Hub with
`--qairt_version 2.49`. The export's own `.bin` embeds 2.50 and does not load
on the fleet runtime. That re-link route no longer exists (2.49 is retired —
see below), which is why `yoloe_mission_qa250.bin` is a straight 2.50 export.
Check any `.bin` with
`strings -n 6 MODEL.bin | grep -oE "v2\.[0-9]{2}\.[0-9]+\.[0-9]{6,}" | sort -u`.

The quant sidecar (`*_quant.yaml`) is generated on-unit from the `.bin`:

```bash
qnn-context-binary-utility --context_binary MODEL.bin --json_file /tmp/m.json
python3 - <<'EOF'
import json, yaml
d = json.load(open('/tmp/m.json'))
outs = d['info']['graphs'][0]['info']['graphOutputs']
q = {t['info']['name']: t['info']['quantizeParams'].get('scaleOffset') for t in outs}
yaml.dump({k: [v['scale'], v['offset']] for k, v in q.items() if v},
          open('out.yaml', 'w'), default_flow_style=None)
EOF
```

`class_idx` is a plain `uint8` output with no quant params, so it is skipped.
The same values are in the export's `metadata.json` (`quantization_parameters`;
the YAML offset is the negated `zero_point`). Every yoloe_seg export so far
(lab, COCO, a 7-class mission export) has had identical scales/offsets.

### Fleet deployment of the .bin files

The three bins (`yoloe_coco80_qa249.bin`, `yoloe_lab_qa249.bin`,
`yoloe_mission_qa250.bin`) are ~46 MB each and NOT in git — they stage
through the qb3rt_base role exactly like the QAIRT debs:

```bash
# once on the control host:
ansible/roles/qb3rt_base/files/yoloe/fetch-yoloe-models.sh
# then per unit / fleet-wide:
ansible-playbook playbooks/base-config.yml [-l qb3rt-N]
```

The role rsyncs them to `/home/ubuntu/models/` (ubuntu-owned) — the path
`perception.launch.py`'s `model:=` default expects. Label/quant yamls ride
the normal project payload (`config/yoloe_*.yaml` → share/QB3rt).

## QAIRT version requirement — THE reason the fleet needs 2.49

Units ship with qairt 2.46 (noble apt). Two hard blockers:

1. The runtime refuses blobs built newer than itself ("Using newer context
   binary on old SDK" → "Inference graph init fail!", and the container
   dies). 2.46 cannot load any AI Hub `.bin`. When the fleet was set up AI Hub
   offered 2.45/2.49/2.50; as of 2026-11-12 it offers only **2.45, 2.50
   (default) and 2.51** (`qai-hub list-frameworks`), so **new 2.49 binaries
   can no longer be built**. Re-linking at 2.49 fails with "QAIRT SDK version
   2.49 is not valid". 2.45 can't build YOLOE either: the link fails with
   "could not create op" on the MatMul nodes. A fresh export therefore needs a
   ≥2.50 runtime on the units. The qcom-ppa tops out at 2.49, so that
   runtime has to come from the QAIRT SDK zip.
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

That 2.49 install runs the existing fleet `.bin`s; for 2.50 exports the role
adds the side-by-side stack described next.

## QAIRT 2.50 side-by-side (`/opt/qairt-2.50`)

AI Hub dropped QAIRT 2.49 and the qcom-ppa has no 2.50 debs, so the runtime
that runs new exports is unpacked from the QAIRT SDK zip to
`/opt/qairt-2.50` — nothing under `/usr` or in dpkg is touched, and the
installed 2.49 stack stays the system default.

Staged files (from `aarch64-ubuntu-gcc9.4` unless noted; the
`qnn-context-binary-utility` binary only ships as `aarch64-oe-linux-gcc11.2`
and runs fine on noble):

- `lib/` — `libQnnHtp.so`, `libQnnHtpPrepare.so`, `libQnnSystem.so`,
  `libQnnHtpV68Stub.so`, `libQnnHtpV68CalculatorStub.so`,
  `libQnnHtpNetRunExtensions.so`, `libQnnSaver.so`, `libQnnModelDlc.so`,
  `libQnnIr.so`, `libQnnGraphTransformer.so`
- `dsp/` (SDK `lib/hexagon-v68/unsigned`) — `libQnnHtpV68.so`,
  `libQnnHtpV68Skel.so`, `libQnnSystem.so`, `libQnnHtpNetRunExtensions.so`,
  `libQnnHtpV68QemuDriver.so`, `libQnnSaver.so`, `libQnnHexagonSkel_dspApp.so`,
  `libQnnNetRunDirectV68Skel.so`
- `bin/` — `qnn-context-binary-utility`

`yolo_npu.launch.py`'s `qairt` arg picks the runtime: `auto` (default) uses
`/opt/qairt-2.50` if `lib/libQnnHtp.so` exists there, else the system libs;
`system` forces `/usr/lib/libQnnHtp.so` with no env changes; any other value
is an explicit QAIRT root with `lib/` and `dsp/`. For a non-system root the
container gets `LD_LIBRARY_PATH=<root>/lib:...` and
`ADSP_LIBRARY_PATH=<root>/dsp;/usr/lib/dsp/cdsp;/usr/lib/rfsa/adsp;/dsp`, and
`backend_option` becomes `<root>/lib/libQnnHtp.so`; the resolved backend is
logged as `yolo_npu: QAIRT backend ...` at startup.

Deploy:

```bash
# once on the control host (reuses ~/QB3rt_rb3/tools/qairt/v2.50.0.260828.zip
# if present; QAIRT_ZIP=/path/to.zip overrides):
ansible/roles/qb3rt_base/files/qairt250/fetch-qairt250.sh
# then per unit / fleet-wide:
ansible-playbook playbooks/base-config.yml [-l qb3rt-N]
```

Verified side-by-side on qb3rt-42 (2026-11-12): the 2.50 mission export ran
at ~12 Hz with `/opt/qairt-2.50` libs in the container's maps and the mission
classes published; the same 2.50 stack ran the fleet's
`yoloe_coco80_qa249.bin` at ~12 Hz (2.50 is backwards-compatible with 2.49
binaries); and the system 2.49 backend on the 2.50 bin reproduced the
expected "Using newer context binary on old SDK" failure.

The `qairt` launch arg itself was verified on qb3rt-42 the same day by
running the launch file by path: `qairt:=auto` picked the 2.50 backend and
logged `yolo_npu: QAIRT backend /opt/qairt-2.50/lib/libQnnHtp.so
(qairt:=auto)`, with the container's `LD_LIBRARY_PATH`/`ADSP_LIBRARY_PATH`
set as described above (~12 Hz on the mission model); `qairt:=system` logged
the `/usr/lib/libQnnHtp.so` backend, set no `ADSP_LIBRARY_PATH` override, and
ran the 2.49 COCO model at ~12 Hz; `qairt:=system` on the 2.50 mission bin
reproduced "Using newer context binary on old SDK" followed by `Inference
graph init fail`; and `qairt:=/nonexistent` aborted the launch with a clear
error before the container started. The Ansible qairt250 block was run
against qb3rt-42 with the same tasks only: first pass fixed owner and modes
(`root:root`, dirs 0755, libs 0644, bin 0755), second pass reported
`changed=0`.

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
