# VLM (camera → vision-language model) on the Adreno GPU

Verified 2026-11-07 on qb3rt-42 (over `Brians iPhone`). Camera frame →
SmolVLM-500M-Instruct via llama.cpp/OpenCL in ~10 s end-to-end.

This is the GPU path. NPU VLMs via Genie need an AI Hub-exported encoder +
split-decoder bundle (account required, and the available exports are 7B —
too big for 5.3 GB RAM). `genie-t2t-run` itself is text-only; see
`docs/genie_npu.md` for the NPU LLM path.

## One-command demo

```bash
~/genie-models/smolvlm-500m/vlm_describe.sh "your prompt here"
```

Captures a fresh 1920x1080 frame from camera 0 (`qtiqmmfsrc` → jpeg →
`/tmp/vlm_frame.jpg`) and runs `llama-mtmd-cli`. Pass an image path as the
second arg to skip capture. Tunables: `MAXTOK`, `TEMP` env vars.

Observed: ~10 tok/s prompt eval (incl. 7.7 s image encode — mmproj is f16,
unquantizable), ~26-32 tok/s decode. Scene description accurate; fine text
OCR hallucinates (it's a 500M model).

## Model files — `~/genie-models/smolvlm-500m/`

From `huggingface.co/ggml-org/SmolVLM-500M-Instruct-GGUF` (public, no login):

- `SmolVLM-500M-Instruct-f16.gguf` → quantized on-unit to
  `SmolVLM-500M-Instruct-q4_0.gguf` (220 MB) via
  `llama-quantize --pure ... Q4_0` (pure Q4_0 is what the Adreno kernels want)
- `mmproj-SmolVLM-500M-Instruct-f16.gguf` (200 MB, stays f16)

Bigger option if quality matters: `radxa/Qwen2-VL-2B-Instruct-NOE` GGUF +
mmproj on ModelScope (~1.7+1.3 GB — still fits).

## llama.cpp build — `~/dev/llm/llama.cpp` @ f6da8cb

Followed the Thundercomm manual (OpenCL-Headers + ICD-Loader into
`~/dev/llm/opencl`, `cmake -G Ninja -DGGML_OPENCL=ON`). Deviations hit on
this unit — expect the same fleet-wide:

1. **dpkg repair first** — the unit's `/var/lib/dpkg/status` still carried
   the QAIRT phantom deps (the ansible repair task hadn't landed here);
   apt was fully broken until the same `qb3rt_base` python repair ran.
2. **`/usr/include/CL` already exists as a real dir** (qcom-adreno-dev) —
   skip the doc's `rm -f /usr/include/CL` symlink step. Its headers include
   `cl_ext_qcom.h`; keep them.
3. **`/etc/OpenCL/vendors/adreno.icd` was missing** — create containing
   `libOpenCL_adreno.so.1`. Without it the loader finds zero vendors.
4. **`/dev/kgsl-3d0` open → ENOENT until `a660_zap.mbn` is loadable** —
   dmesg: "Firmware request for a660_zap.mbn failed -2". The blob ships
   zstd-compressed under `qcom/qcm6490/` but the kernel requests the bare
   name. Fix:
   ```bash
   zstd -dc /lib/firmware/qcom/qcm6490/a660_zap.mbn.zst \
       | sudo tee /lib/firmware/a660_zap.mbn
   ```
   GPU registers on the next open (kgsl retries probe); no reboot needed.
   Diag one-liner:
   `python3 -c "import ctypes; l=ctypes.CDLL('libOpenCL.so.1'); n=ctypes.c_uint(); print(l.clGetPlatformIDs(0,None,ctypes.byref(n)), n.value)"` → `0 1`

Success looks like `llama-cli --version` printing
`device: 'QUALCOMM Adreno(TM) 643'` + `using kernels optimized for Adreno`.

## Camera capture standalone

```bash
timeout --signal=SIGINT 4 gst-launch-1.0 -e \
    qtiqmmfsrc name=qmmf qmmf.video_0 ! \
    'video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1' ! \
    jpegenc ! multifilesink location=/tmp/cap_%02d.jpg
```

Works with cam-server active. Don't use `filesink` for one-shots — it
concatenates every frame into one file. If a later open produces no
frames, `systemctl restart cam-server` (known wedge, see yolo_npu.md).

## ROS2 integration — verified 2026-11-07 on qb3rt-42

Two pieces:

**`qb3rt-vlm-server.service`** (systemd, on-unit) — persistent
`llama-server` with the q4_0 model + mmproj on `127.0.0.1:9876`, OpenAI
`/v1/chat/completions` API, `Restart=on-failure`, `MemoryMax=2G`.
`systemctl start qb3rt-vlm-server`.

**`scripts/vlm_scene_node.py`** (rclpy, std msgs only — no custom
interfaces) — `ros2 launch QB3rt vlm.launch.py`:

- sub `/image_raw/compressed` (or gst `qtiqmmfsrc` fallback when the
  perception stack isn't running), `/odometry/filtered` (yaw), `/scan`
  (stop-range), pub `/cmd_vel`
- `~/describe` (Trigger) → caption latest frame
- `~/find_object` (Trigger) → scan/approach search for `target` param
- `~/cancel` (Trigger); `/vlm/command` topic takes JSON
  `{"cmd":"describe|find|cancel", "prompt|target":"..."}`; results on
  `/vlm/response`, progress on `/vlm/feedback`

Find algorithm: 360° scan in `sample_count` headings (default 8),
`votes_per_sample` (default 2) majority-vote per heading with a
constrained one-word prompt (LEFT/CENTER/RIGHT/NOT-VISIBLE) → bearing
from odom yaw (or commanded-angle dead reckon) → rotate to bearing →
drive forward at `fwd_speed`, re-querying proximity every ~6 s, stop at
`stop_range` on the front ±25° scan sector or when the VLM says CLOSE.

**`drive:=false` is the default** — the whole pipeline runs (rotation
tracked, bearings logged) without publishing cmd_vel. Set `drive:=true`
only when the robot can move. Verified end-to-end bench-side:
describe → caption, find "laptop" → 8-heading scan → bearing → approach
→ `{"found": true, "vlm": "close"}`.

Observed model noise: identical frames were read NOT-VISIBLE 5× then
CENTER/RIGHT — the 500M model is genuinely marginal at bearing queries.
Bump `votes_per_sample` to 3, or swap in Qwen2-VL-2B (bbox JSON gives
real bearings via intrinsics) for anything beyond demo duty.

## Fleet notes / TODO

- Items 3+4 above are golden-image gaps likely present on every unit —
  worth folding into `qb3rt_base` (adreno.icd + decompressed zap blob)
  if GPU inference goes fleet-wide. The zap fix is a straight
  `dpkg-divert`-style file drop; icd is a one-line file.
- Clock was ~3.5 months stale on arrival (fleet https-time-sync issue);
  TLS/apt/git all fail until `date` is set — do it first on any unit.
