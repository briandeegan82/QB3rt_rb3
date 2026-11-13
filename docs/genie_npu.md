# Genie LLM inference on the Hexagon NPU

Verified 2026-11-07 on qb3rt-42 (dev target, over `Brians iPhone`).

## Prerequisites — already fleet-standard

- `genie-t2t-run` ships in `qairt-tools` and `libGenie.so` in `qairt-libs`,
  both installed fleet-wide by the QAIRT 2.49 block in `qb3rt_base`
  (`ansible/roles/qb3rt_base/tasks/main.yml`). Nothing extra to install.
- The 2.49 runtime loads context binaries built with older SDKs; the V68
  skel symlinks repointed by the role are required (same as YOLOE — see
  `docs/yolo_npu.md`).

## Model bundle

Stock `/opt/model/*.bin` are all **V73** — unusable here, same as the rest
of the golden-image ML stack. Working bundle is Radxa's V68 export of
Qwen2.5-0.5B (w4a16, 1k context), pulled anonymously from ModelScope —
no Aplux/AI Hub account needed:

```bash
mkdir -p genie-models/qwen2.5-0.5b-v68/models
cd genie-models/qwen2.5-0.5b-v68
BASE=https://modelscope.cn/models/radxa/Qwen2.5-0.5B-v68/resolve/master
for f in models/qwen2.5-0.5b-1k.bin tokenizer.json \
         qwen2.5-0.5B-1k-htp.json htp_backend_ext_config.json; do
    curl -fLO "$BASE/$f"   # bin lands in CWD; move it into models/
done
mv qwen2.5-0.5b-1k.bin models/   # htp.json references models/<bin>
```

~647 MB total; `qwen2.5-0.5b-1k.bin` sha256 =
`ca1a502fc11a1c4ef4e0d0adb1c31318fbd0b9dc017a0e8292e530fa7077bf39`.

The repo also ships its own `genie-t2t-run`/libs — ignore them and use the
system 2.49 install.

## Run

```bash
cd ~/genie-models/qwen2.5-0.5b-v68
genie-t2t-run -c qwen2.5-0.5B-1k-htp.json -p "<chat-ml prompt>"
```

Qwen2.5 wants ChatML markup (system/user/assistant turns wrapped in
im_start/im_end tags). `run.sh` alongside the bundle on the unit builds
the prompt — the literal tag text breaks some tooling, so it constructs
the markers with `printf '\x3c\x7c...\x3e'` escapes:

```bash
~/genie-models/qwen2.5-0.5b-v68/run.sh "your question here"
```

Observed on qb3rt-42: libGenie 1.20.0, model loads in ~1 s, correct
answers on the first pass. The `rpcmem_android.c: dummy call` lines on
stdout are benign QAIRT log noise.

## Bigger models / official source

The Thundercomm guide uses the Aplux model zoo (`qnn229_qcs6490_cl4096.zip`
for Qwen2.5-0.5B-**Instruct** w8a16, 4k context) which requires a free
Aplux account, or Qualcomm AI Hub (login + `qai_hub` token). The Radxa
ModelScope bundle above is a plain Qwen2.5-0.5B base model — fine for a
demo, but grab the Instruct export from Aplux/AI Hub if chat quality
matters. Other V68 Genie bundles live under `radxa/*-v68` on ModelScope.