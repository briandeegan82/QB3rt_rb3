#!/usr/bin/env bash
# Fetch the QAIRT 2.50 SDK zip from Qualcomm softwarecenter and extract the
# side-by-side runtime staged under /opt/qairt-2.50 on the units.
#
# Why: AI Hub no longer offers QAIRT 2.49, so new yoloe_seg exports build
# 2.50 context binaries that the packaged 2.49 runtime refuses ("Using newer
# context binary on old SDK"). The qcom-ppa has no 2.50 debs either, so the
# 2.50 libs ride side-by-side under /opt/qairt-2.50 — yolo_npu.launch.py picks
# them up via qairt:=auto without touching the system 2.49 install.
#
# The download (~2.6 GB) stalls mid-stream on Qualcomm's CDN, so it is fetched
# with curl -C - and retried. Reuse an existing zip with QAIRT_ZIP=/path/to.zip.
set -euo pipefail
cd "$(dirname "$0")"
URL="https://softwarecenter.qualcomm.com/api/download/software/sdks/Qualcomm_AI_Runtime_Community/All/2.50.0.260828/v2.50.0.260828.zip"
SHA="a346ea0e2c8631b46d57261a4969994cd9cc34124a8355bbc7b08b2c8bd859a5"
SDK="qairt/2.50.0.260828"
ZIP="${QAIRT_ZIP:-$HOME/QB3rt_rb3/tools/qairt/v2.50.0.260828.zip}"

if ! echo "$SHA  $ZIP" | sha256sum -c --status 2>/dev/null; then
    echo "fetching $URL -> $ZIP"
    mkdir -p "$(dirname "$ZIP")"
    for try in 1 2 3 4 5; do
        curl -fL -C - -o "$ZIP" "$URL" && break
        echo "attempt $try failed/stalled; resuming in 10 s" >&2
        sleep 10
    done
    echo "$SHA  $ZIP" | sha256sum -c
fi
echo "zip verified: $ZIP ($(stat -c%s "$ZIP") bytes)"

# host libs (Ubuntu 24.04 build)
for f in libQnnGraphTransformer.so libQnnHtpNetRunExtensions.so \
         libQnnHtpPrepare.so libQnnHtp.so libQnnHtpV68CalculatorStub.so \
         libQnnHtpV68Stub.so libQnnIr.so libQnnModelDlc.so libQnnSaver.so \
         libQnnSystem.so; do
    unzip -o -j "$ZIP" "$SDK/lib/aarch64-ubuntu-gcc9.4/$f" -d lib/ >/dev/null
done
# DSP skels (hexagon-v68, unsigned)
for f in libQnnHexagonSkel_dspApp.so libQnnHtpNetRunExtensions.so \
         libQnnHtpV68QemuDriver.so libQnnHtpV68Skel.so libQnnHtpV68.so \
         libQnnNetRunDirectV68Skel.so libQnnSaver.so libQnnSystem.so; do
    unzip -o -j "$ZIP" "$SDK/lib/hexagon-v68/unsigned/$f" -d dsp/ >/dev/null
done
# tools (no ubuntu build ships this one; the oe-linux binary works on noble)
unzip -o -j "$ZIP" "$SDK/bin/aarch64-oe-linux-gcc11.2/qnn-context-binary-utility" -d bin/ >/dev/null
chmod 0755 bin/qnn-context-binary-utility

echo "staged under files/qairt250:"
find lib dsp bin -type f | sort
