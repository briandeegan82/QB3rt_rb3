#!/usr/bin/env bash
# Fetch the QAIRT 2.49 arm64 runtime debs the fleet needs for AI Hub
# qnn_context_binary models. Stock noble repos ship 2.46, which both rejects
# AI Hub context blobs (built with >= 2.49) and lacks MatMul in the V68 skel.
#
# These come from the ubuntu-qcom-iot qcom-ppa `resolute` suite. The declared
# deps libfastrpc1/hexagon-dsp-binaries/fastrpc-support are functionally
# covered by the image's qcom-fastrpc1 (1.0.15), so the role installs with
# dpkg --force-depends rather than dragging in the resolute fastrpc stack.
set -euo pipefail
cd "$(dirname "$0")"
BASE=https://ppa.launchpadcontent.net/ubuntu-qcom-iot/qcom-ppa/ubuntu/pool/main/q/qairt-sdk
V=2.49.0-0ubuntu2
for p in libqnn1 qairt-dsp-binaries qairt-libs qairt-tools; do
    f="${p}_${V}_arm64.deb"
    [ -f "$f" ] || curl -fLO "$BASE/$f"
    echo "have $f ($(stat -c%s "$f") bytes)"
done
