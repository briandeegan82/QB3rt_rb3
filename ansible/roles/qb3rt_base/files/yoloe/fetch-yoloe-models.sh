#!/bin/bash
# Stage the YOLOE QNN context binaries into this directory for Ansible
# deployment. Source of truth on the dev laptop: ~/QB3rt_rb3/models/
# (AI Hub yoloe_seg exports, QAIRT 2.49 toolchain, V68/QCS6490).
#
#   ansible/roles/qb3rt_base/files/yoloe/fetch-yoloe-models.sh
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${YOLOE_MODELS_DIR:-$HOME/QB3rt_rb3/models}"

for m in yoloe_coco80_qa249.bin yoloe_lab_qa249.bin; do
    # Files live in per-model subdirs (yoloe_coco80/, yoloe_lab/).
    found="$(find "$SRC" -name "$m" -type f | head -1)"
    [ -n "$found" ] || { echo "ERROR: $m not found under $SRC" >&2; exit 1; }
    cp -v "$found" "$HERE/$m"
done
echo "Staged in $HERE — base-config.yml will sync them to /home/ubuntu/models/."
