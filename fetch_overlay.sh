#!/bin/bash
# Download the slim bootstrap overlay from a GitHub Release into ./overlay/.
#
# Configure tag/asset in overlay_release.env (copied from .example if missing).
#
#   ./fetch_overlay.sh
#   OVERLAY_RELEASE_TAG=overlay-2026-08-03 ./fetch_overlay.sh
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="$HERE/overlay_release.env"
EXAMPLE="$HERE/overlay_release.env.example"

if [ ! -f "$ENV_FILE" ] && [ -f "$EXAMPLE" ]; then
    cp "$EXAMPLE" "$ENV_FILE"
    echo "Created $ENV_FILE from example — edit tag/asset if needed."
fi
if [ -f "$ENV_FILE" ]; then
    # shellcheck disable=SC1090
    source "$ENV_FILE"
fi

REPO="${OVERLAY_REPO:-briandeegan82/QB3rt_rb3}"
TAG="${OVERLAY_RELEASE_TAG:-}"
ASSET="${OVERLAY_ASSET:-qb3rt-overlay.tar.gz}"
OUT="$HERE/overlay"

if [ -z "$TAG" ]; then
    echo "ERROR: set OVERLAY_RELEASE_TAG in overlay_release.env (or the environment)." >&2
    echo "  Create a GitHub Release, upload qb3rt-overlay.tar.gz (from ./pack_overlay.sh --tarball)," >&2
    echo "  then put that release tag in overlay_release.env." >&2
    exit 1
fi

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
URL="https://github.com/${REPO}/releases/download/${TAG}/${ASSET}"

echo "Fetching $URL ..."
if command -v gh >/dev/null 2>&1; then
    gh release download "$TAG" --repo "$REPO" --pattern "$ASSET" --dir "$TMP"
else
    curl -fL --retry 3 -o "$TMP/$ASSET" "$URL"
fi

echo "Extracting -> $OUT"
rm -rf "$OUT"
mkdir -p "$OUT"
tar -C "$OUT" -xzf "$TMP/$ASSET"
du -sh "$OUT"
echo "Done. You can now: ./deploy_via_adb.sh --unit <id> --bootstrap"
