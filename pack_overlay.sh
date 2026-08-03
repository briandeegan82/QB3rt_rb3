#!/bin/bash
# Build a slim overlay/ tree (and optional release tarball) for --bootstrap.
#
# The full /usr/share dump (~1 GB, hundreds of stock ROS packages) must NOT go
# on GitHub. This packs only packages listed in bootstrap_packages.list plus
# their libs / ament_index markers / python bindings.
#
# Usage:
#   ./pack_overlay.sh                         # from ../share + ../lib (legacy dump)
#   ./pack_overlay.sh /path/to/dump           # dump must contain share/ and lib/
#   ./pack_overlay.sh --tarball               # also write qb3rt-overlay.tar.gz
#
# Output: ./overlay/{share,lib}/  (gitignored) and optionally qb3rt-overlay.tar.gz
# Upload the tarball as a GitHub Release asset (see SETUP.md / fetch_overlay.sh).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_LIST="$HERE/bootstrap_packages.list"
OUT="$HERE/overlay"
MAKE_TAR=0
SRC=""

while [ $# -gt 0 ]; do
    case "$1" in
        --tarball) MAKE_TAR=1; shift ;;
        -h|--help)
            sed -n '2,16p' "$0" | sed 's/^# \?//'
            exit 0
            ;;
        *) SRC="$1"; shift ;;
    esac
done

if [ -z "$SRC" ]; then
    if [ -d "$HERE/../share" ] && [ -d "$HERE/../lib" ]; then
        SRC="$HERE/.."
    else
        echo "ERROR: pass a dump root containing share/ and lib/, or place them next to this repo." >&2
        exit 1
    fi
fi
[ -d "$SRC/share" ] && [ -d "$SRC/lib" ] || {
    echo "ERROR: $SRC must contain share/ and lib/" >&2
    exit 1
}

mapfile -t PKGS < <(grep -vE '^\s*(#|$)' "$PKG_LIST")
echo "Packing slim overlay from $SRC -> $OUT"
rm -rf "$OUT"
mkdir -p "$OUT"

copy_path() {
    local rel="$1"
    [ -e "$SRC/$rel" ] || return 0
    mkdir -p "$OUT/$(dirname "$rel")"
    cp -a "$SRC/$rel" "$OUT/$rel"
    echo "  + $rel"
}

for pkg in "${PKGS[@]}"; do
    copy_path "share/$pkg"
    copy_path "lib/$pkg"
    for idx in packages package_run_dependencies parent_prefix_path \
               rosidl_interfaces rclcpp_components; do
        copy_path "share/ament_index/resource_index/$idx/$pkg"
    done
    for f in "$SRC"/share/ament_index/resource_index/*__pluginlib__plugin/"$pkg"; do
        [ -e "$f" ] && copy_path "${f#"$SRC"/}"
    done
done

# Dedup: keep ORBvoc only under orb_slam3_ros (skip duplicate share/orb_slam3
# unless Vocabulary is missing there).
if [ ! -f "$OUT/share/orb_slam3_ros/Vocabulary/ORBvoc.txt" ] && \
   [ -f "$SRC/share/orb_slam3/Vocabulary/ORBvoc.txt" ]; then
    copy_path "share/orb_slam3"
fi

# Top-level overlay .so's referenced by ORB-SLAM / slam_toolbox / msgs.
shopt -s nullglob
for so in "$SRC"/lib/*.so; do
    base="$(basename "$so")"
    case "$base" in
        libORB_SLAM3.so|libDBoW2.so|libg2o.so|liborb_slam3_msgs*.so|\
        lib*slam_toolbox*.so|libtoolbox_common.so|libceres_solver_plugin.so|\
        libkartoSlamToolbox.so|libasync_slam*.so|libsync_slam*.so|\
        liblocalization_slam*.so|liblifelong_slam*.so|libmap_and_localization*.so)
            copy_path "lib/$base"
            ;;
    esac
done
shopt -u nullglob

for py in orb_slam3_msgs slam_toolbox; do
    copy_path "lib/python3.12/site-packages/$py"
done

echo "Slim overlay size:"
du -sh "$OUT" "$OUT/share" "$OUT/lib" 2>/dev/null

if [ "$MAKE_TAR" -eq 1 ]; then
    TAR="$HERE/qb3rt-overlay.tar.gz"
    tar -C "$OUT" -czf "$TAR" share lib
    ls -lh "$TAR"
    echo "Upload $TAR as a GitHub Release asset, then point overlay_release.env at that tag."
fi
