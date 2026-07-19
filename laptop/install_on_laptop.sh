#!/bin/bash
# Install/refresh the QB3rt remote-Nav2 bundle onto the laptop.
#
# Run ON THE LAPTOP with the robot sshfs-mounted at ~/mnt/rb3:
#   bash ~/mnt/rb3/root/QB3rt/laptop/install_on_laptop.sh
#
# Copies this directory plus the no-spin behavior trees to ~/qb3rt_laptop
# (a local copy, so Nav2 keeps its files even if the WiFi mount drops).
set -e

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(dirname "$SRC")"
DST="$HOME/qb3rt_laptop"

mkdir -p "$DST/behavior_trees"
cp "$SRC/nav2_laptop.yaml" "$SRC/nav2_laptop.launch.py" "$SRC/qb3rt_env.sh" \
   "$SRC/README.md" "$DST/"
cp "$PROJECT/behavior_trees/"*.xml "$DST/behavior_trees/"

echo "Installed to $DST"
echo "Next: source $DST/qb3rt_env.sh && ros2 launch $DST/nav2_laptop.launch.py"
