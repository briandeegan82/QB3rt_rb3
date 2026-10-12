# rf2o_laser_odometry — prebuilt arm64 binary

`rf2o_laser_odometry.tar.gz` is a merged-install tree for `/opt/qb3rt/install`:
extracting it at the overlay root places the node in
`lib/rf2o_laser_odometry/` and its ament markers under
`share/ament_index/resource_index/`.

## Why a binary instead of a package

- There is no `ros-jazzy-rf2o-laser-odometry` apt package.
- Per-unit colcon builds are unreliable on this fleet: the image's apt state
  is broken by an unmet `fastrpc-support` dependency, and the lab network's
  TLS interception presents a not-yet-valid github cert, so `git clone` on the
  unit fails. The source was cloned on the provisioning laptop and rsynced.
- All units are identical RB3 Gen 2 / Ubuntu 24.04 / ROS 2 Jazzy arm64, so a
  single build is valid everywhere.

## Provenance

- Source: https://github.com/MAPIRlab/rf2o_laser_odometry
- Commit: `b38c68e46387b98845ecbfeb6660292f967a00d3` (jazzy branch)
- Built on qb3rt-12, 2026-07-28: `colcon build --packages-select
  rf2o_laser_odometry --cmake-args -DCMAKE_BUILD_TYPE=Release` in
  `~/ov_ws`, then the node binary was `strip`ped.
- Runtime deps: only base ROS libs already in the Jazzy apt install +
  Eigen (system).

## Rebuilding

On any unit with the workspace intact (or after rsyncing the source):

```bash
cd ~/ov_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select rf2o_laser_odometry \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
strip install/rf2o_laser_odometry/lib/rf2o_laser_odometry/rf2o_laser_odometry_node
mkdir -p /tmp/rf2o_pkg && cp -r install/rf2o_laser_odometry/{lib,share} /tmp/rf2o_pkg/
tar czf rf2o_laser_odometry.tar.gz -C /tmp/rf2o_pkg lib share
```

Replace this tarball and rerun `base-config.yml` or `deploy/update_project.sh`.
