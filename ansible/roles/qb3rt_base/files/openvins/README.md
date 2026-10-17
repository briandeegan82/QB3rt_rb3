# OpenVINS `ov_msckf` — experimental VIO payload

## Status: EXPERIMENTAL — binaries deploy but are not wired in

The nodes publish `/ov_msckf/*` topics when run manually; nothing consumes
them (the EKF's `odom1` slot is for the ORB-SLAM3 `/vio/odometry` path and is
unaffected). Deploying this package is inert — it exists so the fleet already
has the binaries when the bringup is finished.

**Known issue (qb3rt-12, 2026-10-13):** the filter initializes on motion but
immediately diverges (~3 km/s runaway). Suspects, untested:

- `T_imu_cam` in the Kalibr chain was converted by hand from
  `orbslam3_ov9282_imu.yaml` — a rotation-convention flip would diverge
  exactly like this.
- `calib_cam_timeoffset: true` double-corrects: `nv12_to_mono8.py` already
  applies `image_stamp_offset: -0.0195` to image stamps. Try disabling
  online timeoffset calib or setting the relay offset to 0.
- Carry-test init with the camera tilted ~50° at ceiling lighting gives
  nearly no usable features — degenerate dynamic init.

Config ships in `config/openvins/` (deployed with the QB3rt package):
`estimator_config.yaml` + `kalibr_imucam_chain.yaml` + `kalibr_imu_chain.yaml`.
Run on a unit:

```bash
qb3rt-up openvins    # base + camera + IMU + ov_msckf, systemd-managed
# or by hand:
ros2 launch QB3rt openvins.launch.py
```

The tarball is now repacked as a flat merged install (`lib/`, `share/` +
ament index at the top level), so `ros2 run ov_msckf run_subscribe_msckf
<config>` also works directly — the binaries carry no rpath and resolve
their libs via the `install/lib` entry that the overlay's environment
hooks already put on `LD_LIBRARY_PATH`.

Odometry is published on `/odomimu` (plus `/poseimu`, `/pathimu`,
`/points_msckf`, `/trackhist`) — NOT `/ov_msckf/*` as previously assumed.
The odom is in the filter's init-anchored world frame (~arbitrary heading),
so it is not EKF-fusable without a re-anchor step.

**Verified on qb3rt-42 (2026-10-17):** initialized on a hand carry, tracked
a 10.6 m path, loop closure back at the start point was ~0.23 m (~2%),
z drift 1 cm, cam-IMU timeoffset stable at 12 ms. Driven test: VIO tracked
a 1.0 m commanded move at 1.03 m — scale is accurate. The unit-12 runaway
did not reproduce — its failure was likely the degenerate ceiling-facing
init, not the config suspects above. Note: `qrb_ros_imu` emits IMU samples
in duplicate-timestamp pairs at ~400 Hz — cosmetic "Zero DT" warnings, but
worth filtering if it ever feeds a real consumer.

Needs `/image_raw` (mono8) at ~30 Hz and `/imu/data` at ~200 Hz — e.g.
`rb3_vio.launch.py` supplies the camera chain, `imu.launch.py` the IMU.

## Provenance

- Source: https://github.com/rpng/open_vins — commit `69488123ed9362dd44b6f28e7f4680abbff1442b`
- Built on `qb3rt-12` (QCS6490 arm64, Ubuntu 24.04, ROS 2 Jazzy) in
  `/home/ubuntu/ov_ws`.
- **Jazzy compatibility patch:** `jazzy_compat.patch` — upstream still uses
  the `.h` include shims removed in Jazzy (`image_transport.h`,
  `cv_bridge.h`, `tf2_geometry_msgs.h`) plus a `ROS2.cmake` fix. Apply to the
  checkout before building.
- **Build flags that matter on this hardware:** the unit has ~5.3 GB RAM and
  `cc1plus -O3` on the big TUs (`VioManager.cpp`, `Propagator.cpp`,
  `StateHelper.cpp`, `Simulator.cpp`) OOMs the OOM-killer even at `-j1`.
  What worked:

  ```bash
  MAKEFLAGS=-j1 colcon build --packages-select ov_msckf --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
      "-DCMAKE_CXX_FLAGS_RELEASE=-O1 -g0 -DNDEBUG" \
      "-DCMAKE_C_FLAGS_RELEASE=-O1 -g0 -DNDEBUG"
  ```

  (`--executor sequential` alone does NOT limit make parallelism — without
  `MAKEFLAGS=-j1` it ran 8 concurrent `cc1plus` and OOM'd repeatedly.)

- Runtime deps verified present on a provisioned unit: `cv_bridge`,
  `image_transport`, `tf2_geometry_msgs`, boost-filesystem, opencv-4.6,
  ceres/glog/armadillo (all from the stock image). `ldd` on the merged lib
  shows no missing sonames once inside `/opt/qb3rt/install`.
- Tarball is the **stripped** merged-install tree (`ov_core`, `ov_init`,
  `ov_msckf`), ~19 MB. Extracted at `/opt/qb3rt/install` so `lib/` and
  `share/ament_index/` land flat — `$ORIGIN` rpaths resolve against sibling
  ov_* libs only after the merge, not in a side-by-side layout.

## Rebuild

```bash
# on a unit (qb3rt-12 pattern):
mkdir -p ~/ov_ws/src && cd ~/ov_ws
git clone https://github.com/rpng/open_vins.git src/open_vins
cd src/open_vins && git checkout 6948812 && git apply <jazzy_compat.patch>
cd ~/ov_ws
colcon build --packages-up-to ov_msckf --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
    "-DCMAKE_CXX_FLAGS_RELEASE=-O1 -g0 -DNDEBUG" \
    "-DCMAKE_C_FLAGS_RELEASE=-O1 -g0 -DNDEBUG" \
  # (set MAKEFLAGS=-j1 in the environment)
strip/retar as above.
```
