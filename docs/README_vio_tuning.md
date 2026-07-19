# QB3rt OAK-D Basalt VIO Tuning (RB3 Gen2)

## Current stable profile
- Basalt VIO + camera streams locked to 12 Hz:
  - config: /usr/share/QB3rt/config/vio.yaml
  - fallback camera profile: /usr/share/QB3rt/config/rtabmap_cam.yaml
- EKF uses RB3 IMU on /imu/data (not /oak/imu/data):
  - config: /usr/share/QB3rt/config/ekf_oak_vio.yaml

## Validation method
Use lightweight cadence topics for source timing:
- /oak/rgb/camera_info
- /oak/stereo/camera_info
- /oak/vio/odometry

Use:
python3 /usr/share/QB3rt/scripts/qb3_rate_probe.py --duration-sec 20

## Notes
- /oak/imu/data remains ~249 Hz by design of OAK stream path.
- Heavy subscribers on both image_raw topics can reduce observed delivery rate.
