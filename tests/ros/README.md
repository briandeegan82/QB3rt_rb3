# Signal-handling checks (need a ROS 2 Jazzy environment, no hardware)

Run in a stock Jazzy container from the repo root:

```bash
docker run --rm -v "$PWD":/w -w /w ros:jazzy-ros-base bash -c '
  source /opt/ros/jazzy/setup.bash &&
  python3 tests/ros/bridge_shutdown_check.py vendor_overrides/wave_rover_controller/wave_rover_bridge.py &&
  python3 tests/ros/script_signal_check.py scripts/nv12_to_mono8.py scripts/orbslam3_pose_to_odom.py scripts/wait_for_vio_ready.py'
```

- `bridge_shutdown_check.py` runs the rover bridge on a pseudo-terminal, sends
  SIGINT and SIGTERM, and checks it exits 0 and writes a zero-wheel command.
- `script_signal_check.py` checks each node exits 0 without a traceback on
  SIGINT and SIGTERM (systemd and `ros2 launch` escalation both use SIGTERM).
