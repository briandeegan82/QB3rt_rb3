"""Launch the open-loop odometry test / drive calibration.

By default this runs ONLY the test node (assumes the robot stack is already up).
Pass bringup:=true to also start the bridge, robot_state_publisher /
joint_state_publisher, the EKF and the OAK-D VIO.

Examples:
  ros2 launch QB3rt odom_square_test.launch.py mode:=square    bringup:=true
  ros2 launch QB3rt odom_square_test.launch.py mode:=straight  bringup:=true \
      linear_speed:=0.42 side_length:=12.5
  ros2 launch QB3rt odom_square_test.launch.py mode:=deadband  bringup:=true
  ros2 launch QB3rt odom_square_test.launch.py mode:=speed     bringup:=true
  ros2 launch QB3rt odom_square_test.launch.py mode:=turn      bringup:=true
  ros2 launch QB3rt odom_square_test.launch.py mode:=trim      bringup:=true \
      linear_speed:=0.30 side_length:=3.0   # (repeat at a 2nd speed for the slope)
  ros2 launch QB3rt odom_square_test.launch.py mode:=vio_check bringup:=true
  ros2 launch QB3rt odom_square_test.launch.py mode:=turn_check bringup:=true
  ros2 launch QB3rt odom_square_test.launch.py mode:=oneside   bringup:=true \
      oneside_side:=left oneside_duration:=2.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


# Test-node parameters exposed as launch arguments (name -> default).
TEST_ARGS = {
    "mode": "square",
    "cmd_vel_topic": "/cmd_vel",
    "side_length": "1.0",
    "num_sides": "4",
    "linear_speed": "0.15",
    "angular_speed": "0.4",
    "turn_linear_speed": "0.2",
    "turn_angle": "1.5708",
    "publish_rate": "20.0",
    "settle_time": "1.5",
    "direction": "cw",
    # MUST match wave_rover_bridge.yaml max_speed (0.56 = serial-transport
    # tape recalibration 2026-07-13; the old 0.42 here silently overrode the
    # updated config/odom_square_test.yaml and mis-scaled the oneside test).
    "driver_max_speed": "0.56",
    "driver_spin_boost_max": "10.0",
    "driver_spin_boost_k": "1.4",
    "driver_straight_trim": "0.12",
    "driver_node": "waverover_bridge",
    "deadband_start": "0.0",
    "deadband_step": "0.02",
    "deadband_max": "0.6",
    "step_hold": "2.5",
    "move_threshold": "0.02",
    "speed_start": "0.25",
    "speed_step": "0.25",
    "speed_max": "1.0",
    "speed_hold": "3.5",
    "accel_skip": "0.8",
    "speed_alternate": "false",
    "turn_target": "6.2832",
    "turn_settle": "1.0",
    "oneside_side": "left",
    "oneside_duration": "2.0",
    "driver_track_width": "0.15",
}


def generate_launch_description():
    pkg = FindPackageShare("QB3rt")
    bringup = LaunchConfiguration("bringup")

    default_output_dir = PathJoinSubstitution([pkg, "results"])
    script = PathJoinSubstitution([pkg, "scripts", "odom_square_test.py"])
    config = PathJoinSubstitution([pkg, "config", "odom_square_test.yaml"])

    declared = [
        DeclareLaunchArgument("bringup", default_value="false",
                              description="also start bridge + RSP/JSP + EKF + OAK-D VIO"),
        DeclareLaunchArgument("output_dir", default_value=default_output_dir),
    ]
    declared += [DeclareLaunchArgument(name, default_value=default)
                 for name, default in TEST_ARGS.items()]

    # cmd: python3 odom_square_test.py --ros-args --params-file <yaml> -p <overrides>
    cmd = ["python3", script, "--ros-args", "--params-file", config,
           "-p", ["output_dir:=", LaunchConfiguration("output_dir")]]
    for name in TEST_ARGS:
        cmd += ["-p", [f"{name}:=", LaunchConfiguration(name)]]

    test_proc = ExecuteProcess(cmd=cmd, output="screen")

    # --- Optional robot bringup (order -> bridge, RSP, JSP, EKF, OAK-D) ---
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wave_rover_controller"), "launch", "wave_rover_bridge_launch.py"]
            )
        ),
        condition=IfCondition(bringup),
    )
    odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "odometry.launch.py"])
        ),
        condition=IfCondition(bringup),
        launch_arguments={"bringup_base": "true"}.items(),
    )
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "perception.launch.py"])
        ),
        condition=IfCondition(bringup),
        launch_arguments={"bringup_base": "false", "enable_vio": "true"}.items(),
    )

    return LaunchDescription(declared + [bridge, odometry, perception, test_proc])
