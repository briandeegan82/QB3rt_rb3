from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _inc(name, launch_arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("QB3rt"), "launch", name])
        ),
        launch_arguments=launch_arguments.items(),
    )


def generate_launch_description():
    """Step 1 of the manual bring-up: robot foundation + VIO + sensor-fusion EKF,
    but deliberately NO SLAM and NO Nav2.

    The manual two-step workflow (the operator is the gate, replacing the
    automatic /vio/ready gate):

      1. ros2 launch QB3rt odometry_bringup.launch.py
         Wait for /odometry/filtered, run the ORB-SLAM3 figure-8 init ritual,
         then STAND STILL until VIO is trusted: /vio/odometry steady and
         /odometry/filtered twist ~0 while parked (relay logs
         "VIO metric convergence confirmed").
      2. ros2 launch QB3rt slam_toolbox.launch.py
         Starts the RPLIDAR + slam_toolbox (map->odom) only once you have
         confirmed odometry is healthy, so a warming-up VIO can never be baked
         into the map.

    Starts exactly once: base (description/TF + WAVE-ROVER driver + RB3 IMU),
    perception (OV9282 -> ORB-SLAM3 VIO -> /vio/odometry) and the EKF
    (robot_localization -> odom->base_footprint). Nav2 runs on the laptop and is
    not started here."""
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_base_driver = LaunchConfiguration("enable_base_driver")
    enable_vio = LaunchConfiguration("enable_vio")

    common = {"namespace": namespace, "use_sim_time": use_sim_time}

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            # Wheels must move for teleop / laptop Nav2, and their vx feeds the
            # EKF (and the metric-convergence gate needs /odom), so on by default.
            DeclareLaunchArgument("enable_base_driver", default_value="true"),
            # ORB-SLAM3 VIO on by default (the EKF's odom1). Set false for a
            # wheel+gyro-only odom (pair with ekf_config:=ekf_imu_only.yaml).
            DeclareLaunchArgument("enable_vio", default_value="true"),
            # Foundation: robot description/TF + WAVE-ROVER driver + RB3 IMU.
            _inc(
                "base.launch.py",
                {**common, "enable_base_driver": enable_base_driver},
            ),
            # Perception: OV9282 -> ORB-SLAM3 VIO -> /vio/odometry (base already up).
            _inc(
                "perception.launch.py",
                {**common, "bringup_base": "false", "enable_vio": enable_vio},
            ),
            # Sensor fusion: robot_localization EKF -> odom->base_footprint.
            _inc(
                "odometry.launch.py",
                {**common, "bringup_base": "false"},
            ),
        ]
    )
