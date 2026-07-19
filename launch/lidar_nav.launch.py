from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """LIDAR-only nav stack: RPLIDAR + slam_toolbox + Nav2. No camera.

    Stack:
      base        robot description/TF + WAVE-ROVER bridge
      odometry    robot_localization EKF (IMU yaw rate only; ekf_imu_only.yaml)
      lidar_slam  RPLIDAR C1 + slam_toolbox -> map->odom
      nav         Nav2 navigation-only (slam_toolbox owns map and map->odom)

    Run:
      ros2 launch QB3rt lidar_nav.launch.py use_rviz:=true
    """
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_base_driver = LaunchConfiguration("enable_base_driver")
    params_file = LaunchConfiguration("params_file")
    ekf_config = LaunchConfiguration("ekf_config")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    common = {"namespace": namespace, "use_sim_time": use_sim_time}

    default_params = PathJoinSubstitution(
        [FindPackageShare("QB3rt"), "config", "nav2.yaml"]
    )
    default_ekf = PathJoinSubstitution(
        [FindPackageShare("QB3rt"), "config", "ekf_imu_only.yaml"]
    )
    default_rviz = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_base_driver", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            # IMU-only EKF by default (no VIO in this stack). Override with
            # ekf.yaml to re-enable VIO if you attach the camera later.
            DeclareLaunchArgument("ekf_config", default_value=default_ekf),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            # Foundation: robot description/TF + WAVE-ROVER bridge.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("QB3rt"), "launch", "base.launch.py"])
                ),
                launch_arguments={**common, "enable_base_driver": enable_base_driver}.items(),
            ),
            # Sensor fusion: IMU yaw rate -> odom->base_footprint.
            # No VIO in this path; slam_toolbox corrects drift via map->odom.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("QB3rt"), "launch", "odometry.launch.py"])
                ),
                launch_arguments={
                    **common,
                    "bringup_base": "false",
                    "ekf_config": ekf_config,
                }.items(),
            ),
            # LIDAR SLAM: RPLIDAR C1 + slam_toolbox -> map->odom.
            # base and odometry are already started; skip them inside lidar_slam.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("QB3rt"), "launch", "lidar_slam.launch.py"])
                ),
                launch_arguments={
                    **common,
                    "bringup_base": "false",
                    "bringup_odometry": "false",
                    "enable_perception": "false",
                }.items(),
            ),
            # Navigation (Nav2 navigation-only; slam_toolbox provides map->odom).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("QB3rt"), "launch", "nav.launch.py"])
                ),
                launch_arguments={
                    **common,
                    "bringup_base": "false",
                    "params_file": params_file,
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
