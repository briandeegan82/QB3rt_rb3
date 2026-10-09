from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Nav2 in LOCALIZATION (Monte Carlo / AMCL) mode on a saved map.

    Complements slam_toolbox mode: instead of building a map, the robot
    localizes against one saved earlier. AMCL owns map->odom; the EKF still
    owns odom->base_footprint (bringup_base).

      ros2 launch QB3rt localization.launch.py map:=/home/ubuntu/maps/lab.yaml

    Save the map first in mapping mode:
      ros2 run nav2_map_server map_saver_cli -f ~/maps/lab -t /map

    Then seed the pose from RViz ("2D Pose Estimate" -> /initialpose) and drive
    a short arc — AMCL converges within a metre or two of lidar motion.
    """
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    bringup_base = LaunchConfiguration("bringup_base")
    bringup_odometry = LaunchConfiguration("bringup_odometry")
    lidar_start_delay = LaunchConfiguration("lidar_start_delay")
    map_yaml = LaunchConfiguration("map")
    autostart = LaunchConfiguration("autostart")
    # NOTE: unique arg names, NOT "params_file" — include launch_arguments set
    # LaunchConfigurations in the SHARED context, so the first include's
    # params_file= would leak into whatever the second include reads back.
    loc_params = LaunchConfiguration("localization_params_file")
    nav_params = LaunchConfiguration("navigation_params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("bringup_base", default_value="true"),
            # Sensor-fusion EKF (odom->base_footprint) — AMCL needs a live odom
            # source; nav2 navigation needs the wheel driver's /odom + /cmd_vel.
            DeclareLaunchArgument("bringup_odometry", default_value="true"),
            # Same RPLIDAR start delay as lidar_slam (shared USB hub with the
            # rover bridge; a bus reset during bring-up can drop the serial port).
            DeclareLaunchArgument("lidar_start_delay", default_value="5.0"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "localization_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("QB3rt"), "config", "nav2_localize.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "navigation_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("QB3rt"), "config", "nav2.yaml"]
                ),
            ),
            # Robot foundation: description/TF + IMU. Same base as nav.launch.py,
            # but with the WAVE-ROVER driver ON (Nav2 needs /odom and cmd_vel).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("QB3rt"), "launch", "base.launch.py"])
                ),
                condition=IfCondition(bringup_base),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "enable_base_driver": "true",
                }.items(),
            ),
            # Sensor-fusion EKF -> odom->base_footprint (AMCL publishes map->odom).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("QB3rt"), "launch", "odometry.launch.py"])
                ),
                condition=IfCondition(bringup_odometry),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "bringup_base": "false",
                }.items(),
            ),
            # RPLIDAR C1 -> /scan (AMCL's laser input).
            TimerAction(
                period=lidar_start_delay,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [FindPackageShare("QB3rt"), "launch", "rplidar_c1.launch.py"]
                            )
                        ),
                        launch_arguments={
                            "frame_id": "laser_frame",
                            "serial_port": "/dev/rplidar",
                        }.items(),
                    ),
                ],
            ),
            # map_server + AMCL, lifecycle-managed (lifecycle_manager_localization).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("nav2_bringup"), "launch", "localization_launch.py"]
                    )
                ),
                launch_arguments={
                    "namespace": namespace,
                    "map": map_yaml,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": loc_params,
                    "use_composition": "False",
                    # Lidar+camera stack is separate processes on this platform.
                    "container_name": "nav2_localize_container",
                }.items(),
            ),
            # Navigation stack only — identical to live-SLAM mode.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
                    )
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": nav_params,
                    "use_composition": "False",
                    "container_name": "nav2_container",
                }.items(),
            ),
        ]
    )
