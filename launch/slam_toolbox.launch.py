from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Step 2 of the manual bring-up: RPLIDAR C1 + slam_toolbox (online async,
    publishes map->odom), started BY HAND once odometry is healthy.

    Run this only after odometry_bringup.launch.py is up and /odometry/filtered is
    steady while parked (VIO init ritual done). Starting SLAM by hand is the
    simple alternative to the automatic /vio/ready gate: you confirm odom is
    trustworthy, then map, so a warming-up VIO is never mapped.

    Requires the TF chain to already exist (odom->base_footprint from the EKF,
    base_footprint->laser_frame from the URDF) - i.e. odometry_bringup running.

    The RPLIDAR is started here rather than at bring-up on purpose: the OV9282 is
    a CSI/ISP camera (qrb_ros_camera), not the old USB OAK-D, so it does not
    re-enumerate the RPLIDAR's USB bus - the legacy lidar_start_delay workaround
    is unnecessary in this flow."""
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            # Accepted for CLI symmetry with the rest of the bring-up.
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            # RPLIDAR C1. frame_id MUST match the URDF lidar link (laser_frame) or
            # slam_toolbox cannot transform the scan and drops every frame.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("rplidar_ros"), "launch", "rplidar_c1_launch.py"]
                    )
                ),
                launch_arguments={"frame_id": "laser_frame"}.items(),
            ),
            # slam_toolbox online async -> map->odom, with QB3rt's tuned config.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "slam_params_file": PathJoinSubstitution(
                        [FindPackageShare("QB3rt"), "config", "slam_toolbox.yaml"]
                    ),
                }.items(),
            ),
        ]
    )
