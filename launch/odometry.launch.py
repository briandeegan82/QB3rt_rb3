from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Sensor fusion: robot_localization EKF fusing calibrated wheel odometry
    (/odom, vx only), ORB-SLAM3 VIO (/vio/odometry, x/y/yaw differential,
    health-gated in the relay) and the RB3 IMU (/imu/data, yaw rate) into a
    continuous odom->base_footprint transform. VIO is fused as odom1; the relay
    (scripts/orbslam3_pose_to_odom.py) suppresses its output while VIO is not
    gravity-aligned, so the EKF coasts on wheel+gyro when VIO is unhealthy
    (see config/ekf.yaml's odom1 block for the history — VIO was removed
    2026-07-13, re-fused health-gated 2026-07-18).

    The sensor sources themselves are started elsewhere: the RB3 IMU (/imu/data)
    and wheel driver by base.launch.py (which this includes when bringup_base is
    true), and the camera VIO by perception/full_stack. This launch owns the
    fusion (and, via base, the IMU that feeds it)."""
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    bringup_base = LaunchConfiguration("bringup_base")
    ekf_config = LaunchConfiguration("ekf_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            # Bring up the robot foundation too. Set false when a parent launch
            # (e.g. full_stack) already starts base.
            DeclareLaunchArgument("bringup_base", default_value="true"),
            DeclareLaunchArgument(
                "ekf_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("QB3rt"), "config", "ekf.yaml"]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("QB3rt"), "launch", "base.launch.py"])
                ),
                condition=IfCondition(bringup_base),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                namespace=namespace,
                output="screen",
                parameters=[ekf_config, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
