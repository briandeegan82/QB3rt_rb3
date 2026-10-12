from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Sensor fusion: robot_localization EKF fusing calibrated wheel odometry
    (/odom, vx only), RF2O laser odometry (/odom_rf2o, x/y/yaw differential),
    ORB-SLAM3 VIO (/vio/odometry, x/y/yaw differential, health-gated in the
    relay) and the RB3 IMU (/imu/data, yaw rate) into a continuous
    odom->base_footprint transform. VIO is fused as odom1; the relay
    (scripts/orbslam3_pose_to_odom.py) suppresses its output while VIO is not
    gravity-aligned, so the EKF coasts on wheel+gyro when VIO is unhealthy
    (see config/ekf.yaml's odom1 block for the history — VIO was removed
    2026-07-13, re-fused health-gated 2026-07-18).

    RF2O (odom2) is started here because the EKF consumes its output: it is a
    prebuilt binary merged into the overlay (no apt package for Jazzy —
    deploy/update_project.sh and the qb3rt_base role install it). Set
    enable_rf2o:=false on a unit missing the binary or whose lidar is absent.

    The other sensor sources are started elsewhere: the RB3 IMU (/imu/data)
    and wheel driver by base.launch.py (which this includes when bringup_base is
    true), the RPLIDAR (/scan) by slam/localization launches, and the camera
    VIO by perception/full_stack. This launch owns the fusion (and, via base,
    the IMU that feeds it)."""
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    bringup_base = LaunchConfiguration("bringup_base")
    enable_base_driver = LaunchConfiguration("enable_base_driver")
    enable_rf2o = LaunchConfiguration("enable_rf2o")
    ekf_config = LaunchConfiguration("ekf_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            # Bring up the robot foundation too. Set false when a parent launch
            # (e.g. full_stack) already starts base.
            DeclareLaunchArgument("bringup_base", default_value="true"),
            # Start the calibrated WAVE-ROVER driver (cmd_vel->motors, /odom)
            # with base. Needed whenever this launch is used standalone for
            # driveable bring-up (mapping, teleop); leave false when the driver
            # is started elsewhere.
            DeclareLaunchArgument("enable_base_driver", default_value="false"),
            # Scan-match odometry (rf2o_laser_odometry) -> /odom_rf2o, fused by
            # the EKF as odom2. Needs /scan from a running RPLIDAR. Disable on
            # units without the binary installed or without a lidar.
            DeclareLaunchArgument("enable_rf2o", default_value="true"),
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
                    "enable_base_driver": enable_base_driver,
                }.items(),
            ),
            Node(
                package="rf2o_laser_odometry",
                executable="rf2o_laser_odometry_node",
                name="rf2o_laser_odometry",
                namespace=namespace,
                output="screen",
                condition=IfCondition(enable_rf2o),
                parameters=[
                    {
                        "laser_scan_topic": "/scan",
                        "odom_topic": "/odom_rf2o",
                        "publish_tf": False,
                        "base_frame_id": "base_footprint",
                        "odom_frame_id": "odom",
                        "freq": 20.0,
                        # rf2o blocks on an initial pose before its first scan
                        # callback; seed from the wheel-model topic (it only
                        # reads the pose once at startup).
                        "init_pose_from_topic": "/odom",
                    }
                ],
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
