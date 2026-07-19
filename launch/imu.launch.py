from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """RB3-platform IMU driver (ICM-42688 via qrb_ros_imu).

    qrb_ros_imu publishes sensor_msgs/Imu on /imu; we remap it to /imu/data so it
    feeds the robot_localization EKF (imu0 in config/ekf.yaml). This is the
    independent gyro the EKF needs for trustworthy yaw: VIO alone under-observes
    rotation (a real turn read ~9x too small without it), so mode:=turn cannot be
    calibrated against the EKF unless this is running.

    Loaded as a composable node in its own container (matching the package's own
    launch) so the qrb zero-copy transport path is available."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    imu_topic = LaunchConfiguration("imu_topic")

    return LaunchDescription(
        [
            # Accepted for compatibility with the rest of the bringup; the qrb IMU
            # container is global (the hardware is a single board sensor).
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/imu/data",
                description="topic to publish IMU on; must match imu0 in ekf.yaml",
            ),
            ComposableNodeContainer(
                name="imu_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="qrb_ros_imu",
                        plugin="qrb_ros::imu::ImuComponent",
                        name="imu",
                        parameters=[{"use_sim_time": use_sim_time}],
                        # qrb publishes the relative name 'imu' (-> /imu); send it
                        # to /imu/data where the EKF is listening.
                        remappings=[("imu", imu_topic)],
                    ),
                ],
                output="screen",
            ),
        ]
    )
