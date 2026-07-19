from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Robot foundation: description/TF (robot_state_publisher) plus the optional
    WAVE-ROVER base driver. Higher-level launches (odometry/slam/nav/perception/
    full_stack) build on top of this."""
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_base_driver = LaunchConfiguration("enable_base_driver")
    enable_imu = LaunchConfiguration("enable_imu")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            # WAVE-ROVER locomotion driver (cmd_vel -> motors, wheel /odom).
            # Off by default until the controller bringup is finalized; turn on
            # with enable_base_driver:=true.
            DeclareLaunchArgument("enable_base_driver", default_value="false"),
            # RB3 board IMU (qrb_ros_imu -> /imu/data). On by default: it is the
            # EKF's independent gyro (imu0 in ekf.yaml) and is required for
            # trustworthy yaw. Set enable_imu:=false on hardware without it.
            DeclareLaunchArgument("enable_imu", default_value="true"),
            # Robot description + TF.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("QB3rt"), "launch", "static_transforms.launch.py"]
                    )
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            # WAVE-ROVER base driver: the calibrated HTTP bridge (cmd_vel -> motors,
            # wheel /odom). This is the driver that carries the arc-turn calibration
            # (spin_boost, straight_trim, motor_deadband) in wave_rover_bridge.yaml,
            # so Nav2's /cmd_vel is executed with the tuned kinematics. Its publish_tf
            # is set false there so the EKF (odometry.launch.py) solely owns
            # odom->base_footprint.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("wave_rover_controller"), "launch", "wave_rover_bridge_launch.py"]
                    )
                ),
                condition=IfCondition(enable_base_driver),
            ),
            # RB3 board IMU -> /imu/data (the EKF's imu0 gyro).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("QB3rt"), "launch", "imu.launch.py"]
                    )
                ),
                condition=IfCondition(enable_imu),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
        ]
    )
