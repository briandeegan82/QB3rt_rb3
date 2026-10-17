from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """OpenVINS (ov_msckf) mono-inertial VIO — experimental.

    Brings up the full base + camera chain (perception.launch.py with
    enable_vio:=false, i.e. OV9282 -> NV12 -> mono8 on /image_raw, plus the
    RB3 IMU on /imu/data via base.launch.py), then runs ov_msckf on top.

    Publishes /odomimu (+ /poseimu, /pathimu, /points_msckf, /trackhist) in
    the filter's init-anchored world frame — NOT odom-aligned, so it is not
    fused by the EKF. Verified on qb3rt-42 (2026-10-17): ~0.23 m loop
    closure over a 10.6 m carry. See ansible/roles/qb3rt_base/files/
    openvins/README.md for the divergence history and packaging notes.
    """
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    bringup_base = LaunchConfiguration("bringup_base")
    estimator_config = LaunchConfiguration("estimator_config")

    default_config = PathJoinSubstitution(
        [FindPackageShare("QB3rt"), "config", "openvins", "estimator_config.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("bringup_base", default_value="true"),
            DeclareLaunchArgument("estimator_config", default_value=default_config),
            # Base + camera + IMU (everything except a VIO estimator).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("QB3rt"), "launch", "perception.launch.py"]
                    )
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "bringup_base": bringup_base,
                    "enable_vio": "false",
                }.items(),
            ),
            Node(
                package="ov_msckf",
                executable="run_subscribe_msckf",
                name="ov_msckf",
                namespace=namespace,
                arguments=[estimator_config],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )
