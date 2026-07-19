from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_base_driver = LaunchConfiguration("enable_base_driver")
    cam_params_file = LaunchConfiguration("cam_params_file")
    ekf_config = LaunchConfiguration("ekf_config")
    vio_forward_axis = LaunchConfiguration("vio_forward_axis")
    start_imu = LaunchConfiguration("start_imu")

    default_cam_params = PathJoinSubstitution([FindPackageShare("QB3rt"), "config", "vio.yaml"])
    default_ekf = PathJoinSubstitution([FindPackageShare("QB3rt"), "config", "ekf_oak_vio.yaml"])
    oak_vio_relay = PathJoinSubstitution([FindPackageShare("QB3rt"), "scripts", "oak_vio_relay.py"])

    common = {"namespace": namespace, "use_sim_time": use_sim_time}

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enable_base_driver", default_value="true"),
        DeclareLaunchArgument("start_imu", default_value="true"),
        DeclareLaunchArgument("cam_params_file", default_value=default_cam_params),
        DeclareLaunchArgument("ekf_config", default_value=default_ekf),
        DeclareLaunchArgument("vio_forward_axis", default_value="+x"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("QB3rt"), "launch", "base.launch.py"])
            ),
            launch_arguments={**common, "enable_base_driver": enable_base_driver}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("QB3rt"), "launch", "imu.launch.py"])
            ),
            launch_arguments={
                "namespace": namespace,
                "use_sim_time": use_sim_time,
                "imu_topic": "/imu/data",
            }.items(),
        ),

        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([FindPackageShare("depthai_ros_driver_v3"), "launch", "driver.launch.py"])
                    ),
                    launch_arguments={
                        "namespace": namespace,
                        "params_file": cam_params_file,
                    }.items(),
                ),
            ],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="oak_mount_static_tf",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "main_camera_frame",
                "--child-frame-id", "oak_parent_frame",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        ExecuteProcess(
            cmd=[
                "python3", oak_vio_relay,
                "--ros-args",
                "-p", ["use_sim_time:=", use_sim_time],
                "-p", ["forward_axis:=", vio_forward_axis],
            ],
            output="screen",
        ),

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
    ])
