from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Perception: RB3 Gen2 OV9282 mono "tracking" camera via qrb_ros_camera.

    With enable_vio:=true (default) the camera feeds ORB-SLAM3 mono-inertial VIO
    (rb3_vio.launch.py), which publishes /vio/odometry for the robot_localization
    EKF in odometry.launch.py. With enable_vio:=false it runs as a plain mono camera
    (NV12 -> mono8 on /image_raw, no odometry).

    This replaces the previous OAK-D-Lite / depthai_ros_driver_v3 VIO. The IMU that
    the VIO and EKF need (/imu/data) is started separately by base.launch.py
    (rb3_imu_bringup), unchanged.
    """
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    bringup_base = LaunchConfiguration("bringup_base")
    enable_vio = LaunchConfiguration("enable_vio")
    settings_file = LaunchConfiguration("settings_file")
    camera_info_file = LaunchConfiguration("camera_info_file")
    camera_id = LaunchConfiguration("camera_id")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fps = LaunchConfiguration("fps")
    cam_mount_frame = LaunchConfiguration("cam_mount_frame")
    cam_optical_frame = LaunchConfiguration("cam_optical_frame")

    default_settings = PathJoinSubstitution(
        [FindPackageShare("QB3rt"), "config", "orbslam3_ov9282_imu.yaml"]
    )
    default_camera_info = PathJoinSubstitution(
        [FindPackageShare("qrb_ros_camera"), "config", "camera_info_ov9282.yaml"]
    )
    nv12_node = PathJoinSubstitution(
        [FindPackageShare("QB3rt"), "scripts", "nv12_to_mono8.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("bringup_base", default_value="true"),
            DeclareLaunchArgument("enable_vio", default_value="true"),
            DeclareLaunchArgument("settings_file", default_value=default_settings),
            DeclareLaunchArgument("camera_info_file", default_value=default_camera_info),
            # OV9282 mono tracking camera (cameraId 1, native 1280x800).
            # Captured at 1280x720 to match the ov9282 Kalibr calibration (see
            # config/orbslam3_ov9282_imu.yaml); keep in sync with rb3_vio.launch.py.
            DeclareLaunchArgument("camera_id", default_value="1"),
            DeclareLaunchArgument("width", default_value="1280"),
            DeclareLaunchArgument("height", default_value="720"),
            DeclareLaunchArgument("fps", default_value="30"),
            # The tracking camera is rigidly attached to the robot body via the URDF
            # mount frame; the VIO/EKF own odom->base_footprint, so the camera hangs
            # off base_link (no floating VIO TF, matching the old OAK-D handling).
            # tracking_camera_frame is the OV9282's own FLU mount frame (35.3 deg
            # up-tilt in the URDF); main_camera_frame is the OAK-D's.
            DeclareLaunchArgument("cam_mount_frame", default_value="tracking_camera_frame"),
            DeclareLaunchArgument("cam_optical_frame", default_value="tracking_cam_optical"),
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
            # VIO mode: full RB3 mono-inertial chain (camera -> mono8 -> ORB-SLAM3 ->
            # /vio/odometry). rb3_vio.launch.py also publishes the static cam mount TF.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("QB3rt"), "launch", "rb3_vio.launch.py"]
                    )
                ),
                condition=IfCondition(enable_vio),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "settings_file": settings_file,
                    "camera_info_file": camera_info_file,
                    "camera_id": camera_id,
                    "width": width,
                    "height": height,
                    "fps": fps,
                    "cam_mount_frame": cam_mount_frame,
                    "cam_optical_frame": cam_optical_frame,
                }.items(),
            ),
            # Plain camera mode (no VIO): just OV9282 capture + NV12->mono8 on /image_raw.
            ComposableNodeContainer(
                name="rb3_camera_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                condition=UnlessCondition(enable_vio),
                composable_node_descriptions=[
                    ComposableNode(
                        package="qrb_ros_camera",
                        plugin="qrb_ros::camera::CameraNode",
                        name="tracking_camera",
                        parameters=[
                            {
                                "camera_info_path": camera_info_file,
                                "cameraId": camera_id,
                                "width": width,
                                "height": height,
                                "fps": fps,
                                "use_sim_time": use_sim_time,
                            }
                        ],
                        remappings=[("image", "image_nv12")],
                    ),
                ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "python3",
                    nv12_node,
                    "--ros-args",
                    "-p", "input_topic:=image_nv12",
                    "-p", "output_topic:=image_raw",
                    "-p", ["use_sim_time:=", use_sim_time],
                ],
                condition=UnlessCondition(enable_vio),
                output="screen",
            ),
            # Static mount TF for the plain-camera case (VIO case publishes its own in
            # rb3_vio.launch.py). Standard FLU-mount -> optical-axes rotation; the
            # physical tilt lives in the URDF mount joint.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tracking_cam_static_tf",
                condition=UnlessCondition(enable_vio),
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll", "-1.5707963", "--pitch", "0", "--yaw", "-1.5707963",
                    "--frame-id", cam_mount_frame,
                    "--child-frame-id", cam_optical_frame,
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
