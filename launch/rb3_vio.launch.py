from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """RB3-native mono-inertial VIO: OV9282 tracking camera + onboard IMU -> ORB-SLAM3.

    Replaces the OAK-D on-device Basalt VIO. Pipeline:

      qrb_ros_camera (cameraId 1, OV9282, HW ISP, NV12, zero-copy dmabuf)
        -> image_nv12
      nv12_to_mono8 (luma-plane crop)
        -> image_raw (mono8)
      orb_slam3_ros_mono_imu  (image_raw + imu:=/imu/data)
        -> camera_pose (PoseStamped)
      orbslam3_pose_to_odom (optical->REP103 + covariance)
        -> /vio/odometry (nav_msgs/Odometry)  -> robot_localization EKF (odom0)

    The IMU itself (/imu/data) is provided elsewhere (rb3_imu_bringup/imu.launch.py,
    pulled in by base.launch.py), exactly as before; this launch only consumes it.

    Qualcomm HW acceleration: camera capture runs on the Spectra HW ISP via
    qrb_ros_camera with zero-copy transport. ORB-SLAM3 feature extraction is CPU-bound
    (cannot be offloaded without recompiling ORB-SLAM3), so the heavy lifting stays on
    the CPU; the cheap NV12->mono8 step is a luma-plane copy.
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    camera_id = LaunchConfiguration("camera_id")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fps = LaunchConfiguration("fps")
    imu_topic = LaunchConfiguration("imu_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    settings_file = LaunchConfiguration("settings_file")
    voc_file = LaunchConfiguration("voc_file")
    camera_info_file = LaunchConfiguration("camera_info_file")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    cam_mount_frame = LaunchConfiguration("cam_mount_frame")
    cam_optical_frame = LaunchConfiguration("cam_optical_frame")
    show_viewer = LaunchConfiguration("show_viewer")

    default_settings = PathJoinSubstitution(
        [FindPackageShare("QB3rt"), "config", "orbslam3_ov9282_imu.yaml"]
    )
    default_camera_info = PathJoinSubstitution(
        [FindPackageShare("qrb_ros_camera"), "config", "camera_info_ov9282.yaml"]
    )
    nv12_node = PathJoinSubstitution(
        [FindPackageShare("QB3rt"), "scripts", "nv12_to_mono8.py"]
    )
    pose_bridge = PathJoinSubstitution(
        [FindPackageShare("QB3rt"), "scripts", "orbslam3_pose_to_odom.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            # OV9282 = the RB3 Gen2 mono "tracking" camera (cameraId 1, native 1280x800).
            # Captured at 1280x720 to match the Kalibr calibration deployed in
            # config/orbslam3_ov9282_imu.yaml (intrinsics/extrinsics are 720-specific).
            DeclareLaunchArgument("camera_id", default_value="1"),
            DeclareLaunchArgument("width", default_value="1280"),
            DeclareLaunchArgument("height", default_value="720"),
            DeclareLaunchArgument("fps", default_value="30"),
            # IMU shared with the EKF (published by rb3_imu_bringup, remapped to /imu/data).
            DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
            # Kalibr timeshift_cam_imu for the OV9282 rig (t_imu = t_cam + shift):
            # applied to the image stamps by nv12_to_mono8 because stock
            # ORB-SLAM3 has no time-offset field. Without it the ~20 ms skew
            # makes mono-inertial init fragile (initializes under motion, then
            # immediately loses tracking). Set 0.0 to disable; re-measure with
            # Kalibr if the camera/IMU mounting changes.
            DeclareLaunchArgument("image_stamp_offset", default_value="-0.0195"),
            DeclareLaunchArgument("odom_topic", default_value="/vio/odometry"),
            DeclareLaunchArgument("settings_file", default_value=default_settings),
            DeclareLaunchArgument(
                "voc_file",
                default_value="/usr/share/orb_slam3_ros/Vocabulary/ORBvoc.txt",
            ),
            DeclareLaunchArgument("camera_info_file", default_value=default_camera_info),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            # Where the camera is rigidly mounted on the robot (URDF frame).
            # tracking_camera_frame is the OV9282's own FLU mount frame (with
            # the 35.3 deg up-tilt baked into the URDF); main_camera_frame is
            # the OAK-D and was the wrong parent for this camera.
            DeclareLaunchArgument("cam_mount_frame", default_value="tracking_camera_frame"),
            DeclareLaunchArgument("cam_optical_frame", default_value="tracking_cam_optical"),
            # Pangolin GUI off on the headless robot.
            DeclareLaunchArgument("show_viewer", default_value="false"),

            # 1) OV9282 capture on the HW ISP. Publishes NV12 on image_nv12 (+ camera_info).
            ComposableNodeContainer(
                name="rb3_camera_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
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

            # 2) NV12 -> mono8 (luma-plane crop) -> image_raw.
            ExecuteProcess(
                cmd=[
                    "python3",
                    nv12_node,
                    "--ros-args",
                    "-p", "input_topic:=image_nv12",
                    "-p", "output_topic:=image_raw",
                    "-p", ["stamp_offset:=", LaunchConfiguration("image_stamp_offset")],
                    "-p", ["use_sim_time:=", use_sim_time],
                ],
                output="screen",
            ),

            # 3) ORB-SLAM3 mono-inertial. Subscribes image_raw + imu(->/imu/data),
            #    publishes camera_pose (PoseStamped) in the SLAM world frame.
            Node(
                package="orb_slam3_ros",
                executable="orb_slam3_ros_mono_imu",
                name="orb_slam3_mono_imu",
                output="screen",
                parameters=[
                    {
                        "voc_file": voc_file,
                        "settings_file": settings_file,
                        "world_frame_id": odom_frame,
                        "ignore_imu_header": False,
                        "show_viewer": show_viewer,
                        "use_sim_time": use_sim_time,
                    }
                ],
                remappings=[("imu", imu_topic)],
            ),

            # 4) camera_pose (PoseStamped) -> /vio/odometry (Odometry) for the EKF.
            ExecuteProcess(
                cmd=[
                    "python3",
                    pose_bridge,
                    "--ros-args",
                    "-p", "input_topic:=camera_pose",
                    "-p", ["output_topic:=", odom_topic],
                    "-p", ["odom_frame_id:=", odom_frame],
                    "-p", ["child_frame_id:=", base_frame],
                    "-p", "optical_to_ros:=true",
                    "-p", ["use_sim_time:=", use_sim_time],
                ],
                output="screen",
            ),

            # Static mount TF: rigidly hang the tracking camera's optical frame off the
            # robot body (URDF cam_mount_frame). ORB-SLAM3 publishes no TF and the EKF
            # gets odometry directly, so this is for RViz/any TF-based consumer.
            # Standard FLU-mount -> optical-axes rotation (optical z = mount x,
            # optical x = -mount y, optical y = -mount z); the physical tilt lives
            # in the URDF mount joint, not here.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tracking_cam_static_tf",
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
