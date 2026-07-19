from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """RGB-D SLAM nav stack: OAK-D Basalt VIO + OAK-D RGBD + rtabmap + Nav2. No lidar.

    Stack:
      base        robot description/TF + WAVE-ROVER bridge + IMU
      oak_driver  OAK-D depthai driver in VIO+RGBD mode (vio.yaml)
                    -> /oak/vio/odometry  (Basalt stereo VIO)
                    -> /oak/rgb/image_raw + /oak/stereo/image_raw  (RGBD)
      odometry    robot_localization EKF fusing OAK-D VIO + RB3 IMU -> odom->base_footprint
      rtabmap     RGBD loop-closure SLAM (odom from EKF) -> map->odom TF + /map
      nav         Nav2 navigation-only (rtabmap owns map and map->odom)

    The OAK-D is the sole camera: its on-device Basalt VIO feeds the EKF
    (/oak/vio/odometry -> ekf_oak_vio.yaml -> /odometry/filtered) and its
    RGB+Depth feeds rtabmap for loop-closure SLAM. rtabmap uses /odometry/filtered
    (not raw VIO) so it always has odometry even during brief VIO dropouts.

    The OAK-D subtree is grafted onto the robot body via a static TF:
      main_camera_frame -> oak_parent_frame

    To revert to ORB-SLAM3 VIO, override ekf_config and cam_params_file:
      ros2 launch QB3rt rtabmap_nav.launch.py \\
          cam_params_file:=<path>/config/rtabmap_cam.yaml \\
          ekf_config:=<path>/config/ekf.yaml

    NOTE: vio.yaml enables on-device Basalt VIO alongside RGBD. If the OAK-D-LITE
    pipeline builder exits -11, remove the stereo/rgb i_synced entries from vio.yaml
    first (the pipeline may publish them by default).

    Run:
      ros2 launch QB3rt rtabmap_nav.launch.py use_rviz:=true
    """
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_base_driver = LaunchConfiguration("enable_base_driver")
    params_file = LaunchConfiguration("params_file")
    cam_params_file = LaunchConfiguration("cam_params_file")
    rtabmap_params_file = LaunchConfiguration("rtabmap_params_file")
    ekf_config = LaunchConfiguration("ekf_config")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    common = {"namespace": namespace, "use_sim_time": use_sim_time}

    default_params = PathJoinSubstitution(
        [FindPackageShare("QB3rt"), "config", "nav2.yaml"]
    )
    default_cam_params = PathJoinSubstitution(
        [FindPackageShare("QB3rt"), "config", "vio.yaml"]
    )
    default_rtabmap_params = PathJoinSubstitution(
        [FindPackageShare("QB3rt"), "config", "rtabmap_slam.yaml"]
    )
    default_ekf = PathJoinSubstitution(
        [FindPackageShare("QB3rt"), "config", "ekf_oak_vio.yaml"]
    )
    default_rviz = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"]
    )
    oak_vio_relay = PathJoinSubstitution(
        [FindPackageShare("QB3rt"), "scripts", "oak_vio_relay.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_base_driver", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            # vio.yaml by default (OAK-D VIO+RGBD). Override with rtabmap_cam.yaml
            # to revert to plain RGBD + ORB-SLAM3 VIO.
            DeclareLaunchArgument("cam_params_file", default_value=default_cam_params),
            DeclareLaunchArgument("rtabmap_params_file", default_value=default_rtabmap_params),
            # OAK-D Basalt VIO EKF by default. Override with ekf.yaml for ORB-SLAM3.
            DeclareLaunchArgument("ekf_config", default_value=default_ekf),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            # OAK VIO forward axis (see oak_vio_relay.py / vio_axis_probe.py).
            # OAK VIO frame is Y-down; forward is on X (push test: X/Z swapped vs
            # optical). Flip to -x if a forward push reads negative on X.
            DeclareLaunchArgument("vio_forward_axis", default_value="+x"),
            # Foundation: robot description/TF + WAVE-ROVER bridge + IMU.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("QB3rt"), "launch", "base.launch.py"])
                ),
                launch_arguments={**common, "enable_base_driver": enable_base_driver}.items(),
            ),
            # OAK-D driver: VIO+RGBD mode. Publishes:
            #   /oak/vio/odometry          -> EKF odom0
            #   /oak/rgb/image_raw         -> rtabmap rgb/image
            #   /oak/stereo/image_raw      -> rtabmap depth/image
            # Scoped: the depthai driver sets the global `use_composition` launch
            # config to "true", which otherwise leaks into nav2's navigation_launch.py
            # and causes a NameError (eval("not true")).
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [FindPackageShare("depthai_ros_driver_v3"), "launch", "driver.launch.py"]
                            )
                        ),
                        launch_arguments={
                            "namespace": namespace,
                            "params_file": cam_params_file,
                        }.items(),
                    ),
                ],
            ),
            # Static TF: graft the OAK-D subtree (rooted at oak_parent_frame) onto
            # the robot body. The OAK-D's own TF broadcaster is disabled via
            # vio.i_publish_tf: false in vio.yaml so the EKF solely owns
            # odom->base_footprint.
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
            # Covariance relay: Basalt VIO publishes all-zero covariance, which
            # causes robot_localization to give it infinite weight (K->1, no
            # smoothing). This relay inserts a diagonal covariance before the EKF.
            # The EKF (ekf_oak_vio.yaml) subscribes to /oak/vio/odometry_cov.
            # Launched via ExecuteProcess (not Node) because QB3rt is a share-only
            # package with no libexec directory.
            ExecuteProcess(
                cmd=[
                    "python3", oak_vio_relay,
                    "--ros-args",
                    "-p", ["use_sim_time:=", use_sim_time],
                    # OAK VIO frame is Y-down (optical); forward axis confirmed by
                    # scripts/vio_axis_probe.py + a hand-push. Override if the push
                    # test shows forward is not +z:  vio_forward_axis:=-z|+x|-x
                    "-p", ["forward_axis:=", LaunchConfiguration("vio_forward_axis")],
                ],
                output="screen",
            ),
            # Sensor fusion: OAK-D Basalt VIO (/oak/vio/odometry_cov) + RB3 IMU ->
            # odom->base_footprint TF on /odometry/filtered.
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
            # rtabmap SLAM: OAK-D RGB+Depth + EKF odometry -> loop closure ->
            # map->odom TF + /map.
            #
            # odom is /odometry/filtered (EKF output) so rtabmap always has odometry
            # to sync against. Raw /oak/vio/odometry is only published while Basalt is
            # tracking; /odometry/filtered is always published (holds last-known x,y
            # from IMU yaw rate when VIO drops out), keeping rtabmap's sync alive.
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=[rtabmap_params_file, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("rgb/image", "/oak/rgb/image_raw"),
                    ("rgb/camera_info", "/oak/rgb/camera_info"),
                    ("depth/image", "/oak/stereo/image_raw"),
                    ("odom", "/odometry/filtered"),
                ],
            ),
            # Navigation (Nav2 navigation-only; rtabmap provides map->odom and /map).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("QB3rt"), "launch", "nav.launch.py"])
                ),
                launch_arguments={
                    **common,
                    "bringup_base": "false",
                    "params_file": params_file,
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
