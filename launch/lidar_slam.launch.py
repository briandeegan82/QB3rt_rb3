from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


_default_ekf = PathJoinSubstitution([FindPackageShare("QB3rt"), "config", "ekf_imu_only.yaml"])


def generate_launch_description():
    """2D LIDAR SLAM: RPLIDAR C1 + slam_toolbox (publishes map->odom).

    slam_toolbox needs a complete TF chain (map->odom->base_link->laser), so this
    launch also starts base (robot description/TF) and odometry (the EKF that
    supplies odom->base_link). By default perception is OFF (enable_perception:=false)
    so this is a camera-free lidar-only stack; set enable_perception:=true to also
    start the ORB-SLAM3 VIO chain (rb3_vio) and give the EKF a visual odometry source.
    When composed by full_stack, set bringup_base:=false, bringup_odometry:=false and
    enable_perception:=false so none of them are started twice.

    gate_slam_on_vio (default false here, true from full_stack): when true,
    slam_toolbox is held until the ORB-SLAM3 VIO relay latches /vio/ready (VIBA
    calibration done), via wait_for_vio_ready.py + an OnProcessExit handler, so the
    map isn't built from the start-up VIO init ritual. slam_gate_timeout bounds the
    wait so a skipped ritual still yields a map. See _slam_toolbox_actions."""
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    bringup_base = LaunchConfiguration("bringup_base")
    enable_base_driver = LaunchConfiguration("enable_base_driver")
    bringup_odometry = LaunchConfiguration("bringup_odometry")
    ekf_config = LaunchConfiguration("ekf_config")
    enable_perception = LaunchConfiguration("enable_perception")
    enable_vio = LaunchConfiguration("enable_vio")
    lidar_start_delay = LaunchConfiguration("lidar_start_delay")
    gate_slam_on_vio = LaunchConfiguration("gate_slam_on_vio")
    slam_gate_timeout = LaunchConfiguration("slam_gate_timeout")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("bringup_base", default_value="true"),
            # Start the calibrated WAVE-ROVER driver (cmd_vel->motors, wheel /odom)
            # with base. On by default so SLAM-only bring-up can drive (teleop/Nav2)
            # and the EKF gets its wheel-vx (odom1) input. full_stack passes
            # bringup_base:=false here and owns the driver itself, so it isn't
            # double-started.
            DeclareLaunchArgument("enable_base_driver", default_value="true"),
            DeclareLaunchArgument("bringup_odometry", default_value="true"),
            DeclareLaunchArgument("ekf_config", default_value=_default_ekf),
            # Start the ORB-SLAM3 VIO chain so the EKF has visual odometry input.
            # Off by default: lidar_slam is a camera-free stack (use lidar_nav.launch.py
            # for the full lidar+nav2 stack, or full_stack for VIO+lidar+nav2).
            DeclareLaunchArgument("enable_perception", default_value="false"),
            # When perception is on, run the camera in VIO mode (/oak/vio/odometry).
            DeclareLaunchArgument("enable_vio", default_value="true"),
            # WORKAROUND: the RPLIDAR (CP2102N UART bridge) and the OAK-D share a
            # USB hub. When the OAK-D boots it re-enumerates on USB and resets the
            # hub, which drops the lidar's serial bridge and stops its motor. Delay
            # the lidar start until the OAK-D has finished booting and the bus has
            # settled so the lidar opens its port on a stable bus. Set to 0 once the
            # lidar is on its own USB bus / a powered hub.
            DeclareLaunchArgument("lidar_start_delay", default_value="20.0"),
            # Hold slam_toolbox until ORB-SLAM3 VIO has finished calibrating
            # (latched /vio/ready from the pose relay), so it doesn't map the
            # start-up VIO init ritual. OFF by default here: lidar_slam standalone
            # is camera-free (no /vio/ready publisher), so gating would just wait
            # the full timeout. full_stack (which always runs VIO) passes true.
            DeclareLaunchArgument("gate_slam_on_vio", default_value="false"),
            # Fallback: start slam_toolbox anyway this many seconds after launch
            # if VIO never reports ready (skipped/failed init ritual).
            DeclareLaunchArgument("slam_gate_timeout", default_value="60.0"),
            # Robot description / TF (base_link->laser, etc.).
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
            # Sensor-fusion EKF -> odom->base_link (base already started above).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("QB3rt"), "launch", "odometry.launch.py"])
                ),
                condition=IfCondition(bringup_odometry),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "bringup_base": "false",
                    "ekf_config": ekf_config,
                }.items(),
            ),
            # Perception (OAK-D VIO) -> /oak/vio/odometry feeds the EKF above.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("QB3rt"), "launch", "perception.launch.py"])
                ),
                condition=IfCondition(enable_perception),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "bringup_base": "false",
                    "enable_vio": enable_vio,
                }.items(),
            ),
            # RPLIDAR C1 laser scanner. Started after lidar_start_delay seconds so
            # the OAK-D's USB re-enumeration (above) doesn't reset the shared hub
            # out from under the lidar's serial bridge and stop its motor.
            # frame_id must match the lidar link in the URDF (base_link->laser_frame),
            # otherwise slam_toolbox can't transform the scan and drops every frame.
            TimerAction(
                period=lidar_start_delay,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [FindPackageShare("rplidar_ros"), "launch", "rplidar_c1_launch.py"]
                            )
                        ),
                        launch_arguments={
                            "frame_id": "laser_frame",
                            "serial_port": "/dev/rplidar",
                        }.items(),
                    ),
                ],
            ),
        ]
        + _slam_toolbox_actions(use_sim_time, gate_slam_on_vio, slam_gate_timeout)
    )


def _slam_toolbox_actions(use_sim_time, gate_slam_on_vio, slam_gate_timeout):
    """slam_toolbox (online async, publishes map->odom), optionally VIO-gated.

    When gate_slam_on_vio is true, a wait_for_vio_ready.py process blocks until
    the latched /vio/ready goes true (or its timeout elapses) and slam_toolbox is
    started from that process's OnProcessExit handler. When false, slam_toolbox
    starts immediately (the original behavior). Both paths use the same include;
    IfCondition/UnlessCondition select which one is live at runtime.
    """
    def _slam_include(condition=None):
        return IncludeLaunchDescription(
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
            condition=condition,
        )

    # Ungated path: start slam_toolbox directly (gate_slam_on_vio false).
    slam_direct = _slam_include(condition=UnlessCondition(gate_slam_on_vio))

    # Gated path: the gate process only starts under IfCondition, so its
    # OnProcessExit handler (and thus slam_gated) only ever fires when gating is
    # on. slam_gated has no condition of its own - it is reached solely via the
    # handler, never started standalone.
    gate = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution(
                [FindPackageShare("QB3rt"), "scripts", "wait_for_vio_ready.py"]
            ),
            "--ros-args",
            "-p", ["timeout:=", slam_gate_timeout],
            "-p", ["use_sim_time:=", use_sim_time],
        ],
        condition=IfCondition(gate_slam_on_vio),
        output="screen",
    )
    slam_gated = _slam_include()

    return [
        slam_direct,
        gate,
        RegisterEventHandler(
            OnProcessExit(target_action=gate, on_exit=[slam_gated])
        ),
    ]
