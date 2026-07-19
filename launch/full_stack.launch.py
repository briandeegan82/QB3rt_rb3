from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _qb3rt(name, launch_arguments, scoped=False, condition=None):
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("QB3rt"), "launch", name])
        ),
        launch_arguments=launch_arguments.items(),
        condition=condition,
    )
    if not scoped:
        return include
    # Scope ONLY the leaky sub-launch so the launch configurations it sets
    # internally do not leak into sibling sub-launches. perception pulls in the
    # depthai driver, which sets the global `use_composition` config to the
    # lowercase string "true"; without this isolation nav2's navigation_launch.py
    # reads that leaked value into PythonExpression(["not ", use_composition]) ->
    # eval("not true") -> NameError: name 'true' is not defined, aborting the
    # whole stack. We deliberately do NOT scope every include: scoping a launch
    # that resolves a LaunchConfiguration lazily inside a TimerAction (e.g.
    # lidar_slam's `lidar_start_delay`) pops the config before the timer fires,
    # so the RPLIDAR would never start.
    return GroupAction(scoped=True, actions=[include])


def generate_launch_description():
    """Full robot stack: foundation (description/TF + WAVE-ROVER bridge) plus
    perception, sensor-fusion odometry, live SLAM and Nav2.

    The base is started once here; the sub-launches are included with
    bringup_base:=false so the foundation is not started multiple times. Only
    lidar_slam.launch.py starts slam_toolbox; nav.launch.py uses navigation-only
    bring-up, so there is exactly one slam_toolbox in the stack.

    Drive a goal from RViz:
      ros2 launch QB3rt full_stack.launch.py use_rviz:=true
    (enable_base_driver defaults true so Nav2's /cmd_vel actually moves the wheels.)"""
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_base_driver = LaunchConfiguration("enable_base_driver")
    enable_nav = LaunchConfiguration("enable_nav")
    gate_slam_on_vio = LaunchConfiguration("gate_slam_on_vio")
    params_file = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    common = {"namespace": namespace, "use_sim_time": use_sim_time}
    sub = {**common, "bringup_base": "false"}

    default_params = PathJoinSubstitution(
        [FindPackageShare("QB3rt"), "config", "nav2.yaml"]
    )
    default_rviz = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            # Wheels must move for Nav2 to drive, so the calibrated bridge is on by default.
            DeclareLaunchArgument("enable_base_driver", default_value="true"),
            # Set enable_nav:=false to run everything EXCEPT Nav2 on the robot,
            # when Nav2 runs remotely on the laptop (QB3rt/laptop/README.md).
            DeclareLaunchArgument("enable_nav", default_value="true"),
            # Hold slam_toolbox until ORB-SLAM3 VIO has finished its init ritual
            # (latched /vio/ready), so the map isn't built from the start-up VIO
            # calibration motion. On by default: full_stack always runs VIO. If
            # VIO is disabled it simply falls back to slam_gate_timeout.
            DeclareLaunchArgument("gate_slam_on_vio", default_value="true"),
            # Nav2 params (defaults to QB3rt's tuned nav2.yaml for live SLAM).
            DeclareLaunchArgument("params_file", default_value=default_params),
            # RViz for map/scan/costmaps/goal (off by default; headless robots).
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            # Foundation: robot description/TF + (optional) WAVE-ROVER bridge.
            _qb3rt("base.launch.py", {**common, "enable_base_driver": enable_base_driver}),
            # Perception (OAK-D camera / VIO source). Scoped: the depthai driver
            # sets the global `use_composition` launch config to "true", which
            # otherwise leaks into nav2's navigation_launch.py and crashes it.
            _qb3rt("perception.launch.py", sub, scoped=True),
            # Sensor fusion (robot_localization EKF -> odom->base_footprint).
            _qb3rt("odometry.launch.py", sub),
            # LIDAR SLAM (RPLIDAR + slam_toolbox -> map->odom); base, odometry and
            # perception are already started above, so don't start them again here.
            _qb3rt(
                "lidar_slam.launch.py",
                {
                    **sub,
                    "bringup_odometry": "false",
                    "enable_perception": "false",
                    "gate_slam_on_vio": gate_slam_on_vio,
                },
            ),
            # Navigation (Nav2 navigation-only; slam_toolbox provides map->odom).
            # use_localization:=false forces nav.launch.py's bringup_launch.py to skip
            # AMCL/map_server (localization_launch) so Nav2 does NOT fight slam_toolbox,
            # which already owns map->odom and /map here. (nav.launch.py itself defaults
            # use_localization=true for standalone / saved-map navigation.)
            _qb3rt(
                "nav.launch.py",
                {**sub, "params_file": params_file, "use_localization": "False"},
                condition=IfCondition(enable_nav),
            ),
            # RViz visualization + goal sending.
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