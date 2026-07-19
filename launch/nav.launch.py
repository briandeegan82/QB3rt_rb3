from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Navigation: the Nav2 stack (nav2_bringup).

    Nav2 only starts once you provide a params file, e.g.:
      ros2 launch QB3rt nav.launch.py params_file:=/path/nav2.yaml map:=/path/map.yaml
    Until then this launch just brings up the robot foundation."""
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    bringup_base = LaunchConfiguration("bringup_base")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_localization = LaunchConfiguration("use_localization")
    slam = LaunchConfiguration("slam")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("bringup_base", default_value="true"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument("autostart", default_value="true"),
            # NOTE: capitalized Python-literal bools ON PURPOSE. nav2's bringup_launch.py
            # evals these inside PythonExpression([slam,' and ',use_localization]) /
            # ['not ',slam,' and ',use_localization], so lowercase "true"/"false" become
            # eval("... false ...") -> NameError: name 'false' is not defined. Keep them
            # "True"/"False". use_localization=True runs AMCL/map_server for standalone
            # saved-map navigation; full_stack overrides it to "False" (slam_toolbox owns
            # map->odom there, so Nav2 runs navigation-only).
            DeclareLaunchArgument("use_localization", default_value="True"),
            DeclareLaunchArgument("slam", default_value="False"),
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
            # Nav2 — gated on a params_file being supplied (see docstring).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
                    )
                ),
                condition=IfCondition(PythonExpression(["'", params_file, "' != ''"])),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "map": map_yaml,
                    "params_file": params_file,
                    "autostart": autostart,
                    "use_localization": use_localization,
                    "slam": slam,
                }.items(),
            ),
        ]
    )
