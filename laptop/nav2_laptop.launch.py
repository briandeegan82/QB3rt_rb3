"""Remote Nav2 for QB3rt: run the navigation stack on the laptop.

The robot runs sensors + EKF + slam_toolbox onboard and owns map->odom and
odom->base_footprint (ros2 launch QB3rt full_stack.launch.py enable_nav:=false).
This launch runs Nav2 navigation-only (nav2_bringup/navigation_launch.py - no
AMCL/map_server, slam_toolbox on the robot is the localizer) plus RViz.

Launch by path (this file is not an installed package):
    source ~/qb3rt_laptop/qb3rt_env.sh
    ros2 launch ~/qb3rt_laptop/nav2_laptop.launch.py

See README.md in this directory for the full runbook (env, clock sync, order).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    default_rviz = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(_THIS_DIR, "nav2_laptop.yaml"),
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            # Nav2 navigation-only. use_sim_time is always false here: the robot
            # runs on wall clock and TF stamps come from it.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": "false",
                    "params_file": params_file,
                    "autostart": "true",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
