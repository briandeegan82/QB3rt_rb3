#!/usr/bin/env python3
"""VLM scene node — SmolVLM via llama.cpp server on the Adreno GPU.

Requires the qb3rt-vlm-server systemd unit (or any llama.cpp server with
--mmproj at vlm_url). Camera frames come from /image_raw/compressed when
perception.launch.py is running; otherwise the node falls back to a
one-shot qtiqmmfsrc grab.

Usage:
  ros2 launch QB3rt vlm.launch.py target:="red fire extinguisher"
  ros2 launch QB3rt vlm.launch.py drive:=true        # allow motion!

Drive commands (with drive:=true):
  ros2 service call /vlm_scene_node/find_object std_srvs/srv/Trigger
  ros2 service call /vlm_scene_node/cancel      std_srvs/srv/Trigger
  echo '{"cmd":"find","target":"laptop"}' | \
      ros2 topic pub --once /vlm/command std_msgs/msg/String "data: '...'"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "vlm_url", default_value="http://127.0.0.1:9876",
            description="llama.cpp server base URL"),
        DeclareLaunchArgument(
            "target", default_value="red fire extinguisher",
            description="default object for ~/find_object Trigger service"),
        DeclareLaunchArgument(
            "drive", default_value="false",
            description="allow the node to publish cmd_vel (default off — "
                        "set true only when the robot can move safely)"),
        DeclareLaunchArgument(
            "image_topic", default_value="/image_raw/compressed",
            description="compressed image topic (empty/fallback -> gst grab)"),
        DeclareLaunchArgument(
            "odom_topic", default_value="/odometry/filtered",
            description="Odometry for yaw bearings (optional)"),
        DeclareLaunchArgument(
            "scan_topic", default_value="/scan",
            description="LaserScan for forward stop-range (optional)"),
        DeclareLaunchArgument(
            "cmd_vel_topic", default_value="/cmd_vel",
            description="velocity command topic"),
        DeclareLaunchArgument(
            "spin_speed", default_value="0.5",
            description="in-place rotation speed rad/s during scan"),
        DeclareLaunchArgument(
            "fwd_speed", default_value="0.12",
            description="approach speed m/s"),
        DeclareLaunchArgument(
            "stop_range", default_value="0.40",
            description="stop when front sector range < this (m)"),
        DeclareLaunchArgument(
            "sample_count", default_value="8",
            description="headings sampled per 360 scan"),
        DeclareLaunchArgument(
            "votes_per_sample", default_value="2",
            description="VLM queries per heading (majority vote)"),
        DeclareLaunchArgument(
            "max_approach_m", default_value="6.0",
            description="abort approach after travelling this far (m)"),
    ]

    params = {a.name: LaunchConfiguration(a.name) for a in args}
    param_args = []
    for name in params:
        param_args += ["-p", [name, ":=", params[name]]]

    node = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution(
                [FindPackageShare("QB3rt"), "scripts", "vlm_scene_node.py"]),
            "--ros-args",
        ] + param_args,
        output="screen",
    )

    return LaunchDescription(args + [node])
