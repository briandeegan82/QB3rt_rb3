from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=["python3", "/opt/qb3rt/install/share/QB3rt/scripts/nv12_to_mono8.py"], output="screen"),
        ExecuteProcess(cmd=["python3", "/opt/qb3rt/install/share/QB3rt/scripts/wait_for_vio_ready.py",
                            "--ros-args", "-p", "timeout:=600.0"], output="screen"),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["--frame-id", "a", "--child-frame-id", "b"]),
    ])
