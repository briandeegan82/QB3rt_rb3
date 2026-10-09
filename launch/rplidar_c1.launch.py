from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """RPLIDAR C1 driver (rplidar_ros rplidar_node).

    Owned by QB3rt rather than included from rplidar_ros: the apt
    ros-jazzy-rplidar-ros (2.1.2) ships no rplidar_c1_launch.py, and a missing
    include aborts the whole `ros2 launch`, so the visible symptom is unrelated
    nodes segfaulting on shutdown. The legacy QIRP image patched the file into
    the vendor package by hand; keeping it here means deploy/update_project.sh
    ships it and an apt upgrade can't remove it.

    Defaults match the QB3rt fleet, so callers need not override them:
    /dev/rplidar (udev symlink from stamp/) and laser_frame (URDF lidar link).
    """
    args = [
        ("channel_type", "serial", "Lidar channel type"),
        ("serial_port", "/dev/rplidar", "Serial port of the lidar (udev symlink, not ttyUSB*)"),
        ("serial_baudrate", "460800", "Serial baudrate (C1: 460800)"),
        ("frame_id", "laser_frame", "Scan frame_id; must match the URDF lidar link"),
        ("inverted", "false", "Invert scan data"),
        ("angle_compensate", "true", "Enable angle compensation"),
        ("scan_mode", "Standard", "Scan mode (C1 default: Standard)"),
    ]

    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=desc) for name, default, desc in args]
        + [
            Node(
                package="rplidar_ros",
                executable="rplidar_node",
                name="rplidar_node",
                parameters=[{name: LaunchConfiguration(name) for name, _, _ in args}],
                output="screen",
                # The lidar's CP2102N shares a USB hub with the rover bridge; a bus
                # reset makes the serial open fail and the driver abort. Respawn so
                # it self-heals instead of leaving SLAM without /scan.
                respawn=True,
                respawn_delay=5.0,
            ),
        ]
    )
