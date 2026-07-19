from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    publish_joint_states = LaunchConfiguration("publish_joint_states")

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("QB3rt"), "urdf", "QB3rt.urdf.xacro"]
    )
    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            # joint_state_publisher animates the continuous wheel joints. The
            # fixed sensor-mount frames (laser, imu, cameras, rb3) are published
            # by robot_state_publisher regardless, so this is optional.
            DeclareLaunchArgument("publish_joint_states", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=namespace,
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                namespace=namespace,
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                # joint_state_publisher needs the URDF itself to discover the
                # (continuous) wheel joints; without robot_description it publishes
                # no joint states and the wheel link TFs never appear.
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": use_sim_time,
                    }
                ],
                condition=IfCondition(publish_joint_states),
            ),
        ]
    )
