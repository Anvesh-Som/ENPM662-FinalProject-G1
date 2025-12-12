from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    pkg_medibot = get_package_share_directory("medibot")
    urdf_file = os.path.join(pkg_medibot, "urdf", "medibot.urdf")

    robot_description = ParameterValue(
        open(urdf_file).read(),
        value_type=str,
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    return LaunchDescription([rsp])
