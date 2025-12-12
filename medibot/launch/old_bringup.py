from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_medibot = get_package_share_directory("medibot")
    pkg_ros_gz = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    declare_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
    )

    declare_world = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution(
            [pkg_medibot, "worlds", "hospital.sdf"]
        ),
        description="World file",
    )

    urdf_file = f"{pkg_medibot}/urdf/medibot.urdf"

    robot_description = ParameterValue(
        open(urdf_file).read(),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": robot_description,
        }],
        output="screen",
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_ros_gz, "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": ["-r ", world],
        }.items(),
    )

    spawn_medibot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "medibot",
            "-topic", "robot_description",
            "-z", "0.1",
        ],
        output="screen",
    )

    return LaunchDescription([
        declare_sim_time,
        declare_world,
        gz_sim,
        robot_state_publisher,
        joint_state_publisher,
        spawn_medibot,
    ])
