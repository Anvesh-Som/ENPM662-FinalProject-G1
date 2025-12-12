from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os


def generate_launch_description():
    pkg_medibot = get_package_share_directory("medibot")
    pkg_ros_gz = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    declare_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_world = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([pkg_medibot, "worlds", "hospital.sdf"]),
        description="World file",
    )

    # URDF and robot_description
    urdf_file = os.path.join(pkg_medibot, "urdf", "medibot.urdf")
    robot_description = ParameterValue(
        open(urdf_file).read(),
        value_type=str,
    )


    # Retimer for /joint_states
    # joint_state_retimer = Node(
    #     package="medibot",
    #     executable="medibot_joint_state_retimer",
    #     name="joint_state_retimer",
    #     # Important: we let it use wall time for this test
    #     parameters=[{"use_sim_time": False}],
    #     output="screen",
    # )


    robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    # Let RSP live in wall time, ignore sim_time for now
    parameters=[
        {
            "robot_description": robot_description,
            "use_sim_time": True,
            "publish_frequency": 30.0,
            "use_tf_static": False,
        }
    ],
    # remappings=[
    #     ("joint_states", "joint_states_retimed"),
    # ],
    output="screen",
    )

    # Gazebo (Ignition / Gazebo Garden) via ros_gz_sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": ["-r ", world],
        }.items(),
    )

    # Spawn medibot into Gazebo from the robot_description topic
    spawn_medibot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "medibot",
            "-topic",
            "robot_description",
            '-x', '0.0',   # X position
            '-y', '0.0',   # Y position
            '-z', '0.5',    # Small Z offset above the ground
            '-R', '0',
            '-P','0',
            '-Y','0',
        ],
        output="screen",
    )

    # Controller configuration YAML
    controllers_yaml = os.path.join(pkg_medibot, "config", "medibot_controllers.yaml")

    # Controller spawners (ros2_control)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
        # parameters=[{"use_sim_time": False}],
    )

    base_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["base_velocity_controller", "-p", controllers_yaml],
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    arm_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_position_controller", "-p", controllers_yaml],
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    gripper_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_position_controller", "-p", controllers_yaml],
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    # Delay joint_state_broadcaster a bit to give Gazebo + medibot time to come up
    delayed_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner],
    )

    # bridge_params = os.path.join(get_package_share_directory('medibot'),'config','gz_bridge.yaml')
    clock_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',],
    output='screen'
    )

    # === Custom medibot nodes ===

    base_odometry_node = Node(
        package="medibot",
        executable="medibot_base_odometry",
        name="medibot_base_odometry",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "left_wheel_joint": "left_wheel_joint",
                "right_wheel_joint": "right_wheel_joint",
                # wheel_radius and wheel_separation use defaults from the node;
                # tune them via parameters if needed.
            }
        ],
        output="screen",
    )

    base_controller_node = Node(
        package="medibot",
        executable="medibot_base_controller",
        name="medibot_base_controller",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                # PID gains, wheel geometry etc. can also be overridden here if desired.
            }
        ],
        output="screen",
    )

    forward_kinematics_node = Node(
        package="medibot",
        executable="medibot_forward_kinematics",
        name="medibot_forward_kinematics",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
        output="screen",
    )

    arm_controller_node = Node(
        package="medibot",
        executable="medibot_arm_controller",
        name="medibot_arm_controller",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
        output="screen",
    )

    arm_goal_ik_node = Node(
    package="medibot",
    executable="medibot_arm_goal_ik",
    name="medibot_arm_goal_ik",
    parameters=[
        {
            "use_sim_time": use_sim_time,
            "base_frame": "base_link",
            "goal_topic": "/medibot/arm_cartesian_goal",
            "joint_target_topic": "/medibot/arm_joint_targets",
            "joint_names": [
                "shoulder_joint",
                "upper_arm_joint",
                "forearm_joint",
                "wrist_joint",
            ],
        }
    ],
    output="screen",
    )

    # === LaunchDescription and event chaining ===

    ld = LaunchDescription()
    ld.add_action(declare_sim_time)
    ld.add_action(declare_world)
    ld.add_action(gz_sim)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_medibot)
    ld.add_action(clock_bridge)
    ld.add_action(arm_goal_ik_node)

    # Controllers (JSB delayed, others chained after JSB finishes)
    ld.add_action(delayed_joint_state_broadcaster)

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
                    base_velocity_controller_spawner,
                    arm_position_controller_spawner,
                    gripper_position_controller_spawner,
                ],
            )
        )
    )

    # Custom nodes (they can safely wait for /odom, joint_states, etc.)
    ld.add_action(base_odometry_node)
    ld.add_action(base_controller_node)
    ld.add_action(forward_kinematics_node)
    ld.add_action(arm_controller_node)

    return ld
