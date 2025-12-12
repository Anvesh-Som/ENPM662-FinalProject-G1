#!/usr/bin/env python3

from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class ArmControllerNode(Node):
    """
    Arm controller node.

    - Subscribes:
        /medibot/arm_joint_targets (sensor_msgs/JointState)
          The message's `name` and `position` fields specify target joint angles [rad].

    - Publishes:
        /arm_position_controller/commands (std_msgs/Float64MultiArray)
          data = [q_shoulder, q_upper_arm, q_forearm, q_wrist] in rad,
          in the same order as configured in the 'joint_names' parameter.
    """

    def __init__(self) -> None:
        super().__init__("medibot_arm_controller")

        # Parameter: arm joint names and command topic
        self.declare_parameter(
            "joint_names",
            ["shoulder_joint", "upper_arm_joint", "forearm_joint", "wrist_joint"],
        )
        self.declare_parameter(
            "command_topic", "arm_position_controller/commands"
        )

        self.joint_names: List[str] = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        self.command_topic: str = (
            self.get_parameter("command_topic").get_parameter_value().string_value
        )

        self.cmd_pub = self.create_publisher(Float64MultiArray, self.command_topic, 10)
        self.target_sub = self.create_subscription(
            JointState,
            "medibot/arm_joint_targets",
            self.target_callback,
            10,
        )

        self.get_logger().info(
            f"ArmControllerNode started with joints={self.joint_names}, "
            f"command_topic={self.command_topic}"
        )

    def target_callback(self, msg: JointState) -> None:
        name_to_pos: Dict[str, float] = {n: p for n, p in zip(msg.name, msg.position)}

        # Build command in the configured order
        cmd: List[float] = []
        for j in self.joint_names:
            if j not in name_to_pos:
                self.get_logger().warn(
                    f"Target message missing joint '{j}'"
                )
                return
            cmd.append(name_to_pos[j])

        cmd_msg = Float64MultiArray()
        cmd_msg.data = cmd
        self.cmd_pub.publish(cmd_msg)
        self.get_logger().debug(f"Sent arm command: {cmd}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
