#!/usr/bin/env python3

import math
from typing import List

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

# Import IK from our other file
from medibot.forward_kinematics import inverse_kinematics_medibot


class ArmGoalIKNode(Node):
    """
    Node that receives a Cartesian goal for the end-effector and sends joint targets.

    - Subscribes:
        /medibot/arm_cartesian_goal (geometry_msgs/PoseStamped)
          pose.position is the target (x,y,z) in base frame, in meters.

    - Publishes:
        /medibot/arm_joint_targets (sensor_msgs/JointState)
          Joint names and positions [rad] for the arm joints.
    """

    def __init__(self) -> None:
        super().__init__("medibot_arm_goal_ik")

        # Parameters
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter(
            "joint_names",
            ["shoulder_joint", "upper_arm_joint", "forearm_joint", "wrist_joint"],
        )
        self.declare_parameter(
            "goal_topic",
            "/medibot/arm_cartesian_goal",
        )
        self.declare_parameter(
            "joint_target_topic",
            "/medibot/arm_joint_targets",
        )

        self.base_frame: str = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )
        self.joint_names: List[str] = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        self.goal_topic: str = (
            self.get_parameter("goal_topic").get_parameter_value().string_value
        )
        self.joint_target_topic: str = (
            self.get_parameter("joint_target_topic").get_parameter_value().string_value
        )

        # Publisher to the existing ArmControllerNode
        self.joint_pub = self.create_publisher(
            JointState,
            self.joint_target_topic,
            10,
        )

        # Subscriber for Cartesian goals
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10,
        )

        self.get_logger().info(
            f"ArmGoalIKNode started. Listening for goals on '{self.goal_topic}', "
            f"publishing joint targets on '{self.joint_target_topic}'. "
            f"Base frame: '{self.base_frame}', joints={self.joint_names}"
        )

    def goal_callback(self, msg: PoseStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.base_frame:
            self.get_logger().warn(
                f"Received goal in frame '{msg.header.frame_id}', "
                f"but only '{self.base_frame}' is supported (no TF transform applied)."
            )
            return

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        self.get_logger().info(
            f"Received arm goal: frame={msg.header.frame_id or self.base_frame}, "
            f"x={x:.3f}, y={y:.3f}, z={z:.3f}"
        )

        try:
            # IK: returns [q1,q2,q3,q4] in radians
            q = inverse_kinematics_medibot(x, y, z)
        except ValueError as e:
            self.get_logger().error(f"IK failed: {e}")
            return
        except Exception as e:
            self.get_logger().error(f"Unexpected IK error: {e}")
            return

        if len(q) != len(self.joint_names):
            self.get_logger().error(
                f"IK returned {len(q)} joints, but joint_names has {len(self.joint_names)}"
            )
            return

        self.get_logger().info(
            "IK solution [rad]: " + ", ".join(f"{a:.3f}" for a in q)
        )

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = q

        self.joint_pub.publish(js)
        self.get_logger().info("Published joint target to ArmControllerNode.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmGoalIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
