#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateRetimer(Node):
    """
    Subscribes to /joint_states, overwrites the header.stamp with this node's
    current time, and republishes to /joint_states_retimed.
    """

    def __init__(self):
        super().__init__("joint_state_retimer")

        # Sub from original joint_states
        self.sub = self.create_subscription(
            JointState,
            "joint_states",
            self.joint_state_callback,
            10,
        )

        # Pub to retimed joint_states
        self.pub = self.create_publisher(
            JointState,
            "joint_states_retimed",
            10,
        )

        self.get_logger().info("JointStateRetimer started, rewriting header.stamp")

    def joint_state_callback(self, msg: JointState):
        # Overwrite timestamp with *this* node's clock
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRetimer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
