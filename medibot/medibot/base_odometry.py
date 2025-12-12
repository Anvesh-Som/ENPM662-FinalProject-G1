#!/usr/bin/env python3

import math
from typing import Optional, Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class BaseOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("medibot_base_odometry")

        # Parameters
        self.declare_parameter("left_wheel_joint", "left_wheel_joint")
        self.declare_parameter("right_wheel_joint", "right_wheel_joint")
        self.declare_parameter("wheel_radius", 0.1016)
        self.declare_parameter("wheel_separation", 1.2065)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.left_wheel_joint = self.get_parameter(
            "left_wheel_joint"
        ).get_parameter_value().string_value
        self.right_wheel_joint = self.get_parameter(
            "right_wheel_joint"
        ).get_parameter_value().string_value
        self.wheel_radius = self.get_parameter(
            "wheel_radius"
        ).get_parameter_value().double_value
        self.wheel_separation = self.get_parameter(
            "wheel_separation"
        ).get_parameter_value().double_value
        self.odom_frame = self.get_parameter(
            "odom_frame"
        ).get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            "base_frame"
        ).get_parameter_value().string_value

        # Pose state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Previous wheel positions & time (from message header)
        self.prev_left_pos: Optional[float] = None
        self.prev_right_pos: Optional[float] = None
        self.prev_stamp_sec: Optional[float] = None

        # To avoid spamming warnings
        self._missing_joint_warned = False

        # ROS I/O
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_sub = self.create_subscription(
            JointState, "joint_states", self.joint_state_callback, 50
        )

        self.get_logger().info(
            f"BaseOdometryNode started with left='{self.left_wheel_joint}', "
            f"right='{self.right_wheel_joint}', radius={self.wheel_radius}, "
            f"separation={self.wheel_separation}"
        )

    def joint_state_callback(self, msg: JointState) -> None:
        name_to_index: Dict[str, int] = {n: i for i, n in enumerate(msg.name)}

        if (
            self.left_wheel_joint not in name_to_index
            or self.right_wheel_joint not in name_to_index
        ):
            if not self._missing_joint_warned:
                self.get_logger().warn(
                    f"/joint_states does not contain wheel joints "
                    f"'{self.left_wheel_joint}' and '{self.right_wheel_joint}'. "
                    f"Got names: {list(msg.name)}"
                )
                self._missing_joint_warned = True
            return

        left_pos = msg.position[name_to_index[self.left_wheel_joint]]
        right_pos = msg.position[name_to_index[self.right_wheel_joint]]

        # Use the message header stamp if available; fall back to node clock
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            current_time_sec = self.get_clock().now().nanoseconds * 1e-9
        else:
            current_time_sec = stamp.sec + stamp.nanosec * 1e-9

        if self.prev_stamp_sec is None:
            self.prev_left_pos = left_pos
            self.prev_right_pos = right_pos
            self.prev_stamp_sec = current_time_sec
            self.get_logger().info(
                "Received first /joint_states with wheel joints; initializing odometry."
            )
            return

        dt = current_time_sec - self.prev_stamp_sec
        if dt <= 0.0:
            dt = 1e-3

        d_left = (left_pos - self.prev_left_pos) * self.wheel_radius
        d_right = (right_pos - self.prev_right_pos) * self.wheel_radius

        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos
        self.prev_stamp_sec = current_time_sec

        d_s = 0.5 * (d_right + d_left)
        d_theta = (d_right - d_left) / self.wheel_separation

        if abs(d_theta) < 1e-6:
            dx = d_s * math.cos(self.yaw)
            dy = d_s * math.sin(self.yaw)
        else:
            r_icc = d_s / d_theta
            dx = r_icc * (math.sin(self.yaw + d_theta) - math.sin(self.yaw))
            dy = -r_icc * (math.cos(self.yaw + d_theta) - math.cos(self.yaw))

        self.x += dx
        self.y += dy
        self.yaw = normalize_angle(self.yaw + d_theta)

        vx = d_s / dt
        vtheta = d_theta / dt

        # Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = vtheta

        self.odom_pub.publish(odom_msg)

        # TF: odom -> base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = odom_msg.header.stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
    