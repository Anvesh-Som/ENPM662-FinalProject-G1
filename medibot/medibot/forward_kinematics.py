#!/usr/bin/env python3

import math
from typing import List, Dict

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


# --- Arm geometry ---
# Units: meters (was given in inches in submitted ppt).
L1 = 0.18415   # 7.25 in * 0.0254
L2 = 1.6256    # 64 in * 0.0254
L3 = 1.1176    # 44 in * 0.0254
L4 = 0.41148   # 16.2 in * 0.0254


def dht(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """
    Denavit-Hartenberg transform.
    a: link length
    alpha: link twist
    d: link offset
    theta: joint angle
    """
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    ct = math.cos(theta)
    st = math.sin(theta)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def inverse_kinematics_medibot(x: float, y: float, z: float) -> List[float]:
    """
    Args:
        x, y, z: desired end-effector position in base frame (meters).

    Returns:
        joint angles [q1, q2, q3, q4] in radians.
    """
    # Base rotation
    q1g = math.atan2(y, x)

    # Project to arm plane and account for L4
    r = math.sqrt(x ** 2 + y ** 2) - L4
    z_ = z - L1

    D = math.sqrt(r * r + z_ * z_)

    # Law of cosines for elbow
    denom = 2.0 * L2 * L3
    if denom == 0.0:
        raise ValueError("Invalid link lengths (L2 * L3 == 0).")
    cos_q3 = (D * D - L2 * L2 - L3 * L3) / denom

    # Clamp for numerical safety
    cos_q3 = max(min(cos_q3, 1.0), -1.0)
    q3g = math.acos(cos_q3)

    phi = math.atan2(z_, r)
    psi = math.atan2(L3 * math.sin(q3g), L2 + L3 * math.cos(q3g))
    q2g = phi - psi
    q4g = -(q2g + q3g)

    # Apply offsets
    q1 = q1g
    q2 = q2g - math.pi / 2.0
    q3 = q3g + math.pi / 2.0
    q4 = q4g

    return [q1, q2, q3, q4]


def forward_kinematics_medibot(joint_angles: List[float]) -> np.ndarray:
    """
    joint_angles: [q1, q2, q3, q4] in radians.
    Returns 4x4 homogeneous transform base_link to end-effector.
    """
    if len(joint_angles) != 4:
        raise ValueError("Expected 4 joint angles.")

    q1, q2, q3, q4 = joint_angles

    # Offsets and DH parameters as in your snippet
    T1 = dht(0.0, math.radians(90.0), L1, q1)
    T2 = dht(L2, 0.0, 0.0, math.radians(90.0) + q2)
    T3 = dht(L3, 0.0, 0.0, math.radians(-90.0) + q3)
    T4 = dht(L4, 0.0, 0.0, q4)

    T = T1 @ T2 @ T3 @ T4
    return T


def rotation_matrix_to_quaternion(R: np.ndarray) -> List[float]:
    """
    Convert a 3x3 rotation matrix to quaternion [x, y, z, w].
    """
    assert R.shape == (3, 3)
    trace = float(R[0, 0] + R[1, 1] + R[2, 2])

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

    return [qx, qy, qz, qw]


class ForwardKinematicsNode(Node):
    """
    Node that computes and publishes the arm end-effector pose.

    - Subscribes:
        /joint_states
        /odom

    - Publishes:
        /end_effector_pose (PoseStamped, frame_id = odom)

    - TF:
        base_link -> ee_link
    """

    def __init__(self) -> None:
        super().__init__("medibot_forward_kinematics")

        # Parameters
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("ee_frame", "ee_link")
        self.declare_parameter(
            "joint_names",
            ["shoulder_joint", "upper_arm_joint", "forearm_joint", "wrist_joint"],
        )

        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.odom_frame = self.get_parameter("odom_frame").get_parameter_value().string_value
        self.ee_frame = self.get_parameter("ee_frame").get_parameter_value().string_value
        self.joint_names = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )

        # State
        self.current_joint_positions: Dict[str, float] = {}
        self.have_joints = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.have_odom = False

        self.joint_sub = self.create_subscription(
            JointState, "joint_states", self.joint_state_callback, 20
        )
        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )
        self.ee_pub = self.create_publisher(PoseStamped, "end_effector_pose", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f"ForwardKinematicsNode started with joints={self.joint_names}, "
            f"base_frame={self.base_frame}, ee_frame={self.ee_frame}"
        )

    def joint_state_callback(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self.current_joint_positions[name] = pos

        self.have_joints = all(j in self.current_joint_positions for j in self.joint_names)
        if self.have_joints and self.have_odom:
            self.update_end_effector_pose()

    def odom_callback(self, msg: Odometry) -> None:
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        self.current_yaw = math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)
        self.have_odom = True

        if self.have_joints:
            self.update_end_effector_pose()

    def update_end_effector_pose(self) -> None:
        # Collect joint angles in the configured order, in radians
        q = [self.current_joint_positions[j] for j in self.joint_names]

        # Forward kinematics base_link -> ee_link
        T_base_ee = forward_kinematics_medibot(q)
        R_base_ee = T_base_ee[0:3, 0:3]
        p_base_ee = T_base_ee[0:3, 3]

        # Build odom -> base_link transform
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        T_odom_base = np.array(
            [
                [cos_yaw, -sin_yaw, 0.0, self.current_x],
                [sin_yaw, cos_yaw, 0.0, self.current_y],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

        # Compose: odom -> ee
        T_odom_ee = T_odom_base @ T_base_ee
        R_odom_ee = T_odom_ee[0:3, 0:3]
        p_odom_ee = T_odom_ee[0:3, 3]

        # Quaternion for ee pose
        qx, qy, qz, qw = rotation_matrix_to_quaternion(R_odom_ee)

        # Publish PoseStamped in odom frame
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.odom_frame
        pose_msg.pose.position.x = float(p_odom_ee[0])
        pose_msg.pose.position.y = float(p_odom_ee[1])
        pose_msg.pose.position.z = float(p_odom_ee[2])
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.ee_pub.publish(pose_msg)

        # TF: base_link -> ee_link
        qx_b, qy_b, qz_b, qw_b = rotation_matrix_to_quaternion(R_base_ee)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = pose_msg.header.stamp
        tf_msg.header.frame_id = self.base_frame
        tf_msg.child_frame_id = self.ee_frame
        tf_msg.transform.translation.x = float(p_base_ee[0])
        tf_msg.transform.translation.y = float(p_base_ee[1])
        tf_msg.transform.translation.z = float(p_base_ee[2])
        tf_msg.transform.rotation.x = qx_b
        tf_msg.transform.rotation.y = qy_b
        tf_msg.transform.rotation.z = qz_b
        tf_msg.transform.rotation.w = qw_b

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
