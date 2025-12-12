#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PIDController:
    """Simple PID controller."""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_limit: Optional[float] = None) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self.integral = 0.0
        self.prev_error: Optional[float] = None

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = None

    def step(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        p = self.kp * error
        self.integral += error * dt
        i = self.ki * self.integral

        if self.prev_error is None:
            d_error = 0.0
        else:
            d_error = (error - self.prev_error) / dt
        d = self.kd * d_error

        self.prev_error = error

        u = p + i + d
        if self.output_limit is not None:
            u = max(min(u, self.output_limit), -self.output_limit)
        return u


class BaseControllerNode(Node):
    """
    Base PID controller for differential drive.

    - Subscribes:
        /odom      (nav_msgs/Odometry)
        /base_goal (geometry_msgs/Pose2D)

    - Publishes:
        /base_velocity_controller/commands (std_msgs/Float64MultiArray)
        data = [left_wheel_velocity, right_wheel_velocity] in rad/s
    """

    def __init__(self) -> None:
        super().__init__("medibot_base_controller")

        # Parameters
        self.declare_parameter("wheel_radius", 0.1016)
        self.declare_parameter("wheel_separation", 0.3)
        self.declare_parameter("max_linear_velocity", 0.5)
        self.declare_parameter("max_angular_velocity", 1.0)
        self.declare_parameter("position_tolerance", 0.02)
        self.declare_parameter("yaw_tolerance", 0.02)

        self.declare_parameter("linear_kp", 1.0)
        self.declare_parameter("linear_ki", 0.0)
        self.declare_parameter("linear_kd", 0.0)
        self.declare_parameter("angular_kp", 2.0)
        self.declare_parameter("angular_ki", 0.0)
        self.declare_parameter("angular_kd", 0.0)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self.max_linear_velocity = self.get_parameter("max_linear_velocity").get_parameter_value().double_value
        self.max_angular_velocity = self.get_parameter("max_angular_velocity").get_parameter_value().double_value
        self.position_tolerance = self.get_parameter("position_tolerance").get_parameter_value().double_value
        self.yaw_tolerance = self.get_parameter("yaw_tolerance").get_parameter_value().double_value

        lin_kp = self.get_parameter("linear_kp").get_parameter_value().double_value
        lin_ki = self.get_parameter("linear_ki").get_parameter_value().double_value
        lin_kd = self.get_parameter("linear_kd").get_parameter_value().double_value
        ang_kp = self.get_parameter("angular_kp").get_parameter_value().double_value
        ang_ki = self.get_parameter("angular_ki").get_parameter_value().double_value
        ang_kd = self.get_parameter("angular_kd").get_parameter_value().double_value

        self.linear_pid = PIDController(lin_kp, lin_ki, lin_kd, self.max_linear_velocity)
        self.angular_pid = PIDController(ang_kp, ang_ki, ang_kd, self.max_angular_velocity)

        # State: start with a reasonable assumption (0,0,0)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.have_odom = False  # will become True once we see a message

        self.goal: Optional[Pose2D] = None
        self.last_time_sec: Optional[float] = None

        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )
        self.goal_sub = self.create_subscription(
            Pose2D, "base_goal", self.goal_callback, 10
        )
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, "base_velocity_controller/commands", 10
        )

        # 50 Hz control loop
        self.control_timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info(
            f"BaseControllerNode started with wheel_radius={self.wheel_radius}, "
            f"wheel_separation={self.wheel_separation}"
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        self.current_yaw = math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)

        if not self.have_odom:
            self.get_logger().info("Received first odom message.")
        self.have_odom = True

    def goal_callback(self, msg: Pose2D) -> None:
        self.goal = msg
        self.linear_pid.reset()
        self.angular_pid.reset()
        self.last_time_sec = None
        self.get_logger().info(
            f"Received new base goal: x={msg.x:.3f}, y={msg.y:.3f}, theta={msg.theta:.3f}"
        )

    def control_loop(self) -> None:
        # Must at least have a goal; odom will be used when available, but we don't hard-block on it
        if self.goal is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time_sec is None:
            # First iteration after goal: just initialize dt
            self.last_time_sec = now
            return

        dt = now - self.last_time_sec
        self.last_time_sec = now
        if dt <= 0.0:
            dt = 0.02  # fallback to nominal rate

        # Current pose
        x = self.current_x
        y = self.current_y
        yaw = self.current_yaw

        dx = self.goal.x - x
        dy = self.goal.y - y
        rho = math.hypot(dx, dy)

        goal_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(goal_heading - yaw)
        theta_error = normalize_angle(self.goal.theta - yaw)

        if rho > self.position_tolerance:
            linear_cmd = self.linear_pid.step(rho, dt)
            angular_cmd = self.angular_pid.step(heading_error, dt)
        else:
            linear_cmd = 0.0
            angular_cmd = self.angular_pid.step(theta_error, dt)
            if abs(theta_error) < self.yaw_tolerance:
                linear_cmd = 0.0
                angular_cmd = 0.0

        # Clamp
        linear_cmd = max(min(linear_cmd, self.max_linear_velocity), -self.max_linear_velocity)
        angular_cmd = max(min(angular_cmd, self.max_angular_velocity), -self.max_angular_velocity)

        # Diff-drive inverse kinematics
        v_r = linear_cmd + angular_cmd * self.wheel_separation / 2.0
        v_l = linear_cmd - angular_cmd * self.wheel_separation / 2.0

        if self.wheel_radius <= 0.0:
            self.get_logger().error("Wheel radius is non-positive; cannot compute wheel commands.")
            return

        w_r = v_r / self.wheel_radius
        w_l = v_l / self.wheel_radius

        cmd_msg = Float64MultiArray()
        cmd_msg.data = [w_l, w_r]
        self.cmd_pub.publish(cmd_msg)

        self.get_logger().debug(
            f"cmd: dt={dt:.3f}, rho={rho:.3f}, v=({linear_cmd:.3f}), "
            f"w=({angular_cmd:.3f}), w_l={w_l:.3f}, w_r={w_r:.3f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
