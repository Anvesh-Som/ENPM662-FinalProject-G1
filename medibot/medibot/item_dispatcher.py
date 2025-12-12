#!/usr/bin/env python3

import os
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, PointStamped
from std_msgs.msg import String, Float64
from ament_index_python.packages import get_package_share_directory

import yaml


class ItemDispatcherNode(Node):
    """
    Node that maps an item name to:

      - base docking pose (for the base controller)
      - cube center (for arm IK)
      - item height

    and publishes these on ROS topics.

    Usage:
      ros2 run medibot medibot_item_dispatcher --ros-args -p item_name:=<name>
    """

    def __init__(self) -> None:
        super().__init__("medibot_item_dispatcher")

        # Parameters
        self.declare_parameter("item_name", "")
        self.declare_parameter("items_config_file", "")

        self.item_name: str = (
            self.get_parameter("item_name").get_parameter_value().string_value
        )
        self.items_config_file: str = (
            self.get_parameter("items_config_file").get_parameter_value().string_value
        )

        if not self.item_name:
            self.get_logger().error("Parameter 'item_name' must be set.")
            raise RuntimeError("item_name not provided")

        # Load configuration file
        config_path = self._resolve_config_path(self.items_config_file)
        self.get_logger().info(f"Loading items configuration from: {config_path}")

        with open(config_path, "r") as f:
            self.config: Dict[str, Any] = yaml.safe_load(f)

        # Extract config sections
        self.cabinet_frame: str = self.config.get("cabinet_frame", "cabinet")
        self.docking_stations: Dict[str, Any] = self.config.get("docking_stations", {})
        self.cubes: Dict[str, Any] = self.config.get("cubes", {})
        self.items: Dict[str, Any] = self.config.get("items", {})

        if self.item_name not in self.items:
            available = ", ".join(sorted(self.items.keys()))
            self.get_logger().error(
                f"Item '{self.item_name}' not found in configuration. "
                f"Available items: {available}"
            )
            raise RuntimeError("Requested item not in configuration")

        # Publishers
        self.base_goal_pub = self.create_publisher(Pose2D, "base_goal", 10)
        self.ee_target_pub = self.create_publisher(PointStamped, "ee_target", 10)
        self.item_height_pub = self.create_publisher(Float64, "item_height", 10)
        self.requested_item_pub = self.create_publisher(String, "requested_item", 10)

        # Internal state
        self.publish_count = 0
        self.max_publish_count = 5

        # Start timer to publish a few times then shut down
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info(
            f"ItemDispatcherNode started for item '{self.item_name}' in frame '{self.cabinet_frame}'."
        )

    def _resolve_config_path(self, override: str) -> str:
        if override:
            return override
        # Default: medibot/config/medibot_items.yaml
        share_dir = get_package_share_directory("medibot")
        return os.path.join(share_dir, "config", "medibot_items.yaml")

    def timer_callback(self) -> None:
        if self.publish_count >= self.max_publish_count:
            self.get_logger().info("Finished publishing item targets, shutting down.")
            rclpy.shutdown()
            return

        self.publish_count += 1
        self.publish_targets()

    def publish_targets(self) -> None:
        item_info: Dict[str, Any] = self.items[self.item_name]
        cube_id: str = item_info["cube_id"]
        item_height: float = float(item_info.get("height", 0.0))

        if cube_id not in self.cubes:
            self.get_logger().error(
                f"Cube '{cube_id}' not found in 'cubes' section."
            )
            return

        cube_info: Dict[str, Any] = self.cubes[cube_id]
        col_index: int = int(cube_info["col"])
        center = cube_info["center"]  # [x, y, z] in cabinet_frame

        docking_key = f"col{col_index}"
        if docking_key not in self.docking_stations:
            self.get_logger().error(
                f"Docking station '{docking_key}' not found in 'docking_stations'."
            )
            return

        docking = self.docking_stations[docking_key]
        dock_x = float(docking["x"])
        dock_y = float(docking["y"])
        dock_theta = float(docking["theta"])

        # Publish base goal
        base_goal_msg = Pose2D()
        base_goal_msg.x = dock_x
        base_goal_msg.y = dock_y
        base_goal_msg.theta = dock_theta
        self.base_goal_pub.publish(base_goal_msg)

        # Publish ee target in cabinet_frame
        ee_target_msg = PointStamped()
        ee_target_msg.header.stamp = self.get_clock().now().to_msg()
        ee_target_msg.header.frame_id = self.cabinet_frame
        ee_target_msg.point.x = float(center[0])
        ee_target_msg.point.y = float(center[1])
        ee_target_msg.point.z = float(center[2])
        self.ee_target_pub.publish(ee_target_msg)

        # Publish item height
        height_msg = Float64()
        height_msg.data = item_height
        self.item_height_pub.publish(height_msg)

        # Publish item name
        item_msg = String()
        item_msg.data = self.item_name
        self.requested_item_pub.publish(item_msg)

        self.get_logger().info(
            f"Published targets for item '{self.item_name}' "
            f"(cube_id={cube_id}, docking={docking_key}). "
            f"Publish count: {self.publish_count}/{self.max_publish_count}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = ItemDispatcherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        # Initialization errors (e.g., missing item_name) will land here
        rclpy.logging.get_logger("medibot_item_dispatcher").error(str(e))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
