"""Spawn the Aegis URDF through Gazebo's SpawnEntity service.

This avoids a runtime dependency on ``lxml`` (used by gazebo_ros/spawn_entity.py)
and keeps the launch file self-contained.
"""

from __future__ import annotations

import math
import sys

import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node


class ModelSpawner(Node):
    def __init__(self) -> None:
        super().__init__("aegis_spawn_model")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("entity", "aegis_bot")
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("spawn_z", 0.05)
        self.declare_parameter("spawn_yaw", 0.0)
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("timeout_s", 60.0)

        self._client = self.create_client(SpawnEntity, "/spawn_entity")

    def spawn(self) -> bool:
        xml = str(self.get_parameter("robot_description").value)
        if not xml:
            self.get_logger().error("robot_description parameter is empty")
            return False

        timeout = float(self.get_parameter("timeout_s").value)
        if not self._client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("/spawn_entity service did not become available")
            return False

        yaw = float(self.get_parameter("spawn_yaw").value)
        pose = Pose()
        pose.position.x = float(self.get_parameter("spawn_x").value)
        pose.position.y = float(self.get_parameter("spawn_y").value)
        pose.position.z = float(self.get_parameter("spawn_z").value)
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)

        request = SpawnEntity.Request()
        request.name = str(self.get_parameter("entity").value)
        request.xml = xml
        request.robot_namespace = ""
        request.initial_pose = pose
        request.reference_frame = str(self.get_parameter("reference_frame").value)

        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        result = future.result()
        if result is None:
            self.get_logger().error("SpawnEntity call timed out")
            return False
        if not result.success:
            self.get_logger().error(f"SpawnEntity failed: {result.status_message}")
            return False

        self.get_logger().info(f"Spawned {request.name}: {result.status_message}")
        return True


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = ModelSpawner()
    try:
        return 0 if node.spawn() else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
