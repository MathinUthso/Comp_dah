from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, Empty
except Exception:  # pragma: no cover - ROS2 only
    rclpy = None
    Node = object  # type: ignore
    Bool = Empty = object  # type: ignore


@dataclass(slots=True)
class CriticalCommandResult:
    ok: bool
    message: str
    payload: dict[str, Any]


class CriticalRosPublisher:
    def __init__(self, estop_topic: str, halt_topic: str, reset_topic: str) -> None:
        self.estop_topic = estop_topic
        self.halt_topic = halt_topic
        self.reset_topic = reset_topic
        self._lock = threading.Lock()
        self._node = None
        self._publishers = {}
        self._ensure_node()

    def _ensure_node(self) -> None:
        if rclpy is None:
            return
        with self._lock:
            if self._node is not None:
                return
            if not rclpy.ok():
                rclpy.init(args=None)
            self._node = rclpy.create_node("dashboard_critical_publisher")
            self._publishers["estop"] = self._node.create_publisher(Bool, self.estop_topic, 10)
            self._publishers["halt"] = self._node.create_publisher(Bool, self.halt_topic, 10)
            self._publishers["reset"] = self._node.create_publisher(Empty, self.reset_topic, 10)

    def publish_estop(self) -> CriticalCommandResult:
        if rclpy is None:
            return CriticalCommandResult(False, "rclpy unavailable", {"topic": self.estop_topic})
        self._ensure_node()
        msg = Bool()
        msg.data = True
        self._publishers["estop"].publish(msg)
        return CriticalCommandResult(True, "emergency stop published", {"topic": self.estop_topic})

    def publish_motor_halt(self) -> CriticalCommandResult:
        if rclpy is None:
            return CriticalCommandResult(False, "rclpy unavailable", {"topic": self.halt_topic})
        self._ensure_node()
        msg = Bool()
        msg.data = True
        self._publishers["halt"].publish(msg)
        return CriticalCommandResult(True, "motor halt published", {"topic": self.halt_topic})

    def publish_emergency_reset(self) -> CriticalCommandResult:
        if rclpy is None:
            return CriticalCommandResult(False, "rclpy unavailable", {"topic": self.reset_topic})
        self._ensure_node()
        self._publishers["reset"].publish(Empty())
        return CriticalCommandResult(True, "emergency reset published", {"topic": self.reset_topic})
