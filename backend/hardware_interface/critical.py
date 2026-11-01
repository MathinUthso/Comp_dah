from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from std_srvs.srv import Trigger
except Exception:  # pragma: no cover - ROS2 only
    rclpy = None
    Twist = Trigger = object  # type: ignore


@dataclass(slots=True)
class CriticalCommandResult:
    ok: bool
    message: str
    payload: dict[str, Any]


class CriticalRosPublisher:
    def __init__(self, estop_service: str, halt_topic: str, reset_service: str) -> None:
        self.estop_service = estop_service
        self.halt_topic = halt_topic
        self.reset_service = reset_service
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
            self._publishers["halt"] = self._node.create_publisher(Twist, self.halt_topic, 10)
            self._publishers["estop_client"] = self._node.create_client(Trigger, self.estop_service)
            self._publishers["reset_client"] = self._node.create_client(Trigger, self.reset_service)

    def publish_estop(self) -> CriticalCommandResult:
        if rclpy is None:
            return CriticalCommandResult(False, "rclpy unavailable", {"service": self.estop_service})
        self._ensure_node()
        client = self._publishers["estop_client"]
        if not client.wait_for_service(timeout_sec=0.5):
            return CriticalCommandResult(False, "estop service unavailable", {"service": self.estop_service})

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=1.0)
        if not future.done() or future.result() is None:
            return CriticalCommandResult(False, "estop service timeout", {"service": self.estop_service})

        response = future.result()
        return CriticalCommandResult(
            bool(response.success),
            str(response.message or "estop service called"),
            {"service": self.estop_service},
        )

    def publish_motor_halt(self) -> CriticalCommandResult:
        if rclpy is None:
            return CriticalCommandResult(False, "rclpy unavailable", {"topic": self.halt_topic})
        self._ensure_node()
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self._publishers["halt"].publish(msg)
        return CriticalCommandResult(True, "motor halt published as zero /cmd_vel", {"topic": self.halt_topic})

    def publish_emergency_reset(self) -> CriticalCommandResult:
        if rclpy is None:
            return CriticalCommandResult(False, "rclpy unavailable", {"service": self.reset_service})
        self._ensure_node()
        client = self._publishers["reset_client"]
        if not client.wait_for_service(timeout_sec=0.5):
            return CriticalCommandResult(False, "reset service unavailable", {"service": self.reset_service})

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=1.0)
        if not future.done() or future.result() is None:
            return CriticalCommandResult(False, "reset service timeout", {"service": self.reset_service})

        response = future.result()
        return CriticalCommandResult(
            bool(response.success),
            str(response.message or "reset service called"),
            {"service": self.reset_service},
        )
