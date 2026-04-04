from __future__ import annotations

from dataclasses import dataclass
from typing import Any

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
except Exception:  # pragma: no cover - ROS2 runtime only
    rclpy = None
    Node = object  # type: ignore
    Bool = String = object  # type: ignore


@dataclass
class NodeStatus:
    name: str
    healthy: bool = True
    detail: dict[str, Any] | None = None


class ManagedNode(Node):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.status_pub = self.create_publisher(String, f"/{name}/status", 10)
        self.heartbeat_pub = self.create_publisher(Bool, f"/{name}/heartbeat", 10)
        self.status = NodeStatus(name=name)
        self.timer = self.create_timer(1.0, self._tick)

    def _tick(self) -> None:
        msg = String()
        msg.data = f"{self.status.name}:healthy={self.status.healthy}"
        self.status_pub.publish(msg)
        heartbeat = Bool()
        heartbeat.data = True
        self.heartbeat_pub.publish(heartbeat)


class DashboardBridgeNode(ManagedNode):
    def __init__(self) -> None:
        super().__init__("dashboard_bridge")


class CameraBridgeNode(ManagedNode):
    def __init__(self) -> None:
        super().__init__("camera_bridge")


class ThermalBridgeNode(ManagedNode):
    def __init__(self) -> None:
        super().__init__("thermal_bridge")


class HardwareControlNode(ManagedNode):
    def __init__(self) -> None:
        super().__init__("hardware_control")


class FlexBEAdapterNode(ManagedNode):
    def __init__(self) -> None:
        super().__init__("flexbe_adapter")


class DiagnosticsNode(ManagedNode):
    def __init__(self) -> None:
        super().__init__("diagnostics_node")


class EmergencyControlNode(ManagedNode):
    def __init__(self) -> None:
        super().__init__("emergency_control")


def _run(node_cls) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is unavailable in this environment")
    if not rclpy.ok():
        rclpy.init(args=None)
    node = node_cls()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_dashboard_bridge() -> None:
    _run(DashboardBridgeNode)


def main_camera_bridge() -> None:
    _run(CameraBridgeNode)


def main_thermal_bridge() -> None:
    _run(ThermalBridgeNode)


def main_hardware_control() -> None:
    _run(HardwareControlNode)


def main_flexbe_adapter() -> None:
    _run(FlexBEAdapterNode)


def main_diagnostics() -> None:
    _run(DiagnosticsNode)


def main_emergency_control() -> None:
    _run(EmergencyControlNode)
