from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class HardwareAdapter:
    name: str
    node_name: str
    enabled: bool = True
    last_status: dict[str, Any] = field(default_factory=lambda: {"ok": True, "reason": "initialized"})

    def health_check(self) -> dict[str, Any]:
        return {"name": self.name, "node_name": self.node_name, "enabled": self.enabled, "status": self.last_status}

    async def restart(self) -> dict[str, Any]:
        self.last_status = {"ok": True, "reason": "restart requested"}
        return self.health_check()

    async def enable(self) -> dict[str, Any]:
        self.enabled = True
        self.last_status = {"ok": True, "reason": "enabled"}
        return self.health_check()

    async def disable(self) -> dict[str, Any]:
        self.enabled = False
        self.last_status = {"ok": True, "reason": "disabled"}
        return self.health_check()

    async def remap_topic(self, topic: str, remap_to: str) -> dict[str, Any]:
        self.last_status = {"ok": True, "reason": "remap requested", "topic": topic, "remap_to": remap_to}
        return self.health_check()


class OmniDriveAdapter(HardwareAdapter):
    def __init__(self) -> None:
        super().__init__(name="omni_drive", node_name="omni_drive_controller")


class LinearRailAdapter(HardwareAdapter):
    def __init__(self) -> None:
        super().__init__(name="linear_rail", node_name="linear_rail_controller")


class ArmControllerAdapter(HardwareAdapter):
    def __init__(self) -> None:
        super().__init__(name="arm_controller", node_name="arm_controller")


class ThermalSensorAdapter(HardwareAdapter):
    def __init__(self) -> None:
        super().__init__(name="thermal_sensor", node_name="thermal_sensor_node")


class ProximitySensorAdapter(HardwareAdapter):
    def __init__(self) -> None:
        super().__init__(name="proximity_sensor", node_name="proximity_sensor_node")


class VictimDetectionAdapter(HardwareAdapter):
    def __init__(self) -> None:
        super().__init__(name="victim_detection", node_name="victim_detection_node")
