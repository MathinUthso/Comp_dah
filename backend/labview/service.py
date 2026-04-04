from __future__ import annotations

from typing import Any

from backend.camera_manager.manager import CameraManager
from backend.config.service import ConfigService
from backend.hardware_interface.adapters import (
    ArmControllerAdapter,
    HardwareAdapter,
    LinearRailAdapter,
    OmniDriveAdapter,
    ProximitySensorAdapter,
    ThermalSensorAdapter,
    VictimDetectionAdapter,
)
from backend.ros_bridge.client import RosBridgeClient


class LabViewService:
    def __init__(self, camera_manager: CameraManager, config_service: ConfigService, rosbridge: RosBridgeClient) -> None:
        self.camera_manager = camera_manager
        self.config_service = config_service
        self.rosbridge = rosbridge
        self.adapters: dict[str, HardwareAdapter] = {
            "omni": OmniDriveAdapter(),
            "rail": LinearRailAdapter(),
            "arm": ArmControllerAdapter(),
            "thermal_sensor": ThermalSensorAdapter(),
            "proximity": ProximitySensorAdapter(),
            "victim_detection": VictimDetectionAdapter(),
        }

    async def _safe_ros_action(self, action: str, func, **kwargs: Any) -> dict[str, Any]:
        try:
            result = await func()
            if isinstance(result, dict):
                result.setdefault("ok", True)
                return result
            return {"ok": True, "action": action, "result": result, **kwargs}
        except Exception as exc:
            return {"ok": False, "action": action, "error": str(exc), **kwargs}

    async def detect_devices(self) -> dict[str, Any]:
        return self.camera_manager.discover()

    async def assign_source(self, tile: str, device: str) -> dict[str, Any]:
        if tile == "THERMAL":
            return self.camera_manager.set_thermal_device(device)
        return self.camera_manager.assign(tile, device)

    async def rotate_selected_source(self, tile: str, degrees: int) -> dict[str, Any]:
        return self.camera_manager.update_camera(tile, {"rotate_degrees": degrees})

    async def flip_selected_source(self, tile: str, horizontal: bool = False, vertical: bool = False) -> dict[str, Any]:
        return self.camera_manager.update_camera(tile, {"flip_horizontal": horizontal, "flip_vertical": vertical})

    async def change_camera_timing(self, tile: str, fps: int, width: int, height: int) -> dict[str, Any]:
        return self.camera_manager.update_camera(tile, {"fps": fps, "width": width, "height": height})

    async def restart_stream(self, tile: str) -> dict[str, Any]:
        if tile == "THERMAL":
            self.camera_manager.thermal.reconnect()
        else:
            self.camera_manager.hdmi_slots[tile].reconnect()
        return {"ok": True, "tile": tile}

    async def stream_diagnostics(self) -> dict[str, Any]:
        return self.camera_manager.metadata()

    async def ros_graph(self) -> dict[str, Any]:
        return await self._safe_ros_action("ros_graph", self.rosbridge.graph_snapshot)

    async def topic_echo(self, topic: str) -> dict[str, Any]:
        return await self._safe_ros_action("topic_echo", lambda: self.rosbridge.subscribe(topic), topic=topic)

    async def service_browser(self) -> dict[str, Any]:
        return {"ok": True, "services": ["/rosapi/services", "/rosapi/service_type"]}

    async def parameter_edit(self, node: str, parameter: str, value: Any) -> dict[str, Any]:
        return await self._safe_ros_action(
            "parameter_edit",
            lambda: self.rosbridge.publish("/dashboard/parameter_edit", {"node": node, "parameter": parameter, "value": value}),
            node=node,
            parameter=parameter,
            value=value,
        )

    async def node_restart(self, node: str) -> dict[str, Any]:
        return await self._safe_ros_action("node_restart", lambda: self.rosbridge.publish("/dashboard/node_restart", {"node": node}), node=node)

    async def node_enable(self, node: str) -> dict[str, Any]:
        return await self._safe_ros_action("node_enable", lambda: self.rosbridge.publish("/dashboard/node_enable", {"node": node}), node=node)

    async def node_disable(self, node: str) -> dict[str, Any]:
        return await self._safe_ros_action("node_disable", lambda: self.rosbridge.publish("/dashboard/node_disable", {"node": node}), node=node)

    async def remap_topic(self, node: str, topic: str, remap_to: str) -> dict[str, Any]:
        return await self._safe_ros_action(
            "remap_topic",
            lambda: self.rosbridge.publish("/dashboard/remap_topic", {"node": node, "topic": topic, "remap_to": remap_to}),
            node=node,
            topic=topic,
            remap_to=remap_to,
        )

    async def emergency_debug(self) -> dict[str, Any]:
        return {"ok": True, "adapters": {name: adapter.health_check() for name, adapter in self.adapters.items()}}
