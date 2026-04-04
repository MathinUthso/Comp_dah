from __future__ import annotations

from typing import Any

from backend.camera_manager.manager import CameraManager
from backend.config.service import ConfigService
from backend.flexbe_manager.adapter import FlexBEAdapter
from backend.ros_bridge.client import RosBridgeClient
from backend.state_sync.service import StateSyncService


class DiagnosticsCollector:
    def __init__(self, camera_manager: CameraManager, config_service: ConfigService, rosbridge: RosBridgeClient, flexbe: FlexBEAdapter, state_sync: StateSyncService) -> None:
        self.camera_manager = camera_manager
        self.config_service = config_service
        self.rosbridge = rosbridge
        self.flexbe = flexbe
        self.state_sync = state_sync

    async def collect(self) -> dict[str, Any]:
        ros_graph: dict[str, Any]
        try:
            ros_graph = await self.rosbridge.graph_snapshot()
        except Exception as exc:
            ros_graph = {
                "status": "unavailable",
                "source": "rosbridge",
                "ok": False,
                "error": str(exc),
            }

        flexbe_payload: dict[str, Any]
        try:
            flexbe_payload = {
                "active_state": await self.flexbe.active_state(),
                "active_behavior": await self.flexbe.active_behavior(),
                "ok": True,
            }
        except Exception as exc:
            flexbe_payload = {
                "active_state": {"value": "unknown"},
                "active_behavior": {"value": "unknown"},
                "ok": False,
                "error": str(exc),
            }

        return {
            "config": self.config_service.get(),
            "cameras": self.camera_manager.metadata(),
            "state": self.state_sync.snapshot(),
            "flexbe": flexbe_payload,
            "ros_graph": ros_graph,
        }
