from __future__ import annotations

from typing import Any

from backend.hardware_interface.critical import CriticalRosPublisher
from backend.ros_bridge.client import RosBridgeClient


class MissionControlService:
    def __init__(self, critical: CriticalRosPublisher, rosbridge: RosBridgeClient | None = None) -> None:
        self.critical = critical
        self.rosbridge = rosbridge

    async def start(self) -> dict[str, Any]:
        if self.rosbridge:
            await self.rosbridge.publish("/mission/start", {"data": True})
        return {"ok": True, "action": "start"}

    async def reset(self) -> dict[str, Any]:
        result = self.critical.publish_emergency_reset()
        if self.rosbridge:
            await self.rosbridge.publish("/mission/reset", {"data": True})
        return {"ok": result.ok, "action": "reset", "message": result.message}

    async def estop(self) -> dict[str, Any]:
        stop = self.critical.publish_estop()
        halt = self.critical.publish_motor_halt()
        if self.rosbridge:
            await self.rosbridge.publish("/mission/estop", {"data": True})
        return {"ok": stop.ok and halt.ok, "action": "estop", "details": [stop.payload, halt.payload]}

    async def dpad(self, direction: str) -> dict[str, Any]:
        if self.rosbridge:
            await self.rosbridge.publish("/teleop/dpad", {"direction": direction})
        return {"ok": True, "direction": direction}

    async def behavior_command(self, command: str, behavior: str | None = None) -> dict[str, Any]:
        if self.rosbridge:
            await self.rosbridge.publish("/flexbe/command", {"command": command, "behavior": behavior})
        return {"ok": True, "command": command, "behavior": behavior}
