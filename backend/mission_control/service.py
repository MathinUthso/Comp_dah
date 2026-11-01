from __future__ import annotations

import asyncio
from typing import Any

from backend.hardware_interface.critical import CriticalCommandResult, CriticalRosPublisher
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
        # rclpy service calls block; keep them off the asyncio event loop.
        result = await asyncio.to_thread(self.critical.publish_emergency_reset)
        if self.rosbridge:
            await self.rosbridge.publish("/mission/reset", {"data": True})
        return {"ok": result.ok, "action": "reset", "message": result.message}

    def _estop_sync(self) -> tuple[CriticalCommandResult, CriticalCommandResult]:
        return self.critical.publish_estop(), self.critical.publish_motor_halt()

    async def estop(self) -> dict[str, Any]:
        stop, halt = await asyncio.to_thread(self._estop_sync)
        if self.rosbridge:
            await self.rosbridge.publish("/mission/estop", {"data": True})
        return {"ok": stop.ok and halt.ok, "action": "estop", "details": [stop.payload, halt.payload]}

    async def dpad(self, direction: str) -> dict[str, Any]:
        if self.rosbridge:
            mapping = {
                "up": {"linear": {"x": 0.5, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}},
                "down": {"linear": {"x": -0.5, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}},
                "left": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.6}},
                "right": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": -0.6}},
                "stop": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}},
            }
            key = direction.strip().lower()
            if key not in mapping:
                return {"ok": False, "direction": direction, "message": "unknown direction"}
            await self.rosbridge.publish("/cmd_vel", mapping[key])
        return {"ok": True, "direction": direction}

    async def behavior_command(self, command: str, behavior: str | None = None) -> dict[str, Any]:
        if self.rosbridge:
            # Current backend workspace uses /flexbe/execute_behavior (AddTwoInts mock)
            behavior_ids = {
                "move_forward": 1,
                "move_backward": 2,
                "rotate_left": 3,
                "rotate_right": 4,
                "rotate_180": 5,
            }
            cmd = command.strip().lower()
            selected = behavior if behavior is not None else cmd
            behavior_id = behavior_ids.get(selected, 0)
            await self.rosbridge.call_service("/flexbe/execute_behavior", {"a": int(behavior_id), "b": 0})
        return {"ok": True, "command": command, "behavior": behavior}
