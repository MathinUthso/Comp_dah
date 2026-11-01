from __future__ import annotations

from typing import Any

from backend.ros_bridge.client import RosBridgeClient


class FlexBEAdapter:
    def __init__(self, rosbridge: RosBridgeClient) -> None:
        self.rosbridge = rosbridge
        self.state_topic = "/flexbe/status"
        self.behavior_topic = "/flexbe/status"
        self.command_service = "/flexbe/execute_behavior"

    def _behavior_to_id(self, behavior: str) -> int:
        mapping = {
            "move_forward": 1,
            "move_backward": 2,
            "rotate_left": 3,
            "rotate_right": 4,
            "rotate_180": 5,
        }
        return mapping.get(behavior, 0)

    async def active_state(self) -> dict[str, Any]:
        return {"topic": self.state_topic, "value": "requested"}

    async def active_behavior(self) -> dict[str, Any]:
        return {"topic": self.behavior_topic, "value": "requested"}

    async def start_behavior(self, behavior: str) -> dict[str, Any]:
        behavior_id = self._behavior_to_id(behavior)
        result = await self.rosbridge.call_service(self.command_service, {"a": behavior_id, "b": 0})
        return {"ok": True, "behavior": behavior, "result": result}

    async def reset_behavior(self) -> dict[str, Any]:
        result = await self.rosbridge.call_service(self.command_service, {"a": 0, "b": 0})
        return {"ok": True, "result": result}

    async def abort_behavior(self) -> dict[str, Any]:
        result = await self.rosbridge.call_service(self.command_service, {"a": 0, "b": 0})
        return {"ok": True, "result": result}

    async def trigger_transition(self, transition: str) -> dict[str, Any]:
        result = await self.rosbridge.call_service(self.command_service, {"a": 0, "b": 0})
        return {"ok": True, "transition": transition, "result": result}
