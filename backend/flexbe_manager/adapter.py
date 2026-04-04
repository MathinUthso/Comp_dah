from __future__ import annotations

from typing import Any

from backend.ros_bridge.client import RosBridgeClient


class FlexBEAdapter:
    def __init__(self, rosbridge: RosBridgeClient) -> None:
        self.rosbridge = rosbridge
        self.state_topic = "/flexbe/active_state"
        self.behavior_topic = "/flexbe/active_behavior"
        self.command_topic = "/flexbe/command"

    async def active_state(self) -> dict[str, Any]:
        return {"topic": self.state_topic, "value": "requested"}

    async def active_behavior(self) -> dict[str, Any]:
        return {"topic": self.behavior_topic, "value": "requested"}

    async def start_behavior(self, behavior: str) -> dict[str, Any]:
        await self.rosbridge.publish(self.command_topic, {"command": "start", "behavior": behavior})
        return {"ok": True, "behavior": behavior}

    async def reset_behavior(self) -> dict[str, Any]:
        await self.rosbridge.publish(self.command_topic, {"command": "reset"})
        return {"ok": True}

    async def abort_behavior(self) -> dict[str, Any]:
        await self.rosbridge.publish(self.command_topic, {"command": "abort"})
        return {"ok": True}

    async def trigger_transition(self, transition: str) -> dict[str, Any]:
        await self.rosbridge.publish(self.command_topic, {"command": "transition", "transition": transition})
        return {"ok": True, "transition": transition}
