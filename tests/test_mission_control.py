from __future__ import annotations

import asyncio

from backend.hardware_interface.critical import CriticalCommandResult
from backend.mission_control.service import MissionControlService


class FakeCritical:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def publish_estop(self) -> CriticalCommandResult:
        self.calls.append("estop")
        return CriticalCommandResult(True, "estop ok", {"service": "/emergency/estop"})

    def publish_motor_halt(self) -> CriticalCommandResult:
        self.calls.append("halt")
        return CriticalCommandResult(True, "halt ok", {"topic": "/cmd_vel"})

    def publish_emergency_reset(self) -> CriticalCommandResult:
        self.calls.append("reset")
        return CriticalCommandResult(True, "reset ok", {"service": "/emergency/reset"})


class FakeRosBridge:
    def __init__(self) -> None:
        self.published: list[tuple[str, dict]] = []

    async def publish(self, topic: str, message: dict) -> None:
        self.published.append((topic, message))


def test_estop_runs_critical_calls_and_notifies_rosbridge() -> None:
    critical = FakeCritical()
    bridge = FakeRosBridge()

    result = asyncio.run(MissionControlService(critical, bridge).estop())

    assert result["ok"] is True
    assert critical.calls == ["estop", "halt"]
    assert bridge.published == [("/mission/estop", {"data": True})]


def test_reset_uses_direct_critical_service() -> None:
    critical = FakeCritical()

    result = asyncio.run(MissionControlService(critical).reset())

    assert result["ok"] is True
    assert critical.calls == ["reset"]


def test_dpad_publishes_mapped_twist_and_rejects_unknown_direction() -> None:
    bridge = FakeRosBridge()
    service = MissionControlService(FakeCritical(), bridge)

    result = asyncio.run(service.dpad("left"))
    rejected = asyncio.run(service.dpad("sideways"))

    assert result["ok"] is True
    assert bridge.published == [
        ("/cmd_vel", {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.6}})
    ]
    assert rejected["ok"] is False
