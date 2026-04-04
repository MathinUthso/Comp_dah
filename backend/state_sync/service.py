from __future__ import annotations

import asyncio
from typing import Any


class StateSyncService:
    def __init__(self) -> None:
        self._state: dict[str, Any] = {
            "ok": True,
            "autonomous": False,
            "progress": 0,
            "victim_detection": False,
            "rail_status": "idle",
            "omni_status": "idle",
        }
        self._notifier = None

    def set_notifier(self, notifier) -> None:
        self._notifier = notifier

    def update(self, patch: dict[str, Any]) -> dict[str, Any]:
        self._state.update(patch)
        if self._notifier is not None:
            result = self._notifier(dict(self._state))
            if asyncio.iscoroutine(result):
                asyncio.create_task(result)
            return dict(self._state)
        return dict(self._state)

    def snapshot(self) -> dict[str, Any]:
        return dict(self._state)
