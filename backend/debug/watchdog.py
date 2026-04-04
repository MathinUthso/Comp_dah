from __future__ import annotations

import asyncio
import traceback
from typing import Any, Awaitable, Callable


class Watchdog:
    def __init__(self) -> None:
        self._tasks: dict[str, Callable[[], Awaitable[Any]]] = {}

    def register(self, name: str, probe: Callable[[], Awaitable[Any]]) -> None:
        self._tasks[name] = probe

    async def run_once(self) -> dict[str, Any]:
        results: dict[str, Any] = {}
        for name, probe in self._tasks.items():
            try:
                results[name] = await asyncio.wait_for(probe(), timeout=3.0)
            except Exception as exc:
                results[name] = {"ok": False, "error": str(exc), "traceback": traceback.format_exc()}
        return results
