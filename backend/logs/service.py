from __future__ import annotations

import asyncio
from collections import deque
from typing import Any, Callable, Deque

from backend.utils.structured import event_payload


class LogService:
    def __init__(self, capacity: int = 5000) -> None:
        self._records: Deque[dict[str, Any]] = deque(maxlen=capacity)
        self._subscribers: set[Callable[[dict[str, Any]], Any]] = set()
        self._lock = asyncio.Lock()

    async def publish(self, module: str, severity: str, event: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        record = event_payload(module, severity, event, payload)
        async with self._lock:
            self._records.append(record)
            subscribers = list(self._subscribers)
        for subscriber in subscribers:
            try:
                result = subscriber(record)
                if asyncio.iscoroutine(result):
                    await result
            except Exception:
                continue
        return record

    def snapshot(self, limit: int = 250) -> list[dict[str, Any]]:
        return list(self._records)[-limit:]

    def attach(self, subscriber: Callable[[dict[str, Any]], Any]) -> None:
        self._subscribers.add(subscriber)

    def detach(self, subscriber: Callable[[dict[str, Any]], Any]) -> None:
        self._subscribers.discard(subscriber)
