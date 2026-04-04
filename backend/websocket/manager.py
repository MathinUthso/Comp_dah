from __future__ import annotations

import asyncio
from typing import Any

from fastapi import WebSocket, WebSocketDisconnect


class WebSocketManager:
    def __init__(self) -> None:
        self._channels: dict[str, set[WebSocket]] = {
            "logs": set(),
            "telemetry": set(),
            "state": set(),
            "ros_graph": set(),
            "camera_metadata": set(),
        }
        self._locks = {name: asyncio.Lock() for name in self._channels}

    async def connect(self, channel: str, websocket: WebSocket) -> None:
        await websocket.accept()
        async with self._locks[channel]:
            self._channels[channel].add(websocket)

    async def disconnect(self, channel: str, websocket: WebSocket) -> None:
        async with self._locks[channel]:
            self._channels[channel].discard(websocket)

    async def broadcast(self, channel: str, payload: dict[str, Any]) -> None:
        stale: list[WebSocket] = []
        async with self._locks[channel]:
            subscribers = list(self._channels[channel])
        for websocket in subscribers:
            try:
                await websocket.send_json(payload)
            except WebSocketDisconnect:
                stale.append(websocket)
            except Exception:
                stale.append(websocket)
        if stale:
            async with self._locks[channel]:
                for websocket in stale:
                    self._channels[channel].discard(websocket)
