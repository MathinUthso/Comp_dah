from __future__ import annotations

import asyncio
import json
from typing import Any

import websockets


class RosBridgeClient:
    def __init__(self, url: str) -> None:
        self.url = url
        self._ws: Any | None = None
        self._lock = asyncio.Lock()
        self._request_id = 0

    def _is_open(self) -> bool:
        if self._ws is None:
            return False
        closed = getattr(self._ws, "closed", None)
        if isinstance(closed, bool):
            return not closed
        state = getattr(self._ws, "state", None)
        if state is None:
            return True
        state_name = getattr(state, "name", str(state)).upper()
        return "OPEN" in state_name

    async def connect(self) -> None:
        async with self._lock:
            if not self._is_open():
                self._ws = await websockets.connect(self.url, ping_interval=10, ping_timeout=10)

    async def close(self) -> None:
        async with self._lock:
            if self._ws and self._is_open():
                await self._ws.close()
            self._ws = None

    async def _ensure(self) -> Any:
        await self.connect()
        if self._ws is None:
            raise RuntimeError("rosbridge connection unavailable")
        return self._ws

    async def send(self, message: dict[str, Any]) -> None:
        ws = await self._ensure()
        await ws.send(json.dumps(message))

    async def call_service(self, service: str, args: dict[str, Any] | None = None) -> dict[str, Any]:
        self._request_id += 1
        await self.send({"op": "call_service", "service": service, "args": args or {}, "id": str(self._request_id)})
        ws = await self._ensure()
        response = json.loads(await ws.recv())
        return response

    async def publish(self, topic: str, msg: dict[str, Any]) -> None:
        await self.send({"op": "publish", "topic": topic, "msg": msg})

    async def subscribe(self, topic: str) -> None:
        await self.send({"op": "subscribe", "topic": topic})

    async def graph_snapshot(self) -> dict[str, Any]:
        await self.subscribe("/rosapi/nodes")
        return {"status": "requested", "source": "rosbridge"}
