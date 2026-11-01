from __future__ import annotations

import asyncio
import json
import threading
from dataclasses import dataclass
from time import monotonic
from typing import Any

import websockets


@dataclass
class CameraStatusState:
    video2_healthy: bool = False
    webcam_healthy: bool = False
    primary_source: str = "none"
    connected: bool = False
    last_update_monotonic: float = 0.0


class RosCameraStatusService:
    """Tracks camera health/source topics through rosbridge and exposes latest snapshot."""

    VIDEO2_HEALTH_TOPIC = "/camera/video2/healthy"
    WEBCAM_HEALTH_TOPIC = "/camera/webcam/healthy"
    PRIMARY_SOURCE_TOPIC = "/camera/primary_source"
    VIDEO2_HEALTH_TYPE = "std_msgs/Bool"
    WEBCAM_HEALTH_TYPE = "std_msgs/Bool"
    PRIMARY_SOURCE_TYPE = "std_msgs/String"

    def __init__(self, rosbridge_url: str, stale_after_seconds: float = 5.0) -> None:
        self._url = rosbridge_url
        self._stale_after_seconds = stale_after_seconds
        self._state = CameraStatusState()
        self._lock = threading.Lock()
        self._task: asyncio.Task[Any] | None = None
        self._stop_event = asyncio.Event()

    async def start(self) -> None:
        if self._task is not None and not self._task.done():
            return
        self._stop_event = asyncio.Event()
        self._task = asyncio.create_task(self._run(), name="ros-camera-status")

    async def stop(self) -> None:
        self._stop_event.set()
        if self._task is None:
            return
        self._task.cancel()
        try:
            await self._task
        except asyncio.CancelledError:
            pass
        self._task = None

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            age_seconds = monotonic() - self._state.last_update_monotonic if self._state.last_update_monotonic else None
            stale = True if age_seconds is None else age_seconds > self._stale_after_seconds
            return {
                "video2_healthy": self._state.video2_healthy,
                "webcam_healthy": self._state.webcam_healthy,
                "primary_source": self._state.primary_source,
                "connected": self._state.connected,
                "stale": stale,
                "age_seconds": age_seconds,
            }

    async def _run(self) -> None:
        backoff = 1.0
        while not self._stop_event.is_set():
            try:
                async with websockets.connect(self._url, ping_interval=10, ping_timeout=10) as ws:
                    self._set_connected(True)
                    await self._subscribe(ws, self.VIDEO2_HEALTH_TOPIC, self.VIDEO2_HEALTH_TYPE)
                    await self._subscribe(ws, self.WEBCAM_HEALTH_TOPIC, self.WEBCAM_HEALTH_TYPE)
                    await self._subscribe(ws, self.PRIMARY_SOURCE_TOPIC, self.PRIMARY_SOURCE_TYPE)
                    backoff = 1.0

                    while not self._stop_event.is_set():
                        raw = await ws.recv()
                        self._handle_message(raw)
            except asyncio.CancelledError:
                raise
            except Exception:
                self._set_connected(False)
                try:
                    await asyncio.wait_for(self._stop_event.wait(), timeout=backoff)
                except asyncio.TimeoutError:
                    pass
                backoff = min(backoff * 2.0, 8.0)

    async def _subscribe(self, ws: Any, topic: str, topic_type: str) -> None:
        await ws.send(json.dumps({"op": "subscribe", "topic": topic, "type": topic_type}))

    def _set_connected(self, connected: bool) -> None:
        with self._lock:
            self._state.connected = connected

    def _handle_message(self, raw: Any) -> None:
        if isinstance(raw, bytes):
            raw = raw.decode("utf-8", errors="ignore")
        try:
            payload = json.loads(raw)
        except Exception:
            return

        if payload.get("op") != "publish":
            return

        topic = str(payload.get("topic") or "")
        msg = payload.get("msg") if isinstance(payload.get("msg"), dict) else {}

        with self._lock:
            if topic == self.VIDEO2_HEALTH_TOPIC:
                self._state.video2_healthy = bool(msg.get("data", False))
                self._state.last_update_monotonic = monotonic()
            elif topic == self.WEBCAM_HEALTH_TOPIC:
                self._state.webcam_healthy = bool(msg.get("data", False))
                self._state.last_update_monotonic = monotonic()
            elif topic == self.PRIMARY_SOURCE_TOPIC:
                source = str(msg.get("data") or "none").strip().lower()
                if source not in {"video2", "webcam", "none"}:
                    source = "none"
                self._state.primary_source = source
                self._state.last_update_monotonic = monotonic()
