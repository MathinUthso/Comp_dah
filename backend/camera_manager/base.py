from __future__ import annotations

import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any

import cv2


@dataclass
class CameraSettings:
    fps: int = 20
    width: int = 1280
    height: int = 720
    rotate_degrees: int = 0
    flip_horizontal: bool = False
    flip_vertical: bool = False
    extra: dict[str, Any] = field(default_factory=dict)


class BaseCamera(ABC):
    def __init__(self, camera_id: str, device: str | None = None, settings: CameraSettings | None = None) -> None:
        self.camera_id = camera_id
        self.device = device
        self.settings = settings or CameraSettings()
        self._capture: cv2.VideoCapture | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._frame_lock = threading.Lock()
        self._last_frame = None
        self._last_frame_ts = 0.0
        self._health = {"connected": False, "reconnects": 0, "last_error": None}

    @abstractmethod
    def start(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def reconnect(self) -> None:
        raise NotImplementedError

    def get_frame(self):
        with self._frame_lock:
            return self._last_frame

    def update_settings(self, **kwargs: Any) -> None:
        for key, value in kwargs.items():
            if hasattr(self.settings, key):
                setattr(self.settings, key, value)

    def health_check(self) -> dict[str, Any]:
        return dict(self._health, last_frame_ts=self._last_frame_ts, camera_id=self.camera_id, device=self.device)

    def _capture_loop(self, open_capture) -> None:
        while not self._stop.is_set():
            try:
                if self._capture is None:
                    self._capture = open_capture()
                    self._health["connected"] = True
                ok, frame = self._capture.read()
                if not ok or frame is None:
                    self._health["connected"] = False
                    time.sleep(0.25)
                    continue
                frame = self._postprocess_frame(frame)
                with self._frame_lock:
                    self._last_frame = frame
                    self._last_frame_ts = time.time()
                time.sleep(max(0.0, 1.0 / max(self.settings.fps, 1)))
            except Exception as exc:
                self._health["connected"] = False
                self._health["last_error"] = str(exc)
                time.sleep(0.5)

    def _postprocess_frame(self, frame):
        if self.settings.rotate_degrees == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.settings.rotate_degrees == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif self.settings.rotate_degrees == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        if self.settings.flip_horizontal:
            frame = cv2.flip(frame, 1)
        if self.settings.flip_vertical:
            frame = cv2.flip(frame, 0)
        return frame
