from __future__ import annotations

import glob
import os
import threading
from typing import Any

import cv2

from backend.camera_manager.base import BaseCamera


class HDMIReceiverCamera(BaseCamera):
    DEVICE_HINTS = ("HDMI", "Capture", "Cam Link", "UVC", "USB Video", "video capture")

    @classmethod
    def discover_devices(cls) -> list[dict[str, Any]]:
        devices = []
        for path in sorted(glob.glob("/sys/class/video4linux/video*/name")):
            video = f"/dev/{os.path.basename(os.path.dirname(path))}"
            try:
                name = open(path).read().strip()
            except Exception:
                name = "unknown"
            devices.append({"device": video, "name": name, "is_hdmi": any(h.lower() in name.lower() for h in cls.DEVICE_HINTS)})
        return devices

    def start(self) -> None:
        self._stop.clear()
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._capture_loop, args=(self._open_capture,), daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    def reconnect(self) -> None:
        self.stop()
        self._health["reconnects"] += 1
        self.start()

    def _open_capture(self):
        if self.device is None:
            raise RuntimeError(f"camera {self.camera_id} has no device assignment")
        capture = cv2.VideoCapture(self.device)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        capture.set(cv2.CAP_PROP_FPS, self.settings.fps)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.settings.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.settings.height)
        return capture

    def update_settings(self, **kwargs: Any) -> None:
        super().update_settings(**kwargs)
        if self._capture is not None:
            self._capture.set(cv2.CAP_PROP_FPS, self.settings.fps)
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.settings.width)
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.settings.height)
