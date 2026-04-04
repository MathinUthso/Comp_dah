from __future__ import annotations

import glob
import os
import threading
from typing import Any

import cv2

from backend.camera_manager.base import BaseCamera, CameraSettings
from backend.thermal_manager.processor import ThermalProcessor


class ThermalUSBCamera(BaseCamera):
    def __init__(self, camera_id: str = "THERMAL", device: str | None = None, settings: CameraSettings | None = None) -> None:
        super().__init__(camera_id, device, settings)
        self.processor = ThermalProcessor(rotation=self.settings.rotate_degrees, flip_horizontal=self.settings.flip_horizontal, flip_vertical=self.settings.flip_vertical)

    @classmethod
    def discover_devices(cls) -> list[dict[str, Any]]:
        devices = []
        for path in sorted(glob.glob("/sys/class/video4linux/video*/name")):
            video = f"/dev/{os.path.basename(os.path.dirname(path))}"
            try:
                name = open(path).read().strip()
            except Exception:
                name = "unknown"
            thermal = any(keyword in name.lower() for keyword in ("thermal", "ir", "flir", "seek", "infrared"))
            devices.append({"device": video, "name": name, "is_thermal": thermal})
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
            raise RuntimeError(f"thermal camera {self.camera_id} has no device assignment")
        capture = cv2.VideoCapture(self.device)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def _postprocess_frame(self, frame):
        return self.processor.process(frame)

    def update_settings(self, **kwargs: Any) -> None:
        super().update_settings(**kwargs)
        self.processor.rotation = self.settings.rotate_degrees
        self.processor.flip_horizontal = self.settings.flip_horizontal
        self.processor.flip_vertical = self.settings.flip_vertical
