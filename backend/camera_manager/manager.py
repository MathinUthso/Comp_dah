from __future__ import annotations

from dataclasses import asdict
from typing import Any

from backend.camera_manager.hdmi import HDMIReceiverCamera
from backend.camera_manager.thermal import ThermalUSBCamera


class CameraManager:
    def __init__(self) -> None:
        self.hdmi_slots = {f"CAM_{idx:02d}": HDMIReceiverCamera(camera_id=f"CAM_{idx:02d}") for idx in range(1, 5)}
        self.thermal = ThermalUSBCamera()
        self._assignments: dict[str, str | None] = {slot: None for slot in self.hdmi_slots}

    def discover(self) -> dict[str, Any]:
        return {
            "hdmi": HDMIReceiverCamera.discover_devices(),
            "thermal": ThermalUSBCamera.discover_devices(),
            "assignments": dict(self._assignments),
        }

    def assign(self, slot: str, device: str) -> dict[str, Any]:
        if slot not in self.hdmi_slots:
            raise ValueError(f"unknown slot {slot}")
        self._assignments[slot] = device
        camera = self.hdmi_slots[slot]
        camera.device = device
        camera.reconnect()
        return {"slot": slot, "device": device}

    def set_thermal_device(self, device: str) -> dict[str, Any]:
        self.thermal.device = device
        self.thermal.reconnect()
        return {"device": device}

    def start_all(self) -> None:
        discovered = self.discover()
        hdmi_devices = [item["device"] for item in discovered["hdmi"] if item.get("is_hdmi")]
        thermal_devices = [item["device"] for item in discovered["thermal"] if item.get("is_thermal")]
        all_hdmi_devices = [item["device"] for item in discovered["hdmi"]]
        all_thermal_devices = [item["device"] for item in discovered["thermal"]]

        if not hdmi_devices:
            hdmi_devices = all_hdmi_devices
        if not thermal_devices:
            thermal_devices = all_thermal_devices
        for slot, device in zip(self.hdmi_slots, hdmi_devices, strict=False):
            if self._assignments[slot] is None:
                self._assignments[slot] = device
                self.hdmi_slots[slot].device = device
        if self.thermal.device is None and thermal_devices:
            self.thermal.device = thermal_devices[0]
        for camera in self.hdmi_slots.values():
            camera.start()
        self.thermal.start()

    def stop_all(self) -> None:
        for camera in self.hdmi_slots.values():
            camera.stop()
        self.thermal.stop()

    def update_camera(self, slot: str, settings: dict[str, Any]) -> dict[str, Any]:
        camera = self.thermal if slot == "THERMAL" else self.hdmi_slots[slot]
        camera.update_settings(**settings)
        return {"slot": slot, "settings": settings}

    def metadata(self) -> dict[str, Any]:
        return {
            "slots": {name: {"device": cam.device, "health": cam.health_check(), "settings": asdict(cam.settings)} for name, cam in self.hdmi_slots.items()},
            "thermal": {"device": self.thermal.device, "health": self.thermal.health_check(), "settings": asdict(self.thermal.settings)},
            "assignments": dict(self._assignments),
        }

    def frame_snapshot(self, slot: str):
        camera = self.thermal if slot == "THERMAL" else self.hdmi_slots[slot]
        return camera.get_frame()
