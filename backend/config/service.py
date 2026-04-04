from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from backend.core.settings import Settings


class ConfigService:
    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self.path = Path(settings.runtime_config_path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._config: dict[str, Any] = self._load()

    def _load(self) -> dict[str, Any]:
        if not self.path.exists():
            return self.default_config()
        return json.loads(self.path.read_text())

    def default_config(self) -> dict[str, Any]:
        return {
            "mission": {"mode": "idle", "autonomous_enabled": False},
            "cameras": {"fps": self.settings.default_camera_fps, "width": self.settings.default_camera_width, "height": self.settings.default_camera_height},
            "thermal": {"rotation": self.settings.default_thermal_rotation, "flip": False},
            "hardware": {"watchdog_enabled": True},
        }

    def get(self) -> dict[str, Any]:
        return self._config

    def update(self, patch: dict[str, Any]) -> dict[str, Any]:
        self._config = self._deep_merge(self._config, patch)
        self.path.write_text(json.dumps(self._config, indent=2, sort_keys=True))
        return self._config

    def reload(self) -> dict[str, Any]:
        self._config = self._load()
        return self._config

    def _deep_merge(self, base: dict[str, Any], patch: dict[str, Any]) -> dict[str, Any]:
        merged = dict(base)
        for key, value in patch.items():
            if isinstance(value, dict) and isinstance(merged.get(key), dict):
                merged[key] = self._deep_merge(merged[key], value)
            else:
                merged[key] = value
        return merged
