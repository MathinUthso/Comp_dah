from __future__ import annotations

from functools import lru_cache
from pathlib import Path

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

PROJECT_ROOT = Path(__file__).resolve().parents[2]


class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_prefix="RESCUE_", env_file=".env", extra="ignore")

    app_name: str = "rescue-robot-dashboard-backend"
    api_host: str = "0.0.0.0"
    api_port: int = 8000
    log_level: str = "INFO"

    rosbridge_url: str = "ws://127.0.0.1:9090"
    ros_namespace: str = "/"
    critical_estop_service: str = "/emergency/estop"
    critical_halt_topic: str = "/cmd_vel"
    critical_reset_service: str = "/emergency/reset"

    runtime_config_path: Path = Field(default=PROJECT_ROOT / "backend" / "config" / "runtime.json")
    camera_device_glob: str = "/dev/video*"
    default_camera_fps: int = 20
    default_camera_width: int = 1280
    default_camera_height: int = 720
    default_thermal_rotation: int = 90

    websocket_heartbeat_seconds: int = 15
    diagnostics_refresh_seconds: int = 2
    enable_web_assets: bool = True


@lru_cache
def get_settings() -> Settings:
    return Settings()
