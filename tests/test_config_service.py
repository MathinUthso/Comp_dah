from __future__ import annotations

from pathlib import Path

from backend.config.service import ConfigService
from backend.core.settings import Settings


def test_runtime_config_path_defaults_to_absolute_repo_path() -> None:
    assert Settings().runtime_config_path.is_absolute()


def test_config_service_creates_defaults_and_deep_merges(tmp_path: Path) -> None:
    config_path = tmp_path / "runtime.json"
    service = ConfigService(Settings(runtime_config_path=config_path))

    assert service.get()["thermal"]["rotation"] == 90
    updated = service.update({"thermal": {"rotation": 180}})

    assert updated["thermal"]["rotation"] == 180
    assert updated["thermal"]["flip"] is False
    assert config_path.exists()


def test_config_service_reloads_from_disk(tmp_path: Path) -> None:
    config_path = tmp_path / "runtime.json"
    service = ConfigService(Settings(runtime_config_path=config_path))
    service.update({"mission": {"mode": "running"}})

    reloaded = service.reload()

    assert reloaded["mission"]["mode"] == "running"
