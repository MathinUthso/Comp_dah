from __future__ import annotations

import logging
from typing import Any

from backend.logs.service import LogService


class StructuredLogger:
    def __init__(self, module: str, log_service: LogService) -> None:
        self.module = module
        self.log_service = log_service

    async def info(self, event: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        return await self.log_service.publish(self.module, "INFO", event, payload)

    async def warn(self, event: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        return await self.log_service.publish(self.module, "WARN", event, payload)

    async def error(self, event: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        return await self.log_service.publish(self.module, "ERROR", event, payload)

    async def debug(self, event: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        return await self.log_service.publish(self.module, "DEBUG", event, payload)


def configure_python_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )
