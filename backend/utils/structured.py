from __future__ import annotations

import datetime as dt
import traceback
from typing import Any


def utc_timestamp() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def exception_payload(exc: Exception) -> dict[str, Any]:
    return {
        "error_type": type(exc).__name__,
        "message": str(exc),
        "traceback": traceback.format_exc(),
    }


def event_payload(module: str, severity: str, event: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
    return {
        "timestamp": utc_timestamp(),
        "module": module,
        "severity": severity.upper(),
        "event": event,
        "payload": payload or {},
    }
