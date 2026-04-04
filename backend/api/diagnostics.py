from __future__ import annotations

from fastapi import APIRouter, Request

router = APIRouter()


def _svc(request: Request):
    return request.app.state.services["diagnostics"]


@router.get("")
async def diagnostics(request: Request):
    return await _svc(request).collect()


@router.get("/state")
async def state_snapshot(request: Request):
    return request.app.state.services["state_sync"].snapshot()
