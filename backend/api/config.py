from __future__ import annotations

from fastapi import APIRouter, Request

router = APIRouter()


def _svc(request: Request):
    return request.app.state.services["config"]


@router.get("")
async def get_config(request: Request):
    return _svc(request).get()


@router.put("")
async def update_config(request: Request, patch: dict):
    return _svc(request).update(patch)


@router.post("/reload")
async def reload_config(request: Request):
    return _svc(request).reload()
