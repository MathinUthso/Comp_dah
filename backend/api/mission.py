from __future__ import annotations

from fastapi import APIRouter, Request

router = APIRouter()


def _svc(request: Request):
    return request.app.state.services["mission"]


@router.post("/start")
async def start_mission(request: Request):
    return await _svc(request).start()


@router.post("/reset")
async def reset_mission(request: Request):
    return await _svc(request).reset()


@router.post("/estop")
async def estop_mission(request: Request):
    return await _svc(request).estop()


@router.post("/dpad/{direction}")
async def dpad_command(direction: str, request: Request):
    return await _svc(request).dpad(direction)


@router.post("/behavior/{command}")
async def behavior_command(command: str, request: Request, behavior: str | None = None):
    return await _svc(request).behavior_command(command, behavior)
