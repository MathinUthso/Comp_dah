from __future__ import annotations

import cv2
import time

from fastapi import APIRouter, Request
from fastapi.responses import StreamingResponse

router = APIRouter()


def _svc(request: Request):
    return request.app.state.services["labview"]


def _camera_manager(request: Request):
    return request.app.state.services["camera_manager"]


@router.get("/discover")
async def discover(request: Request):
    return await _svc(request).detect_devices()


@router.get("/metadata")
async def metadata(request: Request):
    return await _svc(request).stream_diagnostics()


@router.post("/assign")
async def assign_source(request: Request, tile: str, device: str):
    return await _svc(request).assign_source(tile, device)


@router.post("/rotate")
async def rotate_source(request: Request, tile: str, degrees: int):
    return await _svc(request).rotate_selected_source(tile, degrees)


@router.post("/flip")
async def flip_source(request: Request, tile: str, horizontal: bool = False, vertical: bool = False):
    return await _svc(request).flip_selected_source(tile, horizontal=horizontal, vertical=vertical)


@router.post("/timing")
async def timing(request: Request, tile: str, fps: int, width: int, height: int):
    return await _svc(request).change_camera_timing(tile, fps, width, height)


@router.post("/{tile}/restart")
async def restart_stream(tile: str, request: Request):
    return await _svc(request).restart_stream(tile)


@router.get("/{tile}/stream.mjpeg")
async def mjpeg_stream(tile: str, request: Request):
    camera_manager = _camera_manager(request)

    def frame_generator():
        boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
        while True:
            frame = camera_manager.frame_snapshot(tile)
            if frame is None:
                time.sleep(0.05)
                continue
            ok, encoded = cv2.imencode(".jpg", frame)
            if not ok:
                time.sleep(0.05)
                continue
            yield boundary + encoded.tobytes() + b"\r\n"
            time.sleep(1.0 / max(camera_manager.hdmi_slots[tile].settings.fps if tile != "THERMAL" else camera_manager.thermal.settings.fps, 1))

    return StreamingResponse(frame_generator(), media_type="multipart/x-mixed-replace; boundary=frame")
