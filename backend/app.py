from __future__ import annotations

from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse

from backend.api.routes import router as api_router
from backend.camera_manager.manager import CameraManager
from backend.config.service import ConfigService
from backend.core.logging import configure_python_logging
from backend.core.settings import get_settings
from backend.debug.watchdog import Watchdog
from backend.diagnostics.collector import DiagnosticsCollector
from backend.flexbe_manager.adapter import FlexBEAdapter
from backend.hardware_interface.critical import CriticalRosPublisher
from backend.labview.service import LabViewService
from backend.logs.service import LogService
from backend.mission_control.service import MissionControlService
from backend.ros_bridge.camera_status import RosCameraStatusService
from backend.ros_bridge.client import RosBridgeClient
from backend.state_sync.service import StateSyncService
from backend.websocket.manager import WebSocketManager


def build_services() -> dict[str, Any]:
    settings = get_settings()
    configure_python_logging(settings.log_level)
    log_service = LogService()
    rosbridge = RosBridgeClient(settings.rosbridge_url)
    camera_status = RosCameraStatusService(settings.rosbridge_url)
    critical = CriticalRosPublisher(
        settings.critical_estop_service,
        settings.critical_halt_topic,
        settings.critical_reset_service,
    )
    camera_manager = CameraManager()
    config_service = ConfigService(settings)
    state_sync = StateSyncService()
    flexbe = FlexBEAdapter(rosbridge)
    mission = MissionControlService(critical, rosbridge)
    labview = LabViewService(camera_manager, config_service, rosbridge)
    diagnostics = DiagnosticsCollector(camera_manager, config_service, rosbridge, flexbe, state_sync)
    websocket_manager = WebSocketManager()
    watchdog = Watchdog()
    services = {
        "settings": settings,
        "logs": log_service,
        "rosbridge": rosbridge,
        "camera_status": camera_status,
        "critical": critical,
        "camera_manager": camera_manager,
        "config": config_service,
        "state_sync": state_sync,
        "flexbe": flexbe,
        "mission": mission,
        "labview": labview,
        "diagnostics": diagnostics,
        "websocket_manager": websocket_manager,
        "watchdog": watchdog,
    }
    return services


@asynccontextmanager
async def lifespan(app: FastAPI):
    services = app.state.services
    services["state_sync"].set_notifier(lambda state: services["websocket_manager"].broadcast("state", {"channel": "state", "payload": state}))
    services["logs"].attach(lambda record: services["websocket_manager"].broadcast("logs", record))
    await services["logs"].publish("app", "INFO", "startup", {"app": app.title})
    try:
        await services["rosbridge"].connect()
    except Exception as exc:
        await services["logs"].publish("rosbridge", "WARN", "connect_failed", {"error": str(exc)})
    await services["camera_status"].start()
    camera_capture = services["settings"].enable_camera_capture
    if camera_capture:
        services["camera_manager"].start_all()
    else:
        await services["logs"].publish("camera", "INFO", "local_capture_disabled", {"owner": "camera_bridge"})
    services["watchdog"].register("diagnostics", services["diagnostics"].collect)
    yield
    await services["camera_status"].stop()
    if camera_capture:
        services["camera_manager"].stop_all()
    await services["rosbridge"].close()


def create_app() -> FastAPI:
    settings = get_settings()
    app = FastAPI(title=settings.app_name, lifespan=lifespan)
    app.state.services = build_services()
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )
    app.include_router(api_router, prefix="/api/v1")

    project_root = Path(__file__).resolve().parent.parent
    frontend_root = project_root / "frontend"

    @app.get("/")
    async def index() -> Any:
        return FileResponse(frontend_root / "index.html")

    @app.get("/{asset_path:path}")
    async def frontend_asset(asset_path: str) -> Any:
        if asset_path.startswith("api/") or asset_path.startswith("ws/"):
            raise HTTPException(status_code=404, detail="not found")
        frontend_dir = frontend_root.resolve()
        candidate = (frontend_dir / asset_path).resolve()
        if candidate.is_relative_to(frontend_dir) and candidate.is_file():
            return FileResponse(candidate)
        raise HTTPException(status_code=404, detail="not found")

    async def websocket_channel(websocket: WebSocket, channel: str) -> None:
        manager: WebSocketManager = app.state.services["websocket_manager"]
        if channel not in {"logs", "telemetry", "state", "ros_graph", "camera_metadata"}:
            await websocket.close(code=1008)
            return
        await manager.connect(channel, websocket)
        try:
            while True:
                message = await websocket.receive_text()
                if channel == "logs":
                    await manager.broadcast("logs", {"channel": channel, "message": message})
                else:
                    await manager.broadcast(channel, {"channel": channel, "message": message})
        except WebSocketDisconnect:
            await manager.disconnect(channel, websocket)

    @app.websocket("/ws/logs")
    async def websocket_logs(websocket: WebSocket) -> None:
        await websocket_channel(websocket, "logs")

    @app.websocket("/ws/telemetry")
    async def websocket_telemetry(websocket: WebSocket) -> None:
        await websocket_channel(websocket, "telemetry")

    @app.websocket("/ws/state")
    async def websocket_state(websocket: WebSocket) -> None:
        await websocket_channel(websocket, "state")

    @app.websocket("/ws/ros-graph")
    async def websocket_ros_graph(websocket: WebSocket) -> None:
        await websocket_channel(websocket, "ros_graph")

    @app.websocket("/ws/ros_graph")
    async def websocket_ros_graph_alias(websocket: WebSocket) -> None:
        await websocket_channel(websocket, "ros_graph")

    @app.websocket("/ws/cameras")
    async def websocket_camera_metadata(websocket: WebSocket) -> None:
        await websocket_channel(websocket, "camera_metadata")

    @app.websocket("/ws/camera_metadata")
    async def websocket_camera_metadata_alias(websocket: WebSocket) -> None:
        await websocket_channel(websocket, "camera_metadata")

    return app


app = create_app()
