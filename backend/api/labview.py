from __future__ import annotations

from fastapi import APIRouter, Request

router = APIRouter()


def _svc(request: Request):
    return request.app.state.services["labview"]


@router.get("/hardware")
async def hardware_debug(request: Request):
    return await _svc(request).emergency_debug()


@router.get("/ros-graph")
async def ros_graph(request: Request):
    return await _svc(request).ros_graph()


@router.post("/topic-echo")
async def topic_echo(request: Request, topic: str):
    return await _svc(request).topic_echo(topic)


@router.post("/service-browser")
async def service_browser(request: Request):
    return await _svc(request).service_browser()


@router.post("/parameter-edit")
async def parameter_edit(request: Request, node: str, parameter: str, value):
    return await _svc(request).parameter_edit(node, parameter, value)


@router.post("/node/{action}")
async def node_action(action: str, request: Request, node: str, topic: str | None = None, remap_to: str | None = None):
    service = _svc(request)
    if action == "restart":
        return await service.node_restart(node)
    if action == "enable":
        return await service.node_enable(node)
    if action == "disable":
        return await service.node_disable(node)
    if action == "remap":
        return await service.remap_topic(node, topic or "", remap_to or "")
    return {"ok": False, "error": f"unsupported action {action}"}
