from __future__ import annotations

from fastapi import APIRouter

from backend.api.cameras import router as cameras_router
from backend.api.config import router as config_router
from backend.api.diagnostics import router as diagnostics_router
from backend.api.labview import router as labview_router
from backend.api.mission import router as mission_router

router = APIRouter()
router.include_router(mission_router, prefix="/mission", tags=["mission"])
router.include_router(cameras_router, prefix="/cameras", tags=["cameras"])
router.include_router(config_router, prefix="/config", tags=["config"])
router.include_router(diagnostics_router, prefix="/diagnostics", tags=["diagnostics"])
router.include_router(labview_router, prefix="/labview", tags=["labview"])
