"""main.py
Point d'entrée du serveur FastAPI du Robot Agricole ESP32-S3.
"""

from pathlib import Path
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

try:
    from dotenv import load_dotenv
    _BASE = Path(__file__).resolve().parent
    load_dotenv(_BASE.parent / ".env", override=False)
    load_dotenv(_BASE / ".env", override=False)
except Exception:
    pass

from backend.api.server_context import lifespan
from backend.api.routes import router
from backend.api.network_utils import detect_local_wifi_ip  # noqa: F401
from backend.config.robot_config import get_deployment_target, is_robot_uc_mode  # noqa: F401

BASE_DIR: Path = Path(__file__).resolve().parent
STATIC_DIR: Path = BASE_DIR / "static"

app = FastAPI(
    title="Robot Agricole ESP32-S3 Firmware v2 API",
    description="API temps réel & WebSockets multi-clients pour Cockpit 3D & Mobile",
    version="2.2.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.middleware("http")
async def add_no_cache_headers(request: Request, call_next):
    response = await call_next(request)
    response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    response.headers["Pragma"] = "no-cache"
    response.headers["Expires"] = "0"
    return response

app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")
if (STATIC_DIR / "js").exists():
    app.mount("/js", StaticFiles(directory=str(STATIC_DIR / "js")), name="js")
app.include_router(router)


@app.get("/")
async def get_dashboard():
    return FileResponse(
        str(STATIC_DIR / "index.html"),
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
            "Pragma": "no-cache",
            "Expires": "0",
        },
    )


@app.get("/mobile")
async def get_mobile_app():
    return FileResponse(
        str(STATIC_DIR / "mobile.html"),
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
            "Pragma": "no-cache",
            "Expires": "0",
        },
    )


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
