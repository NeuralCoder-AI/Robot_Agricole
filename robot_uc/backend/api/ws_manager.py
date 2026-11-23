"""backend/api/ws_manager.py - Gestionnaire multi-clients WebSocket."""
import asyncio
import json
import logging
import time
from typing import Any, Dict, List, Optional, Set
from fastapi import WebSocket
from backend.hardware.hardware_bridge import is_console_noise_line

logger = logging.getLogger("WsManager")
class ConnectionManager:
    """Gestionnaire des connexions WebSocket multi-clients pour la diffusion temps réel."""

    def __init__(self):
        self.active_connections: Set[WebSocket] = set()
        self.loop: asyncio.AbstractEventLoop = None  # type: ignore

    def set_loop(self, loop: asyncio.AbstractEventLoop):
        self.loop = loop

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.add(websocket)
        logger.info("Nouveau client WebSocket connecté. Total clients actifs: %d", len(self.active_connections))

    def disconnect(self, websocket: WebSocket):
        self.active_connections.discard(websocket)
        logger.info("Client WebSocket déconnecté. Clients restants: %d", len(self.active_connections))

    def broadcast_sync(self, message: Dict[str, Any]):
        """Diffusé depuis les threads d'arrière-plan (reader, telemetry, missions, etc.)."""
        if not self.active_connections or not self.loop or self.loop.is_closed():
            return
        asyncio.run_coroutine_threadsafe(self.broadcast(message), self.loop)

    async def broadcast(self, message: Dict[str, Any], exclude: Optional[WebSocket] = None):
        dead_connections = set()
        msg_str = json.dumps(message)
        for connection in list(self.active_connections):
            if exclude is not None and connection is exclude:
                continue
            try:
                await connection.send_text(msg_str)
            except Exception:
                dead_connections.add(connection)
        for dead in dead_connections:
            self.active_connections.discard(dead)


ws_manager = ConnectionManager()


