"""backend/api/__init__.py"""

from backend.api.routes import router, websocket_endpoint
from backend.api.server_context import robot as controller
