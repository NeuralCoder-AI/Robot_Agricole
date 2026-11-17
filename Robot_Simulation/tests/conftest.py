"""tests/conftest.py
Fixtures pytest partagées pour le simulateur ESP32-S3 et le client FastAPI.
"""

import os
import sys
from pathlib import Path
import tempfile
import pytest

APP_ROOT = Path(__file__).resolve().parent.parent
if str(APP_ROOT) not in sys.path:
    sys.path.insert(0, str(APP_ROOT))

from fastapi.testclient import TestClient

from backend.hardware.esp32_simulator import VirtualESP32FirmwareV2
from backend.navigation.robot_controller import RobotController
from main import app


@pytest.fixture
def sim():
    """Fixture créant une instance isolée du simulateur avec un fichier NVS temporaire."""
    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
        temp_nvs = tf.name

    simulator = VirtualESP32FirmwareV2(nvs_path=temp_nvs)
    yield simulator
    simulator.close()
    if os.path.exists(temp_nvs):
        try:
            os.remove(temp_nvs)
        except OSError:
            pass


@pytest.fixture
def client():
    """Fixture fournissant un client de test FastAPI."""
    with TestClient(app) as test_client:
        yield test_client


def ws_receive_type(ws, expected_type: str, max_reads: int = 25):
    """Lit les messages WebSocket en ignorant les trames intermédiaires #T ou serial_log."""
    for _ in range(max_reads):
        msg = ws.receive_json()
        if msg.get("type") == expected_type:
            return msg
    raise AssertionError(f"Message WebSocket de type '{expected_type}' non reçu après {max_reads} lectures")
