"""backend/api/server_context.py - Instance du RobotController et cycle de vie FastAPI."""
import asyncio
import json
import logging
import os
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Dict
from fastapi import FastAPI

from backend.navigation.robot_controller import RobotController, RobotMode
from backend.hardware.hardware_bridge import ExecutionMode, is_console_noise_line
from backend.api.ws_manager import ws_manager
from backend.api.network_utils import detect_local_wifi_ip, detect_all_local_ips, SERVER_PORT, EXPO_PORT
from backend.config.robot_config import get_deployment_target, is_robot_uc_mode
from backend.system.auto_deploy_manager import auto_deploy_manager

logger = logging.getLogger("ServerContext")
BASE_DIR: Path = Path(__file__).resolve().parent.parent.parent

robot: RobotController = RobotController(use_simulator=True)

def build_state_payload(msg_type: str = "state") -> Dict[str, Any]:
    """Construit le paquet d'état complet synchronisé (avec mission_snapshot & caméra) en réévaluant l'IP en direct."""
    live_ip = detect_local_wifi_ip()
    all_ips = detect_all_local_ips()
    cam_status: Dict[str, Any] = {}
    try:
        from backend.vision.camera_manager import camera_manager
        cam_status = camera_manager.get_status()
    except Exception:
        cam_status = {}
    return {
        "type": msg_type,
        "armed": robot.armed if robot else 0,
        "is_armed": robot.is_armed if robot else False,
        "estop": robot.estop_active if robot else True,
        "mission_running": robot.mission_running if robot else False,
        "mission_active": bool(robot and robot.mission_running),
        "mission_paused": bool(robot and robot.mission_paused),
        "state": robot.state if robot else "ESTOP",
        "mode": robot.mode.value if robot else "MANUAL",
        "mission": robot.current_mission_name if robot else None,
        "mission_state": robot.get_mission_state() if robot else {},
        "mission_snapshot": robot.get_mission_snapshot() if robot else {},
        "telemetry": robot.last_telemetry if robot else {},
        "agronomics": robot.get_agronomic_metrics() if robot else {},
        "robot_specs": robot.get_robot_specs() if robot else {},
        "hardware": robot.get_hardware_status() if robot else {},
        "camera": cam_status,
        "local_ip": live_ip,
        "all_ips": all_ips,
        "expo_url": f"exp://{live_ip}:{EXPO_PORT}",
        "mobile_url": f"http://{live_ip}:{SERVER_PORT}/mobile",
        "ws_url": f"ws://{live_ip}:{SERVER_PORT}/ws",
        "connected_clients": len(ws_manager.active_connections),
    }


@asynccontextmanager


async def lifespan(app: FastAPI):
    global robot
    loop = asyncio.get_running_loop()
    ws_manager.set_loop(loop)

    live_ip = detect_local_wifi_ip()
    banner = (
        "\n"
        "======================================================================\n"
        "  [ROBOT AGRICOLE ESP32-S3] SERVEUR TEMPS REEL DYNAMIQUE (AUTO-IP)\n"
        "======================================================================\n"
        f"  * IP Locale Active Detectee  : {live_ip} (Reevaluee en direct)\n"
        f"  * Jumeau Numerique 3D (PC)   : http://localhost:{SERVER_PORT}\n"
        f"  * Telecommande Web Mobile    : http://{live_ip}:{SERVER_PORT}/mobile\n"
        f"  * Serveur Expo Go (Mobile)   : exp://{live_ip}:{EXPO_PORT}\n"
        f"  * WebSocket Temps Reel (/ws) : ws://{live_ip}:{SERVER_PORT}/ws\n"
        "======================================================================"
    )
    print(banner, flush=True)
    logger.info("Adresse IP locale dynamique initiale : %s", live_ip)

    logger.info("Initialisation du contrôleur de robot...")
    if not robot.device or not getattr(robot.device, "is_open", False):
        robot = RobotController(use_simulator=True)

    # Liaison Safe-Reload Guard & Notifications WebSocket Auto-Deploy
    auto_deploy_manager.set_mission_running_guard(lambda: bool(robot and robot.mission_running))

    def _on_deploy_status(payload: Dict[str, Any]):
        ws_manager.broadcast_sync({"type": "system_update_status", **payload})

    auto_deploy_manager.on_status_change_fn = _on_deploy_status
    robot.on_mission_stopped_callbacks.append(auto_deploy_manager.check_and_flush_pending_update)

    if is_robot_uc_mode():
        logger.info("🤖 DEPLOYMENT_TARGET=ROBOT_UC : Mode Embarqué Zéro Simulation activé !")
        robot.set_execution_mode(ExecutionMode.REAL_ZERO_SIM.value)
        import threading
        threading.Thread(target=robot.auto_connect_uc_hardware, name="BootUCAutoHardware", daemon=True).start()
        auto_deploy_manager.start_watcher(interval_sec=10.0)


    # Connexion des callbacks du contrôleur au gestionnaire WebSocket
    def on_telemetry(data: Dict[str, Any]):
        ws_manager.broadcast_sync(
            {
                "type": "telemetry",
                "data": data,
                "armed": robot.armed,
                "mode": robot.mode.value,
                "mission": robot.current_mission_name,
                "mission_state": robot.get_mission_state(),
                "agronomics": robot.get_agronomic_metrics(),
                "hardware": robot.get_hardware_status(),
            }
        )

    def on_event(event_name: str, payload: Any):
        ws_manager.broadcast_sync(
            {
                "type": "event",
                "event": event_name,
                "payload": payload,
                "armed": robot.armed,
                "mode": robot.mode.value,
                "mission": robot.current_mission_name,
                "mission_running": robot.mission_running,
                "mission_active": robot.mission_running,
                "mission_paused": robot.mission_paused,
                "mission_state": robot.get_mission_state(),
                "mission_snapshot": robot.get_mission_snapshot(),
                "agronomics": robot.get_agronomic_metrics(),
                "hardware": robot.get_hardware_status(),
            }
        )

    def on_log(direction: str, line: str):
        # Interdiction absolue d'émettre PING, PONG ou #T dans la console
        if is_console_noise_line(line):
            return
        ws_manager.broadcast_sync(
            {
                "type": "serial_log",
                "direction": direction,
                "line": line,
                "ts": int(time.time() * 1000),
            }
        )

    robot.telemetry_callbacks.append(on_telemetry)
    robot.event_callbacks.append(on_event)
    robot.log_callbacks.append(on_log)

    # Activer la télémétrie par défaut à 100 ms
    robot.enable_telemetry(100)

    yield

    logger.info("Fermeture du contrôleur de robot...")
    auto_deploy_manager.stop_watcher()
    robot.close()
