"""backend/api/routes.py - Routes REST et WebSocket /ws."""
import asyncio
import json
import logging
import time
from typing import Any, Dict, List, Optional
import anyio
from fastapi import APIRouter, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from pydantic import BaseModel

logger = logging.getLogger("routes")

from backend.api import server_context
from backend.api.server_context import build_state_payload
from backend.api.ws_manager import ws_manager
from backend.api.network_utils import detect_local_wifi_ip, detect_all_local_ips, SERVER_PORT, EXPO_PORT
from backend.hardware.hardware_bridge import ESP32HardwareBridge, ExecutionMode
from backend.navigation.robot_controller import RobotMode
from backend.vision.camera_manager import camera_manager
from backend.config.robot_config import robot_config, get_deployment_target, is_robot_uc_mode
from backend.system.auto_deploy_manager import auto_deploy_manager

router = APIRouter()

class CommandRequest(BaseModel):
    command: str
    with_checksum: bool = False


class DriveRequest(BaseModel):
    a: int
    d: int
    ms: int | None = None


class PulseRequest(BaseModel):
    motor: int
    speed: int
    ms: int


class MissionRequest(BaseModel):
    name: str
    steps: List[List[int]] | None = None
    field_length_m: float | None = None
    field_width_m: float | None = None
    spacing_cm: int | None = None
    seed_with_tines_down: bool | None = None
    progression_direction: int | None = 1
    start_x: float | None = None
    start_z: float | None = None
    boundary_mode: str | None = None


# ------------------------------------------------------------------------------
# Endpoints REST (diffusent également l'état à tous les clients WebSocket)
# ------------------------------------------------------------------------------
@router.get("/api/status")
async def get_status(request: Request):
    """Retourne l'état actuel complet du robot avec IP réévaluée en temps réel."""
    live_ip = detect_local_wifi_ip(request_host=request.headers.get("host"))
    return {
        "armed": server_context.robot.armed,
        "mode": server_context.robot.mode.value,
        "mission": server_context.robot.current_mission_name,
        "mission_state": server_context.robot.get_mission_state(),
        "use_simulator": server_context.robot.use_simulator,
        "last_telemetry": server_context.robot.last_telemetry,
        "agronomics": server_context.robot.get_agronomic_metrics(),
        "robot_specs": server_context.robot.get_robot_specs(),
        "hardware": server_context.robot.get_hardware_status(),
        "solar_bms": server_context.robot.solar_bms.get_status(),
        "ekf_localization": server_context.robot.ekf_localizer.get_state(),
        "seed_map": server_context.robot.seed_map.summary(),
        "local_ip": live_ip,
        "all_ips": detect_all_local_ips(),
        "expo_url": f"exp://{live_ip}:{EXPO_PORT}",
        "mobile_url": f"http://{live_ip}:{SERVER_PORT}/mobile",
        "ws_url": f"ws://{live_ip}:{SERVER_PORT}/ws",
        "connected_clients": len(ws_manager.active_connections),
    }


@router.get("/api/hardware/status")
async def get_hardware_status():
    """Retourne l'état du mode (SIMULATION_ONLY / COUPLED) et de la liaison ESP32-S3."""
    return server_context.robot.get_hardware_status()


@router.get("/api/hardware/ports")
async def get_hardware_ports():
    """Scanne et retourne les ports série USB/UART disponibles (/dev/ttyUSB*, /dev/ttyACM*)."""
    return {"ports": ESP32HardwareBridge.scan_serial_ports()}


@router.get("/api/hardware/serial-ports")
async def get_hardware_serial_ports():
    """Alias pour /api/hardware/ports demandée par le cockpit matériel."""
    return {"ports": ESP32HardwareBridge.scan_serial_ports()}


@router.get("/api/camera/devices")
async def get_camera_devices():
    """Scanne et liste les périphériques vidéo USB /dev/video*."""
    return {"devices": camera_manager.scan_camera_devices()}


@router.get("/api/camera/status")
async def get_camera_status():
    """Retourne l'état courant de la caméra, des modèles Dual-ONNX (YOLOv8n + MobileNetV3) et du pipeline vision."""
    status = camera_manager.get_status()
    status.setdefault("yolo_onnx_loaded", bool(getattr(camera_manager.yolo_pipeline, "yolo_onnx_loaded", False)))
    status.setdefault("yolo_model_name", str(getattr(camera_manager.yolo_pipeline, "yolo_model_name", "yolov8n.onnx")))
    return status


@router.post("/api/camera/connect")
async def post_camera_connect(payload: Dict[str, Any]):
    """Connecte la Caméra USB (78°) et démarre le flux."""
    dev = payload.get("device_index", payload.get("index", 0))
    w = payload.get("width", 640)
    h = payload.get("height", 480)
    fps = payload.get("fps", 30)
    resolution = payload.get("resolution")
    k1 = payload.get("k1")
    tilt = payload.get("camera_tilt_deg", payload.get("tilt_deg", payload.get("tilt")))
    mission = payload.get("current_mission", payload.get("mission"))
    status = await asyncio.to_thread(
        camera_manager.connect,
        device_index=dev,
        width=w,
        height=h,
        fps=fps,
        resolution=resolution,
        k1=k1,
        tilt_deg=tilt,
        mission=mission,
    )
    await ws_manager.broadcast({"type": "camera_status", "camera": status})
    return status


@router.post("/api/camera/disconnect")
async def post_camera_disconnect():
    """Déconnecte la Caméra USB (78°)."""
    status = await asyncio.to_thread(camera_manager.disconnect)
    await ws_manager.broadcast({"type": "camera_status", "camera": status})
    return status


@router.post("/api/camera/config")
async def post_camera_config(payload: Dict[str, Any]):
    """Configure la distorsion k1, l'inclinaison de caméra ou la mission."""
    k1 = payload.get("k1")
    tilt = payload.get("camera_tilt_deg", payload.get("tilt"))
    mission = payload.get("current_mission", payload.get("mission"))
    status = camera_manager.set_config(k1=k1, camera_tilt_deg=tilt, current_mission=mission)
    await ws_manager.broadcast({"type": "camera_status", "camera": status})
    return status


@router.get("/api/camera/stream")
def get_camera_stream():
    """Stream MJPEG temps réel de la Caméra USB (78°) avec HUD vision."""
    return StreamingResponse(
        camera_manager.generate_mjpeg_stream(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )



@router.post("/api/hardware/mode")
async def post_hardware_mode(payload: Dict[str, Any]):
    """Change le mode d'exécution (SIMULATION_ONLY vs COUPLED) et diffuse l'état."""
    mode = str(payload.get("execution_mode") or payload.get("mode") or "SIMULATION_ONLY")
    hw = server_context.robot.set_execution_mode(mode)
    await ws_manager.broadcast({"type": "hardware_status", "hardware": hw})
    await ws_manager.broadcast(build_state_payload("state"))
    return hw


@router.post("/api/hardware/connect")
async def post_hardware_connect(payload: Dict[str, Any]):
    """Configure et teste la connexion au vrai robot ESP32-S3 (PING -> PONG + VER + STATUS)."""
    logger.info("Connexion matériel demandée: transport=%s host=%s port=%s serial=%s", payload.get("transport"), payload.get("host"), payload.get("port"), payload.get("serial_port"))
    hw = await anyio.to_thread.run_sync(server_context.robot.configure_and_connect_hardware, payload)
    logger.info("Résultat connexion: connected=%s last_error=%s latency=%s", hw.get("connected"), hw.get("last_error"), hw.get("latency_ms"))
    await ws_manager.broadcast({"type": "hardware_status", "hardware": hw})
    await ws_manager.broadcast(build_state_payload("state"))
    return hw


@router.post("/api/hardware/disconnect")
async def post_hardware_disconnect():
    """Ferme la connexion au robot physique ESP32-S3."""
    hw = await anyio.to_thread.run_sync(server_context.robot.disconnect_hardware)
    await ws_manager.broadcast({"type": "hardware_status", "hardware": hw})
    await ws_manager.broadcast(build_state_payload("state"))
    return hw


@router.post("/api/hardware/ping")
async def post_hardware_ping(payload: Optional[Dict[str, Any]] = None):
    """Teste directement PING -> PONG sur le pont matériel ESP32-S3."""
    res = await anyio.to_thread.run_sync(server_context.robot.ping_hardware, payload or {})
    hw = server_context.robot.get_hardware_status()
    await ws_manager.broadcast({"type": "hardware_status", "hardware": hw})
    return res


@router.post("/api/hardware/read-config")
async def post_hardware_read_config():
    """Lit VER, STATUS, GCFG, PERF et CFG:1..8 sur l'ESP32-S3."""
    res = await anyio.to_thread.run_sync(server_context.robot.read_all_hardware_configs)
    await ws_manager.broadcast({"type": "hardware_config_dump", "data": res, "hardware": res["hardware"]})
    await ws_manager.broadcast(build_state_payload("state"))
    return res


@router.post("/api/hardware/calibrate")
async def post_hardware_calibrate(payload: Dict[str, Any]):
    """Applique les réglages NVS d'un moteur (INV, MIN, MAX, RAMP, TMAX)."""
    res = server_context.robot.calibrate_motor_nvs(
        motor_id=int(payload.get("motor", 1)),
        inv=payload.get("inv"),
        min_pwm=payload.get("min_pwm"),
        max_pwm=payload.get("max_pwm"),
        ramp=payload.get("ramp"),
        tmax=payload.get("tmax"),
    )
    await ws_manager.broadcast({"type": "hardware_status", "hardware": res["hardware"]})
    await ws_manager.broadcast(build_state_payload("state"))
    return res


@router.get("/api/config/specs")
async def get_config_specs():
    """Retourne les dimensions dynamiques persistantes du robot (mètres, angles caméra, délais)."""
    specs = robot_config.get_specs()
    return {**specs, "specs": specs}


@router.put("/api/config/specs")
@router.post("/api/config/specs")
async def put_config_specs(payload: Dict[str, Any]):
    """Met à jour et persiste les dimensions dynamiques du robot dans robot_specs.sqlite / robot_specs.json."""
    updated = robot_config.update_specs(payload)
    if server_context.robot:
        server_context.robot.sync_dynamic_config(updated)
    await ws_manager.broadcast(
        {
            "type": "config_specs_updated",
            "specs": updated,
            "robot_specs": server_context.robot.get_robot_specs() if server_context.robot else updated,
        }
    )
    await ws_manager.broadcast(build_state_payload("state"))
    return {"status": "updated", **updated, "specs": updated}


@router.post("/api/config/specs/reset")
async def reset_config_specs():
    """Réinitialise les dimensions dynamiques aux valeurs par défaut."""
    reset_val = robot_config.reset_specs()
    if server_context.robot:
        server_context.robot.sync_dynamic_config(reset_val)
    await ws_manager.broadcast(
        {
            "type": "config_specs_updated",
            "specs": reset_val,
        }
    )
    await ws_manager.broadcast(build_state_payload("state"))
    return {"status": "reset", **reset_val, "specs": reset_val}


@router.get("/api/robot-specs")
async def get_robot_specs():
    """Retourne les dimensions mécaniques actuelles du robot (en cm)."""
    return server_context.robot.get_robot_specs()


@router.post("/api/robot-specs")
async def post_robot_specs(specs: Dict[str, Any]):
    """Met à jour les dimensions du robot à la volée, les sauvegarde dans robot_specs.json et diffuse specs_updated."""
    updated = server_context.robot.update_robot_specs(specs)
    await ws_manager.broadcast(
        {
            "type": "specs_updated",
            "robot_specs": updated,
            "agronomics": server_context.robot.get_agronomic_metrics(),
        }
    )
    await ws_manager.broadcast(build_state_payload("state"))
    return {"status": "updated", "robot_specs": updated}


@router.post("/api/robot-specs/reset")
async def post_robot_specs_reset():
    """Réinitialise toutes les dimensions aux vraies mesures du relevé terrain (DEFAULT_ROBOT_SPECS)."""
    reset_specs = server_context.robot.reset_robot_specs()
    await ws_manager.broadcast(
        {
            "type": "specs_updated",
            "robot_specs": reset_specs,
            "agronomics": server_context.robot.get_agronomic_metrics(),
        }
    )
    await ws_manager.broadcast(build_state_payload("state"))
    return {"status": "reset", "robot_specs": reset_specs}


@router.get("/api/vision")
async def get_vision_analysis():
    """Retourne la dernière analyse temps réel du Cerveau de Vision IA OpenCV."""
    if not server_context.robot or not hasattr(server_context.robot, "vision_brain"):
        return {"status": "unavailable"}
    return server_context.robot.vision_brain.last_analysis


@router.post("/api/arm")
async def post_arm():
    res = server_context.robot.arm()
    await ws_manager.broadcast(build_state_payload("state"))
    return {"status": res, "armed": server_context.robot.armed}


@router.post("/api/estop")
async def post_estop():
    res = server_context.robot.estop()
    await ws_manager.broadcast(build_state_payload("state"))
    return {"status": res, "armed": server_context.robot.armed, "is_armed": server_context.robot.is_armed, "estop": server_context.robot.estop_active}


@router.post("/api/stop")
async def post_stop():
    res = server_context.robot.stop()
    await ws_manager.broadcast(build_state_payload("state"))
    return {"status": res}


@router.post("/api/command")
async def post_command(req: CommandRequest):
    if req.command.strip().upper().split("*")[0].strip() == "ESTOP":
        res = server_context.robot.estop()
    elif req.command.strip().upper().split("*")[0].strip() == "ARM":
        res = server_context.robot.arm()
    else:
        res = server_context.robot.send_command(req.command, with_checksum=req.with_checksum)
    await ws_manager.broadcast(build_state_payload("state"))
    return {"command": req.command, "response": res}


@router.post("/api/drive")
async def post_drive(req: DriveRequest):
    res = server_context.robot.drive(req.a, req.d, ms=req.ms)
    await ws_manager.broadcast(build_state_payload("state"))
    return {"status": res}


@router.post("/api/mission")
async def post_mission(req: MissionRequest):
    steps = [tuple(s) for s in (req.steps or [])]
    extra: Dict[str, Any] = {"steps": steps}
    if req.field_length_m is not None:
        extra["field_length_m"] = req.field_length_m
    if req.field_width_m is not None:
        extra["field_width_m"] = req.field_width_m
    if req.spacing_cm is not None:
        extra["spacing_cm"] = req.spacing_cm
    if req.seed_with_tines_down is not None:
        extra["seed_with_tines_down"] = req.seed_with_tines_down
    if req.progression_direction is not None:
        extra["progression_direction"] = req.progression_direction
    if req.start_x is not None:
        extra["start_x"] = req.start_x
    if req.start_z is not None:
        extra["start_z"] = req.start_z
    if req.boundary_mode is not None:
        extra["boundary_mode"] = req.boundary_mode
    ok = server_context.robot.start_mission_async(req.name, **extra)
    if not ok:
        raise HTTPException(status_code=400, detail="Impossible de lancer la mission (occupé ou inconnue)")
    await ws_manager.broadcast(
        {
            "type": "mission_status",
            "mission": req.name,
            "started": ok,
            "mode": server_context.robot.mode.value,
            "mission_state": server_context.robot.get_mission_state(),
        }
    )
    return {"status": "started", "mission": req.name, "mission_state": server_context.robot.get_mission_state()}


@router.post("/api/pause")
async def post_pause():
    ok = server_context.robot.pause_mission()
    await ws_manager.broadcast(build_state_payload("state"))
    return {"status": "paused" if ok else "not_paused", "paused": server_context.robot.mission_paused}


@router.post("/api/resume")
async def post_resume():
    ok = server_context.robot.resume_mission()
    await ws_manager.broadcast(build_state_payload("state"))
    return {"status": "resumed" if ok else "not_resumed", "paused": server_context.robot.mission_paused}


@router.post("/api/abort")
async def post_abort():
    server_context.robot.abort_mission()
    server_context.robot.stop()
    await ws_manager.broadcast(
        {"type": "mission_aborted", "mode": server_context.robot.mode.value, "mission_state": server_context.robot.get_mission_state()}
    )
    return {"status": "aborted"}


# ------------------------------------------------------------------------------
# Endpoint WebSocket bidirectionnel multi-clients (Mobile + Dashboard 3D PC)
# ------------------------------------------------------------------------------
@router.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await ws_manager.connect(websocket)

    # Envoi de l'état initial au client qui vient de se connecter
    await websocket.send_text(json.dumps(build_state_payload("state")))
    # Notification aux autres clients du nombre de connexions actives
    await ws_manager.broadcast(
        {"type": "clients_update", "connected_clients": len(ws_manager.active_connections)},
        exclude=websocket,
    )

    try:
        while True:
            data_text = await websocket.receive_text()
            try:
                msg = json.loads(data_text)
            except json.JSONDecodeError:
                # Si c'est du texte brut, on le traite comme une commande série directe
                msg = {"type": "cmd", "command": data_text}

            msg_type = msg.get("type", "")

            if msg_type == "ping":
                # Mesure de latence PING/PONG + maintien du watchdog UART + renvoi de l'IP live
                if server_context.robot.simulator:
                    server_context.robot.simulator.last_cmd_time = time.perf_counter()
                live_ip = detect_local_wifi_ip()
                await websocket.send_text(
                    json.dumps(
                        {
                            "type": "pong",
                            "ts": msg.get("ts"),
                            "server_ts": int(time.time() * 1000),
                            "armed": server_context.robot.armed,
                            "mode": server_context.robot.mode.value,
                            "local_ip": live_ip,
                            "ws_url": f"ws://{live_ip}:{SERVER_PORT}/ws",
                        }
                    )
                )

            elif msg_type == "cmd":
                cmd = msg.get("command", "")
                with_crc = bool(msg.get("with_checksum", False))
                res = server_context.robot.send_command(cmd, with_checksum=with_crc)
                await websocket.send_text(json.dumps({"type": "cmd_result", "command": cmd, "response": res}))
                await ws_manager.broadcast(build_state_payload("state"), exclude=websocket)

            elif msg_type == "arm":
                res = server_context.robot.arm()
                await websocket.send_text(json.dumps({"type": "cmd_result", "command": "ARM", "response": res}))
                await ws_manager.broadcast(build_state_payload("state"))

            elif msg_type == "estop":
                res = server_context.robot.estop()
                await websocket.send_text(json.dumps({"type": "cmd_result", "command": "ESTOP", "response": res}))
                await ws_manager.broadcast(build_state_payload("state"))

            elif msg_type == "stop":
                motor_idx = msg.get("motor")
                if motor_idx is not None:
                    res = server_context.robot.stop_motor(int(motor_idx))
                    cmd_str = f"STOP:{int(motor_idx)}"
                else:
                    res = server_context.robot.stop()
                    cmd_str = "STOP"
                await websocket.send_text(json.dumps({"type": "cmd_result", "command": cmd_str, "response": res}))
                await ws_manager.broadcast(build_state_payload("state"), exclude=websocket)

            elif msg_type == "drive":
                a = int(msg.get("a", 0))
                d = int(msg.get("d", 0))
                ms = msg.get("ms")
                res = server_context.robot.drive(a, d, ms=int(ms) if ms is not None else None)
                cmd_str = f"DRIVE:{a},{d}:{int(ms)}" if ms is not None else f"DRIVE:{a},{d}"
                await websocket.send_text(json.dumps({"type": "cmd_result", "command": cmd_str, "response": res}))
                await ws_manager.broadcast(build_state_payload("state"), exclude=websocket)

            elif msg_type == "tank":
                l = int(msg.get("l", 0))
                r = int(msg.get("r", 0))
                ms = msg.get("ms")
                res = server_context.robot.tank(l, r, ms=int(ms) if ms is not None else None)
                cmd_str = f"TANK:{l},{r}:{int(ms)}" if ms is not None else f"TANK:{l},{r}"
                await websocket.send_text(json.dumps({"type": "cmd_result", "command": cmd_str, "response": res}))
                await ws_manager.broadcast(build_state_payload("state"), exclude=websocket)

            elif msg_type == "motor":
                n = int(msg.get("motor", 1))
                v = int(msg.get("speed", 0))
                ms = msg.get("ms")
                res = server_context.robot.motor(n, v, ms=int(ms) if ms is not None else None)
                cmd_str = f"M{n}:{v}:{int(ms)}" if ms is not None else f"M{n}:{v}"
                await websocket.send_text(json.dumps({"type": "cmd_result", "command": cmd_str, "response": res}))
                await ws_manager.broadcast(build_state_payload("state"), exclude=websocket)

            elif msg_type == "pulse":
                n = int(msg.get("motor", 1))
                v = int(msg.get("speed", 0))
                ms = int(msg.get("ms", 1000))
                res = server_context.robot.pulse(n, v, ms)
                await websocket.send_text(
                    json.dumps({"type": "cmd_result", "command": f"PULSE:{n}:{v}:{ms}", "response": res})
                )
                await ws_manager.broadcast(build_state_payload("state"), exclude=websocket)

            elif msg_type == "mode":
                new_mode = msg.get("mode", "MANUAL")
                server_context.robot.set_mode(RobotMode(new_mode))
                await ws_manager.broadcast(
                    {
                        "type": "mode_changed",
                        "mode": server_context.robot.mode.value,
                        "armed": server_context.robot.armed,
                        "mission": server_context.robot.current_mission_name,
                        "mission_state": server_context.robot.get_mission_state(),
                    }
                )

            elif msg_type in ("mission", "mission_start"):
                mission_name = msg.get("name") or msg.get("mission") or ""
                raw_steps = msg.get("steps")
                steps = [tuple(s) for s in raw_steps] if raw_steps else []
                mission_kwargs: Dict[str, Any] = {"steps": steps}
                if msg.get("field_length_m") is not None:
                    mission_kwargs["field_length_m"] = float(msg["field_length_m"])
                if msg.get("field_width_m") is not None:
                    mission_kwargs["field_width_m"] = float(msg["field_width_m"])
                if msg.get("spacing_cm") is not None:
                    mission_kwargs["spacing_cm"] = int(msg["spacing_cm"])
                if msg.get("seed_with_tines_down") is not None:
                    mission_kwargs["seed_with_tines_down"] = bool(msg["seed_with_tines_down"])
                elif msg.get("use_tines") is not None:
                    mission_kwargs["seed_with_tines_down"] = bool(msg["use_tines"])
                if msg.get("progression_direction") is not None:
                    mission_kwargs["progression_direction"] = int(msg["progression_direction"])
                if msg.get("start_x") is not None:
                    mission_kwargs["start_x"] = float(msg["start_x"])
                if msg.get("start_z") is not None:
                    mission_kwargs["start_z"] = float(msg["start_z"])
                if msg.get("boundary_mode") is not None:
                    mission_kwargs["boundary_mode"] = str(msg["boundary_mode"])
                ok = server_context.robot.start_mission_async(mission_name, **mission_kwargs)
                await ws_manager.broadcast(
                    {
                        "type": "mission_status",
                        "mission": mission_name,
                        "started": ok,
                        "mode": server_context.robot.mode.value,
                        "mission_state": server_context.robot.get_mission_state(),
                    }
                )

            elif msg_type == "pause_mission":
                ok = server_context.robot.pause_mission()
                await ws_manager.broadcast(
                    {
                        "type": "mission_paused",
                        "paused": server_context.robot.mission_paused,
                        "ok": ok,
                        "mission_state": server_context.robot.get_mission_state(),
                    }
                )

            elif msg_type == "resume_mission":
                ok = server_context.robot.resume_mission()
                await ws_manager.broadcast(
                    {
                        "type": "mission_resumed",
                        "paused": server_context.robot.mission_paused,
                        "ok": ok,
                        "mission_state": server_context.robot.get_mission_state(),
                    }
                )

            elif msg_type == "abort_mission":
                server_context.robot.abort_mission()
                server_context.robot.stop()
                await ws_manager.broadcast(
                    {
                        "type": "mission_aborted",
                        "mode": server_context.robot.mode.value,
                        "mission_state": server_context.robot.get_mission_state(),
                    }
                )

            elif msg_type == "seed_config":
                spacing_cm = msg.get("spacing_cm")
                auto_seed = msg.get("auto_seed")
                use_tines = msg.get("use_tines")
                if use_tines is None and msg.get("seed_with_tines_down") is not None:
                    use_tines = msg.get("seed_with_tines_down")
                field_length_m = msg.get("field_length_m")
                field_width_m = msg.get("field_width_m")
                metrics = server_context.robot.set_seeding_config(
                    spacing_cm=int(spacing_cm) if spacing_cm is not None else None,
                    auto_seed=bool(auto_seed) if auto_seed is not None else None,
                    use_tines=bool(use_tines) if use_tines is not None else None,
                    field_length_m=float(field_length_m) if field_length_m is not None else None,
                    field_width_m=float(field_width_m) if field_width_m is not None else None,
                )
                await ws_manager.broadcast(
                    {
                        "type": "agronomics",
                        "agronomics": metrics,
                    }
                )
                await ws_manager.broadcast(
                    {
                        "type": "specs_updated",
                        "robot_specs": server_context.robot.get_robot_specs(),
                        "agronomics": server_context.robot.get_agronomic_metrics(),
                    }
                )
                await ws_manager.broadcast(build_state_payload("state"))

            elif msg_type == "update_specs":
                raw_specs = msg.get("specs") or msg.get("robot_specs") or {}
                if isinstance(raw_specs, dict):
                    updated = server_context.robot.update_robot_specs(raw_specs)
                    await ws_manager.broadcast(
                        {
                            "type": "specs_updated",
                            "robot_specs": updated,
                            "agronomics": server_context.robot.get_agronomic_metrics(),
                        }
                    )
                    await ws_manager.broadcast(build_state_payload("state"))

            elif msg_type == "reset_specs":
                reset_specs = server_context.robot.reset_robot_specs()
                await ws_manager.broadcast(
                    {
                        "type": "specs_updated",
                        "robot_specs": reset_specs,
                        "agronomics": server_context.robot.get_agronomic_metrics(),
                    }
                )
                await ws_manager.broadcast(build_state_payload("state"))

            elif msg_type == "set_execution_mode":
                exec_mode = str(msg.get("execution_mode") or msg.get("mode") or "SIMULATION_ONLY")
                hw = server_context.robot.set_execution_mode(exec_mode)
                await ws_manager.broadcast({"type": "hardware_status", "hardware": hw})
                await ws_manager.broadcast(build_state_payload("state"))

            elif msg_type == "hardware_connect":
                hw = await asyncio.to_thread(server_context.robot.configure_and_connect_hardware, msg)
                await ws_manager.broadcast({"type": "hardware_status", "hardware": hw})
                await ws_manager.broadcast(build_state_payload("state"))

            elif msg_type == "hardware_disconnect":
                hw = server_context.robot.disconnect_hardware()
                await ws_manager.broadcast({"type": "hardware_status", "hardware": hw})
                await ws_manager.broadcast(build_state_payload("state"))

            elif msg_type == "hardware_read_config":
                dump = await asyncio.to_thread(server_context.robot.read_all_hardware_configs)
                await ws_manager.broadcast(
                    {
                        "type": "hardware_config_dump",
                        "data": dump,
                        "hardware": dump["hardware"],
                    }
                )
                await ws_manager.broadcast(build_state_payload("state"))

            elif msg_type == "nvs_calibrate":
                cal_res = await asyncio.to_thread(
                    server_context.robot.calibrate_motor_nvs,
                    motor_id=int(msg.get("motor", 1)),
                    inv=msg.get("inv"),
                    min_pwm=msg.get("min_pwm"),
                    max_pwm=msg.get("max_pwm"),
                    ramp=msg.get("ramp"),
                    tmax=msg.get("tmax"),
                )
                await ws_manager.broadcast({"type": "hardware_status", "hardware": cal_res["hardware"]})
                await ws_manager.broadcast(build_state_payload("state"))

            elif msg_type == "camera_connect":
                dev = msg.get("device_index", msg.get("index", 0))
                w = msg.get("width", 640)
                h = msg.get("height", 480)
                fps = msg.get("fps", 30)
                cam_status = await asyncio.to_thread(
                    camera_manager.connect,
                    device_index=dev,
                    width=w,
                    height=h,
                    fps=fps,
                    resolution=msg.get("resolution"),
                    k1=msg.get("k1"),
                    tilt_deg=msg.get("camera_tilt_deg", msg.get("tilt_deg", msg.get("tilt"))),
                    mission=msg.get("current_mission", msg.get("mission")),
                )
                await ws_manager.broadcast({"type": "camera_status", "camera": cam_status})

            elif msg_type == "camera_disconnect":
                cam_status = await asyncio.to_thread(camera_manager.disconnect)
                await ws_manager.broadcast({"type": "camera_status", "camera": cam_status})

            elif msg_type == "camera_config":
                cam_status = camera_manager.set_config(
                    k1=msg.get("k1"),
                    camera_tilt_deg=msg.get("camera_tilt_deg", msg.get("tilt")),
                    current_mission=msg.get("current_mission", msg.get("mission")),
                )
                await ws_manager.broadcast({"type": "camera_status", "camera": cam_status})


            elif msg_type == "terrain_feedback":
                p = float(msg.get("pitch", 0.0))
                r = float(msg.get("roll", 0.0))
                obs = bool(msg.get("obstacle", False))
                dist = float(msg.get("distance", 99.0))
                dense = bool(msg.get("dense_weed", False))
                speed_kmh = msg.get("speed_kmh")
                weeds_destroyed = msg.get("weeds_destroyed")
                weeds_total = msg.get("weeds_total")
                seeds_sown = msg.get("seeds_sown")
                tines_depth = msg.get("tines_depth")
                solar_angle = msg.get("solar_angle")
                distance_traveled_m = msg.get("distance_traveled_m")
                robot_x = msg.get("robot_x")
                robot_z = msg.get("robot_z")
                robot_yaw = msg.get("robot_yaw")
                weeds_in_fov = msg.get("weeds_in_fov")
                obstacles_in_fov = msg.get("obstacles_in_fov")
                crop_rows_x = msg.get("crop_rows_x")
                server_context.robot.update_terrain_sensor_data(
                    pitch=p,
                    roll=r,
                    obstacle=obs,
                    distance=dist,
                    dense_weed=dense,
                    speed_kmh=float(speed_kmh) if speed_kmh is not None else None,
                    weeds_destroyed=int(weeds_destroyed) if weeds_destroyed is not None else None,
                    weeds_total=int(weeds_total) if weeds_total is not None else None,
                    seeds_sown=int(seeds_sown) if seeds_sown is not None else None,
                    tines_depth=float(tines_depth) if tines_depth is not None else None,
                    solar_angle=float(solar_angle) if solar_angle is not None else None,
                    distance_traveled_m=float(distance_traveled_m) if distance_traveled_m is not None else None,
                    robot_x=float(robot_x) if robot_x is not None else None,
                    robot_z=float(robot_z) if robot_z is not None else None,
                    robot_yaw=float(robot_yaw) if robot_yaw is not None else None,
                    weeds_in_fov=weeds_in_fov if isinstance(weeds_in_fov, list) else None,
                    obstacles_in_fov=obstacles_in_fov if isinstance(obstacles_in_fov, list) else None,
                    crop_rows_x=crop_rows_x if isinstance(crop_rows_x, list) else None,
                )
                # Diffuser les métriques agronomiques 3D + HUD Vision IA vers tous les clients (Mobile & PC)
                await ws_manager.broadcast(
                    {
                        "type": "agronomics",
                        "agronomics": server_context.robot.get_agronomic_metrics(),
                    }
                )

    except WebSocketDisconnect:
        ws_manager.disconnect(websocket)
        await ws_manager.broadcast({"type": "clients_update", "connected_clients": len(ws_manager.active_connections)})
    except Exception as e:
        logger.error("Erreur WebSocket : %s", e)
        ws_manager.disconnect(websocket)

@router.get("/api/network")
async def get_network_info(request: Request):
    """Réévalue en temps réel l'adresse IP locale active (Wi-Fi ou Hotspot) à chaque appel."""
    host_header = request.headers.get("host")
    ip = detect_local_wifi_ip(request_host=host_header, deep_scan=True)
    all_ips = detect_all_local_ips(deep_scan=True)
    return {
        "local_ip": ip,
        "all_ips": all_ips,
        "port": SERVER_PORT,
        "bridge_port": 8082,
        "expo_port": EXPO_PORT,
        "expo_url": f"exp://{ip}:{EXPO_PORT}",
        "dashboard_url": f"http://{ip}:{SERVER_PORT}/",
        "mobile_url": f"http://{ip}:{SERVER_PORT}/mobile",
        "ws_url": f"ws://{ip}:{SERVER_PORT}/ws",
        "bridge_ws_url": f"ws://{ip}:8082/ws",
        "connected_clients": len(ws_manager.active_connections),
        "timestamp": int(time.time() * 1000),
    }


@router.get("/api/system/info")
async def get_system_info():
    """Retourne le profil de déploiement (ROBOT_UC vs DEV_FULL), l'état simulation et le mode forcé."""
    target = get_deployment_target()
    robot_uc = target == "ROBOT_UC"
    if robot_uc and server_context.robot and server_context.robot.execution_mode != ExecutionMode.REAL_ZERO_SIM.value:
        server_context.robot.execution_mode = ExecutionMode.REAL_ZERO_SIM.value
    cam_pc = getattr(camera_manager.pipeline, "peanut_classifier", None)
    return {
        "deployment_target": target,
        "is_robot_uc": robot_uc,
        "simulation_enabled": not robot_uc,
        "forced_execution_mode": ExecutionMode.REAL_ZERO_SIM.value if robot_uc else None,
        "execution_mode": (
            ExecutionMode.REAL_ZERO_SIM.value
            if robot_uc
            else (server_context.robot.execution_mode if server_context.robot else ExecutionMode.SIMULATION_ONLY.value)
        ),
        "auto_hardware_boot": robot_uc,
        "mission_running": bool(server_context.robot and server_context.robot.mission_running),
        "onnx_loaded": bool(cam_pc is not None and getattr(cam_pc, "onnx_loaded", False)),
        "yolo_onnx_loaded": bool(getattr(camera_manager.yolo_pipeline, "yolo_onnx_loaded", False)),
        "yolo_model_name": str(getattr(camera_manager.yolo_pipeline, "yolo_model_name", "yolov8n.onnx")),
        "version": auto_deploy_manager.get_version_info(),
    }


@router.get("/api/system/version")
async def get_system_version():
    """Retourne le hash court du commit Git actif, le message du dernier commit, l'horodatage et le mode."""
    return auto_deploy_manager.get_version_info()


@router.post("/api/system/trigger-update")
async def post_system_trigger_update(payload: Optional[Dict[str, Any]] = None):
    """Déclenche une mise à jour Git / redémarrage avec Safe-Reload Guard (bloqué si mission en cours)."""
    body = payload or {}
    branch = body.get("branch")
    commit_hash = body.get("commit_hash")
    force_apply_local = bool(body.get("force_apply_local", True))

    # Garder le pointeur live sur le contrôleur actif
    auto_deploy_manager.set_mission_running_guard(
        lambda: bool(server_context.robot and server_context.robot.mission_running)
    )
    res = await asyncio.to_thread(
        auto_deploy_manager.trigger_update,
        branch=branch,
        commit_hash=commit_hash,
        force_apply_local=force_apply_local,
    )
    await ws_manager.broadcast({"type": "system_update_status", **res})
    return res



