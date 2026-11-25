"""tests/test_real_zero_sim_mode.py - Tests du Mode Réel (Zéro Simulation), Hardware & Caméra USB (78°)."""
import pytest
import numpy as np
from fastapi.testclient import TestClient

from main import app
from backend.hardware.hardware_bridge import ExecutionMode, ESP32HardwareBridge
from backend.vision.real_vision_pipeline import RealVisionPipeline
from backend.vision.camera_manager import CameraManager


@pytest.fixture
def client():
    return TestClient(app)


def test_hardware_serial_ports_endpoint(client):
    """Vérifie l'endpoint GET /api/hardware/serial-ports."""
    res = client.get("/api/hardware/serial-ports")
    assert res.status_code == 200
    data = res.json()
    assert "ports" in data
    assert isinstance(data["ports"], list)


def test_camera_devices_and_status_endpoints(client):
    """Vérifie les endpoints GET /api/camera/devices et GET /api/camera/status."""
    res_dev = client.get("/api/camera/devices")
    assert res_dev.status_code == 200
    dev_data = res_dev.json()
    assert "devices" in dev_data
    assert isinstance(dev_data["devices"], list)

    res_stat = client.get("/api/camera/status")
    assert res_stat.status_code == 200
    stat_data = res_stat.json()
    assert "connected" in stat_data
    assert "resolution" in stat_data
    assert "k1" in stat_data
    assert "camera_tilt_deg" in stat_data
    assert "current_mission" in stat_data


def test_camera_config_post(client):
    """Vérifie la mise à jour des paramètres caméra via POST /api/camera/config."""
    payload = {
        "k1": -0.22,
        "camera_tilt_deg": 40.0,
        "current_mission": "WEEDING",
    }
    res = client.post("/api/camera/config", json=payload)
    assert res.status_code == 200
    data = res.json()
    assert data["k1"] == pytest.approx(-0.22, abs=0.01)
    assert data["camera_tilt_deg"] == pytest.approx(40.0, abs=0.1)
    assert data["current_mission"] == "WEEDING"


def test_execution_mode_switching(client):
    """Vérifie la transition entre les 3 modes : SIM_ONLY, COUPLED, REAL_ZERO_SIM."""
    # 1. Mode SIM_ONLY
    res1 = client.post("/api/hardware/mode", json={"execution_mode": "SIM_ONLY"})
    assert res1.status_code == 200
    assert res1.json()["execution_mode"] in ("SIM_ONLY", "SIMULATION_ONLY")

    # 2. Mode REAL_ZERO_SIM
    res2 = client.post("/api/hardware/mode", json={"execution_mode": "REAL_ZERO_SIM"})
    assert res2.status_code == 200
    assert res2.json()["execution_mode"] == "REAL_ZERO_SIM"

    # 3. Mode COUPLED
    res3 = client.post("/api/hardware/mode", json={"execution_mode": "COUPLED"})
    assert res3.status_code == 200
    assert res3.json()["execution_mode"] == "COUPLED"

    # Reset à SIM_ONLY
    client.post("/api/hardware/mode", json={"execution_mode": "SIM_ONLY"})


def test_real_vision_pipeline_4_missions():
    """Vérifie le traitement OpenCV Caméra USB (78°) pour les modes agricoles & conduite manuelle."""
    pipeline = RealVisionPipeline(
        chassis_width_m=1.21,
        tool_spacing_m=0.46,
        camera_height_m=0.75,
        camera_tilt_deg=35.0,
        k1=-0.18,
    )

    # Image de test simulant le sol agricole avec sillon et feuillage
    test_frame = pipeline.generate_synthetic_farm_frame(width=640, height=480, mission="SEEDING")
    assert test_frame.shape == (480, 640, 3)

    # Mission 1 : SEEDING (Guidage ligne directrice centrale + semence M8)
    vis_seeding, telem_seeding = pipeline.process_frame(test_frame, mission="SEEDING")
    assert vis_seeding.shape == (480, 640, 3)
    assert telem_seeding["mode"] == "SEEDING"
    assert "guideline_detected" in telem_seeding
    assert "lateral_error_cm" in telem_seeding
    assert "recommended_steer" in telem_seeding
    assert "seeder_m8_active" in telem_seeding

    # Mission 2 : RESEEDING (Détection de manques > 25 cm dans les rangs arachide)
    vis_reseeding, telem_reseeding = pipeline.process_frame(test_frame, mission="RESEEDING")
    assert vis_reseeding.shape == (480, 640, 3)
    assert telem_reseeding["mode"] == "RESEEDING"
    assert "gaps_count" in telem_reseeding
    assert "reseed_pulse_triggered" in telem_reseeding

    # Mission 3 : WEEDING (Désherbage inter-rang ±23 cm, protection rang central, dents M5 / fraise M6)
    vis_weeding, telem_weeding = pipeline.process_frame(test_frame, mission="WEEDING")
    assert vis_weeding.shape == (480, 640, 3)
    assert telem_weeding["mode"] == "WEEDING"
    assert "weeds_detected_count" in telem_weeding
    assert "tines_m5_lowered" in telem_weeding
    assert "cutter_m6_active" in telem_weeding

    # Sécurité Obstacles (Couloir 1.21 m + Caméra USB 78°)
    vis_obs, telem_obs = pipeline.process_frame(test_frame, mission="OBSTACLE_AVOIDANCE")
    assert vis_obs.shape == (480, 640, 3)
    assert telem_obs["mode"] == "OBSTACLE_AVOIDANCE"
    assert "obstacle_in_corridor" in telem_obs
    assert "min_distance_m" in telem_obs
    assert "emergency_stop" in telem_obs

    # Mode 4 : MANUAL (Conduite Manuelle avec sécurité obstacles active)
    vis_man, telem_man = pipeline.process_frame(test_frame, mission="MANUAL")
    assert vis_man.shape == (480, 640, 3)
    assert telem_man["mode"] == "MANUAL"
    assert "obstacle_in_corridor" in telem_man
    assert "CONDUITE MANUELLE" in telem_man["robot_order_str"] or "STOP OBSTACLE" in telem_man["robot_order_str"]


def test_camera_connect_and_disconnect_no_deadlock(client):
    """Vérifie POST /api/camera/connect (avec width/height/fps) et /api/camera/disconnect sans TypeError ni deadlock."""
    res_conn = client.post(
        "/api/camera/connect",
        json={"device_index": 99, "width": 640, "height": 480, "fps": 30},
    )
    assert res_conn.status_code == 200
    data = res_conn.json()
    assert data["connected"] is True
    assert data["resolution"] == "640x480"
    assert "telemetry" in data

    res_disc = client.post("/api/camera/disconnect")
    assert res_disc.status_code == 200
    assert res_disc.json()["connected"] is False


def test_hardware_ping_endpoint(client):
    """Vérifie l'endpoint POST /api/hardware/ping."""
    res = client.post("/api/hardware/ping", json={"transport": "USB_UART", "serial_port": "/dev/null", "timeout": 0.2})
    assert res.status_code == 200
    data = res.json()
    assert "ok" in data
    assert "response" in data
    assert "latency_ms" in data
    assert "connected" in data


def test_real_zero_sim_arm_and_camera_guidance():
    """Vérifie la synchronisation d'armement ARM et le guidage Caméra USB (78°) en mode REAL_ZERO_SIM."""
    from backend.api import server_context

    robot = server_context.robot
    prev_mode = robot.execution_mode
    try:
        robot.set_execution_mode("REAL_ZERO_SIM")
        # Simuler une connexion matérielle active répondant 'OK' à ARM
        robot.hardware_bridge.connected = True
        orig_send = robot.hardware_bridge.send_command
        robot.hardware_bridge.send_command = lambda cmd, **kw: "OK" if cmd.startswith("ARM") else "OK"
        try:
            res_arm = robot.send_command("ARM")
            assert res_arm == "OK"
            assert robot.armed == 1
            assert robot.is_armed is True
            assert robot.estop_active is False

            res_drive = robot.send_command("DRIVE:25,0:400")
            assert res_drive == "OK"

            # Vérifier que _follow_corridor en mode REAL_ZERO_SIM exploite bien la Caméra USB (78°)
            ok_corr = robot._follow_corridor(
                corridor_center_x=0.0,
                heading_rad=0.0,
                length_units=1.0,
                label="Test Real Zero Sim",
                pct_start=10,
                pct_end=50,
                weeding=False,
                reseeding=False,
                fallback_steps=1,
            )
            assert ok_corr is True
        finally:
            robot.hardware_bridge.send_command = orig_send
            robot.hardware_bridge.connected = False
    finally:
        robot.set_execution_mode(prev_mode)


def test_real_zero_sim_vision_only_dry_run_without_esp32():
    """Vérifie que le mode REAL_ZERO_SIM fonctionne à 100 % sans ESP32 branché (Dry-Run Hardware)."""
    from backend.api import server_context

    robot = server_context.robot
    prev_mode = robot.execution_mode
    logged_lines = []
    cb = lambda d, msg: logged_lines.append((d, msg))
    robot.log_callbacks.append(cb)
    try:
        robot.hardware_bridge.connected = False
        hw = robot.set_execution_mode("REAL_ZERO_SIM")
        assert hw["execution_mode"] == "REAL_ZERO_SIM"
        assert hw["simulation_locked"] is False
        assert hw["vision_test_mode"] is True
        assert hw["status_badge"] == "🤖 Robot : Mode Test Vision (ESP32 non requis)"
        assert robot.is_simulation_locked() is False
        assert robot.is_armed is True

        # Envoi d'une commande DRIVE sans ESP32 connecté : doit réussir en Dry-Run et émettre [VISION IA -> ORDRE ROBOT]
        res = robot.send_command("DRIVE:22,-4:250")
        assert not res.startswith("ERR:")
        assert any("[VISION IA -> ORDRE ROBOT]" in msg for _, msg in logged_lines)

        # Vérification du pipeline 16:9 (1280x720) rectilinéaire (k1=0.0) pour la T'nB INCAM1080A (78°)
        pipeline = RealVisionPipeline(k1=0.0, fov_h_deg=78.0)
        hd_frame = pipeline.generate_synthetic_farm_frame(width=1280, height=720, mission="SEEDING")
        vis_hd, telem_hd = pipeline.process_frame(hd_frame, mission="SEEDING")
        assert vis_hd.shape == (720, 1280, 3)
        assert telem_hd["aspect_ratio"] == "16:9"
        assert "DRIVE:" in telem_hd["robot_order_str"]
        assert "Semoir M8:" in telem_hd["robot_order_str"]
    finally:
        if cb in robot.log_callbacks:
            robot.log_callbacks.remove(cb)
        robot.set_execution_mode(prev_mode)

