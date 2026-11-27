"""tests/test_mission_persistence_and_multisession.py
Tests unitaires et d'intégration pour la persistance de mission & la synchronisation multi-appareils :
1. Déconnexion/reconnexion WebSocket (F5 ou 0 client connecté) pendant une mission active :
   aucune interruption de la mission, ni déconnexion ESP32/caméra.
2. Nouveau client frais (`GET /api/status` et `/ws`) reçoit le `mission_snapshot` complet
   (`mission_active`, `mission_paused`, `mission_status`, `mission_type`, `mission_label`,
   `current_action`, `row_index`, `total_rows_est`, `distance_m`, `elapsed_s`,
   `hardware_connected`, `camera_connected`, `camera_pitch_deg`, `auto_pitch`, `recent_logs`).
3. Buffer circulaire `recent_logs` (30 dernières lignes max, zéro bruit PING/PONG/#T).
4. Protection côté serveur (`Mission Lock` HTTP 409 Conflict + WS `mission_lock_rejected`)
   pendant une mission active OU en pause, tout en autorisant `pause`, `resume`, `stop`, `abort`, `estop` et la lecture.
5. Vérification structurelle Desktop (`index.html`, `wizard_navigator.js`, `ws_client.js`, `state_store.js`)
   et Mobile (`mobile.html`, `mobile_main.js`, `mobile_missions.js`) : zéro commande destructive au unload/close,
   hydratation prioritaire `bootstrapFromServerState()`, saut direct Étape 4 / vue active mobile.
"""

from pathlib import Path
import time
from fastapi.testclient import TestClient

from backend.api import server_context
from backend.config.robot_config import robot_config
from backend.vision.camera_manager import camera_manager


def _setup_simulated_running_mission(robot, mission_name: str = "smart_weeding"):
    """Place le contrôleur dans un état déterministe de mission en cours sans dépendance matérielle."""
    robot.arm()
    robot.current_mission_name = mission_name
    robot._last_mission_type = robot._normalize_mission_type(mission_name)
    robot.mission_running = True
    robot.mission_paused = False
    robot._last_mission_status = "RUNNING"
    robot._mission_started_ts = time.time() - 12.5
    robot._mission_ended_ts = None
    robot.current_corridor = 2
    robot.corridors_total = 10
    robot.distance_traveled_m = 8.4
    robot.field_length_m = 20.0
    robot.field_width_m = 4.6
    robot.progression_direction = -1
    robot.direction_mode = "RIGHT"
    robot.locked_side = "RIGHT"
    robot.mission_step_label = "Suivi du rang 2 — Fraise active"
    robot._emit_log("RX", "MISSION DÉMARRÉE : 3. Désherbage IA (Rang 2/10)")


def _cleanup_mission_state(robot):
    robot.mission_running = False
    robot.mission_paused = False
    robot.current_mission_name = None
    robot._last_mission_status = "IDLE"
    robot._mission_started_ts = None
    robot._mission_ended_ts = None
    robot.mission_step_label = "Prêt"
    robot.last_mission_alert = None


def test_ws_disconnect_and_reconnect_preserves_active_mission_and_delivers_snapshot(client: TestClient):
    """1 & 2. Une déconnexion WebSocket (F5) n'interrompt pas la mission, et un 2e client frais
    reçoit immédiatement tout le `mission_snapshot` sur `GET /api/status` et sur `/ws`.
    """
    robot = server_context.robot
    prev_hw_conn = robot.hardware_bridge.connected
    prev_cam_conn = camera_manager.connected
    try:
        robot.hardware_bridge.connected = True
        camera_manager.connected = True
        _setup_simulated_running_mission(robot, "smart_weeding")

        # Client 1 ouvre /ws, reçoit l'état initial, puis se déconnecte (simule F5 / onglet fermé)
        with client.websocket_connect("/ws") as ws1:
            msg1 = ws1.receive_json()
            assert msg1["type"] == "state"
            snap1 = msg1["mission_snapshot"]
            assert snap1["mission_active"] is True
            assert snap1["mission_type"] == "weeding"
            assert snap1["mission_label"] == "3. Désherbage IA"
            assert "Suivi du rang 2" in snap1["current_action"]

        # Après déconnexion totale (0 client WS) : RIEN ne doit être arrêté ni déconnecté !
        assert robot.mission_running is True
        assert robot.current_mission_name == "smart_weeding"
        assert robot.hardware_bridge.connected is True
        assert camera_manager.connected is True

        # Client 2 (nouveau téléphone / PC) interroge GET /api/status
        res_status = client.get("/api/status")
        assert res_status.status_code == 200
        body = res_status.json()
        assert body["mission_active"] is True
        assert body["mission_paused"] is False
        snap = body["mission_snapshot"]

        required_fields = [
            "mission_active",
            "mission_paused",
            "mission_status",
            "mission_type",
            "mission_label",
            "current_action",
            "phase_label",
            "row_index",
            "total_rows_est",
            "distance_m",
            "elapsed_s",
            "parcel_length_m",
            "parcel_width_m",
            "direction_mode",
            "locked_side",
            "hardware_connected",
            "hardware_port",
            "armed",
            "camera_connected",
            "camera_device",
            "camera_pitch_deg",
            "auto_pitch",
            "recent_logs",
        ]
        for field in required_fields:
            assert field in snap, f"Champ '{field}' manquant dans mission_snapshot"

        assert snap["mission_active"] is True
        assert snap["mission_status"] == "RUNNING"
        assert snap["row_index"] == 2
        assert snap["total_rows_est"] == 10
        assert snap["distance_m"] == 8.4
        assert snap["elapsed_s"] >= 12.0
        assert snap["parcel_length_m"] == 20.0
        assert snap["parcel_width_m"] == 4.6
        assert snap["direction_mode"] == "RIGHT"
        assert snap["locked_side"] == "RIGHT"
        assert snap["hardware_connected"] is True
        assert snap["camera_connected"] is True
        assert isinstance(snap["recent_logs"], list) and len(snap["recent_logs"]) >= 1
        assert any("Désherbage IA" in e["line"] for e in snap["recent_logs"])

        # Client 2 ouvre /ws : le 1er message contient aussi tout le mission_snapshot
        with client.websocket_connect("/ws") as ws2:
            first_msg = ws2.receive_json()
            assert first_msg["type"] == "state"
            assert first_msg["mission_snapshot"]["mission_active"] is True
            assert first_msg["mission_snapshot"]["mission_type"] == "weeding"
            assert first_msg["mission_snapshot"]["row_index"] == 2
            assert any("Désherbage IA" in e["line"] for e in first_msg["mission_snapshot"]["recent_logs"])
    finally:
        _cleanup_mission_state(robot)
        robot.hardware_bridge.connected = prev_hw_conn
        camera_manager.connected = prev_cam_conn


def test_recent_logs_circular_buffer_30_lines_and_noise_filtered():
    """3. Vérifie le buffer circulaire de 30 lignes max et l'exclusion absolue de PING/PONG/#T."""
    robot = server_context.robot
    robot._recent_logs.clear()
    for i in range(45):
        robot._emit_log("TX", f"DRIVE:20,0:400 #{i}")
        robot._emit_log("TX", "PING")
        robot._emit_log("RX", "PONG")
        robot._emit_log("RX", "#T 1 20,20,20,20,0,0,0,0 12345")

    logs = robot.get_recent_logs(30)
    assert len(logs) == 30
    assert logs[0]["line"] == "DRIVE:20,0:400 #15"
    assert logs[-1]["line"] == "DRIVE:20,0:400 #44"
    assert not any(e["line"] in ("PING", "PONG") or e["line"].startswith("#T ") for e in logs)


def test_server_mission_lock_409_and_allowed_control_endpoints(client: TestClient):
    """4. Pendant une mission active ou en pause :
    - 409 Conflict sur 2e mission, config/specs, robot-specs, calibration, reconnexion matériel/caméra.
    - 200 OK sur pause, resume, stop, abort, estop et lecture /api/status.
    - Rejet WS `mission_lock_rejected` (409) sur tentative concurrente.
    """
    robot = server_context.robot
    try:
        _setup_simulated_running_mission(robot, "seeding")

        # Toutes les mutations concurrentes doivent renvoyer 409 Conflict
        assert client.post("/api/mission", json={"name": "weeding"}).status_code == 409
        assert client.put("/api/config/specs", json={"camera_pitch_deg": 40.0}).status_code == 409
        assert client.post("/api/config/specs/reset").status_code == 409
        assert client.post("/api/robot-specs", json={"chassis_length_top_cm": 170.0}).status_code == 409
        assert client.post("/api/hardware/mode", json={"execution_mode": "SIMULATION_ONLY"}).status_code == 409
        assert client.post("/api/hardware/connect", json={"transport": "USB_UART"}).status_code == 409
        assert client.post("/api/hardware/disconnect").status_code == 409
        assert client.post("/api/hardware/calibrate", json={"motor": 1, "inv": 1}).status_code == 409
        assert client.post("/api/camera/config", json={"camera_pitch_deg": 40.0}).status_code == 409
        assert client.post("/api/camera/connect", json={"device_index": 0}).status_code == 409
        assert client.post("/api/camera/disconnect").status_code == 409
        assert client.post("/api/camera/calibrate-pitch", json={"mode": "vanishing_point"}).status_code == 409

        # Vérification WS : tentative concurrente rejetée en 409 sans casser la mission
        with client.websocket_connect("/ws") as ws:
            _ = ws.receive_json()  # état initial
            ws.send_json({"type": "mission", "name": "weeding"})
            rej = None
            for _ in range(8):
                cand = ws.receive_json()
                if cand.get("type") == "mission_lock_rejected":
                    rej = cand
                    break
            assert rej is not None, "Message WS mission_lock_rejected non reçu"
            assert rej["status_code"] == 409
            assert robot.current_mission_name == "seeding"

        # PAUSE autorisée -> état passe à PAUSED -> 409 toujours maintenu pendant la pause !
        res_pause = client.post("/api/pause")
        assert res_pause.status_code == 200
        assert res_pause.json()["paused"] is True
        assert res_pause.json()["mission_snapshot"]["mission_status"] == "PAUSED"
        assert client.post("/api/mission", json={"name": "weeding"}).status_code == 409
        assert client.put("/api/config/specs", json={"camera_pitch_deg": 40.0}).status_code == 409

        # RESUME autorisée -> revient à RUNNING
        res_resume = client.post("/api/resume")
        assert res_resume.status_code == 200
        assert res_resume.json()["paused"] is False
        assert res_resume.json()["mission_snapshot"]["mission_status"] == "RUNNING"

        # STOP et ABORT autorisés -> déverrouillage immédiat
        assert client.post("/api/stop").status_code == 200
        res_abort = client.post("/api/abort")
        assert res_abort.status_code == 200
        assert robot.is_mission_locked() is False

        # Après arrêt, la modification de configuration redevient autorisée (200 OK)
        res_ok = client.put("/api/config/specs", json={"camera_pitch_deg": 32.0})
        assert res_ok.status_code == 200

        # ESTOP autorisé même pendant une mission active -> coupe et déverrouille
        _setup_simulated_running_mission(robot, "reseeding")
        res_estop = client.post("/api/estop")
        assert res_estop.status_code == 200
        assert robot.is_mission_locked() is False
        assert robot.armed == 0
    finally:
        _cleanup_mission_state(robot)
        robot_config.reset_specs()


def test_desktop_and_mobile_frontends_implement_persistence_and_direct_active_view(client: TestClient):
    """5. Vérifie la structure frontend Desktop & Mobile :
    - Aucune commande destructive sur beforeunload / pagehide / onclose.
    - Hydratation prioritaire `bootstrapFromServerState()` -> saut direct Étape 4 / Vue active mobile.
    - Bandeau `ROBOT EN COURS DE TRAVAIL`, replay `recent_logs`, contrôles Pause/Reprendre/Arrêter/ESTOP.
    """
    repo_root = Path(__file__).resolve().parent.parent
    web_dir = repo_root / "robot_uc" / "web"

    index_html = client.get("/").text
    wiz_js = (web_dir / "js" / "ui" / "wizard_navigator.js").read_text(encoding="utf-8")
    ws_js = (web_dir / "js" / "core" / "ws_client.js").read_text(encoding="utf-8")
    store_js = (web_dir / "js" / "core" / "state_store.js").read_text(encoding="utf-8")
    mobile_html = client.get("/mobile").text
    mob_main_js = (web_dir / "js" / "mobile" / "mobile_main.js").read_text(encoding="utf-8")
    mob_miss_js = (web_dir / "js" / "mobile" / "mobile_missions.js").read_text(encoding="utf-8")

    # 1. Zéro envoi STOP/ESTOP/disconnect dans beforeunload/pagehide
    assert 'addEventListener("beforeunload"' not in index_html
    assert 'addEventListener("pagehide"' not in index_html
    assert 'addEventListener("beforeunload"' not in mob_main_js

    # 2. Desktop : bootstrapFromServerState() + attachToActiveMission() + bandeau travail + recent_logs
    assert "bootstrapFromServerState(" in index_html
    assert "applyServerMissionSnapshot(" in index_html
    assert "replayServerRecentLogs(" in index_html
    assert 'id="liveMissionWorkBanner"' in index_html
    assert "ROBOT EN COURS DE TRAVAIL" in index_html
    assert "togglePauseResumeMission()" in index_html
    assert "mission_lock_rejected" in index_html
    assert "attachToActiveMission(" in wiz_js
    assert "snap.mission_active || snap.mission_paused" in wiz_js

    # 3. Core shared modules : state_store.js & ws_client.js
    assert "missionSnapshot" in store_js
    assert "bootstrapFromServerState()" in ws_js
    assert "_replayRecentLogs(" in ws_js
    assert "pauseMission()" in ws_js
    assert "resumeMission()" in ws_js
    assert "abortMission()" in ws_js

    # 4. Mobile : vue mission active auto-ouverte + masquage connexion + flux vidéo + Pause/Arrêter + verrouillage onglets
    assert 'id="mobActiveMissionCard"' in mobile_html
    assert 'id="mobConnectionCard"' in mobile_html
    assert 'id="mobLiveCameraStream"' in mobile_html
    assert 'src="/api/camera/stream"' in mobile_html
    assert 'id="mobBtnPauseResume"' in mobile_html
    assert 'id="mobBtnAbortMission"' in mobile_html
    assert "await wsClient.bootstrapFromServerState()" in mob_main_js
    assert "connCard.style.display = isMissionLive ? \"none\" : \"block\"" in mob_main_js
    assert "isLockedByActiveMission" in mob_miss_js
