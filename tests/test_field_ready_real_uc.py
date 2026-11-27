"""tests/test_field_ready_real_uc.py
Tests unitaires et d'intégration dédiés au premier test terrain réel (`ROBOT_UC`) :
1. Refus de démarrage + alerte « Aucune ligne détectée par le robot » sur sol nu / absence de ligne
   (zéro commande DRIVE envoyée) + blocage sécurité si ESP32 connecté avec caméra synthétique.
2. Propagation exacte de l'angle caméra (`camera_pitch_deg`), de la hauteur (`camera_height_m`)
   et du mode `auto_pitch` (Manuel vs Auto) depuis l'Étape 2 vers `RealVisionPipeline`,
   `YoloVisionPipeline`, `CameraManager` et `RobotController` (avec impact réel sur D_sol).
3. Convention exacte du signe de braquage `DRIVE:a,d` (ligne à droite -> d > 0 ; ligne à gauche -> d < 0).
4. Cycle complet en `REAL_ZERO_SIM` : détection ligne -> suivi continu -> fin de ligne (5 frames)
   -> arrêt outils + relevage M5 -> demi-tour temporisé -> ligne suivante présente (poursuite)
   puis absence de ligne suivante (5 frames) -> « Fin de parcelle — Aucune ligne suivante détectée ».
5. Priorité `/dev/video0` en mode `ROBOT_UC`, coupure immédiate `ESTOP` / `STOP` ESP32,
   et conformité structurelle du parcours 4 étapes + calibration banc + bannière d'alerte UI.
"""

from pathlib import Path
import cv2
import numpy as np
import pytest
from fastapi.testclient import TestClient

from backend.api import server_context
from backend.config.robot_config import robot_config
from backend.vision.camera_manager import CameraManager, camera_manager
from backend.vision.real_vision_pipeline import RealVisionPipeline
from backend.vision.yolo_pipeline import YoloVisionPipeline


def _make_bare_soil_frame(width: int = 640, height: int = 480) -> np.ndarray:
    """Sol sableux uniforme sans sillon sombre ni végétation."""
    frame = np.full((height, width, 3), (112, 162, 198), dtype=np.uint8)
    rng = np.random.default_rng(7)
    noise = rng.integers(-3, 4, size=(height, width, 3), dtype=np.int16)
    return np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)


def _make_furrow_frame(x_offset_px: int = 0, width: int = 640, height: int = 480) -> np.ndarray:
    """Sol sableux avec un sillon sombre net décalé de `x_offset_px` par rapport au centre."""
    frame = _make_bare_soil_frame(width=width, height=height)
    cx = (width // 2) + x_offset_px
    cv2.line(frame, (cx, int(height * 0.30)), (cx, height - 1), (28, 52, 74), 14)
    return frame


def test_no_line_bare_soil_blocks_autonomous_start_and_alerts(monkeypatch):
    """1. Sur sol nu (aucune ligne), le pipeline renvoie a=0 / outils OFF et le contrôleur
    refuse de démarrer avec zéro DRIVE et l'alerte « Aucune ligne détectée par le robot ».
    """
    bare = _make_bare_soil_frame()
    pipeline = RealVisionPipeline()

    for mode in ("SEEDING", "RESEEDING", "WEEDING"):
        _, telem = pipeline.process_frame(bare, mission=mode)
        assert telem["target_detected"] is False, f"{mode}: ne doit pas détecter de ligne sur sol nu"
        assert telem["is_aligned"] is False, f"{mode}: ne doit pas être aligné sans ligne"
        assert telem["recommended_drive"]["a"] == 0, f"{mode}: vitesse d'avance doit être 0 sans ligne"
        assert telem["seeder_m8_active"] is False
        assert telem["cutter_m6_active"] is False
        assert telem["tines_m5_lowered"] is False
        assert "STOP" in telem["robot_order_str"]

    robot = server_context.robot
    prev_mode = robot.execution_mode
    prev_state = robot.state
    sent_cmds = []
    alerts = []
    orig_send = robot.send_command

    def spy_send(cmd, *args, **kwargs):
        sent_cmds.append(cmd)
        return orig_send(cmd, *args, **kwargs)

    def on_event(ev_name, payload):
        if ev_name == "MISSION_ALERT":
            alerts.append(payload)

    monkeypatch.setattr(robot, "send_command", spy_send)
    monkeypatch.setattr(
        camera_manager,
        "get_latest_telemetry",
        lambda: {
            "guideline_detected": False,
            "row_detected": False,
            "target_detected": False,
            "is_valid_peanut_row": False,
            "hybrid_boundary_alert": True,
            "recommended_drive": {"a": 0, "d": 0, "ms": 250},
            "obstacle_stop": False,
        },
    )
    # Accélérer la boucle d'échantillonnage pré-départ pour le test
    monkeypatch.setattr(
        robot,
        "_verify_initial_line_presence",
        lambda mission_type="SEEDING": type(robot)._verify_initial_line_presence(
            robot, mission_type=mission_type, sample_frames=6, frame_delay_s=0.01
        ),
    )

    robot.event_callbacks.append(on_event)
    try:
        robot.hardware_bridge.connected = False
        robot.set_execution_mode("REAL_ZERO_SIM")
        robot.last_mission_alert = None

        ok = robot.mission_autonomous_seeding()
        assert ok is False
        assert robot.state == "ABORTED_NO_LINE"
        assert not any(c.upper().startswith("DRIVE:") for c in sent_cmds), f"Aucun DRIVE permis sans ligne: {sent_cmds}"
        assert "STOP" in sent_cmds
        assert "STOP:6" in sent_cmds
        assert "STOP:8" in sent_cmds
        assert robot.last_mission_alert is not None
        assert "Aucune ligne détectée par le robot" in robot.last_mission_alert["message"]
        assert any("Aucune ligne détectée par le robot" in a["message"] for a in alerts)
    finally:
        if on_event in robot.event_callbacks:
            robot.event_callbacks.remove(on_event)
        robot.state = prev_state
        robot.set_execution_mode(prev_mode)


def test_real_esp32_connected_blocks_synthetic_camera_motion(monkeypatch):
    """1-bis. Si l'ESP32-S3 physique est connecté alors que la caméra est synthétique ou déconnectée,
    le départ autonome est bloqué par sécurité.
    """
    robot = server_context.robot
    prev_mode = robot.execution_mode
    prev_state = robot.state
    prev_hw_conn = robot.hardware_bridge.connected
    prev_cam_conn = camera_manager.connected
    prev_cam_synth = camera_manager.is_synthetic
    sent_cmds = []

    monkeypatch.setattr(robot.hardware_bridge, "send_command", lambda cmd, **kw: (sent_cmds.append(cmd), "OK")[1])
    try:
        robot.set_execution_mode("REAL_ZERO_SIM")
        robot.hardware_bridge.connected = True
        camera_manager.connected = True
        camera_manager.is_synthetic = True
        robot.armed = 1
        robot.is_armed = True
        robot.estop_active = False

        allowed = robot._verify_initial_line_presence("SEEDING", sample_frames=5, frame_delay_s=0.01)
        assert allowed is False
        assert robot.state == "CAMERA_SAFETY_BLOCKED"
        assert "STOP" in sent_cmds
        assert robot.last_mission_alert is not None
        assert robot.last_mission_alert["code"] == "SYNTHETIC_CAMERA_BLOCKED"
    finally:
        camera_manager.connected = prev_cam_conn
        camera_manager.is_synthetic = prev_cam_synth
        robot.hardware_bridge.connected = prev_hw_conn
        robot.state = prev_state
        robot.set_execution_mode(prev_mode)


def test_step2_camera_pitch_height_and_auto_pitch_propagation(client: TestClient):
    """2. Vérifie la propagation exacte de camera_pitch_deg, camera_height_m et auto_pitch
    vers CameraManager, RealVisionPipeline, YoloVisionPipeline et RobotController, et l'impact sur D_sol.
    """
    try:
        # 1. Angle manuel strict à 20° / hauteur 1.05 m via les deux endpoints de l'Étape 2
        res_spec = client.put(
            "/api/config/specs",
            json={"camera_pitch_deg": 20.0, "camera_height_m": 1.05, "auto_pitch": False},
        )
        assert res_spec.status_code == 200
        res_cam = client.post(
            "/api/camera/config",
            json={"camera_pitch_deg": 20.0, "camera_height_m": 1.05, "auto_pitch": False},
        )
        assert res_cam.status_code == 200
        cam_data = res_cam.json()

        assert cam_data["camera_pitch_deg"] == pytest.approx(20.0, abs=0.1)
        assert cam_data["camera_tilt_deg"] == pytest.approx(20.0, abs=0.1)
        assert cam_data["camera_height_m"] == pytest.approx(1.05, abs=0.01)
        assert cam_data["auto_pitch"] is False

        assert camera_manager.pipeline.camera_tilt_deg == pytest.approx(20.0, abs=0.1)
        assert camera_manager.pipeline.camera_height_m == pytest.approx(1.05, abs=0.01)
        assert camera_manager.pipeline.auto_pitch is False
        assert server_context.robot.robot_specs["camera_tilt_deg"] == pytest.approx(20.0, abs=0.1)
        assert server_context.robot.robot_specs["camera_height_cm"] == pytest.approx(105.0, abs=0.2)

        # Impact mathématique réel sur la distance au sol D_sol
        d_20 = camera_manager.pipeline._estimate_distance_from_y(py=240, h=480)

        # 2. Changement manuel à 45° -> D_sol doit diminuer nettement et HUD indiquer "Manuel : 45.0°"
        client.put("/api/config/specs", json={"camera_pitch_deg": 45.0, "camera_height_m": 1.05, "auto_pitch": False})
        d_45 = camera_manager.pipeline._estimate_distance_from_y(py=240, h=480)
        assert d_45 < d_20, f"À 45° la distance au centre ({d_45}m) doit être inférieure à celle à 20° ({d_20}m)"

        test_frame = _make_furrow_frame(0)
        _, telem_manual = camera_manager.yolo_pipeline.process_frame(test_frame, mission="SEEDING")
        assert telem_manual["auto_pitch"] is False
        assert telem_manual["camera_pitch_deg"] == pytest.approx(45.0, abs=0.1)
        assert "Manuel : 45.0°" in telem_manual["hud_pitch_label"]

        # 3. Calibration automatique via POST /api/camera/calibrate-pitch (Point de fuite & Mire 1.00 m)
        res_vp = client.post("/api/camera/calibrate-pitch", json={"mode": "vanishing_point", "camera_height_m": 0.90})
        assert res_vp.status_code == 200
        vp_json = res_vp.json()
        assert vp_json["ok"] is True
        assert vp_json["auto_pitch"] is True
        assert 5.0 <= vp_json["camera_pitch_deg"] <= 80.0

        res_mk = client.post(
            "/api/camera/calibrate-pitch",
            json={"mode": "ground_marker", "marker_distance_m": 1.0, "y_marker": 320.0, "camera_height_m": 0.90},
        )
        assert res_mk.status_code == 200
        mk_json = res_mk.json()
        assert mk_json["ok"] is True
        assert 5.0 <= mk_json["camera_pitch_deg"] <= 80.0
    finally:
        client.post("/api/config/specs/reset")
        robot_config.reset_specs()


def test_steering_sign_matches_esp32_drive_convention():
    """3. Vérifie le signe de braquage conforme au firmware ESP32 DRIVE:a,d (L=a+d, R=a-d) :
    - Ligne décalée à DROITE de l'image -> err_cm > 0 -> d > 0 (tourne à DROITE).
    - Ligne décalée à GAUCHE de l'image -> err_cm < 0 -> d < 0 (tourne à GAUCHE).
    """
    pipe_right = RealVisionPipeline()
    _, t_right = pipe_right.process_frame(_make_furrow_frame(x_offset_px=+85), mission="SEEDING")
    assert t_right["guideline_detected"] is True
    assert t_right["lateral_error_cm"] > 4.0
    assert t_right["recommended_steer"] > 0, f"Ligne à droite doit produire d > 0, reçu {t_right['recommended_steer']}"
    assert t_right["recommended_drive"]["d"] > 0

    pipe_left = RealVisionPipeline()
    _, t_left = pipe_left.process_frame(_make_furrow_frame(x_offset_px=-85), mission="SEEDING")
    assert t_left["guideline_detected"] is True
    assert t_left["lateral_error_cm"] < -4.0
    assert t_left["recommended_steer"] < 0, f"Ligne à gauche doit produire d < 0, reçu {t_left['recommended_steer']}"
    assert t_left["recommended_drive"]["d"] < 0


def test_full_cycle_line_follow_end_of_row_uturn_and_field_end(monkeypatch):
    """4. Scénario complet REAL_ZERO_SIM :
    détection ligne -> suivi continu (DRIVE) -> fin de ligne (5 frames sans ligne)
    -> STOP + outils OFF + relevage M5 -> demi-tour temporisé -> ligne suivante détectée (Passe 2)
    -> fin de Passe 2 -> absence ligne suivante (5 frames) -> « Fin de parcelle — Aucune ligne suivante détectée ».
    """
    robot = server_context.robot
    prev_mode = robot.execution_mode
    prev_state = robot.state
    prev_tines = robot.seeding_use_tines
    prev_len = robot.field_length_m
    prev_wid = robot.field_width_m
    sent_cmds = []
    orig_send = robot.send_command

    def spy_send(cmd, *args, **kwargs):
        sent_cmds.append(cmd)
        return orig_send(cmd, *args, **kwargs)

    # Séquence de télémétrie vision couvrant 2 passes complètes puis fin de parcelle :
    # - 2 frames pré-départ OK
    # - Passe 1 : 3 frames suivi OK -> 5 frames sans ligne (fin de Passe 1)
    # - Vérif après demi-tour 1 : 1 frame OK (ligne 2 présente !)
    # - Passe 2 : 2 frames suivi OK -> 5 frames sans ligne (fin de Passe 2)
    # - Vérif après demi-tour 2 : 6 frames sans ligne -> Fin de parcelle !
    seq = (
        [True] * 2
        + [True] * 3
        + [False] * 5
        + [True] * 1
        + [True] * 2
        + [False] * 5
        + [False] * 10
    )
    idx_holder = {"i": 0}

    def fake_telem():
        i = idx_holder["i"]
        idx_holder["i"] += 1
        present = seq[i] if i < len(seq) else False
        return {
            "guideline_detected": present,
            "row_detected": present,
            "target_detected": present,
            "is_valid_peanut_row": present,
            "hybrid_boundary_alert": not present,
            "lateral_error_cm": 5.0 if present else 0.0,
            "heading_error_deg": 0.0,
            "recommended_steer": 4 if present else 0,
            "recommended_drive": {"a": 22, "d": 4, "ms": 250} if present else {"a": 0, "d": 0, "ms": 250},
            "obstacle_stop": False,
        }

    monkeypatch.setattr(robot, "send_command", spy_send)
    monkeypatch.setattr(camera_manager, "get_latest_telemetry", fake_telem)
    monkeypatch.setattr(robot, "wait_for_done", lambda motors, timeout=5.0: True)
    monkeypatch.setattr(
        robot,
        "_verify_initial_line_presence",
        lambda mission_type="SEEDING": type(robot)._verify_initial_line_presence(
            robot, mission_type=mission_type, sample_frames=6, frame_delay_s=0.0
        ),
    )

    try:
        robot.hardware_bridge.connected = False
        robot.set_execution_mode("REAL_ZERO_SIM")
        robot.seeding_use_tines = True
        robot.field_length_m = 10.0
        robot.field_width_m = 4.6  # > 3 passes prévues, mais doit s'arrêter dès que la passe 3 n'existe pas !
        robot.progression_direction = -1  # DROITE
        robot.last_mission_alert = None

        ok = robot.mission_autonomous_seeding()
        assert ok is True

        drive_cmds = [c for c in sent_cmds if c.startswith("DRIVE:")]
        tank_cmds = [c for c in sent_cmds if c.startswith("TANK:")]
        lift_cmds = [c for c in sent_cmds if c.startswith("PULSE:5:-50")]

        assert len(drive_cmds) >= 5, f"Le suivi continu devait envoyer plusieurs DRIVE: {drive_cmds}"
        assert any("DRIVE:22,4:" in c for c in drive_cmds), f"La correction steer d=+4 devait être transmise: {drive_cmds}"
        assert len(lift_cmds) >= 2, f"Les dents M5 devaient être relevées en fin des 2 passes: {lift_cmds}"
        assert len(tank_cmds) >= 4, f"Deux demi-tours complets (2×2 TANK) attendus: {tank_cmds}"
        # Vérifier l'inversion correcte du sens de rotation entre le 1er demi-tour (DROITE: TANK:35,-35) et le 2ème (GAUCHE: TANK:-35,35)
        assert tank_cmds[0].startswith("TANK:35,-35"), f"1er demi-tour vers DROITE attendu TANK:35,-35, reçu {tank_cmds[0]}"
        assert tank_cmds[2].startswith("TANK:-35,35"), f"2ème demi-tour en retour attendu TANK:-35,35, reçu {tank_cmds[2]}"
        assert "STOP" in sent_cmds
        assert "STOP:8" in sent_cmds
        assert robot.state == "MISSION_COMPLETED_BY_HYBRID_VISION"
        assert robot.last_mission_alert is not None
        assert "Fin de parcelle — Aucune ligne suivante détectée" in robot.last_mission_alert["message"]
    finally:
        robot.seeding_use_tines = prev_tines
        robot.field_length_m = prev_len
        robot.field_width_m = prev_wid
        robot.state = prev_state
        robot.set_execution_mode(prev_mode)


def test_robot_uc_prioritizes_dev_video0_and_instant_estop_stop(monkeypatch):
    """5. En mode ROBOT_UC, /dev/video0 est prioritaire par défaut, et ESTOP/STOP coupent immédiatement l'ESP32."""
    import sys

    cam_mod = sys.modules["backend.vision.camera_manager"]
    monkeypatch.setenv("DEPLOYMENT_TARGET", "ROBOT_UC")

    # Cas A : aucun /dev/video* branché -> fallback par défaut sur /dev/video0 (et non /dev/video2)
    monkeypatch.setattr(cam_mod.glob, "glob", lambda pattern: [])
    devs_empty = CameraManager.scan_camera_devices()
    assert devs_empty[0]["index"] == 0
    assert devs_empty[0]["path"] == "/dev/video0"

    # Cas B : /dev/video0 et /dev/video2 présents sur HP t530 -> /dev/video0 arrive en premier
    monkeypatch.setattr(cam_mod.glob, "glob", lambda pattern: ["/dev/video2", "/dev/video0"])
    monkeypatch.setattr(cam_mod.os.path, "exists", lambda p: False)
    devs_both = CameraManager.scan_camera_devices()
    assert devs_both[0]["index"] == 0
    assert devs_both[0]["path"] == "/dev/video0"

    # Coupure instantanée STOP et ESTOP sur pont matériel connecté
    robot = server_context.robot
    prev_mode = robot.execution_mode
    prev_conn = robot.hardware_bridge.connected
    hw_sent = []
    monkeypatch.setattr(robot.hardware_bridge, "send_command", lambda cmd, **kw: (hw_sent.append(cmd), f"OK {cmd}")[1])
    try:
        robot.set_execution_mode("REAL_ZERO_SIM")
        robot.hardware_bridge.connected = True
        robot.armed = 1
        robot.is_armed = True
        robot.estop_active = False
        robot.speed_kmh = 1.8

        res_stop = robot.stop()
        assert "STOP" in hw_sent
        assert robot.speed_kmh == 0.0
        assert "OK" in res_stop

        res_estop = robot.estop()
        assert "ESTOP" in hw_sent
        assert robot.armed == 0
        assert robot.is_armed is False
        assert robot.estop_active is True
        assert res_estop == "OK ESTOP"
    finally:
        robot.hardware_bridge.connected = prev_conn
        robot.set_execution_mode(prev_mode)


def test_robot_uc_4_step_wizard_and_calibration_ui_structure(client: TestClient):
    """6. Vérifie l'intégrité HTML/JS du parcours 4 étapes ROBOT_UC, du banc calibration et de la bannière d'alerte."""
    repo_root = Path(__file__).resolve().parent.parent
    wiz_js = (repo_root / "robot_uc" / "web" / "js" / "ui" / "wizard_navigator.js").read_text(encoding="utf-8")
    html = client.get("/").text

    # Étape 1 UC : Connexion ESP32 + Calibration Moteurs + Mode Test Vision
    assert "Étape 1/4 — Connexion au Vrai Robot (ESP32-S3)" in wiz_js
    assert "btnWizUcOpenCalibration" in wiz_js
    assert "Continuer en Mode Test Vision sans ESP32" in wiz_js
    assert "Suivant : Réglage Caméra →" in wiz_js

    # Étape 2 UC : Caméra /dev/video0 + Stream + Angle Manuel / Auto + Sauvegarde double endpoint
    assert "Étape 2/4 — Réglage &amp; Calibration de la Caméra" in wiz_js
    assert "/api/camera/stream" in wiz_js
    assert "wizPitchModeManual" in wiz_js
    assert "wizPitchModeAuto" in wiz_js
    assert "/api/camera/calibrate-pitch" in wiz_js
    assert "/api/config/specs" in wiz_js
    assert "/api/camera/config" in wiz_js
    assert "Suivant : Choix de la Mission →" in wiz_js

    # Étape 3 UC : Boutons retour Caméra et Robot
    assert "btnWizUcBackToCamera" in wiz_js
    assert "btnWizUcBackToRobot" in wiz_js

    # Banc de calibration dans #hardwareConfigDialog (ARM, ESTOP, DRIVE:30,0, DRIVE:0,30, Test 20%, regex :=)
    assert "DRIVE:30,0:800" in html
    assert "DRIVE:0,30:800" in html
    assert "PULSE:${m.id}:20:1000" in html
    assert "INV[:=]" in html

    # Bannière d'alerte mission visible en UI
    assert 'id="missionStatusAlertBanner"' in html
    assert "MISSION_ALERT" in html
    assert "showMissionStatusAlert" in html

