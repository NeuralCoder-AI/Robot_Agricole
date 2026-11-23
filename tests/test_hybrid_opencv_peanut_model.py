"""tests/test_hybrid_opencv_peanut_model.py
Tests unitaires de la Simplification Binaire Stricte (Arachide vs Sol Nu) & MobileNetV3 ONNX :
1. Présence & chargement effectif de `backend/models/mobilenetv3_peanut.onnx` (`cv2.dnn` & `/api/camera/status`).
2. Certification binaire positive : rang aligné d'arachide (`is_valid_peanut_row=True`, `"Arachide détectée"`).
3. Détection Sol Nu (fin de ligne) + règle stricte des 5 frames consécutives (`hybrid_line_end_confirmed=True` à la 5ème).
4. Arrêt immédiat dans `KinematicEngine` après 5 frames invalides : demi-tour avant `field_length=50m` puis arrêt définitif `MISSION_COMPLETED_BY_HYBRID_VISION` (`MISSION TERMINÉE — FIN DES LIGNES D'ARACHIDE`).
"""

from pathlib import Path
import cv2
import numpy as np

from backend.navigation.kinematic_engine import KinematicEngine
from backend.vision.peanut_classifier import PeanutPlantClassifier, SPECIES_BARE_SOIL, SPECIES_PEANUT
from backend.vision.real_vision_pipeline import RealVisionPipeline, validate_peanut_row_hybrid


def _make_sandy_frame(width: int = 640, height: int = 480) -> np.ndarray:
    """Crée une image synthétique de sol sableux agricole sans végétation."""
    frame = np.full((height, width, 3), (115, 165, 202), dtype=np.uint8)
    rng = np.random.default_rng(42)
    noise = rng.integers(-6, 7, size=(height, width, 3), dtype=np.int16)
    return np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)


def _draw_peanut_rosette(frame: np.ndarray, cx: int, cy: int, scale: float = 1.0) -> None:
    """Dessine un plant d'arachide synthétique à 4 folioles ovales."""
    r_major = max(6, int(11 * scale))
    r_minor = max(4, int(7 * scale))
    offset = max(5, int(9 * scale))
    for dx, dy, angle in [
        (-offset, -offset // 2, 25),
        (offset, -offset // 2, -25),
        (-offset, offset // 2, -25),
        (offset, offset // 2, 25),
    ]:
        cv2.ellipse(
            frame,
            (cx + dx, cy + dy),
            (r_major, r_minor),
            angle,
            0,
            360,
            (48, 168, 52),
            -1,
        )
    cv2.circle(frame, (cx, cy), max(2, int(3 * scale)), (38, 145, 42), -1)


def test_mobilenetv3_onnx_loaded_and_api_camera_status(client):
    """Vérifie le fichier ONNX réel sur disque, le chargement cv2.dnn et `onnx_loaded == True` dans `/api/camera/status`."""
    onnx_path = Path(__file__).resolve().parent.parent / "robot_uc" / "backend" / "models" / "mobilenetv3_peanut.onnx"
    assert onnx_path.is_file(), f"Fichier ONNX manquant : {onnx_path}"
    assert onnx_path.stat().st_size > 1_000_000

    clf = PeanutPlantClassifier()
    assert clf.onnx_loaded is True
    assert "MobileNetV3" in clf.model_backend

    resp = client.get("/api/camera/status")
    assert resp.status_code == 200
    data = resp.json()
    assert data.get("onnx_loaded") is True
    assert data.get("telemetry", {}).get("onnx_loaded") is True


def test_hybrid_binary_accepts_peanut_row():
    """Un rang aligné d'arachide (Arachis hypogaea) est certifié par MobileNetV3 ONNX + OpenCV ExG."""
    frame = _make_sandy_frame(640, 480)
    for idx, y in enumerate([210, 255, 305, 360, 420]):
        _draw_peanut_rosette(frame, 320, y, scale=0.85 + 0.12 * idx)

    pipeline = RealVisionPipeline()
    assert pipeline.peanut_classifier.onnx_loaded is True

    res = pipeline.validate_peanut_row_hybrid(frame, mission_type="WEEDING")
    assert res["opencv_line_aligned"] is True
    assert res["is_peanut"] is True
    assert res["is_valid_peanut_row"] is True
    assert res["peanut_confidence"] >= 0.65
    assert res["detected_species_label"] == SPECIES_PEANUT
    assert res["hud_alert"] is False
    assert "Arachide détectée" in res["hud_badge"]
    assert res["onnx_loaded"] is True

    mod_res = validate_peanut_row_hybrid(frame, mission_type="WEEDING")
    assert mod_res["is_valid_peanut_row"] is True


def test_hybrid_binary_rejects_bare_soil_and_debounces_5_frames():
    """Sur sol nu (fin de ligne), is_peanut=False immédiatement et l'arrêt ferme est confirmé sur 5 frames consécutives."""
    bare_soil = _make_sandy_frame(640, 480)
    # Ajouter un sillon de terre sombre sans végétation verte (fin de ligne en terre travaillée)
    cv2.line(bare_soil, (320, 140), (320, 475), (48, 72, 98), 10)

    pipeline = RealVisionPipeline()
    res = pipeline.validate_peanut_row_hybrid(bare_soil, mission_type="WEEDING")
    assert res["is_peanut"] is False
    assert res["is_valid_peanut_row"] is False
    assert res["detected_species_label"] == SPECIES_BARE_SOIL
    assert res["hud_alert"] is True
    assert "FIN DE LIGNE D'ARACHIDE" in res["hud_badge"]

    # Frames 1 à 4 : pas encore d'arrêt définitif (filtrage anti-faux-positif)
    for i in range(1, 5):
        _, telem = pipeline.process_frame(bare_soil, mission="WEEDING")
        assert telem["is_valid_peanut_row"] is False
        assert telem["consecutive_non_peanut_frames"] == i
        assert telem["hybrid_line_end_confirmed"] is False

    # 5ème frame consécutive de Sol Nu -> arrêt confirmé (DRIVE 0, M6/M8 coupés, dents relevées)
    _, telem5 = pipeline.process_frame(bare_soil, mission="WEEDING")
    assert telem5["consecutive_non_peanut_frames"] == 5
    assert telem5["hybrid_line_end_confirmed"] is True
    assert telem5["hybrid_line_end_detected"] is True
    assert telem5["recommended_drive"]["a"] == 0
    assert "STOP" in telem5["robot_order_str"]

    # Une frame valide d'arachide réinitialise immédiatement le compteur
    peanut_frame = _make_sandy_frame(640, 480)
    for idx, y in enumerate([210, 255, 305, 360, 420]):
        _draw_peanut_rosette(peanut_frame, 320, y, scale=0.85 + 0.12 * idx)
    _, telem_reset = pipeline.process_frame(peanut_frame, mission="WEEDING")
    assert telem_reset["is_valid_peanut_row"] is True
    assert telem_reset["consecutive_non_peanut_frames"] == 0
    assert telem_reset["hybrid_line_end_confirmed"] is False


def test_kinematic_engine_stops_after_5_non_peanut_frames_and_completes_at_last_row():
    """Règle des 5 frames dans KinematicEngine : demi-tour immédiat puis MISSION TERMINÉE — FIN DES LIGNES D'ARACHIDE."""
    engine = KinematicEngine(
        row_spacing=0.46,
        field_length=50.0,
        field_width=20.0,
        start_x=0.0,
        start_z=0.0,
        step_dir=1,
    )
    engine.init_mission()

    for _ in range(25):
        telem = engine.step_simulation(dt=0.05, hybrid_row_valid=True)
    assert engine.state == "ROW_FOLLOWING"
    assert abs(engine.z) < 50.0

    # 4 frames invalides : ne doit PAS encore couper (anti-bruit)
    for _ in range(4):
        telem = engine.step_simulation(dt=0.05, hybrid_row_valid=False)
        assert engine.state == "ROW_FOLLOWING"

    # 5ème frame consécutive Sol Nu -> STOP + M6/M8 coupés + levage M5 + U_TURN_SHIFT
    telem = engine.step_simulation(dt=0.05, hybrid_row_valid=False)
    assert engine.state == "U_TURN_SHIFT", f"Demi-tour attendu à la 5ème frame : {engine.state}"
    assert abs(engine.z) < 50.0

    # Fin du demi-tour puis 5 frames Sol Nu sur la ligne suivante -> arrêt terminal définitif
    final_telem = telem
    for _ in range(500):
        final_telem = engine.step_simulation(dt=0.05, hybrid_row_valid=False)
        if engine.state == "MISSION_COMPLETED_BY_HYBRID_VISION":
            break

    assert engine.state == "MISSION_COMPLETED_BY_HYBRID_VISION"
    assert final_telem["state"] == "MISSION_COMPLETED_BY_HYBRID_VISION"
    assert "MISSION TERMINÉE — FIN DES LIGNES D'ARACHIDE" in engine.completion_message
    assert final_telem["motors"] == [0] * 8
    assert abs(engine.z) < 50.0
    assert abs(engine.x) < 20.0


def test_robot_controller_real_zero_sim_stops_on_5_bare_soil_frames(monkeypatch):
    """Dans RobotController (REAL_ZERO_SIM), 5 frames Sol Nu consécutives stoppent la passe, coupent M6/M8, lèvent M5 et arrêtent si sol nu à côté."""
    from backend.api import server_context
    from backend.vision.camera_manager import camera_manager

    robot = server_context.robot
    prev_mode = robot.execution_mode
    prev_state = robot.state
    sent_cmds = []
    orig_send = robot.send_command

    def spy_send(cmd, *args, **kwargs):
        sent_cmds.append(cmd)
        return orig_send(cmd, *args, **kwargs)

    monkeypatch.setattr(robot, "send_command", spy_send)
    # 3 frames valides puis sol nu continu
    call_counter = {"n": 0}

    def fake_telem():
        call_counter["n"] += 1
        valid = call_counter["n"] <= 2
        return {
            "recommended_drive": {"a": 22, "d": 0},
            "lateral_error_cm": 0.0,
            "heading_error_deg": 0.0,
            "is_valid_peanut_row": valid,
            "hybrid_boundary_alert": not valid,
            "hybrid_line_end_confirmed": call_counter["n"] >= 7,
            "hybrid_hud_badge": "🛑 FIN DE LIGNE D'ARACHIDE : Sol nu détecté -> Arrêt / Demi-tour" if not valid else "🥜 OK",
            "obstacle_stop": False,
        }

    was_cam_connected = camera_manager.connected
    monkeypatch.setattr(camera_manager, "get_latest_telemetry", fake_telem)
    try:
        robot.set_execution_mode("REAL_ZERO_SIM")
        robot.arm()
        ok = robot._follow_corridor(
            corridor_center_x=0.0,
            heading_rad=0.0,
            length_units=50.0,
            label="Test Arrêt 5 Frames",
            pct_start=10,
            pct_end=80,
            weeding=True,
            fallback_steps=15,
        )
        assert ok is True
        # A dû s'arrêter dès la 7ème itération (2 valides + 5 invalides), bien avant fallback_steps=15
        assert call_counter["n"] <= 8
        assert "STOP" in sent_cmds
        assert "STOP:6" in sent_cmds
        assert "STOP:8" in sent_cmds
        assert any(c.startswith("PULSE:5:-50") for c in sent_cmds)

        # Après demi-tour, la ligne adjacente est également Sol Nu sur 5 frames -> fin définitive
        has_next_peanut = robot._verify_peanut_row_present_after_turn(required_invalid_frames=5)
        assert has_next_peanut is False
        assert robot.state == "MISSION_COMPLETED_BY_HYBRID_VISION"
    finally:
        if not was_cam_connected and camera_manager.connected:
            camera_manager.disconnect()
        robot.state = prev_state
        robot.set_execution_mode(prev_mode)
