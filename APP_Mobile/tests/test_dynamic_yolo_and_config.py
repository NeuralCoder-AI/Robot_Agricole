"""tests/test_dynamic_yolo_and_config.py
Tests unitaires et d'intégration validant les 5 nouvelles briques d'architecture avancée :
1. Stockage persistant dynamique des mesures du robot (`backend/config/robot_config.py`, routes GET/PUT `/api/config/specs`)
   et recalculation automatique des Waypoints trapézoïdaux (WP0..WP3) et du couloir de collision.
2. Auto-calibration de l'angle d'inclinaison caméra (theta_pitch) par Point de Fuite (y_vp)
   et calcul trigonométrique exact de la distance au sol (D_sol, X_sol) par `y_bottom`.
3. État d'attente devant un obstacle mobile (`WAITING_MOVING_OBSTACLE`) :
   - Reprise directe de la ligne (`WORKING` / `ROW_FOLLOWING`) si l'obstacle quitte le couloir avant la fin du délai.
   - Contournement trapézoïdal (`BYPASS_REQUIRED` / `APPROACH_AVOIDANCE`) si le délai expire ou si l'obstacle est statique.
4. Détection automatique de fin de ligne (`LINE_END_DETECTED` après > 0.50 m d'avancement sans ligne)
   et de fin de parcelle (`FIELD_END_DETECTED`).
5. Présence de la modale dynamique, du verrouillage de mission UI et du sélecteur Fin de Ligne Intelligente.
"""

import math
from pathlib import Path
import cv2
from fastapi.testclient import TestClient

from backend.config.robot_config import RobotConfigManager, robot_config
from backend.navigation.kinematic_engine import KinematicEngine
from backend.vision.yolo_pipeline import YoloVisionPipeline


def test_dynamic_robot_config_persistence_and_waypoint_recalculation(tmp_path: Path, client: TestClient):
    """1. Vérifie la persistance SQLite/JSON, les formules dérivées (tool_spacing/2, tool_spacing*2)
    et la recalculation immédiate des Waypoints (WP0..WP3) et du couloir de collision.
    """
    sqlite_file = tmp_path / "test_specs.sqlite"
    json_file = tmp_path / "test_specs.json"
    cfg = RobotConfigManager(sqlite_path=sqlite_file, json_path=json_file)

    # Valeurs par défaut
    s0 = cfg.get_specs()
    assert s0["chassis_width_m"] == 1.21
    assert s0["chassis_half_width_m"] == 0.605
    assert s0["tool_spacing_m"] == 0.46
    assert s0["tool_half_offset_m"] == 0.23
    assert s0["central_guide_spacing_m"] == 0.92
    assert s0["moving_obstacle_wait_sec"] == 5.0

    engine = KinematicEngine(config_manager=cfg)
    engine.init_mission()

    obs = {"id": "rock_1", "x": 0.0, "z": 5.0, "width_m": 0.50, "length_m": 0.50}
    wp_before = engine.compute_trapezoid_waypoints(obs, side_x=1)
    chk_before = engine.check_obstacle_collision({"x": 0.90, "z": 1.5, "width_m": 0.40, "length_m": 0.40})

    # Mise à jour dynamique des dimensions mécaniques
    updated = cfg.update_specs(
        {
            "chassis_width_m": 1.50,
            "nose_offset_m": 1.00,
            "rear_teeth_offset_m": 1.30,
            "tool_spacing_m": 0.60,
            "lateral_safety_margin_m": 0.45,
            "teeth_rear_clearance_m": 0.35,
            "moving_obstacle_wait_sec": 4.0,
        }
    )

    assert updated["chassis_width_m"] == 1.50
    assert updated["chassis_half_width_m"] == 0.75
    assert updated["tool_spacing_m"] == 0.60
    assert updated["tool_half_offset_m"] == 0.30
    assert updated["central_guide_spacing_m"] == 1.20
    assert updated["moving_obstacle_wait_sec"] == 4.0

    # Vérifier la persistance en rechargeant une nouvelle instance depuis SQLite
    cfg_reloaded = RobotConfigManager(sqlite_path=sqlite_file, json_path=json_file)
    assert cfg_reloaded.chassis_width_m == 1.50
    assert cfg_reloaded.tool_half_offset_m == 0.30
    assert cfg_reloaded.central_guide_spacing_m == 1.20

    # Vérifier que KinematicEngine a recalculé automatiquement ses Waypoints et son couloir
    wp_after = engine.compute_trapezoid_waypoints(obs, side_x=1)
    assert wp_after["bypass_x"] > wp_before["bypass_x"]
    # bypass_x = 0.0 + 1 * (0.25 + 0.75 + 0.45) = 1.45 m
    assert math.isclose(wp_after["bypass_x"], 1.45, abs_tol=1e-3)
    # WP1.z = front_z (4.75) - 1.00 = 3.75 m
    assert math.isclose(wp_after["WP1"]["z"], 4.75 - 1.00, abs_tol=1e-3)
    # WP2.z = rear_z (5.25) + (1.30 + 0.35) = 6.90 m
    assert math.isclose(wp_after["WP2"]["z"], 5.25 + 1.65, abs_tol=1e-3)

    chk_after = engine.check_obstacle_collision({"x": 0.90, "z": 1.5, "width_m": 0.40, "length_m": 0.40})
    assert chk_after["collision_half_width"] > chk_before["collision_half_width"]

    # Test des routes FastAPI GET /api/config/specs et PUT /api/config/specs
    res_get = client.get("/api/config/specs")
    assert res_get.status_code == 200
    assert "chassis_width_m" in res_get.json()

    res_put = client.put("/api/config/specs", json={"tool_spacing_m": 0.50, "moving_obstacle_wait_sec": 6.0})
    assert res_put.status_code == 200
    data_put = res_put.json()
    assert data_put["tool_spacing_m"] == 0.50
    assert data_put["tool_half_offset_m"] == 0.25
    assert data_put["central_guide_spacing_m"] == 1.00
    assert data_put["moving_obstacle_wait_sec"] == 6.0

    # Remettre les valeurs par défaut pour les autres tests
    client.post("/api/config/specs/reset")
    robot_config.reset_specs()


def test_trigonometric_ground_distance_y_bottom_and_auto_pitch(tmp_path: Path):
    """2. Vérifie l'auto-calcul de theta_pitch par Point de Fuite (y_vp)
    et la formule trigonométrique exacte de distance au sol (D_sol, X_sol) par `y_bottom`.
    """
    cfg = RobotConfigManager(sqlite_path=tmp_path / "cam.sqlite", json_path=tmp_path / "cam.json")
    cfg.update_specs(
        {
            "camera_height_m": 0.90,
            "camera_fov_deg": 78.0,
            "camera_pitch_deg": 32.0,
            "auto_pitch": True,
        }
    )
    yolo = YoloVisionPipeline(config_manager=cfg)

    img_w, img_h = 640, 480
    cx, cy = img_w / 2.0, img_h / 2.0
    fx, fy = yolo.compute_focal_lengths(img_width=img_w, img_height=img_h, fov_v_deg=78.0, fov_h_deg=78.0)

    expected_fy = (480.0 / 2.0) / math.tan(math.radians(39.0))
    assert math.isclose(fy, expected_fy, rel_tol=1e-4)

    # Test 1 : Auto-calibration de theta_pitch pour un point de fuite produisant exactement 32.0°
    y_vp_for_32deg = cy - fy * math.tan(math.radians(32.0))
    pitch_deg = yolo.calibrate_pitch_from_vanishing_point(y_vp=y_vp_for_32deg, img_height=img_h, fov_v_deg=78.0)
    assert math.isclose(pitch_deg, 32.0, abs_tol=0.05)

    # Test 2 : Distance au sol au centre optique (y_bottom == c_y) -> D_sol = camera_height_m / tan(theta_pitch)
    proj_center = yolo.compute_ground_distance_and_lateral(
        y_bottom=cy,
        x_center=cx,
        img_width=img_w,
        img_height=img_h,
        camera_height_m=0.90,
        pitch_deg=32.0,
        fov_v_deg=78.0,
        fov_h_deg=78.0,
    )
    expected_d_center = 0.90 / math.tan(math.radians(32.0))
    assert math.isclose(proj_center["d_sol_m"], expected_d_center, abs_tol=1e-3)
    assert math.isclose(proj_center["x_sol_m"], 0.0, abs_tol=1e-4)
    assert "📐 Angle Caméra Auto : 32.0°" in proj_center["hud_pitch_label"]

    # Test 3 : Plus y_bottom descend dans l'image (pied de l'obstacle proche du châssis), plus D_sol diminue
    proj_near = yolo.compute_ground_distance_and_lateral(
        y_bottom=420.0,
        x_center=cx + 80.0,
        img_width=img_w,
        img_height=img_h,
        camera_height_m=0.90,
        pitch_deg=32.0,
        fov_v_deg=78.0,
        fov_h_deg=78.0,
    )
    expected_alpha = math.atan((420.0 - cy) / fy)
    expected_d_near = 0.90 / math.tan(math.radians(32.0) + expected_alpha)
    expected_x_near = expected_d_near * (80.0 / fx)
    assert math.isclose(proj_near["d_sol_m"], expected_d_near, abs_tol=1e-3)
    assert math.isclose(proj_near["x_sol_m"], expected_x_near, abs_tol=1e-3)
    assert proj_near["d_sol_m"] < proj_center["d_sol_m"]


def test_waiting_moving_obstacle_resumes_or_bypasses(tmp_path: Path):
    """3. Vérifie l'état `WAITING_MOVING_OBSTACLE` :
    - Cas A : Un obstacle mobile (personne / animal) entre dans le couloir -> arrêt immédiat + attente.
              S'il quitte le couloir avant `moving_obstacle_wait_sec`, le robot reprend sa ligne (`WORKING` / `ROW_FOLLOWING`) sans contourner.
    - Cas B : Si le délai `moving_obstacle_wait_sec` expire et qu'il est toujours là, le contournement trapézoïdal est déclenché.
    """
    cfg = RobotConfigManager(sqlite_path=tmp_path / "wait.sqlite", json_path=tmp_path / "wait.json")
    cfg.update_specs({"moving_obstacle_wait_sec": 3.0})
    yolo = YoloVisionPipeline(config_manager=cfg)

    # --- CAS A : Obstacle mobile dans le couloir qui s'en va avant 3.0 s ---
    moving_person = [
        {
            "track_id": 7,
            "class_name": "person",
            "bbox": [280, 220, 360, 410],
            "cx": 320.0,
            "cy": 315.0,
            "y_bottom": 410.0,
            "dist_m": 0.85,
            "x_m": 0.0,
            "is_moving": True,
        }
    ]
    res1 = yolo.evaluate_obstacle_behavior(moving_person, dt=0.5, now_ts=100.0)
    assert res1["obstacle_state"] == "WAITING_MOVING_OBSTACLE"
    assert res1["stop_robot"] is True
    assert res1["cut_m6"] is True
    assert res1["bypass_triggered"] is False
    assert "⏳ OBSTACLE EN MOUVEMENT — ATTENTE" in res1["hud_banner"]

    # 1.2 s plus tard, toujours en attente (< 3.0 s)
    res2 = yolo.evaluate_obstacle_behavior(moving_person, dt=1.2, now_ts=101.2)
    assert res2["obstacle_state"] == "WAITING_MOVING_OBSTACLE"
    assert res2["bypass_triggered"] is False

    # À t = 102.0 s (< 3.0 s), la personne quitte le couloir !
    res_clear = yolo.evaluate_obstacle_behavior([], dt=0.8, now_ts=102.0)
    assert res_clear["state"] == "WORKING"
    assert res_clear["obstacle_state"] == "CLEAR"
    assert res_clear["bypass_triggered"] is False
    assert res_clear["resumed_after_moving_obstacle"] is True

    # --- CAS B : Obstacle mobile qui reste bloqué au-delà du délai de 3.0 s ---
    yolo.reset_obstacle_state()
    r_start = yolo.evaluate_obstacle_behavior(moving_person, dt=0.1, now_ts=200.0)
    assert r_start["obstacle_state"] == "WAITING_MOVING_OBSTACLE"

    r_timeout = yolo.evaluate_obstacle_behavior(moving_person, dt=3.2, now_ts=203.2)
    assert r_timeout["obstacle_state"] == "BYPASS_REQUIRED"
    assert r_timeout["bypass_triggered"] is True
    assert r_timeout["reason"] == "MOVING_OBSTACLE_TIMEOUT_EXCEEDED"

    # --- Vérification identique dans KinematicEngine ---
    engine = KinematicEngine(config_manager=cfg)
    engine.init_mission()
    obs_mobile = {
        "id": "cow_1",
        "class_name": "cow",
        "is_moving": True,
        "x": 0.0,
        "z": engine.z + engine.ROBOT_NOSE_EXTENT + 0.70,
        "width_m": 0.45,
        "length_m": 0.45,
    }
    telem1 = engine.step_simulation(dt=0.5, obstacles=[obs_mobile])
    assert telem1["state"] == "WAITING_MOVING_OBSTACLE"
    assert telem1["motors"][5] == 0  # M6 coupé

    # L'animal part à t = 1.0 s -> reprise directe sur la ligne sans contournement
    telem_resume = engine.step_simulation(dt=0.5, obstacles=[])
    assert telem_resume["state"] == "ROW_FOLLOWING"
    assert telem_resume["resumed_after_moving_obstacle"] is True


def test_smart_line_end_and_field_end_detection(tmp_path: Path):
    """4. Vérifie la détection automatique de fin de ligne (`LINE_END_DETECTED` après > 0.50 m sans ligne)
    et la fin automatique de parcelle (`FIELD_END_DETECTED` / `MISSION_COMPLETE`) lorsqu'aucune nouvelle ligne n'existe.
    """
    cfg = RobotConfigManager(sqlite_path=tmp_path / "line.sqlite", json_path=tmp_path / "line.json")
    cfg.update_specs({"line_end_confirm_dist_m": 0.50, "tool_spacing_m": 0.46})
    yolo = YoloVisionPipeline(config_manager=cfg)

    # Tant que la ligne est détectée : TRACKING_LINE
    ev0 = yolo.evaluate_smart_line_continuity(line_detected_in_bottom=True, step_distance_m=0.30)
    assert ev0["line_state"] == "TRACKING_LINE"
    assert ev0["line_end_detected"] is False

    # La ligne disparaît sur 0.30 m (< 0.50 m) : pas encore déclenché
    ev1 = yolo.evaluate_smart_line_continuity(line_detected_in_bottom=False, step_distance_m=0.30)
    assert ev1["line_end_detected"] is False
    assert math.isclose(ev1["line_missing_distance_m"], 0.30, abs_tol=1e-3)

    # +0.25 m supplémentaires (total = 0.55 m >= 0.50 m) -> LINE_END_DETECTED !
    ev2 = yolo.evaluate_smart_line_continuity(line_detected_in_bottom=False, step_distance_m=0.25)
    assert ev2["line_state"] == "LINE_END_DETECTED"
    assert ev2["line_end_detected"] is True
    assert ev2["action"] == "TRIGGER_UTURN"
    assert math.isclose(ev2["shift_m"], 0.92, abs_tol=1e-3)

    # Après le demi-tour latéral, aucune nouvelle ligne n'est présente -> FIELD_END_DETECTED !
    ev_end = yolo.evaluate_smart_line_continuity(
        line_detected_in_bottom=False,
        step_distance_m=0.0,
        after_uturn_shift=True,
    )
    assert ev_end["line_state"] == "FIELD_END_DETECTED"
    assert ev_end["field_end_detected"] is True
    assert "Fin de parcelle détectée automatiquement" in ev_end["message"]

    # Test dans KinematicEngine en mode SMART_LINE_END
    engine = KinematicEngine(config_manager=cfg, boundary_mode="SMART_LINE_END")
    engine.init_mission(boundary_mode="SMART_LINE_END")
    # Avance sans ligne pendant 1.0 s à 0.65 m/s (= 0.65 m >= 0.50 m)
    engine.step_simulation(dt=0.5, line_detected=False)
    t_uturn = engine.step_simulation(dt=0.5, line_detected=False)
    assert t_uturn["line_end_detected"] is True
    assert t_uturn["state"] == "U_TURN_SHIFT"

    # Finir la transition latérale puis constater qu'aucune nouvelle ligne n'existe
    for _ in range(30):
        if engine.state != "U_TURN_SHIFT":
            break
        engine.step_simulation(dt=0.1, line_detected=False)

    assert engine.state == "MISSION_COMPLETE"
    assert engine.field_end_detected is True
    assert "Fin de parcelle détectée automatiquement" in engine.completion_message


def test_yolov8n_onnx_loaded_detects_person_and_exposes_status(tmp_path: Path, client: TestClient):
    """6. Vérifie le chargement natif `yolov8n.onnx` via `cv2.dnn` (`yolo_onnx_loaded == True`),
    la détection réelle d'une personne/obstacle avec NMS + transition `WAITING_MOVING_OBSTACLE`,
    le repli propre si le fichier ONNX est absent, et les champs `/api/camera/status`.
    """
    onnx_path = Path(__file__).resolve().parent.parent / "backend" / "models" / "yolov8n.onnx"
    assert onnx_path.is_file(), f"Modèle yolov8n.onnx introuvable : {onnx_path}"
    assert onnx_path.stat().st_size > 10_000_000

    # Repli propre si le modèle est absent
    fallback_yolo = YoloVisionPipeline(onnx_model_path=str(tmp_path / "missing_yolo.onnx"))
    assert fallback_yolo.yolo_onnx_loaded is False
    assert fallback_yolo.detect_objects_onnx(cv2.imread(str(Path(__file__).resolve().parent / "assets" / "yolo_test_person_bus.jpg"))) == []

    # Chargement effectif du modèle réel yolov8n.onnx (~12.2 Mo)
    yolo = YoloVisionPipeline()
    assert yolo.yolo_onnx_loaded is True
    assert yolo.yolo_model_name == "yolov8n.onnx"

    sample_path = Path(__file__).resolve().parent / "assets" / "yolo_test_person_bus.jpg"
    assert sample_path.is_file(), f"Image de test manquante : {sample_path}"
    frame = cv2.imread(str(sample_path))
    assert frame is not None and frame.size > 0

    dets = yolo.detect_objects_onnx(frame, conf_thresh=0.40, nms_thresh=0.45)
    assert len(dets) >= 1
    person_dets = [d for d in dets if d["class_name"] == "person"]
    assert len(person_dets) >= 1
    assert person_dets[0]["conf"] >= 0.40
    assert person_dets[0]["is_mobile_class"] is True
    assert person_dets[0]["source"] == "yolov8n.onnx"

    # Vérification de l'intégration complète process_frame() -> WAITING_MOVING_OBSTACLE & HUD
    annotated, telem = yolo.process_frame(frame, mission="WEEDING")
    assert annotated.shape == frame.shape
    assert telem.get("yolo_onnx_loaded") is True
    assert telem.get("yolo_model_name") == "yolov8n.onnx"
    assert telem.get("obstacle_state") == "WAITING_MOVING_OBSTACLE"
    assert telem.get("waiting_moving_obstacle") is True
    assert "MOBILE" in str(telem.get("hud_obstacle_summary", ""))

    # Vérification REST `/api/camera/status` et `/api/system/info`
    cam_resp = client.get("/api/camera/status")
    assert cam_resp.status_code == 200
    cam_json = cam_resp.json()
    assert cam_json.get("yolo_onnx_loaded") is True
    assert cam_json.get("yolo_model_name") == "yolov8n.onnx"
    assert cam_json.get("onnx_loaded") is True

    sys_resp = client.get("/api/system/info")
    assert sys_resp.status_code == 200
    assert sys_resp.json().get("yolo_onnx_loaded") is True


