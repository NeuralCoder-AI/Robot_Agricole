"""tests/test_hybrid_opencv_peanut_model.py
Tests unitaires de la Fusion OpenCV + Modèle Pré-entraîné Arachis hypogaea :
1. Validation positive : rang d'arachide aligné (folioles ovales -> is_valid_peanut_row=True + badge vert).
2. Rejet frontière : culture étrangère rectiligne (mil/sorgho/graminées -> opencv_line_aligned=True MAIS is_peanut=False -> is_valid_peanut_row=False + alerte rouge).
3. Arrêt prioritaire fin de parcelle dans KinematicEngine : demi-tour immédiat avant field_length=50m puis arrêt terminal MISSION_COMPLETED_BY_HYBRID_VISION avant field_width=20m.
"""

import cv2
import numpy as np

from backend.navigation.kinematic_engine import KinematicEngine
from backend.vision.real_vision_pipeline import RealVisionPipeline, validate_peanut_row_hybrid


def _make_sandy_frame(width: int = 640, height: int = 480) -> np.ndarray:
    """Crée une image synthétique de sol sableux agricole sans artefacts."""
    frame = np.full((height, width, 3), (115, 165, 202), dtype=np.uint8)
    # Légère texture sableuse naturelle
    rng = np.random.default_rng(42)
    noise = rng.integers(-6, 7, size=(height, width, 3), dtype=np.int16)
    return np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)


def _draw_peanut_rosette(frame: np.ndarray, cx: int, cy: int, scale: float = 1.0) -> None:
    """Dessine un plant d'arachide synthétique à 4 folioles ovales paripennées."""
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
    # Tige centrale discrète
    cv2.circle(frame, (cx, cy), max(2, int(3 * scale)), (38, 145, 42), -1)


def _draw_millet_tuft(frame: np.ndarray, cx: int, cy: int, scale: float = 1.0) -> None:
    """Dessine une touffe étroite et élancée de mil / graminée linéaire."""
    h_blade = max(24, int(44 * scale))
    for dx_top in (-14, -6, 0, 7, 15):
        cv2.line(
            frame,
            (cx, cy + h_blade // 3),
            (cx + int(dx_top * scale), cy - h_blade // 2),
            (54, 166, 56),
            2,
        )


def test_hybrid_accepts_aligned_peanut_foliage():
    """CAS 1 : Un rang aligné de plants d'arachide (folioles ovales) est certifié par OpenCV + IA botanique."""
    frame = _make_sandy_frame(640, 480)
    # Rang d'arachide rectiligne centré dans la moitié basse/médiane de l'image
    for idx, y in enumerate([210, 255, 305, 360, 420]):
        scale = 0.85 + 0.12 * idx
        _draw_peanut_rosette(frame, 320, y, scale=scale)

    pipeline = RealVisionPipeline()
    res = pipeline.validate_peanut_row_hybrid(frame, mission_type="WEEDING")

    assert res["opencv_line_aligned"] is True, f"Ligne OpenCV non détectée : {res}"
    assert res["is_peanut"] is True, f"Arachis hypogaea rejeté à tort : {res}"
    assert res["is_valid_peanut_row"] is True
    assert res["peanut_confidence"] >= 0.65
    assert "Arachis hypogaea" in res["detected_species_label"]
    assert res["hud_alert"] is False
    assert "Arachis hypogaea confirmé" in res["hud_badge"]
    assert "Rang aligné" in res["hud_badge"]

    # Vérification via l'entrée module-level + process_frame complet
    mod_res = validate_peanut_row_hybrid(frame, mission_type="WEEDING")
    assert mod_res["is_valid_peanut_row"] is True

    _, telem = pipeline.process_frame(frame, mission="WEEDING")
    assert telem["is_valid_peanut_row"] is True
    assert telem["hybrid_boundary_alert"] is False


def test_hybrid_rejects_aligned_foreign_crop_row():
    """CAS 2 : Une rangée rectiligne de culture étrangère (mil/graminées) est rejetée malgré l'alignement géométrique OpenCV."""
    frame = _make_sandy_frame(640, 480)
    # Touffes élancées de mil formant une ligne géométriquement bien droite
    for idx, y in enumerate([210, 260, 312, 368, 425]):
        scale = 0.85 + 0.12 * idx
        _draw_millet_tuft(frame, 320, y, scale=scale)

    pipeline = RealVisionPipeline()
    res = pipeline.validate_peanut_row_hybrid(frame, mission_type="WEEDING")

    assert res["opencv_line_aligned"] is True, f"La géométrie alignée aurait dû être vue par OpenCV : {res}"
    assert res["is_peanut"] is False, f"Culture étrangère classée à tort comme arachide : {res}"
    assert res["is_valid_peanut_row"] is False
    assert res["peanut_confidence"] < 0.65
    assert res["hud_alert"] is True
    assert "ALERTE FRONTIÈRE" in res["hud_badge"]

    _, telem = pipeline.process_frame(frame, mission="WEEDING")
    assert telem["is_valid_peanut_row"] is False
    assert telem["hybrid_boundary_alert"] is True
    assert telem.get("hybrid_line_end_detected") is True


def test_hybrid_stops_mission_at_last_peanut_row():
    """CAS 3 : Même avec de grandes dimensions saisies (50m × 20m), le rejet hybride force le demi-tour puis l'arrêt terminal."""
    engine = KinematicEngine(
        row_spacing=0.46,
        field_length=50.0,
        field_width=20.0,
        start_x=0.0,
        start_z=0.0,
        step_dir=1,
    )
    engine.init_mission()

    # 1. Avance normale sur le rang d'arachide (validation hybride positive)
    for _ in range(25):
        telem = engine.step_simulation(dt=0.05, hybrid_row_valid=True)
    assert engine.state == "ROW_FOLLOWING"
    assert abs(engine.z) < 50.0

    # 2. En fin de rang : la vision hybride détecte une sortie d'arachide -> demi-tour immédiat avant 50 m
    telem = engine.step_simulation(dt=0.05, hybrid_row_valid=False)
    assert engine.state != "ROW_FOLLOWING", f"Le demi-tour immédiat aurait dû démarrer : state={engine.state}"
    assert abs(engine.z) < 50.0

    # 3. Exécution du demi-tour jusqu'à l'inspection du rang suivant (qui échoue aussi la validation hybride)
    final_telem = telem
    for _ in range(500):
        # Une fois le demi-tour achevé, le contrôleur inspecte la nouvelle ligne (hybrid_row_valid=False)
        final_telem = engine.step_simulation(dt=0.05, hybrid_row_valid=False)
        if engine.state == "MISSION_COMPLETED_BY_HYBRID_VISION":
            break

    assert engine.state == "MISSION_COMPLETED_BY_HYBRID_VISION", f"État final inattendu : {engine.state}"
    assert final_telem["state"] == "MISSION_COMPLETED_BY_HYBRID_VISION"
    assert final_telem["motors"] == [0] * 8
    assert abs(engine.z) < 50.0
    assert abs(engine.x) < 20.0
