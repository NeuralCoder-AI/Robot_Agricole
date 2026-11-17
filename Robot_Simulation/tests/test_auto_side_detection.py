"""tests/test_auto_side_detection.py
Validation unitaire de l'Auto-Détection Intelligente du Sens de Progression (Gauche / Droite / Analyse de l'État du Sol)
verrouillée dès la 1ère ligne :
1. `test_auto_side_only_right_line_exists` -> verrouille `RIGHT` (-1) pour tous les tours.
2. `test_auto_side_only_left_line_exists` -> verrouille `LEFT` (+1) pour tous les tours.
3. `test_auto_side_both_lines_left_already_worked` -> détecte le sol traité à gauche et verrouille `RIGHT` (côté vierge).
4. `test_auto_side_both_lines_untouched_autonomous_fallback` -> choisit automatiquement un côté et garde strictement
   le même sens aux tours 1, 2, 3 et 4.
"""

import math
import numpy as np

from backend.navigation.kinematic_engine import KinematicEngine
from backend.vision.yolo_pipeline import YoloVisionPipeline


def test_auto_side_only_right_line_exists():
    """CAS 1A : Ligne présente uniquement à DROITE (bord gauche de la parcelle) -> verrouille `RIGHT` (-1) pour tous les tours."""
    yolo = YoloVisionPipeline()
    yolo.reset_side_detection_lock()

    decision = yolo.detect_next_row_direction(left_line_exists=False, right_line_exists=True)
    assert decision["direction"] == "RIGHT"
    assert decision["side_sign"] == -1
    assert decision["locked"] is True
    assert yolo.locked_field_side_sign == -1

    # Même si au tour suivant une ligne apparaît à gauche (ligne qu'on vient de quitter), le verrouillage reste RIGHT (-1) !
    decision_pass_2 = yolo.detect_next_row_direction(left_line_exists=True, right_line_exists=True)
    assert decision_pass_2["direction"] == "RIGHT"
    assert decision_pass_2["side_sign"] == -1

    # Vérification dans KinematicEngine sur plusieurs tours
    engine = KinematicEngine(row_spacing=0.46, field_length=10.0, field_width=6.0, start_x=0.0, start_z=0.0, step_dir="AUTO")
    engine.init_mission(step_dir="AUTO")
    engine.left_line_exists = False
    engine.right_line_exists = True

    visited_x = [engine.target_row_x]
    for _ in range(3):
        assert engine.advance_row_cursor() is True
        visited_x.append(engine.target_row_x)

    assert engine.locked_field_side_sign == -1
    assert visited_x == [0.0, -0.92, -1.84, -2.76]


def test_auto_side_only_left_line_exists():
    """CAS 1B : Ligne présente uniquement à GAUCHE (bord droit de la parcelle) -> verrouille `LEFT` (+1) pour tous les tours."""
    yolo = YoloVisionPipeline()
    yolo.reset_side_detection_lock()

    decision = yolo.detect_next_row_direction(left_line_exists=True, right_line_exists=False)
    assert decision["direction"] == "LEFT"
    assert decision["side_sign"] == 1
    assert decision["locked"] is True
    assert yolo.locked_field_side_sign == 1

    # Vérification que le verrouillage persiste aux tours suivants
    decision_pass_2 = yolo.detect_next_row_direction(left_line_exists=True, right_line_exists=True)
    assert decision_pass_2["direction"] == "LEFT"
    assert decision_pass_2["side_sign"] == 1

    # Vérification dans KinematicEngine sur plusieurs tours
    engine = KinematicEngine(row_spacing=0.46, field_length=10.0, field_width=6.0, start_x=0.0, start_z=0.0, step_dir="AUTO")
    engine.init_mission(step_dir="AUTO")
    engine.left_line_exists = True
    engine.right_line_exists = False

    visited_x = [engine.target_row_x]
    for _ in range(3):
        assert engine.advance_row_cursor() is True
        visited_x.append(engine.target_row_x)

    assert engine.locked_field_side_sign == 1
    assert visited_x == [0.0, 0.92, 1.84, 2.76]


def test_auto_side_both_lines_left_already_worked():
    """CAS 2 : Lignes présentes des DEUX côtés, mais le sol à GAUCHE est déjà traité
    (empreintes de roues cages + 0 adventice vs sol vierge avec adventices à droite) -> verrouille `RIGHT` (-1).
    """
    yolo = YoloVisionPipeline()
    yolo.reset_side_detection_lock()

    # Simulation d'images ROI OpenCV :
    # - ROI gauche : sol travaillé (stries hautes fréquences de roues cages, 0 adventice verte)
    left_img = np.full((120, 160, 3), (85, 145, 195), dtype=np.uint8)
    for y in range(0, 120, 6):
        left_img[y : y + 2, :, :] = (30, 55, 80)  # Stries d'empreintes de roues cages et dents

    # - ROI droite : sable lisse vierge avec adventices vertes non désherbées
    right_img = np.full((120, 160, 3), (95, 170, 225), dtype=np.uint8)
    right_img[30:70, 40:100] = (30, 180, 40)  # Adventices vertes dans l'inter-rang

    soil_state = yolo.analyze_soil_treatment_state(left_roi=left_img, right_roi=right_img, mission="WEEDING")
    assert soil_state["left_worked"] is True
    assert soil_state["right_worked"] is False
    assert soil_state["recommended_side"] == "RIGHT"
    assert soil_state["recommended_sign"] == -1
    assert "Côté Gauche déjà traité -> Sens DROITE verrouillé" in soil_state["hud_message"]

    decision = yolo.detect_next_row_direction(
        left_line_exists=True,
        right_line_exists=True,
        left_roi=left_img,
        right_roi=right_img,
        mission="WEEDING",
    )
    assert decision["direction"] == "RIGHT"
    assert decision["side_sign"] == -1
    assert decision["locked"] is True

    # Vérification également dans KinematicEngine
    engine = KinematicEngine(row_spacing=0.46, field_length=10.0, field_width=6.0, start_x=0.0, start_z=0.0, step_dir="AUTO")
    engine.init_mission(step_dir="AUTO")
    engine.left_line_exists = True
    engine.right_line_exists = True
    engine.left_already_worked = True
    engine.right_already_worked = False

    assert engine.advance_row_cursor() is True
    assert engine.locked_field_side_sign == -1
    assert engine.target_row_x == -0.92


def test_auto_side_both_lines_untouched_autonomous_fallback():
    """CAS 3 : Les deux côtés sont 100 % vierges (non traités) -> choisit automatiquement un côté (`RIGHT` / -1)
    dès la 1ère ligne et garde strictement le même sens aux tours 1, 2, 3 et 4 sans jamais revenir en arrière.
    """
    yolo = YoloVisionPipeline()
    yolo.reset_side_detection_lock()

    decision = yolo.detect_next_row_direction(
        left_line_exists=True,
        right_line_exists=True,
        left_roi={"worked": False, "weeds_ratio": 0.12, "texture_var": 35.0},
        right_roi={"worked": False, "weeds_ratio": 0.12, "texture_var": 35.0},
    )
    assert decision["direction"] == "RIGHT"
    assert decision["side_sign"] == -1
    assert decision["locked"] is True
    assert "Choix autonome : Vers la DROITE verrouillé pour tous les tours" in decision["hud_message"]

    # Vérification dans KinematicEngine sur les tours 1, 2, 3 et 4
    engine = KinematicEngine(row_spacing=0.46, field_length=10.0, field_width=8.0, start_x=0.0, start_z=0.0, step_dir="AUTO")
    engine.init_mission(step_dir="AUTO")
    assert engine.locked_field_side_sign is None

    visited_rows = [engine.target_row_x]
    locked_signs = []

    for pass_num in range(1, 5):
        has_next = engine.advance_row_cursor()
        assert has_next is True, f"Échec de progression au tour {pass_num}"
        assert engine.locked_field_side_sign is not None
        locked_signs.append(engine.locked_field_side_sign)
        visited_rows.append(engine.target_row_x)

    # Le sens verrouillé doit rester strictement identique aux tours 1, 2, 3 et 4
    assert len(set(locked_signs)) == 1
    assert locked_signs == [-1, -1, -1, -1]

    # Les abscisses X progressent de manière strictement monotone (-0.92 m à chaque tour) sans jamais revenir en arrière
    assert visited_rows == [0.0, -0.92, -1.84, -2.76, -3.68]
    for i in range(len(visited_rows) - 1):
        step_delta = visited_rows[i + 1] - visited_rows[i]
        assert math.isclose(step_delta, -0.92, abs_tol=1e-3)
