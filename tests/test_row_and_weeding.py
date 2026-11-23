"""tests/test_row_and_weeding.py
Validation de la spécialisation agronomique Arachide :
- Détection vision, labels des rangs et couloir de désherbage M5/M6.
- Verrouillage visuel (|Δθ| < 2.0°, |Δx| < 2.5 cm) avant descente des dents M5.
- Règle de sélection des rangs : espacement <= 50 cm -> saut d'un rang (+2), > 50 cm -> rang suivant (+1).
"""

from backend.navigation.robot_controller import RobotController
from backend.vision.ai_vision_brain import AIVisionBrain
from tests.conftest import ws_receive_type


def test_peanut_specialization_vision_and_corridors():
    """Vérifie la spécialisation Arachide (vision inter-rang 46 cm, couloir M5/M6, adventices)."""
    brain = AIVisionBrain(width=320, height=200)
    res = brain.analyze_scene(
        robot_x=0.05,
        robot_z=2.0,
        robot_yaw=0.04,
        weeds_in_fov=[{"x": 0.02, "z": 3.8}],
        obstacles_in_fov=[],
    )
    assert res["crop_type"] == "ARACHIDE"
    assert res["left_row_label"] == "RANG ARACHIDE"
    assert res["right_row_label"] == "RANG ARACHIDE"
    assert res["corridor_label"] == "COULOIR DÉSHERBAGE DENTS/FRAISE"
    assert res["wheel_contact_ignored"] is True
    assert any("ADVENTICE INTER-RANG" in b["label"] for b in res["weed_boxes"])
    assert "M6:70" in res["recommended_tools"]
    assert "PULSE:5:50:4000" in res["recommended_tools"]


def test_seed_config_and_agronomic_parameters(client):
    """Vérifie la transmission des paramètres de semis et dimensions du champ via WebSocket."""
    with client.websocket_connect("/ws") as ws:
        ws_receive_type(ws, "state")
        ws.send_json({
            "type": "seed_config",
            "spacing_cm": 20,
            "use_tines": True,
            "field_length_m": 15.0,
            "field_width_m": 4.6,
        })
        agro_msg = ws_receive_type(ws, "agronomics")
        assert agro_msg["agronomics"]["seed_spacing_cm"] == 20
        assert agro_msg["agronomics"]["field_length_m"] == 15.0
        assert agro_msg["agronomics"]["field_width_m"] == 4.6


def test_row_skip_agronomic_rule():
    """Vérifie la sélection de ligne : écart <= 50 cm -> saut de 1 rang (i+2), > 50 cm -> rang suivant (i+1)."""
    ctrl = RobotController()

    # Cas 1 : Espacement 46 cm (standard arachide, <= 50 cm) -> Saut d'une ligne (+2 rangs)
    rows_46cm = [round(i * 0.46, 3) for i in range(-4, 5)]
    idx_0 = rows_46cm.index(0.0)
    sel_46 = ctrl.select_next_peanut_row(current_x=0.0, crop_rows_x=rows_46cm, direction_sign=1)
    assert abs(sel_46["spacing_m"] - 0.46) < 1e-4
    assert sel_46["spacing_cm"] == 46
    assert sel_46["skip_row"] is True
    assert sel_46["row_step"] == 2
    assert sel_46["next_index"] == idx_0 + 2
    assert abs(sel_46["next_x"] - 0.92) < 1e-3

    # Cas 2 : Espacement 75 cm (maïs / coton, > 50 cm) -> Rang immédiatement consécutif (+1 rang)
    rows_75cm = [round(i * 0.75, 3) for i in range(-3, 4)]
    idx_75 = rows_75cm.index(0.0)
    sel_75 = ctrl.select_next_peanut_row(current_x=0.0, crop_rows_x=rows_75cm, direction_sign=1)
    assert abs(sel_75["spacing_m"] - 0.75) < 1e-4
    assert sel_75["skip_row"] is False
    assert sel_75["row_step"] == 1
    assert sel_75["next_index"] == idx_75 + 1
    assert abs(sel_75["next_x"] - 0.75) < 1e-3


def test_inter_row_locking_accuracy():
    """Vérifie la précision de verrouillage avant engagement des outils M5/M6."""
    brain = AIVisionBrain(width=320, height=200)

    # Scène parfaitement centrée : Δθ et Δx quasi-nuls
    centered = brain.analyze_scene(
        robot_x=0.0,
        robot_z=1.0,
        robot_yaw=0.0,
        weeds_in_fov=[{"x": 0.0, "z": 2.5}],
        obstacles_in_fov=[],
    )
    assert abs(centered.get("heading_error_deg", 0.0)) < 2.0
    assert abs(centered.get("lateral_offset_m", 0.0)) < 0.025
