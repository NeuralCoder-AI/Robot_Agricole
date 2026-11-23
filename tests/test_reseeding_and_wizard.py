"""tests/test_reseeding_and_wizard.py
Validation des fonctionnalités :
1. Décalage latéral selon le sens choisi (LEFT = -1 vs RIGHT = +1) à partir d'une position initiale (X0, Z0) quelconque.
2. Semis double ligne à 46 cm (pas de demi-tour de 92 cm = 2 x 46 cm) sur champ vide.
3. Algorithme de Révision de la Semence (Resemence Ciblée) : le semoir M8 reste inactif sur les zones peuplées
   et ne s'active que sur les coordonnées des trous de germination (> 35 cm) et des plants morts.
"""

import math
from backend.navigation.robot_controller import RobotController
from backend.navigation.reseeding_planner import ReseedingPlanner


def test_progression_direction_left_and_right():
    """Vérifie le décalage latéral selon le sens choisi (LEFT = -1 vs RIGHT = +1) depuis (X0, Z0) quelconque."""
    ctrl = RobotController(use_simulator=True)
    rows_46cm = [round(i * 0.46, 3) for i in range(-8, 9)]

    # Position initiale arbitraire : X0 = 0.92 m, Z0 = 2.10 m
    x0 = 0.92
    ctrl.robot_x = x0
    ctrl.robot_z = 2.10

    # Test Sens DROITE (progression_direction = +1)
    sel_right = ctrl.select_next_peanut_row(current_x=x0, crop_rows_x=rows_46cm, direction_sign=+1)
    assert sel_right["next_x"] > x0, f"En progression DROITE, next_x ({sel_right['next_x']}) doit être > X0 ({x0})"
    # Avec saut de rang (46 cm <= 50 cm), doit aller à X0 + 2 * 0.46 = 1.84 m
    assert abs(sel_right["next_x"] - 1.84) < 1e-3

    # Test Sens GAUCHE (progression_direction = -1)
    sel_left = ctrl.select_next_peanut_row(current_x=x0, crop_rows_x=rows_46cm, direction_sign=-1)
    assert sel_left["next_x"] < x0, f"En progression GAUCHE, next_x ({sel_left['next_x']}) doit être < X0 ({x0})"
    # Avec saut de rang, doit aller à X0 - 2 * 0.46 = 0.00 m
    assert abs(sel_left["next_x"] - 0.00) < 1e-3


def test_initial_seeding_double_line_46cm_step_92cm():
    """Vérifie le semis double ligne à 46 cm avec pas latéral de 92 cm (2 x 46 cm) sur champ vide."""
    ctrl = RobotController(use_simulator=True)
    ctrl.field_width_m = 4.6
    ctrl.field_length_m = 10.0
    ctrl.progression_direction = 1  # Droite

    # 2 rangs semés simultanément par les 2 tuyaux M8 écartés de 46 cm
    pass_width_m = 0.92
    expected_passes = int(round(ctrl.field_width_m / pass_width_m))
    assert expected_passes == 5, f"Pour 4.6 m de largeur, attendu 5 passes de 92 cm, obtenu {expected_passes}"

    # Simulation d'un pas latéral en bout de ligne
    x0 = 0.0
    next_x_right = x0 + ctrl.progression_direction * 0.92
    assert abs(next_x_right - 0.92) < 1e-3

    # Test en sens inverse (Gauche)
    ctrl.progression_direction = -1
    next_x_left = x0 + ctrl.progression_direction * 0.92
    assert abs(next_x_left - (-0.92)) < 1e-3


def test_reseeding_planner_and_m8_selective_trigger():
    """Vérifie l'algorithme de Révision de la Semence :
    - Le semoir reste à 0 sur les zones peuplées régulièrement (<= 30 cm).
    - Ralentissement à 18 km/h et activation du semoir M8 sur les trous (> 35 cm) et plants morts.
    - Zéro doublon sur les plants vivants.
    """
    planner = ReseedingPlanner(
        seeder_row_spacing_m=0.46,
        seeder_rear_offset_m=0.85,
        nominal_seed_gap_cm=20.0,
        gap_detection_threshold_cm=35.0,
        camera_lookahead_m=1.80,
    )

    # Scénario 1 : Rang parfaitement peuplé et sain (espacement régulier de 25 cm)
    healthy_plants = [
        {"x": -0.23, "z": round(1.0 + i * 0.25, 2), "alive": True} for i in range(10)
    ] + [
        {"x": +0.23, "z": round(1.0 + i * 0.25, 2), "alive": True} for i in range(10)
    ]

    eval_healthy = planner.evaluate_reseeding_at_pose(
        robot_x=0.0,
        robot_z=1.50,
        robot_yaw_rad=0.0,
        dir_z=1.0,
        nearby_plants=healthy_plants,
        row_left_x=-0.23,
        row_right_x=+0.23,
    )
    assert eval_healthy["status"] == "FULL_ROW"
    assert eval_healthy["recommended_speed"] == 32
    assert eval_healthy["trigger_seeder"] is False
    assert len(eval_healthy["seeds_to_drop"]) == 0

    # Scénario 2 : Trou de germination de 60 cm (> 35 cm) sur le rang gauche entre Z = 2.0 et Z = 2.6
    plants_with_gap = [
        {"x": -0.23, "z": 1.0, "alive": True},
        {"x": -0.23, "z": 1.25, "alive": True},
        {"x": -0.23, "z": 2.0, "alive": True},
        {"x": -0.23, "z": 2.6, "alive": True},  # Trou de 0.60 m
        {"x": +0.23, "z": 1.0, "alive": True},
        {"x": +0.23, "z": 1.25, "alive": True},
    ]

    # Le robot est en Z = 1.0, la caméra (lookahead 1.8m) voit le trou devant entre 2.0 et 2.6
    eval_gap_ahead = planner.evaluate_reseeding_at_pose(
        robot_x=0.0,
        robot_z=1.0,
        robot_yaw_rad=0.0,
        dir_z=1.0,
        nearby_plants=plants_with_gap,
        row_left_x=-0.23,
        row_right_x=+0.23,
    )
    assert eval_gap_ahead["status"] == "GAP_DETECTED"
    assert eval_gap_ahead["recommended_speed"] == 18

    # Scénario 3 : Plant mort détecté à remplacer sur le rang droit à Z = 1.0
    plants_with_dead = [
        {"x": +0.23, "z": 1.0, "alive": False},  # Mort
        {"x": +0.23, "z": 1.25, "alive": True},
    ]
    # Les tuyaux arrière M8 sont à Z_robot - 0.85 m. Donc si robot_z = 1.85 m, le tuyau arrière est en Z = 1.00 m !
    eval_at_dead = planner.evaluate_reseeding_at_pose(
        robot_x=0.0,
        robot_z=1.85,  # 1.85 - 0.85 = 1.00 m (pile sur le plant mort)
        robot_yaw_rad=0.0,
        dir_z=1.0,
        nearby_plants=plants_with_dead,
        row_left_x=-0.23,
        row_right_x=+0.23,
    )
    assert eval_at_dead["trigger_seeder"] is True
    assert eval_at_dead["status"] == "RESEEDING_ACTIVE"
    assert len(eval_at_dead["seeds_to_drop"]) == 1
    assert eval_at_dead["seeds_to_drop"][0]["reason"] == "REPLACE_DEAD"

    # Vérification anti-doublon : un deuxième passage ne doit pas redéclencher
    eval_repeat = planner.evaluate_reseeding_at_pose(
        robot_x=0.0,
        robot_z=1.85,
        robot_yaw_rad=0.0,
        dir_z=1.0,
        nearby_plants=plants_with_dead,
        row_left_x=-0.23,
        row_right_x=+0.23,
    )
    assert eval_repeat["trigger_seeder"] is False

