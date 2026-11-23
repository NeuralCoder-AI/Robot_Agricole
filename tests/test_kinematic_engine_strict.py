"""tests/test_kinematic_engine_strict.py
Tests unitaires stricts validant les 4 règles inviolables du KinematicEngine :
1. Faux positifs éliminés : un obstacle hors couloir latéral est ignoré à 100%.
2. Collision des dents éliminée : les dents arrière restent toujours à >= 0.45 m du rocher (+Z et -Z).
3. Non-retour au 2ème tour : progression stricte (0 -> 2 -> 4) sans retour sur la ligne précédente.
4. Respect des grands espacements : espacement > 0.50 m entraîne rowStep = 1 (visite 0 -> 1 -> 2 -> 3).
"""

import math
import pytest
from backend.navigation.kinematic_engine import KinematicEngine


def test_rule_1_lateral_corridor_box_ignores_lateral_obstacles():
    """RÈGLE 1 : Un obstacle à X = 1.20 m sur une ligne X = 0.00 m est ignoré à 100%."""
    engine = KinematicEngine(row_spacing=0.46, field_length=15.0, field_width=4.6, start_x=0.0, start_z=0.0)
    engine.init_mission()
    assert engine.target_row_x == 0.0
    assert engine.pass_direction == 1

    # Obstacle à X=1.20 m, largeur 0.40 m, profondeur 0.40 m, Z=4.0 m
    # collisionHalfWidth = 0.40 / 2 + 0.605 + 0.08 = 0.885 m
    # lateral_dist = 1.20 m > 0.885 m -> DOIT ÊTRE IGNORÉ
    obs_outside = {
        "id": "rock_lane_2",
        "x": 1.20,
        "z": 4.0,
        "width_m": 0.40,
        "length_m": 0.40,
    }

    chk = engine.check_obstacle_collision(obs_outside)
    assert not chk["is_blocking"], f"L'obstacle latéral a été détecté à tort : {chk}"
    assert chk["reason"] == "HORS_COULOIR"
    assert chk["lateral_dist"] == 1.20
    assert chk["collision_half_width"] == 0.885

    # En revanche, un obstacle centré à X=0.05 m sur la ligne X=0.00 m est détecté dans le couloir
    obs_inside = {
        "id": "rock_on_row",
        "x": 0.05,
        "z": engine.z + engine.ROBOT_NOSE_EXTENT + 0.80,  # forward_dist = 0.80 m (<= 1.20 m)
        "width_m": 0.40,
        "length_m": 0.40,
    }
    chk_inside = engine.check_obstacle_collision(obs_inside)
    assert chk_inside["is_blocking"], f"L'obstacle sur le rang devrait être bloquant : {chk_inside}"
    assert chk_inside["forward_dist"] <= 1.20


def test_rule_2_rear_tines_safety_clearance_forward_and_backward():
    """RÈGLE 2 : Les dents arrière (à 1.15 m derrière le centre) restent à >= 0.18 m du rocher (+Z et -Z)."""
    for dir_z in [1, -1]:
        start_z = 0.0 if dir_z > 0 else 10.0
        obs_z = 4.0 if dir_z > 0 else 6.0
        obs = {
            "id": f"rock_dir_{dir_z}",
            "x": 0.0,
            "z": obs_z,
            "width_m": 0.50,
            "length_m": 0.50,
            "min_z": obs_z - 0.25,
            "max_z": obs_z + 0.25,
        }

        engine = KinematicEngine(row_spacing=0.46, field_length=10.0, field_width=4.6, start_x=0.0, start_z=start_z)
        engine.init_mission()
        engine.pass_direction = dir_z
        engine.z = start_z

        waypoints_plan = engine.compute_trapezoid_waypoints(obs, side_x=1)
        wp0 = waypoints_plan["WP0"]
        wp1 = waypoints_plan["WP1"]
        wp2 = waypoints_plan["WP2"]
        wp3 = waypoints_plan["WP3"]

        # À WP2 (fin du segment parallèle) :
        # Le robot s'est avancé jusqu'à rear_z + dir_z * 1.35
        # Les dents arrière sont à 1.15 m derrière le centre -> position dents = rear_z + dir_z * (1.35 - 1.15) = rear_z + dir_z * 0.20 m
        # La marge par rapport à la face arrière du rocher est strictement 0.20 m >= 0.18 m !
        rear_face_z = waypoints_plan["rear_z"]
        tines_box_at_wp2 = engine.get_tines_bounding_box(wp2["x"], wp2["z"], dir_z=dir_z)

        # Boîte du rocher : [x - w/2, min_z, x + w/2, max_z]
        obs_box = (obs["x"] - 0.25, obs["min_z"], obs["x"] + 0.25, obs["max_z"])

        dist_at_wp2 = engine.box_distance(tines_box_at_wp2, obs_box)
        assert dist_at_wp2 >= 0.18, (
            f"Direction {dir_z}: Marge de sécurité des dents à WP2 insuffisante : {dist_at_wp2:.3f} m < 0.18 m"
        )

        # Simulation pas à pas du contournement (dt = 0.05s) et vérification continue de non-collision
        engine.trigger_avoidance_plan(obs, side_x=1)
        max_steps = 800
        step_count = 0
        min_recorded_distance = 999.0

        while engine.state in ("APPROACH_AVOIDANCE", "TRAJ_1_EXIT", "TRAJ_2_PARALLEL", "TRAJ_3_RETURN") and step_count < max_steps:
            engine.step_simulation(dt=0.05)
            step_count += 1
            t_box = engine.get_tines_bounding_box(engine.x, engine.z, dir_z=dir_z)
            d = engine.box_distance(t_box, obs_box)
            if d < min_recorded_distance:
                min_recorded_distance = d

        assert min_recorded_distance >= 0.18, (
            f"Direction {dir_z}: Collision potentielle détectée pendant le contournement ! Min dist={min_recorded_distance:.3f} m"
        )


def test_rule_3_row_progression_no_backtracking():
    """RÈGLE 3 : Fin du bug du 2ème tour — progression stricte 0 -> 2 -> 4 et inversion de cap."""
    engine = KinematicEngine(row_spacing=0.46, field_length=5.0, field_width=4.6, start_x=0.0, start_z=0.0)
    engine.init_mission()

    assert engine.row_step == 2, "Pour row_spacing = 0.46 m (<= 0.50 m), row_step doit valoir 2"
    assert engine.current_row_index == 0
    assert engine.target_row_x == 0.0
    assert engine.pass_direction == 1

    visited_indices = [engine.current_row_index]
    visited_xs = [engine.target_row_x]

    # Tour 1 -> Tour 2
    has_next = engine.advance_row_cursor()
    assert has_next is True
    assert engine.current_row_index == 2, f"Au 2ème tour, index attendu = 2, obtenu = {engine.current_row_index}"
    assert engine.target_row_x == round(0.0 - 2 * 0.46, 3)  # -0.92 m (vers la droite)
    assert engine.pass_direction == -1
    visited_indices.append(engine.current_row_index)
    visited_xs.append(engine.target_row_x)

    # Tour 2 -> Tour 3
    has_next = engine.advance_row_cursor()
    assert has_next is True
    assert engine.current_row_index == 4, f"Au 3ème tour, index attendu = 4, obtenu = {engine.current_row_index}"
    assert engine.target_row_x == round(0.0 - 4 * 0.46, 3)  # -1.84 m (continue vers la droite)
    assert engine.pass_direction == 1
    visited_indices.append(engine.current_row_index)
    visited_xs.append(engine.target_row_x)

    # Vérification stricte d'absence de régression
    assert visited_indices == [0, 2, 4]
    assert visited_xs == [0.0, -0.92, -1.84]
    assert len(set(visited_indices)) == 3, "Un rang a été visité deux fois !"


def test_rule_4_wide_row_spacing_visits_every_single_row():
    """RÈGLE 4 : Grands espacements (> 0.50 m) -> row_step = 1, le robot ne saute AUCUNE ligne."""
    engine = KinematicEngine(row_spacing=0.80, field_length=10.0, field_width=4.0, start_x=0.0, start_z=0.0)
    engine.init_mission()

    assert engine.row_step == 1, "Pour row_spacing = 0.80 m (> 0.50 m), row_step doit valoir 1"
    assert engine.current_row_index == 0
    assert engine.target_row_x == 0.0

    visited_indices = [engine.current_row_index]
    visited_xs = [engine.target_row_x]

    # Progression sur 4 passes successives
    for expected_idx in [1, 2, 3]:
        has_next = engine.advance_row_cursor()
        assert has_next is True
        assert engine.current_row_index == expected_idx
        assert engine.target_row_x == round(-expected_idx * 0.80, 3)
        visited_indices.append(engine.current_row_index)
        visited_xs.append(engine.target_row_x)

    assert visited_indices == [0, 1, 2, 3]
    assert visited_xs == [0.0, -0.80, -1.60, -2.40]


def test_check_real_ground_collision_zero_false_alert():
    """RÈGLE 5 : L'empreinte 2D OBB exacte au sol élimine à 100% les fausses alertes de collision."""
    # Obstacle rocher central : centre (0.0, 5.2), rayon 0.50 m
    obs_x = 0.0
    obs_z = 5.2
    obs_radius = 0.50

    # 1. Cas normal de contournement : robot en diagonale à 45° avec 30 à 38 cm de marge latérale
    # Position robot : x = 0.88, z = 4.35, cap = 45° (PI/4)
    # L'ancienne THREE.Box3 AABB renvoyait True (fausse alerte), mais check_real_ground_collision DOIT renvoyer False !
    hit_at_45deg = KinematicEngine.check_real_ground_collision(
        robot_x=0.88,
        robot_z=4.35,
        robot_heading=math.pi / 4,
        obs_x=obs_x,
        obs_z=obs_z,
        obs_radius=obs_radius,
    )
    assert hit_at_45deg is False, "Fausse alerte détectée à 45° alors que le robot a une marge de 30+ cm !"

    # 2. Cas du segment parallèle (Trajectoire 2) : robot longeant le rocher à x = 1.235 m, cap = 0°
    hit_parallel = KinematicEngine.check_real_ground_collision(
        robot_x=1.235,
        robot_z=5.20,
        robot_heading=0.0,
        obs_x=obs_x,
        obs_z=obs_z,
        obs_radius=obs_radius,
    )
    assert hit_parallel is False, "Fausse alerte détectée sur le segment parallèle !"

    # 3. Cas de collision frontale réelle : robot fonce directement dans le rocher
    hit_head_on = KinematicEngine.check_real_ground_collision(
        robot_x=0.0,
        robot_z=4.60,
        robot_heading=0.0,
        obs_x=obs_x,
        obs_z=obs_z,
        obs_radius=obs_radius,
    )
    assert hit_head_on is True, "Une vraie collision frontale doit être détectée !"


def test_progression_direction_left_and_right():
    """RÈGLE 6 : Le sens de progression 'Vers la DROITE' (-1) vs 'Vers la GAUCHE' (+1) et espacement strict 0.46 m."""
    # 1. Sélection 'Vers la DROITE' (step_dir = -1, repère Three.js)
    engine_right = KinematicEngine(row_spacing=0.46, field_length=10.0, field_width=4.6, start_x=0.0, start_z=0.0)
    engine_right.init_mission(step_dir=-1)

    assert engine_right.step_dir == -1
    assert engine_right.pass_direction == 1  # Démarre toujours vers +Z
    assert engine_right.target_row_x == 0.0

    # Vérification espacement strictement identique 0.46 m sur toutes les lignes
    for i in range(len(engine_right.rows_x) - 1):
        gap = abs(engine_right.rows_x[i + 1] - engine_right.rows_x[i])
        assert round(gap, 3) == 0.46, f"Écart non uniforme entre ligne {i} et {i+1} : {gap}"

    # Ligne 1 -> U-turn -> Ligne 2
    has_next = engine_right.advance_row_cursor()
    assert has_next is True
    assert engine_right.target_row_x == round(0.0 - 2 * 0.46, 3)  # -0.92 m (à droite visuelle !)
    assert engine_right.pass_direction == -1

    # Ligne 2 -> U-turn -> Ligne 3
    has_next = engine_right.advance_row_cursor()
    assert has_next is True
    assert engine_right.target_row_x == round(0.0 - 4 * 0.46, 3)  # -1.84 m (continue à droite !)
    assert engine_right.pass_direction == 1

    # 2. Sélection 'Vers la GAUCHE' (step_dir = +1)
    engine_left = KinematicEngine(row_spacing=0.46, field_length=10.0, field_width=4.6, start_x=0.0, start_z=0.0)
    engine_left.init_mission(step_dir=1)

    assert engine_left.step_dir == 1
    assert engine_left.pass_direction == 1  # Démarre toujours vers +Z
    assert engine_left.target_row_x == 0.0

    # Vérification espacement strictement identique 0.46 m
    for i in range(len(engine_left.rows_x) - 1):
        gap = abs(engine_left.rows_x[i + 1] - engine_left.rows_x[i])
        assert round(gap, 3) == 0.46, f"Écart non uniforme entre ligne {i} et {i+1} : {gap}"

    # Ligne 1 -> U-turn -> Ligne 2
    has_next = engine_left.advance_row_cursor()
    assert has_next is True
    assert engine_left.target_row_x == round(0.0 + 2 * 0.46, 3)  # +0.92 m (à gauche visuelle !)
    assert engine_left.pass_direction == -1

    # Ligne 2 -> U-turn -> Ligne 3
    has_next = engine_left.advance_row_cursor()
    assert has_next is True
    assert engine_left.target_row_x == round(0.0 + 4 * 0.46, 3)  # +1.84 m (continue à gauche !)
    assert engine_left.pass_direction == 1


