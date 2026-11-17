"""tests/test_obstacle_avoidance.py
Validation du contournement d'obstacles par AgriNavEngine et RobotController :
- Extraction Box3 3D réelle (width_m, length_m).
- Polygone orienté Shapely (2.20 m hors-tout x 1.21 m) avec zéro collision et marge >= 0.25 m.
- Contournement oblique calculé (angle alpha entre 15° et 60°, sans virage à 90°).
- Scan complet du couloir (choix du côté libre, détection des 2èmes rochers).
- Verrouillage dur LOCK_BYPASS_PLAN et suivi rail strict W0 -> W1 -> W2 -> W3.
"""

import math
from backend.navigation.agri_nav_engine import AgriNavEngine
from backend.navigation.robot_controller import RobotController
from backend.navigation.obstacle_avoidance import SymmetricTrapezoidPlanner
from backend.config.constants import (
    CHASSIS_LENGTH_TOP_CM,
    TOTAL_WIDTH_WITH_WHEELS_CM,
    TOTAL_ROBOT_LENGTH_M,
    TOTAL_ROBOT_WIDTH_M,
    FRONT_OVERHANG_M,
    REAR_OVERHANG_M,
)


def get_default_engine() -> AgriNavEngine:
    return AgriNavEngine({
        "chassis_length_top_cm": CHASSIS_LENGTH_TOP_CM,
        "front_overhang_m": FRONT_OVERHANG_M,
        "rear_overhang_m": REAR_OVERHANG_M,
        "total_width_with_wheels_cm": TOTAL_WIDTH_WITH_WHEELS_CM,
        "ground_clearance_cm": 24.0,
        "wheelbase_cm": 110.0,
        "track_width_cm": 103.0,
        "safety_margin_m": 0.35,
    })


def test_obstacle_box3_extraction_and_footprint():
    """Vérifie le polygone orienté Shapely avec les dimensions physiques exactes."""
    engine = get_default_engine()
    footprint = engine.get_robot_footprint(0.0, 0.0, 0.0, include_margin=False)

    # Dimensions : largeur 1.21 m, longueur totale 2.20 m (1.05 avant + 1.15 arrière)
    minx, miny, maxx, maxy = footprint.bounds
    width = maxx - minx
    length = maxy - miny
    assert abs(width - (TOTAL_WIDTH_WITH_WHEELS_CM / 100.0)) < 0.02
    assert abs(length - TOTAL_ROBOT_LENGTH_M) < 0.02


def test_oblique_avoidance_no_90_degree_turn():
    """Vérifie que la trajectoire oblique respecte alpha entre 15° et 60° et zéro collision."""
    engine = get_default_engine()
    rock = {
        "x": 0.0,
        "z": 3.6,
        "width_m": 1.20,
        "length_m": 1.10,
        "radius": 0.60,
        "type": "ROCHER",
    }

    plan = engine.find_lateral_bypass_corridor(
        corridor_x=0.0,
        target_heading_rad=0.0,
        primary_obs=rock,
        all_obstacles=[rock],
    )
    assert plan["success"] is True
    alpha_deg = plan["alpha_deg"]
    assert 15.0 <= abs(alpha_deg) <= 60.0, f"Angle oblique hors intervalle: {alpha_deg}°"
    assert abs(alpha_deg - 90.0) > 30.0, "Interdiction stricte des virages à 90°"
    assert plan["l_approche_m"] >= 1.35

    # Vérification collision sur les 3 segments
    rock_poly = engine.get_obstacle_polygon(rock)
    for seg_idx, (p_start, p_end) in enumerate([
        (plan["w0"], plan["w1"]),
        (plan["w1"], plan["w2"]),
        (plan["w2"], plan["w3"]),
    ]):
        for k in range(11):
            t = k / 10.0
            rx = p_start[0] + (p_end[0] - p_start[0]) * t
            rz = p_start[1] + (p_end[1] - p_start[1]) * t
            ryaw = 0.0 if seg_idx == 1 else plan["alpha_rad"] * plan["best_direction"] * (1 if seg_idx == 0 else -1)
            fp = engine.get_robot_footprint(rx, rz, ryaw, include_margin=False)
            assert not fp.intersects(rock_poly), f"Collision détectée sur segment {seg_idx} à ({rx:.2f}, {rz:.2f})"


def test_corridor_scanning_and_locked_bypass_plan():
    """Vérifie le choix du côté libre lors de la présence d'un 2ème rocher bloquant et le verrouillage."""
    engine = get_default_engine()
    rock_1 = {
        "id": "rock_1",
        "x": 0.0,
        "z": 3.6,
        "width_m": 1.45,
        "length_m": 1.35,
        "min_x": -0.725,
        "max_x": 0.725,
        "min_z": 2.925,
        "max_z": 4.275,
        "radius": 0.725,
        "type": "ROCHER",
    }
    # Rocher bloquant le couloir gauche
    rock_2_left = {
        "id": "rock_2_left",
        "x": -1.65,
        "z": 6.2,
        "width_m": 1.50,
        "length_m": 1.50,
        "radius": 0.75,
        "type": "ROCHER",
    }

    plan = engine.find_lateral_bypass_corridor(
        corridor_x=0.0,
        target_heading_rad=0.0,
        primary_obs=rock_1,
        all_obstacles=[rock_1, rock_2_left],
    )
    assert plan["success"] is True
    # Doit choisir impérativement la DROITE (+1)
    assert plan["best_direction"] == 1, f"Doit choisir la droite (+1) car la gauche est encombrée, obtenu: {plan['best_direction']}"

    # Verrouillage du plan
    engine.lock_bypass_plan(plan)
    plan_locked = engine.find_lateral_bypass_corridor(
        corridor_x=0.0,
        target_heading_rad=0.0,
        primary_obs=rock_1,
        all_obstacles=[rock_1, rock_2_left],
        force_replan=False,
    )
    assert plan_locked["best_direction"] == 1
    engine.unlock_bypass_plan()


def test_strict_rail_follower_w0_w1_w2_w3():
    """Vérifie la génération des 4 sommets W0->W1->W2->W3 et le calcul de suivi strict."""
    engine = get_default_engine()
    rock = {
        "x": 0.0,
        "z": 3.6,
        "width_m": 1.45,
        "length_m": 1.35,
        "min_x": -0.725,
        "max_x": 0.725,
        "min_z": 2.925,
        "max_z": 4.275,
        "radius": 0.725,
        "type": "ROCHER",
    }

    plan = engine.find_lateral_bypass_corridor(
        corridor_x=0.0,
        target_heading_rad=0.0,
        primary_obs=rock,
        all_obstacles=[rock],
    )
    assert len(plan["waypoints"]) == 4
    w0, w1, w2, w3 = plan["waypoints"]

    # Test d'une commande sur le rail W1->W2
    cmd = engine.compute_strict_rail_command(
        rx=w1[0],
        rz=(w1[1] + w2[1]) / 2.0,
        ryaw=0.0,
        w_start=w1,
        w_end=w2,
        is_parallel_pass=True,
        dir_z=1.0,
        is_aligned=True,
        obs_z=rock["z"],
        obs_r=rock["radius"],
    )
    assert abs(cmd["e_ct"]) < 0.03
    assert cmd["cmd_type"] == "DRIVE"


def test_symmetric_trapezoid_locked_parameters_and_x3_equals_x0():
    """Vérifie le Contournement Trapézoïdal Symétrique en 3 Trajectoires Exactes :
    - Trajectoire 1 : angle alpha et distance D1, clairance franche latérale >= 0.55m.
    - Trajectoire 2 : D2 = d_vers_bord_avant + obs_length + 2.15m + 0.50m (ou safety_margin).
    - Trajectoire 3 : angle -alpha et même distance D1 ramenant exactement à X3 == X0 (|X3 - X0| < 0.02m).
    - Zéro collision sur tous les segments (marge >= 0.20m).
    """
    from backend.navigation.obstacle_avoidance import SymmetricTrapezoidPlanner

    planner = SymmetricTrapezoidPlanner(
        robot_width_m=1.21,
        robot_length_m=2.15,
        front_overhang_m=1.05,
        rear_overhang_m=1.10,
        safety_margin_m=0.55,
        approach_l_m=1.40,
    )

    rock = {
        "x": 0.0,
        "z": 3.6,
        "width_m": 1.45,
        "length_m": 1.35,
        "min_x": -0.725,
        "max_x": 0.725,
        "min_z": 2.925,
        "max_z": 4.275,
        "radius": 0.725,
        "type": "ROCHER",
    }

    # Test avec cap 0.0 (sens croissant des Z)
    plan = planner.plan_symmetric_trapezoid(
        x0=0.0,
        z0=1.5,
        initial_heading_rad=0.0,
        primary_obs=rock,
        all_obstacles=[rock],
        field_width_m=4.6,
    )

    assert plan["success"] is True
    # 1. Mémorisation des variables demandées
    alpha_1 = plan["locked_alpha_rad"]
    d_1 = plan["locked_dist_1_m"]
    d_2 = plan["locked_d2_m"]
    x0 = plan["initial_line_x"]
    w0, w1, w2, w3 = plan["waypoints"]

    # Clairance franche latérale >= 0.55m (1.45/2 + 1.21/2 + 0.55 = 1.88m)
    assert abs(abs(w1[0] - x0) - 1.88) < 1e-2, f"Écartement latéral {abs(w1[0] - x0)} != 1.88m"

    # 2. Vérification Trajectoire 2 : D2 = max(dist_to_rear_edge + 2.80, obs_depth + 3.20)
    z1 = w1[1]
    obs_rear_z = rock["max_z"]
    dist_to_rear_edge = max(0.0, obs_rear_z - z1)
    obs_depth = abs(rock["max_z"] - rock["min_z"])
    required_d2 = round(max(dist_to_rear_edge + 2.80, obs_depth + 3.20), 3)
    assert abs(d_2 - required_d2) < 1e-3, f"D2 calculé {d_2} != attendu {required_d2}"

    # 3. Vérification Trajectoire 3 : Retour symétrique exact (-alpha_1, D1) et X3 == X0
    alpha_3 = planner.wrap_angle(plan["theta_seg_3"] - plan["initial_heading_rad"])
    assert abs(alpha_3 + alpha_1) < 1e-3, f"Traj 3 angle {alpha_3} doit valoir -alpha_1 ({-alpha_1})"

    d_3 = math.hypot(w3[0] - w2[0], w3[1] - w2[1])
    assert abs(d_3 - d_1) < 1e-3, f"Distance D3 {d_3} != D1 {d_1}"

    # X3 doit être égal à X0 à moins de 2 cm (ici exactement 0.000m)
    assert abs(w3[0] - x0) < 0.02, f"X3={w3[0]} doit être égal à X0={x0} à < 2cm"

    # 4. Zéro collision tout au long des 3 segments
    obs_poly = planner.get_obstacle_polygon(rock)
    assert plan["min_clearance_m"] >= 0.20, f"Marge de sécurité insuffisante: {plan['min_clearance_m']}m"


def test_robot_controller_symmetric_avoidance_execution():
    """Vérifie l'exécution séquentielle complète du contrôleur en mode automatique."""
    controller = RobotController(use_simulator=True)
    controller.robot_x = 0.0
    controller.robot_z = 1.5
    controller.robot_yaw = 0.0
    controller._last_3d_feedback_time = 0.0  # mode déconnecté / mock rapide

    rock = {
        "x": 0.0,
        "z": 3.6,
        "width_m": 1.45,
        "length_m": 1.35,
        "min_x": -0.725,
        "max_x": 0.725,
        "min_z": 2.925,
        "max_z": 4.275,
        "radius": 0.725,
        "type": "ROCHER",
    }
    controller._last_all_obstacles = [rock]

    # Déclencher le contournement
    result = controller._handle_obstacle_avoidance(
        corridor_center_x=0.0,
        heading_rad=0.0,
        pct=50,
    )

    assert result == "RESUMED_SAME_LINE"
    assert controller.obstacle_state == "CLEAR"
    assert controller.weeding_state == "SYNCHRONOUS_ROW_WEEDING"
    assert controller.bypass_plan_locked is False


def test_d2_lock_and_rear_teeth_clearance_both_directions():
    """Vérifie le double verrou mathématique de D2 et le dégagement des dents arrière pour +Z et -Z."""
    planner = SymmetricTrapezoidPlanner()

    rock = {
        "x": 0.0,
        "z": 4.0,
        "width_m": 1.20,
        "length_m": 1.00,
        "min_x": -0.60,
        "max_x": 0.60,
        "min_z": 3.50,
        "max_z": 4.50,
        "radius": 0.60,
        "type": "ROCHER",
    }

    # 1. Sens +Z (heading = 0)
    plan_pos_z = planner.plan_symmetric_trapezoid(
        x0=0.0,
        z0=2.0,
        initial_heading_rad=0.0,
        primary_obs=rock,
    )
    assert plan_pos_z["success"] is True
    z1_pos = plan_pos_z["z1"]
    obs_rear_z_pos = rock["max_z"]
    dist_to_rear_pos = max(0.0, obs_rear_z_pos - z1_pos)
    req_d2_pos = max(dist_to_rear_pos + 2.80, 1.00 + 3.20)
    assert abs(plan_pos_z["locked_d2_m"] - req_d2_pos) < 1e-3

    # Vérification des dents à la fin de Seg2 (+Z) :
    z2_pos = plan_pos_z["z2"]
    rear_teeth_z_pos = z2_pos - 1.40
    assert rear_teeth_z_pos >= rock["max_z"] + 0.50, f"Dents arrière {rear_teeth_z_pos} pas assez dégagées"

    # 2. Sens -Z (heading = pi)
    plan_neg_z = planner.plan_symmetric_trapezoid(
        x0=0.0,
        z0=6.0,
        initial_heading_rad=math.pi,
        primary_obs=rock,
    )
    assert plan_neg_z["success"] is True
    z1_neg = plan_neg_z["z1"]
    obs_rear_z_neg = rock["min_z"]
    dist_to_rear_neg = max(0.0, z1_neg - obs_rear_z_neg)
    req_d2_neg = max(dist_to_rear_neg + 2.80, 1.00 + 3.20)
    assert abs(plan_neg_z["locked_d2_m"] - req_d2_neg) < 1e-3

    # Vérification des dents à la fin de Seg2 (-Z) :
    z2_neg = plan_neg_z["z2"]
    rear_teeth_z_neg = z2_neg - (-1.0) * 1.40  # z2 + 1.40
    assert rear_teeth_z_neg <= rock["min_z"] - 0.50, f"Dents arrière {rear_teeth_z_neg} pas assez dégagées en -Z"


def test_obstacle_avoidance_return_pass_neg_z():
    """Test obligatoire de contournement au RETOUR (Cap = 180°, dir_z = -1, de Z=10 vers Z=0) :
    - Obstacle en Z=5.0 (min_z=4.5, max_z=5.5)
    - Face avant en retour = max_z (5.5)
    - Recul de sécurité obligatoire : Z > 6.5 (Z_0 >= 5.5 + 1.55 = 7.05m)
    - Contournement sans collision (clearance >= 0.20m)
    - Fin de dépassement D2 : ne se rabat qu'à Z < 2.2m (dents arrière à Z_dents <= min_z - 0.50m)
    - Aucun plant d'arachide détruit (alignement sur les couloirs inter-rangs)
    """
    planner = SymmetricTrapezoidPlanner()

    rock = {
        "x": 0.0,
        "z": 5.0,
        "width_m": 1.20,
        "length_m": 1.00,
        "min_x": -0.60,
        "max_x": 0.60,
        "min_z": 4.50,
        "max_z": 5.50,
        "radius": 0.60,
        "type": "ROCHER",
    }

    dir_z = -1.0
    initial_heading = math.pi
    front_face_z = rock["max_z"]  # 5.50m face au robot venant de Z=10
    rear_face_z = rock["min_z"]   # 4.50m

    # 1. Position après marche arrière de dégagement : dist_to_front >= 1.55m
    z0 = front_face_z - dir_z * 1.55  # 5.50 - (-1.0) * 1.55 = 7.05m
    assert z0 > 6.5, f"Recul insuffisant : z0={z0} <= 6.5m"

    plan = planner.plan_symmetric_trapezoid(
        x0=0.0,
        z0=z0,
        initial_heading_rad=initial_heading,
        primary_obs=rock,
    )
    assert plan["success"] is True

    # 2. Vérification Trajectoire 1 (Sortie vers -Z)
    z1 = plan["z1"]
    assert z1 < z0, f"En marche vers -Z, z1 ({z1}) doit être inférieur à z0 ({z0})"

    # 3. Vérification Trajectoire 2 (Ligne droite D2 au retour)
    z2 = plan["z2"]
    # En sens -Z, z2 doit avoir largement dépassé le rocher :
    assert z2 < 2.2, f"Le robot se rabat trop tôt au retour : z2={z2} >= 2.2m"

    # Vérification des dents arrière à la fin de Traj 2
    rear_teeth_z = z2 - dir_z * 1.40  # z2 + 1.40
    assert rear_teeth_z <= rear_face_z - 0.50, (
        f"Dents arrière {rear_teeth_z:.2f}m n'ont pas dépassé rear_face_z {rear_face_z:.2f}m avec 0.50m de marge"
    )

    # 4. Trajectoire 3 et retour sur la ligne
    z3 = plan["z3"]
    assert abs(plan["x3"] - plan["initial_line_x"]) < 0.02

    # 5. Zéro collision
    assert plan["min_clearance_m"] >= 0.20



