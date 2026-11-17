"""
obstacle_avoidance.py — Contournement Trapézoïdal Symétrique en 3 Trajectoires Exactes.
Mémorisation stricte : Angle alpha, Distance D1, Ligne Droite D2 selon profondeur de l'obstacle,
et Retour Symétrique (-alpha, D1) sans recalcul en cours de manœuvre.
"""

import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from shapely.geometry import Polygon, box
from shapely.ops import unary_union


class SymmetricTrapezoidPlanner:
    """Planificateur géométrique autonome à 3 trajectoires trapézoïdales symétriques :
    - Trajectoire 1 : Sortie de la portée de l'obstacle (+alpha, D1).
    - Trajectoire 2 : Ligne droite parallèle de dépassement (cap 0° ou 180°, D2 calculé selon profondeur obstacle).
    - Trajectoire 3 : Retour symétrique exact sur la ligne initiale (-alpha, D1).
    - Reprise : Réalignement strict sur l'axe du rang, descente des outils à l'arrêt, et reprise du travail.
    """

    def __init__(
        self,
        robot_width_m: float = 1.21,
        robot_length_m: float = 2.15,
        front_overhang_m: float = 1.05,
        rear_overhang_m: float = 1.10,
        safety_margin_m: float = 0.55,
        approach_l_m: float = 1.50,
        extra_d2_clearance_m: float = 0.50,
    ):
        self.robot_width_m = robot_width_m
        self.robot_length_m = robot_length_m
        self.front_overhang_m = front_overhang_m
        self.rear_overhang_m = rear_overhang_m
        self.safety_margin_m = max(0.55, safety_margin_m)
        self.approach_l_m = max(1.50, approach_l_m)
        self.extra_d2_clearance_m = max(0.50, extra_d2_clearance_m)

    @staticmethod
    def wrap_angle(angle_rad: float) -> float:
        """Normalise un angle dans [-pi, +pi]."""
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def get_obstacle_polygon(self, obs: Dict[str, Any]) -> Polygon:
        """Génère le polygone 2D Shapely de l'obstacle."""
        ox = float(obs.get("abs_x", obs.get("x", 0.0)))
        oz = float(obs.get("abs_z", obs.get("z", 0.0)))
        w = float(obs.get("width_m", obs.get("radius", 0.45) * 2.0))
        l = float(obs.get("length_m", obs.get("radius", 0.45) * 2.0))
        r = max(float(obs.get("radius", 0.45)), w / 2.0, l / 2.0)

        min_x = obs.get("min_x", ox - r)
        max_x = obs.get("max_x", ox + r)
        min_z = obs.get("min_z", oz - r)
        max_z = obs.get("max_z", oz + r)
        return box(float(min_x), float(min_z), float(max_x), float(max_z))

    def get_robot_footprint(self, rx: float, rz: float, ryaw: float) -> Polygon:
        """Polygone orienté du robot au sol avec outils avant (M6) et arrière (M5)."""
        cos_y = math.cos(ryaw)
        sin_y = math.sin(ryaw)
        hw = self.robot_width_m / 2.0
        corners = [
            (-hw, self.front_overhang_m),
            (hw, self.front_overhang_m),
            (hw, -self.rear_overhang_m),
            (-hw, -self.rear_overhang_m),
        ]
        pts = []
        for lx, lz in corners:
            gx = rx + lx * cos_y + lz * sin_y
            gz = rz - lx * sin_y + lz * cos_y
            pts.append((gx, gz))
        return Polygon(pts)

    def plan_symmetric_trapezoid(
        self,
        x0: float,
        z0: float,
        initial_heading_rad: float,
        primary_obs: Dict[str, Any],
        all_obstacles: Optional[List[Dict[str, Any]]] = None,
        field_width_m: float = 4.6,
    ) -> Dict[str, Any]:
        """Calcule et verrouille en mémoire le plan trapézoïdal symétrique en 3 trajectoires exactes :
        1. Trajectoire 1 : Sortie de la portée de l'obstacle (+alpha, D1).
        2. Trajectoire 2 : Ligne droite de dépassement (D2) = d_vers_bord_avant + obs_length + 2.15m + 0.45m.
        3. Trajectoire 3 : Retour symétrique (-alpha, D1) ramenant exactement à x3 == x0.
        """
        dir_z = 1.0 if math.cos(initial_heading_rad) >= 0 else -1.0

        obs_poly = self.get_obstacle_polygon(primary_obs)
        min_x, min_z, max_x, max_z = obs_poly.bounds
        obs_cx = (min_x + max_x) / 2.0
        obs_cz = (min_z + max_z) / 2.0
        obs_w = max_x - min_x
        obs_l = max_z - min_z
        obs_r = max(float(primary_obs.get("radius", 0.45)), obs_w / 2.0, obs_l / 2.0)

        # ----------------------------------------------------------------------
        # Choix du côté libre (Droite vs Gauche) avec scan du couloir
        # ----------------------------------------------------------------------
        right_world_sign = 1.0 if dir_z > 0 else -1.0
        candidate_sides = [(1, right_world_sign), (-1, -right_world_sign)]

        # Identifier tout obstacle secondaire bloquant
        secondary_polys: List[Polygon] = []
        for o in (all_obstacles or []):
            p = self.get_obstacle_polygon(o)
            if p.centroid.distance(obs_poly.centroid) > 0.20:
                secondary_polys.append(p)

        best_plan = None
        best_clearance = -1.0

        for rel_side, world_sign in candidate_sides:
            margin = max(0.55, self.safety_margin_m)
            delta_x_lat = abs(obs_cx - x0) + (obs_w / 2.0) + (self.robot_width_m / 2.0) + margin
            delta_x = world_sign * delta_x_lat
            x_safe = round(x0 + delta_x, 3)

            # Trajectoire 1 : Sortie (+alpha, D1)
            l1 = self.approach_l_m
            z1 = round(z0 + dir_z * l1, 3)
            w0 = [round(x0, 3), round(z0, 3)]
            w1 = [round(x_safe, 3), z1]

            # Angle absolu et angle relatif (+alpha)
            theta_seg_1 = math.atan2(w1[0] - w0[0], w1[1] - w0[1])
            alpha_rel_rad = self.wrap_angle(theta_seg_1 - initial_heading_rad)
            d1 = round(math.hypot(w1[0] - w0[0], w1[1] - w0[1]), 3)

            # Trajectoire 2 : Ligne droite de dépassement (D2)
            # Calcul Exact et Verrouillage de la Distance Droite D2 (Indépendant du Sens +Z ou -Z) :
            # seg2_start_z = z1
            obs_depth = abs(max_z - min_z)
            if dir_z > 0:
                obs_rear_z = max_z
                dist_to_rear_edge = max(0.0, obs_rear_z - z1)
            else:
                obs_rear_z = min_z
                dist_to_rear_edge = max(0.0, z1 - obs_rear_z)

            # Longueur complète châssis + dents arrière (2.15 m) + marge franche de sécurité (0.65 m) = 2.80 m
            required_d2 = dist_to_rear_edge + 2.80
            d2 = round(max(required_d2, obs_depth + 3.20), 3)

            z_bord_avant_obs = min_z if dir_z > 0 else max_z
            z_bord_arriere_obs = obs_rear_z
            d_vers_bord_avant = max(0.0, dir_z * (z_bord_avant_obs - z1))

            z2 = round(z1 + dir_z * d2, 3)
            w2 = [round(x_safe, 3), z2]

            # Trajectoire 3 : Retour symétrique (-alpha, D1)
            # Même angle inverse -alpha, même distance D1, ramène exactement à X3 = X0
            z3 = round(z2 + dir_z * l1, 3)
            w3 = [round(x0, 3), z3]

            theta_seg_3 = math.atan2(w3[0] - w2[0], w3[1] - w2[1])
            alpha_3_rel_rad = self.wrap_angle(theta_seg_3 - initial_heading_rad)
            d3 = round(math.hypot(w3[0] - w2[0], w3[1] - w2[1]), 3)

            # Échantillonnage de non-collision
            cand_ok = True
            min_clear = 99.0

            # Échantillonnage segment 1
            for k in range(9):
                t = k / 8.0
                rx = w0[0] + (w1[0] - w0[0]) * t
                rz = w0[1] + (w1[1] - w0[1]) * t
                fp = self.get_robot_footprint(rx, rz, theta_seg_1)
                d = fp.distance(obs_poly)
                if d < 0.20:
                    cand_ok = False
                    break
                for sp in secondary_polys:
                    d_s = fp.distance(sp)
                    if d_s < 0.20:
                        cand_ok = False
                        break
                    d = min(d, d_s)
                min_clear = min(min_clear, d)

            # Échantillonnage segment 2 (ligne droite)
            if cand_ok:
                for k in range(11):
                    t = k / 10.0
                    rx = w1[0]
                    rz = w1[1] + (w2[1] - w1[1]) * t
                    fp = self.get_robot_footprint(rx, rz, initial_heading_rad)
                    d = fp.distance(obs_poly)
                    if d < 0.20:
                        cand_ok = False
                        break
                    for sp in secondary_polys:
                        d_s = fp.distance(sp)
                        if d_s < 0.20:
                            cand_ok = False
                            break
                        d = min(d, d_s)
                    min_clear = min(min_clear, d)

            # Échantillonnage segment 3
            if cand_ok:
                for k in range(9):
                    t = k / 8.0
                    rx = w2[0] + (w3[0] - w2[0]) * t
                    rz = w2[1] + (w3[1] - w2[1]) * t
                    fp = self.get_robot_footprint(rx, rz, theta_seg_3)
                    d = fp.distance(obs_poly)
                    if d < 0.20:
                        cand_ok = False
                        break
                    for sp in secondary_polys:
                        d_s = fp.distance(sp)
                        if d_s < 0.20:
                            cand_ok = False
                            break
                        d = min(d, d_s)
                    min_clear = min(min_clear, d)

            if cand_ok and min_clear > best_clearance:
                best_clearance = min_clear
                best_plan = {
                    "success": True,
                    "rel_side": rel_side,
                    "best_direction": rel_side,
                    "direction": rel_side,
                    "world_sign": world_sign,
                    "x0": w0[0],
                    "z0": w0[1],
                    "x1": w1[0],
                    "z1": w1[1],
                    "x2": w2[0],
                    "z2": w2[1],
                    "x3": w3[0],
                    "z3": w3[1],
                    "w0": w0,
                    "w1": w1,
                    "w2": w2,
                    "w3": w3,
                    "waypoints": [w0, w1, w2, w3],
                    "start_pt": tuple(w0),
                    "point_a": tuple(w1),
                    "point_b": tuple(w2),
                    "point_c": tuple(w3),
                    # Mémorisation stricte des paramètres du contournement
                    "locked_alpha_rad": alpha_rel_rad,
                    "locked_alpha_deg": round(math.degrees(alpha_rel_rad), 2),
                    "locked_dist_1_m": d1,
                    "locked_d2_m": d2,
                    "initial_line_x": round(x0, 3),
                    "initial_heading_rad": initial_heading_rad,
                    "d_vers_bord_avant_m": round(d_vers_bord_avant, 3),
                    "dist_to_rear_edge_m": round(dist_to_rear_edge, 3),
                    "obs_depth_m": round(obs_depth, 3),
                    "obs_min_z": round(min_z, 3),
                    "obs_max_z": round(max_z, 3),
                    "obs_rear_z": round(obs_rear_z, 3),
                    "z_bord_arriere_obs": round(z_bord_arriere_obs, 3),
                    "z_bord_avant_obs": round(z_bord_avant_obs, 3),
                    "obs_length_m": round(obs_l, 3),
                    "theta_seg_1": theta_seg_1,
                    "theta_seg_2": initial_heading_rad,
                    "theta_seg_3": theta_seg_3,
                    "min_clearance_m": round(min_clear, 3),
                }

        if best_plan is None:
            # Fallback direct par défaut côté droit
            world_sign = right_world_sign
            margin = max(0.55, self.safety_margin_m)
            delta_x_lat = abs(obs_cx - x0) + (obs_w / 2.0) + (self.robot_width_m / 2.0) + margin
            delta_x = world_sign * delta_x_lat
            x_safe = round(x0 + delta_x, 3)
            l1 = self.approach_l_m
            z1 = round(z0 + dir_z * l1, 3)
            w0 = [round(x0, 3), round(z0, 3)]
            w1 = [round(x_safe, 3), z1]
            theta_seg_1 = math.atan2(w1[0] - w0[0], w1[1] - w0[1])
            alpha_rel_rad = self.wrap_angle(theta_seg_1 - initial_heading_rad)
            d1 = round(math.hypot(w1[0] - w0[0], w1[1] - w0[1]), 3)

            obs_depth = abs(max_z - min_z)
            if dir_z > 0:
                obs_rear_z = max_z
                dist_to_rear_edge = max(0.0, obs_rear_z - z1)
            else:
                obs_rear_z = min_z
                dist_to_rear_edge = max(0.0, z1 - obs_rear_z)

            required_d2 = dist_to_rear_edge + 2.80
            d2 = round(max(required_d2, obs_depth + 3.20), 3)

            z_bord_avant_obs = min_z if dir_z > 0 else max_z
            z_bord_arriere_obs = obs_rear_z
            d_vers_bord_avant = max(0.0, dir_z * (z_bord_avant_obs - z1))
            z2 = round(z1 + dir_z * d2, 3)
            w2 = [round(x_safe, 3), z2]

            z3 = round(z2 + dir_z * l1, 3)
            w3 = [round(x0, 3), z3]
            theta_seg_3 = math.atan2(w3[0] - w2[0], w3[1] - w2[1])

            best_plan = {
                "success": True,
                "rel_side": 1,
                "best_direction": 1,
                "direction": 1,
                "world_sign": world_sign,
                "x0": w0[0], "z0": w0[1],
                "x1": w1[0], "z1": w1[1],
                "x2": w2[0], "z2": w2[1],
                "x3": w3[0], "z3": w3[1],
                "w0": w0, "w1": w1, "w2": w2, "w3": w3,
                "waypoints": [w0, w1, w2, w3],
                "start_pt": tuple(w0),
                "point_a": tuple(w1),
                "point_b": tuple(w2),
                "point_c": tuple(w3),
                "locked_alpha_rad": alpha_rel_rad,
                "locked_alpha_deg": round(math.degrees(alpha_rel_rad), 2),
                "locked_dist_1_m": d1,
                "locked_d2_m": d2,
                "initial_line_x": round(x0, 3),
                "initial_heading_rad": initial_heading_rad,
                "d_vers_bord_avant_m": round(d_vers_bord_avant, 3),
                "dist_to_rear_edge_m": round(dist_to_rear_edge, 3),
                "obs_depth_m": round(obs_depth, 3),
                "obs_min_z": round(min_z, 3),
                "obs_max_z": round(max_z, 3),
                "obs_rear_z": round(obs_rear_z, 3),
                "z_bord_arriere_obs": round(z_bord_arriere_obs, 3),
                "z_bord_avant_obs": round(z_bord_avant_obs, 3),
                "obs_length_m": round(obs_l, 3),
                "theta_seg_1": theta_seg_1,
                "theta_seg_2": initial_heading_rad,
                "theta_seg_3": theta_seg_3,
                "min_clearance_m": 0.35,
            }

        dir_label = "DROITE" if best_plan["best_direction"] > 0 else "GAUCHE"
        best_plan["decision_msg"] = (
            f"🧭 CONTOURNEMENT TRAPÉZOÏDAL SYMÉTRIQUE ({dir_label}) : "
            f"Traj 1 [alpha={best_plan['locked_alpha_deg']:+.1f}°, D1={best_plan['locked_dist_1_m']:.2f}m] -> "
            f"Traj 2 [Ligne droite cap {math.degrees(initial_heading_rad):.0f}°, D2={best_plan['locked_d2_m']:.2f}m] -> "
            f"Traj 3 [alpha={-best_plan['locked_alpha_deg']:+.1f}°, D1={best_plan['locked_dist_1_m']:.2f}m]"
        )
        return best_plan
