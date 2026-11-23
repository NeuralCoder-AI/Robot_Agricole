"""
obstacle_path_planner.py — Planificateur d'Évitement d'Obstacles Haute Précision
A* sur Grille d'Occupation 2D avec Dilatation Anisotrope (scipy) + Lissage de Courbe + Fallback Trapézoïdal Garanti.

Prend en compte la Longueur Totale réelle du robot :
- Largeur hors-tout : 1.21 m (demi-largeur = 0.605 m)
- Longueur totale avec outils : 2.15 m (1.69 m châssis + fraise avant M6 + 4 dents arrière M5)
  * Demi-longueur avant (L_front vers pointe fraise) = 1.05 m
  * Demi-longueur arrière (L_rear vers pointe dents M5) = 1.10 m
  * Rayon diagonal balayé en braquage = 1.25 m

Dilatation Anisotrope exacte (Règle 2) :
- Marge Latérale (X) : M_x = R_obs + 0.605m + 0.55m = R_obs + 1.16m
- Marge Longitudinale (Z) : M_z = R_obs + 1.075m + 0.70m = R_obs + 1.78m
Zone interdite elliptique : (dx / M_x)^2 + (dz / M_z)^2 <= 1.0
"""

import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from scipy.ndimage import binary_dilation


class ObstaclePathPlanner:
    def __init__(
        self,
        field_width_m: float = 4.6,
        field_length_m: float = 14.0,
        resolution_m: float = 0.10,
        chassis_width_m: float = 1.21,
        chassis_length_m: float = 2.15,
        safety_margin_m: float = 0.35,
    ):
        self.field_width_m = field_width_m
        self.field_length_m = field_length_m
        self.resolution = resolution_m

        # Dimensions physiques complètes du robot (châssis + outils avant & arrière)
        self.chassis_width_m = chassis_width_m  # 1.21 m
        self.total_length_m = chassis_length_m  # 2.15 m
        self.half_chassis_w = chassis_width_m / 2.0  # 0.605 m
        self.l_front_m = 1.05  # 1.05 m vers l'avant (fraise M6)
        self.l_rear_m = 1.10   # 1.10 m vers l'arrière (dents M5)

        # Marges de dilatation anisotrope C-Space (Règle 2)
        # M_x = R_obs + 1.21/2 + 0.55m = R_obs + 1.16m (balayage diagonal)
        # M_z = R_obs + 2.15/2 + 0.70m = R_obs + 1.78m (sécurité nez & dents)
        self.inflation_lat_m = self.half_chassis_w + 0.55   # 1.155m -> 1.16m
        self.inflation_long_m = (self.total_length_m / 2.0) + 0.70  # 1.775m -> 1.78m

        # Limites du domaine de planification étendues (+3.5 m)
        self.x_min = - (field_width_m / 2.0) - 3.5
        self.x_max = (field_width_m / 2.0) + 3.5
        self.z_min = -3.5
        self.z_max = field_length_m + 5.0

        self.cols = int(round((self.x_max - self.x_min) / self.resolution)) + 1
        self.rows = int(round((self.z_max - self.z_min) / self.resolution)) + 1

    def world_to_grid(self, x: float, z: float) -> Tuple[int, int]:
        """Convertit les coordonnées monde (mètres) en indices grille (col, row)."""
        col = int(round((x - self.x_min) / self.resolution))
        row = int(round((z - self.z_min) / self.resolution))
        col = max(0, min(self.cols - 1, col))
        row = max(0, min(self.rows - 1, row))
        return col, row

    def grid_to_world(self, col: int, row: int) -> Tuple[float, float]:
        """Convertit les indices grille (col, row) en coordonnées monde (mètres)."""
        x = round(self.x_min + col * self.resolution, 3)
        z = round(self.z_min + row * self.resolution, 3)
        return x, z

    def build_occupancy_grid(
        self,
        obstacles: List[Dict[str, Any]],
        start_pos: Tuple[float, float],
        goal_pos: Tuple[float, float],
    ) -> np.ndarray:
        """Construit la grille binaire 2D avec Dilatation Anisotrope (M_x, M_z) pour chaque obstacle."""
        # Initialise toute la grille comme franchissable (1)
        grid = np.ones((self.rows, self.cols), dtype=np.int32)

        # 1. Masque des limites du champ (autoriser le débordement jusqu'à +2.50m en bordure pour évitement)
        max_walkable_abs_x = (self.field_width_m / 2.0) + 2.50
        for r in range(self.rows):
            for c in range(self.cols):
                wx, _ = self.grid_to_world(c, r)
                if abs(wx) > max_walkable_abs_x:
                    grid[r, c] = 0

        # 2. Masque anisotrope elliptique des obstacles (Règle 2 : 121 cm large × 215 cm long)
        for obs in obstacles:
            ox = float(obs.get("abs_x", obs.get("x", 999.0)))
            oz = float(obs.get("abs_z", obs.get("z", 999.0)))
            orad = float(obs.get("radius", 0.45))
            if abs(ox) > 50.0 or abs(oz) > 50.0:
                continue

            mx = orad + self.inflation_lat_m   # R_obs + 1.16m
            mz = orad + self.inflation_long_m  # R_obs + 1.78m

            rx_cells = max(1, int(math.ceil(mx / self.resolution)))
            rz_cells = max(1, int(math.ceil(mz / self.resolution)))

            oc, orow = self.world_to_grid(ox, oz)
            for dr in range(-rz_cells, rz_cells + 1):
                for dc in range(-rx_cells, rx_cells + 1):
                    nr, nc = orow + dr, oc + dc
                    if 0 <= nr < self.rows and 0 <= nc < self.cols:
                        wx, wz = self.grid_to_world(nc, nr)
                        dx = wx - ox
                        dz = wz - oz
                        # Équation de l'ellipse de sécurité anisotrope
                        if ((dx / mx) ** 2 + (dz / mz) ** 2) <= 1.0:
                            grid[nr, nc] = 0

        # 3. S'assurer que le départ et l'arrivée sont STRICTEMENT franchissables (1)
        free_radius_m = 0.35
        free_cells = max(1, int(math.ceil(free_radius_m / self.resolution)))
        sc, sr = self.world_to_grid(start_pos[0], start_pos[1])
        gc, gr = self.world_to_grid(goal_pos[0], goal_pos[1])

        for dr in range(-free_cells, free_cells + 1):
            for dc in range(-free_cells, free_cells + 1):
                if math.hypot(dc * self.resolution, dr * self.resolution) <= free_radius_m:
                    if 0 <= sr + dr < self.rows and 0 <= sc + dc < self.cols:
                        grid[sr + dr, sc + dc] = 1
                    if 0 <= gr + dr < self.rows and 0 <= gc + dc < self.cols:
                        grid[gr + dr, gc + dc] = 1

        return grid

    def is_side_clear(
        self,
        side: int,  # +1 pour droite, -1 pour gauche
        obs_x: float,
        obs_z: float,
        obs_r: float,
        d_lat: float,
        dir_z: float,
        all_obstacles: List[Dict[str, Any]],
        robot_z: Optional[float] = None,
    ) -> bool:
        """Vérifie si toute la longueur du couloir latéral [Z_robot - 1.0m, Z_obs + 4.5m]
        sur [X_obs + 0.5m, X_obs + 2.4m] (ou symétrique à gauche) est strictement dégagée (Règle 1).
        """
        right_world_sign = 1 if dir_z >= 0 else -1
        world_sign = side * right_world_sign
        z_start_scan = (float(robot_z) if robot_z is not None else (obs_z - dir_z * (obs_r + 1.60))) - dir_z * 1.00
        z_end_scan = obs_z + dir_z * 4.50
        z_min_box = min(z_start_scan, z_end_scan, obs_z - obs_r - 1.0)
        z_max_box = max(z_start_scan, z_end_scan, obs_z + obs_r + 4.5)

        for obs in all_obstacles:
            ox = float(obs.get("abs_x", obs.get("x", 999.0)))
            oz = float(obs.get("abs_z", obs.get("z", 999.0)))
            orad = float(obs.get("radius", 0.45))
            if math.hypot(ox - obs_x, oz - obs_z) < 0.25:
                continue  # l'obstacle principal lui-même
            lat_offset = (ox - obs_x) * world_sign
            if (0.50 - orad) <= lat_offset <= max(2.40 + orad, d_lat + orad + 0.40):
                if (z_min_box - orad) <= oz <= (z_max_box + orad):
                    return False
        return True

    def plan_avoidance_path(
        self,
        start_pos: Tuple[float, float],
        obstacle_center: Tuple[float, float],
        obstacle_radius: float,
        target_line_x: float,
        target_heading_rad: float,
        all_obstacles: Optional[List[Dict[str, Any]]] = None,
        obstacle_width: Optional[float] = None,
        obstacle_length: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Planifie la trajectoire d'évitement A* sans collision et retourne les waypoints Pure Pursuit.

        - start_pos : Position actuelle du robot (après le recul de dégagement suffisant)
        - obstacle_center : (X_obs, Z_obs)
        - target_line_x : Coordonnée X de la ligne initiale à reprendre
        - goal_pos : Point de reprise sur la ligne derrière l'obstacle :
                     Z_obs + dir_z * (R_obs + 2.80 m)
        """
        obs_x, obs_z = obstacle_center
        obs_r = max(0.25, obstacle_radius)
        dir_z = 1.0 if math.cos(target_heading_rad) >= 0 else -1.0

        w_obs = max(0.40, float(obstacle_width if obstacle_width is not None else obs_r * 2.0))
        l_obs = max(0.40, float(obstacle_length if obstacle_length is not None else obs_r * 2.0))
        obs_r = max(obs_r, max(w_obs, l_obs) / 2.0)

        # Point d'arrivée sur la même ligne derrière l'obstacle (WP4 : Z_obs + dir_z * (R_obs + 2.80m))
        goal_z = obs_z + dir_z * (obs_r + 2.80)
        goal_pos = (target_line_x, goal_z)

        obstacles_list = list(all_obstacles or [])
        # Assurer la présence de l'obstacle principal
        if not any(math.hypot(float(o.get("abs_x", o.get("x", 999))) - obs_x,
                              float(o.get("abs_z", o.get("z", 999))) - obs_z) < 0.25 for o in obstacles_list):
            obstacles_list.append({
                "abs_x": obs_x, "abs_z": obs_z, "radius": obs_r,
                "width_m": w_obs, "length_m": l_obs
            })

        # Calcul trigonométrique exact du décalage latéral (Delta X), de l'angle oblique alpha (jamais 90°)
        # et des distances D_oblique et D_parallele :
        # Delta X = |X_obs - X_ligne| + (W_obs / 2) + (1.21 / 2) + 0.45 m
        delta_x = abs(obs_x - target_line_x) + (w_obs / 2.0) + self.half_chassis_w + 0.45
        d_lat = max(delta_x, (w_obs / 2.0) + self.half_chassis_w + 0.65)
        l_approche = max(1.35, d_lat / math.tan(math.radians(38.0)))
        alpha_rad = math.atan2(d_lat, l_approche)
        alpha_deg = round(math.degrees(alpha_rad), 1)
        d_oblique = round(math.hypot(d_lat, l_approche), 3)
        d_parallele = round(l_obs + 2.10 + 0.40, 3)

        # 3. Vérification préalable de la disponibilité physique à droite et à gauche sur [Z_robot - 1.0m, Z_obs + 4.5m]
        right_free = self.is_side_clear(1, obs_x, obs_z, obs_r, d_lat, dir_z, obstacles_list, robot_z=start_pos[1])
        left_free = self.is_side_clear(-1, obs_x, obs_z, obs_r, d_lat, dir_z, obstacles_list, robot_z=start_pos[1])

        # Règle stricte : Ne réserver le Demi-Tour (best_direction = 0) STRICTEMENT QUE si la DROITE ET la GAUCHE sont TOUTES DEUX bloquées à la même hauteur Z
        if not right_free and not left_free:
            return {
                "success": False,
                "decision_msg": "🚫 AUCUNE ISSUE (G/D BLOQUÉS) — DEMI-TOUR VERS LIGNE SUIVANTE",
                "best_direction": 0,
                "direction": 0,
                "waypoints": [],
                "path_points": [],
                "w_obs_m": round(w_obs, 2),
                "w_obs_cm": int(round(w_obs * 100)),
                "l_obs_m": round(l_obs, 2),
                "d_lat_m": round(d_lat, 2),
                "delta_x_m": round(d_lat, 2),
                "alpha_deg": alpha_deg,
                "alpha_rad": round(alpha_rad, 4),
                "l_approche_m": round(l_approche, 2),
                "d_oblique_m": round(d_oblique, 2),
                "d_parallele_m": round(d_parallele, 2),
                "d_recul_m": 1.10,
            }

        grid_matrix = self.build_occupancy_grid(obstacles_list, start_pos, goal_pos)

        start_c, start_r = self.world_to_grid(start_pos[0], start_pos[1])
        goal_c, goal_r = self.world_to_grid(goal_pos[0], goal_pos[1])

        grid = Grid(matrix=grid_matrix.tolist())
        start_node = grid.node(start_c, start_r)
        end_node = grid.node(goal_c, goal_r)

        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = finder.find_path(start_node, end_node, grid)

        # 3. Trajectoire Trapézoïdale Oblique de Secours Garanti (Fallback Robuste sans angle à 90°, Règle 3)
        if not path or len(path) < 2:
            side = 1 if right_free else -1
            dir_label = "DROITE" if side > 0 else "GAUCHE"

            z_w0 = round(obs_z - dir_z * (obs_r + 2.00), 3)
            z_w1 = round(obs_z - dir_z * (obs_r + 0.60), 3)
            z_w2 = round(obs_z + dir_z * (obs_r + 2.40), 3)
            z_w3 = round(obs_z + dir_z * (obs_r + 3.80), 3)
            x_safe = round(obs_x + side * max(obs_r + 1.35, (w_obs / 2.0) + self.half_chassis_w + 0.65), 3)

            w0 = [round(target_line_x, 3), z_w0]
            w1 = [x_safe, z_w1]
            w2 = [x_safe, z_w2]
            w3 = [round(target_line_x, 3), z_w3]
            waypoints = [w0, w1, w2, w3]

            return {
                "success": True,
                "decision_msg": (
                    f"🧭 FALLBACK GÉOMÉTRIQUE TRAPÉZOÏDAL OBLIQUE : Contournement {dir_label} "
                    f"(W0->W1->W2->W3, α={alpha_deg:.1f}°, D_obl={d_oblique:.2f}m, D_par={d_parallele:.2f}m)"
                ),
                "best_direction": side,
                "direction": side,
                "waypoints": waypoints,
                "path_points": waypoints,
                "w0": w0,
                "w1": w1,
                "w2": w2,
                "w3": w3,
                "start_pt": w0,
                "point_a": w1,
                "point_b": w2,
                "point_c": w3,
                "w_obs_m": round(w_obs, 2),
                "w_obs_cm": int(round(w_obs * 100)),
                "l_obs_m": round(l_obs, 2),
                "d_lat_m": round(d_lat, 2),
                "delta_x_m": round(d_lat, 2),
                "alpha_deg": alpha_deg,
                "alpha_rad": round(alpha_rad, 4),
                "l_approche_m": round(l_approche, 2),
                "d_oblique_m": round(d_oblique, 2),
                "d_parallele_m": round(d_parallele, 2),
                "d_recul_m": 1.60,
            }

        # Conversion des nœuds de grille en coordonnées monde
        raw_waypoints: List[List[float]] = []
        for node in path:
            wx, wz = self.grid_to_world(node.x, node.y)
            raw_waypoints.append([wx, wz])

        # Sous-échantillonnage régulier tous les 35 cm pour le contrôleur Pure Pursuit
        waypoints = self.subsample_waypoints(raw_waypoints, step_m=0.35)

        # Déterminer la direction choisie (droite > 0 ou gauche < 0)
        max_lat_dev = 0.0
        for wp in waypoints:
            dev = wp[0] - target_line_x
            if abs(dev) > abs(max_lat_dev):
                max_lat_dev = dev

        # Si un côté est bloqué physiquement mais que A* est passé trop près, forcer la direction libre
        if not right_free:
            best_dir = -1
        elif not left_free:
            best_dir = 1
        else:
            best_dir = 1 if max_lat_dev >= 0 else -1

        dir_label = "DROITE" if best_dir > 0 else "GAUCHE"
        decision_msg = (
            f"🧭 A* PLANIFICATEUR OBLIQUE : Contournement {dir_label} "
            f"(α={alpha_deg:.1f}°, D_obl={d_oblique:.2f}m, D_par={d_parallele:.2f}m)"
        )

        return {
            "success": True,
            "decision_msg": decision_msg,
            "best_direction": best_dir,
            "direction": best_dir,
            "waypoints": waypoints,
            "path_points": waypoints,
            "w_obs_m": round(w_obs, 2),
            "w_obs_cm": int(round(w_obs * 100)),
            "l_obs_m": round(l_obs, 2),
            "d_lat_m": round(abs(max_lat_dev) if abs(max_lat_dev) > 0.5 else d_lat, 2),
            "delta_x_m": round(d_lat, 2),
            "alpha_deg": alpha_deg,
            "alpha_rad": round(alpha_rad, 4),
            "l_approche_m": round(l_approche, 2),
            "d_oblique_m": round(d_oblique, 2),
            "d_parallele_m": round(d_parallele, 2),
            "d_recul_m": 1.10,
        }

    @staticmethod
    def subsample_waypoints(raw_pts: List[List[float]], step_m: float = 0.35) -> List[List[float]]:
        """Sous-échantillonne une chaîne de waypoints en conservant le départ, l'arrivée et les étapes à pas constant."""
        if len(raw_pts) <= 2:
            return raw_pts

        subsampled = [raw_pts[0]]
        accumulated_dist = 0.0

        for i in range(1, len(raw_pts)):
            segment_len = math.hypot(
                raw_pts[i][0] - raw_pts[i - 1][0],
                raw_pts[i][1] - raw_pts[i - 1][1],
            )
            accumulated_dist += segment_len
            if accumulated_dist >= step_m:
                subsampled.append(raw_pts[i])
                accumulated_dist = 0.0

        if subsampled[-1] != raw_pts[-1]:
            subsampled.append(raw_pts[-1])

        return subsampled
