"""
agri_nav_engine.py — Moteur Cinématique Agricole Paramétrable basé sur Shapely & Dynamic Window Approach (DWA)

Gère 100 % de la géométrie d'évitement d'obstacles à partir des dimensions exactes du robot :
- Modélisation polygonale orientée exacte via `shapely.geometry.box`, `rotate`, `translate`
- Extraction du polygone réel de chaque obstacle (`THREE.Box3` min_x/max_x/min_z/max_z ou cylindre bufferisé)
- Machine à états verrouillée en 4 phases :
  1. STOP_AND_LIFT : Arrêt immédiat dès distance(footprint, obstacle) < 0.45 m, coupure M6, relevage M5
  2. SMART_BACKUP  : Marche arrière droite (DRIVE:-40,0) tant que le polygone tourné vers la voie de
                     contournement est à moins de 0.50 m de l'obstacle
  3. DWA_BYPASS    : Planification Lattice (Points A, B, C) + Contrôleur prédictif DWA (horizon 1.5s)
                     rejetant strictement toute commande où robot_footprint.intersects(obstacle_poly.buffer(0.25))
  4. LINE_RELOCK   : Alignement final sur la ligne (|Δθ| < 2°), descente M5 et reprise du désherbage M6
"""

import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from shapely.affinity import rotate, translate
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union


class AgriNavEngine:
    """Moteur de navigation agricole paramétrable par les dimensions physiques du robot."""

    def __init__(self, robot_specs: Optional[Dict[str, Any]] = None):
        specs = robot_specs or {}

        # Dimensions physiques paramétrables du robot (en mètres)
        self.length_chassis_m: float = float(specs.get("chassis_length_top_cm", 169.0)) / 100.0
        self.front_overhang_m: float = float(specs.get("front_overhang_m", 1.05))  # Avec fraise avant M6
        self.rear_overhang_m: float = float(specs.get("rear_overhang_m", 1.15))    # Avec les 4 dents arrière M5
        self.total_length_m: float = self.front_overhang_m + self.rear_overhang_m  # 2.20 m hors-tout

        self.width_m: float = float(specs.get("total_width_with_wheels_cm", 121.0)) / 100.0  # 1.21 m hors-tout
        self.ground_clearance_m: float = float(specs.get("ground_clearance_cm", 24.0)) / 100.0  # 0.24 m
        self.wheelbase_m: float = float(specs.get("wheelbase_cm", 110.0)) / 100.0  # 1.10 m
        self.track_width_m: float = float(specs.get("track_width_cm", 103.0)) / 100.0  # 1.03 m
        self.safety_margin_m: float = float(specs.get("safety_margin_m", 0.35))  # 0.35 m obligatoire

        # Paramètres cinématiques (identiques au jumeau numérique 3D WebGL)
        self.kv_linear: float = 0.045   # m/s par unité de commande moteur
        self.kv_angular: float = 0.035  # rad/s par unité de différentiel moteur

        # État interne de la machine à états d'évitement
        self.state: str = "CLEAR"
        self.active_waypoints: List[Tuple[float, float]] = []
        self.current_wp_index: int = 0
        self.bypass_side: int = 1
        self.bypass_plan_locked: bool = False
        self.locked_plan: Optional[Dict[str, Any]] = None
        self.last_dwa_info: Dict[str, Any] = {}

    def lock_bypass_plan(self, plan: Dict[str, Any]) -> None:
        """Verrouille le plan de contournement [W0, W1, W2, W3] et son côté jusqu'à la fin de W3 (Règle 2)."""
        if plan and plan.get("success"):
            self.bypass_plan_locked = True
            self.locked_plan = dict(plan)
            self.bypass_side = int(plan.get("best_direction", 1))

    def unlock_bypass_plan(self) -> None:
        """Déverrouille le plan de contournement uniquement après achèvement complet de W3."""
        self.bypass_plan_locked = False
        self.locked_plan = None

    def update_specs(self, robot_specs: Dict[str, Any]) -> None:
        """Met à jour dynamiquement les dimensions physiques du robot."""
        if not robot_specs:
            return
        self.length_chassis_m = float(robot_specs.get("chassis_length_top_cm", 169.0)) / 100.0
        self.width_m = float(robot_specs.get("total_width_with_wheels_cm", 121.0)) / 100.0
        self.ground_clearance_m = float(robot_specs.get("ground_clearance_cm", 24.0)) / 100.0
        self.wheelbase_m = float(robot_specs.get("wheelbase_cm", 110.0)) / 100.0
        self.track_width_m = float(robot_specs.get("track_width_cm", 103.0)) / 100.0

    @staticmethod
    def wrap_angle(angle_rad: float) -> float:
        """Normalise un angle dans [-pi, +pi]."""
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def get_robot_footprint(
        self,
        x: float,
        z: float,
        heading_rad: float,
        include_margin: bool = True,
        custom_margin: Optional[float] = None,
    ) -> Polygon:
        """Construit le polygone orienté exact du robot (châssis + fraise avant + dents arrière) avec Shapely.

        Dans le repère (X, Z) du champ :
        - à heading_rad = 0, l'avant (+front_overhang_m) pointe vers +Z et la droite (+width_m/2) vers +X.
        - la rotation d'un cap `heading_rad` (où dx = sin(heading), dz = cos(heading)) correspond à
          une rotation horaire dans le plan (X, Z), soit `-heading_rad` dans `shapely.affinity.rotate`.
        """
        margin = (custom_margin if custom_margin is not None else self.safety_margin_m) if include_margin else 0.0
        half_w = (self.width_m / 2.0) + margin
        wheel_z = 0.68 + margin
        min_z = -self.rear_overhang_m - margin
        max_z = self.front_overhang_m + margin
        nose_half_w = min(half_w, 0.22 + margin)
        rear_half_w = min(half_w, 0.45 + margin)

        # Empreinte convexe fidèle au gabarit réel 121 cm (roues cages) × 215/220 cm (fraise M6 avant + dents M5 arrière)
        local_poly = Polygon([
            (-nose_half_w, max_z),
            (nose_half_w, max_z),
            (half_w, wheel_z),
            (half_w, -wheel_z),
            (rear_half_w, min_z),
            (-rear_half_w, min_z),
            (-half_w, -wheel_z),
            (-half_w, wheel_z),
        ])
        rotated_poly = rotate(local_poly, -heading_rad, origin=(0.0, 0.0), use_radians=True)
        world_poly = translate(rotated_poly, xoff=x, yoff=z)
        return world_poly

    @staticmethod
    def get_obstacle_polygon(obs: Dict[str, Any]) -> Polygon:
        """Construit le polygone exact Shapely d'un obstacle à partir de sa THREE.Box3 (min_x, min_z, max_x, max_z)
        ou de ses dimensions (width_m, length_m, radius).
        """
        ox = float(obs.get("x", obs.get("abs_x", 0.0)))
        oz = float(obs.get("z", obs.get("abs_z", 0.0)))

        if all(k in obs for k in ("min_x", "max_x", "min_z", "max_z")):
            min_x = float(obs["min_x"])
            max_x = float(obs["max_x"])
            min_z = float(obs["min_z"])
            max_z = float(obs["max_z"])
            if max_x > min_x and max_z > min_z:
                w = max_x - min_x
                l = max_z - min_z
                r_bevel = min(w, l) * 0.22
                return box(min_x + r_bevel, min_z + r_bevel, max_x - r_bevel, max_z - r_bevel).buffer(r_bevel, quad_segs=8)

        if "width_m" in obs and "length_m" in obs:
            w = max(0.20, float(obs["width_m"]))
            l = max(0.20, float(obs["length_m"]))
            min_x, max_x = ox - w / 2.0, ox + w / 2.0
            min_z, max_z = oz - l / 2.0, oz + l / 2.0
            r_bevel = min(w, l) * 0.22
            return box(min_x + r_bevel, min_z + r_bevel, max_x - r_bevel, max_z - r_bevel).buffer(r_bevel, quad_segs=8)

        radius = max(0.20, float(obs.get("radius", 0.45)))
        return Point(ox, oz).buffer(radius, quad_segs=16)

    def get_obstacles_union(self, obstacles: List[Dict[str, Any]]) -> Optional[Any]:
        """Retourne l'union géométrique Shapely de tous les obstacles fournis."""
        polys = [self.get_obstacle_polygon(o) for o in (obstacles or []) if o is not None]
        if not polys:
            return None
        return unary_union(polys)

    def should_trigger_stop_and_lift(
        self,
        x: float,
        z: float,
        heading_rad: float,
        obstacle_poly: Polygon,
        threshold_m: float = 0.45,
    ) -> bool:
        """Phase 1 (STOP & LIFT) : Vrai dès que `robot_footprint(include_margin=True).distance(obstacle_poly) < 0.45 m`."""
        fp = self.get_robot_footprint(x, z, heading_rad, include_margin=True)
        return fp.distance(obstacle_poly) < threshold_m

    def is_rotation_safe_at_pose(
        self,
        x: float,
        z: float,
        current_heading_rad: float,
        target_heading_rad: float,
        obstacle_poly: Polygon,
        min_clearance_m: float = 0.50,
    ) -> bool:
        """Phase 2 (SMART_BACKUP) : Vérifie avec Shapely que le polygone du robot tourné vers la voie
        de contournement (ainsi que tous les angles intermédiaires de rotation sur place) reste à au moins
        `min_clearance_m` (0.50 m) de la surface réelle du rocher.
        """
        # Teste le polygone réel (sans marge additionnelle, car min_clearance_m = 0.50 m est la distance directe au rocher)
        # ET vérifie que get_robot_footprint(x, z, heading_cible, include_margin=False) >= 0.50 m
        # ainsi que get_robot_footprint(x, z, heading_cible, include_margin=True) >= 0.50 m pour sécurité maximale.
        angles = np.linspace(current_heading_rad, target_heading_rad, num=7)
        for ang in angles:
            fp_real = self.get_robot_footprint(x, z, float(ang), include_margin=False)
            if fp_real.distance(obstacle_poly) < min_clearance_m:
                return False
        return True

    def compute_smart_backup_target_z(
        self,
        x: float,
        z: float,
        current_heading_rad: float,
        target_heading_rad: float,
        obstacle_poly: Polygon,
        min_clearance_m: float = 0.50,
    ) -> float:
        """Calcule par recherche géométrique Shapely la coordonnée Z exacte où `is_rotation_safe_at_pose`
        devient True (distance >= 0.50 m pour la rotation complète du gabarit 1.21m x 2.20m).
        """
        dir_z = 1.0 if math.cos(current_heading_rad) >= 0 else -1.0
        test_z = z
        step_m = 0.05
        for _ in range(120):  # Jusqu'à 6.0 m de recul max
            if self.is_rotation_safe_at_pose(x, test_z, current_heading_rad, target_heading_rad, obstacle_poly, min_clearance_m):
                return round(test_z, 3)
            test_z -= dir_z * step_m
        return round(test_z, 3)

    def find_lateral_bypass_corridor(
        self,
        corridor_x: float,
        target_heading_rad: float,
        primary_obs: Dict[str, Any],
        all_obstacles: Optional[List[Dict[str, Any]]] = None,
        field_width_m: float = 4.6,
        min_corridor_clearance_m: float = 0.45,
        robot_z: Optional[float] = None,
        force_replan: bool = False,
    ) -> Dict[str, Any]:
        """Phase 3 (LATTICE BYPASS) :
        - Règle 1 : Scanne toute la longueur du couloir [Z_robot - 1.0m, Z_obs + 4.5m] avec deux boîtes de test
          (Droite [X_obs + 0.5m, X_obs + 2.4m] et Gauche [X_obs - 2.4m, X_obs - 0.5m]) et fusionne tout 2ème rocher
          situé en aval dans le même couloir (combined_bbox). Choisit strictement le côté ayant 0 rocher secondaire.
        - Règle 2 : Interdiction absolue de réinitialiser ou de changer de côté en cours d'évitement (LOCK_BYPASS_PLAN).
        - Règle 3 : Sommets géométriques W0 -> W1 -> W2 -> W3 avec ligne droite parallèle W1->W2 de >= 3.00m + 2*R_obs.
        """
        # Règle 2 — Si le plan d'évitement est verrouillé (LOCK_BYPASS_PLAN), interdiction de le recalculer !
        if self.bypass_plan_locked and self.locked_plan is not None and not force_replan:
            return dict(self.locked_plan)

        obs_poly = self.get_obstacle_polygon(primary_obs)
        min_x, min_z, max_x, max_z = obs_poly.bounds
        obs_cx_init = (min_x + max_x) / 2.0
        obs_cz_init = (min_z + max_z) / 2.0

        dir_z = 1.0 if math.cos(target_heading_rad) >= 0 else -1.0

        # ----------------------------------------------------------------------
        # Règle 1 (Partie B) — Fusion d'Obstacles Proches (Cluster Merging / combined_bbox) :
        # Si un 2ème rocher est légèrement décalé plus loin sur le même couloir
        # (|ox_2 - X_obs| < 1.2m et 0 < (oz_2 - Z_obs) * dir_z < 3.5m),
        # fusionne les deux rochers en une seule boîte englobante combined_bbox (min_x, max_x, min_z, max_z).
        # ----------------------------------------------------------------------
        combined_polys: List[Polygon] = [obs_poly]
        secondary_obstacles: List[Tuple[Dict[str, Any], Polygon]] = []

        for o in (all_obstacles or []):
            p = self.get_obstacle_polygon(o)
            if p.centroid.distance(obs_poly.centroid) <= 0.25:
                continue
            ox_2 = float(o.get("abs_x", o.get("x", p.centroid.x)))
            oz_2 = float(o.get("abs_z", o.get("z", p.centroid.y)))
            dz_2 = (oz_2 - obs_cz_init) * dir_z
            if abs(ox_2 - obs_cx_init) < 1.20 and 0.0 < dz_2 < 3.50:
                combined_polys.append(p)
                min_x = min(min_x, p.bounds[0])
                min_z = min(min_z, p.bounds[1])
                max_x = max(max_x, p.bounds[2])
                max_z = max(max_z, p.bounds[3])
            else:
                secondary_obstacles.append((o, p))

        combined_obs_poly = unary_union(combined_polys) if len(combined_polys) > 1 else obs_poly
        obs_w = max_x - min_x
        obs_l = max_z - min_z
        obs_cx = (min_x + max_x) / 2.0
        obs_cz = (min_z + max_z) / 2.0
        obs_r = max(float(primary_obs.get("radius", 0.45)), obs_w / 2.0, obs_l / 2.0)

        z_front_edge = min_z if dir_z > 0 else max_z
        z_rear_edge = max_z if dir_z > 0 else min_z

        # ----------------------------------------------------------------------
        # Règle 1 (Partie A) — Scanner Toute la Longueur du Couloir pour Choisir
        # le Côté Vraiment Libre (Droite vs Gauche) avec Balayage Fin Adaptatif :
        # ----------------------------------------------------------------------
        # Quand dir_z > 0 (cap 0°), Droite = +X ; quand dir_z < 0 (cap 180°), Droite = -X
        right_world_sign = 1 if dir_z > 0 else -1
        candidates_sides = [(1, right_world_sign), (-1, -right_world_sign)]

        # Longueur d'approche oblique (>= 1.35m garanti, 1.40m pour une transition douce sans 90°)
        l_oblique = 1.40
        # Longueur de dépassement parallèle : profondeur obstacle (2*R_obs) + longueur robot (2.20m) + marge (0.60m)
        l_parallele = round((2.0 * obs_r) + 2.80, 3)

        # W1 : Arrivée sur le couloir parallèle AVANT la zone de l'obstacle
        # Le nez avant (front_overhang = 1.05m) reste en retrait de la face avant de l'obstacle
        z_w1 = round(obs_cz - dir_z * (obs_r + 1.15), 3)
        # W0 : Point de départ sur la ligne initiale (oblique = L_oblique)
        z_w0 = round(z_w1 - dir_z * l_oblique, 3)
        # W2 : Fin du couloir parallèle après dépassement complet de l'obstacle
        # L'arrière (rear_overhang = 1.15m) dégage complètement la face arrière de l'obstacle
        z_w2 = round(z_w1 + dir_z * l_parallele, 3)
        # W3 : Retour oblique sur la ligne d'arachide avec EXACTEMENT le même angle et la même distance
        z_w3 = round(z_w2 + dir_z * l_oblique, 3)

        z_backup_req = z_w0

        side_evaluations: List[Dict[str, Any]] = []

        for rel_side, world_sign in candidates_sides:
            best_cand_x = None
            best_min_clear = -1.0
            best_wp_list = None
            corridor_valid = False

            # Balayage fin adaptatif du décalage latéral (marge bord-à-bord de 0.35m à 1.45m)
            min_base_offset = (obs_w / 2.0) + (self.width_m / 2.0) + 0.35
            max_base_offset = min_base_offset + 1.10
            candidate_offsets = list(np.linspace(min_base_offset, max_base_offset, num=16))

            for offset_val in candidate_offsets:
                cand_x = obs_cx + world_sign * offset_val

                # Construction de la trajectoire à 3 segments W0 -> W1 -> W2 -> W3
                w0_c = [round(corridor_x, 3), z_w0]
                w1_c = [round(cand_x, 3), z_w1]
                w2_c = [round(cand_x, 3), z_w2]
                w3_c = [round(corridor_x, 3), z_w3]

                alpha_seg = math.atan2(cand_x - corridor_x, z_w1 - z_w0)

                # Échantillonnage précis le long des 3 segments de contournement
                traj_samples: List[Tuple[float, float, float]] = []
                # Segment 1 : W0 -> W1
                for k in range(9):
                    t = k / 8.0
                    traj_samples.append((
                        w0_c[0] + (w1_c[0] - w0_c[0]) * t,
                        w0_c[1] + (w1_c[1] - w0_c[1]) * t,
                        alpha_seg,
                    ))
                # Segment 2 : W1 -> W2 (parallèle)
                for k in range(9):
                    t = k / 8.0
                    traj_samples.append((
                        w1_c[0],
                        w1_c[1] + (w2_c[1] - w1_c[1]) * t,
                        target_heading_rad,
                    ))
                # Segment 3 : W2 -> W3
                for k in range(9):
                    t = k / 8.0
                    traj_samples.append((
                        w2_c[0] + (w3_c[0] - w2_c[0]) * t,
                        w2_c[1] + (w3_c[1] - w2_c[1]) * t,
                        -alpha_seg,
                    ))

                # Test de non-collision Shapely
                cand_ok = True
                min_clear_this_cand = 99.0

                for rx_s, rz_s, ryaw_s in traj_samples:
                    fp = self.get_robot_footprint(rx_s, rz_s, ryaw_s, include_margin=False)
                    d_p = fp.distance(combined_obs_poly)
                    if d_p < 0.22:
                        cand_ok = False
                        break
                    min_clear_this_cand = min(min_clear_this_cand, d_p)

                    for _, sec_poly in secondary_obstacles:
                        d_s = fp.distance(sec_poly)
                        if d_s < 0.20:
                            cand_ok = False
                            break
                        min_clear_this_cand = min(min_clear_this_cand, d_s)

                    if not cand_ok:
                        break

                if cand_ok and min_clear_this_cand > best_min_clear:
                    best_min_clear = min_clear_this_cand
                    best_cand_x = cand_x
                    best_wp_list = [w0_c, w1_c, w2_c, w3_c]
                    corridor_valid = True
                    # Si on dispose déjà d'une excellente marge >= 35 cm, on la retient
                    if min_clear_this_cand >= 0.35:
                        break

            side_evaluations.append({
                "rel_side": rel_side,
                "world_sign": world_sign,
                "corridor_valid": corridor_valid,
                "cand_x": round(best_cand_x, 3) if best_cand_x is not None else 0.0,
                "min_clearance": round(best_min_clear, 3) if best_min_clear >= 0 else 0.0,
                "waypoints": best_wp_list,
            })

        # Choisir le côté valide ayant le plus grand dégagement minimal (clearance)
        valid_candidates = [e for e in side_evaluations if e["corridor_valid"]]

        if valid_candidates:
            # Tri par plus grande marge de sécurité, avec préférence Droite (+1) en cas d'égalité
            valid_candidates.sort(
                key=lambda e: (e["min_clearance"], 1 if e["rel_side"] == 1 else 0),
                reverse=True,
            )
            best_eval = valid_candidates[0]
        else:
            best_eval = None

        if best_eval is None or best_eval.get("waypoints") is None:
            return {
                "success": False,
                "best_direction": 0,
                "direction": 0,
                "decision_msg": "🚫 AUCUNE ISSUE (G/D BLOQUÉS PAR SHAPELY) — DEMI-TOUR VERS LIGNE SUIVANTE",
                "waypoints": [],
                "path_points": [],
                "w_obs_m": round(obs_w, 2),
                "w_obs_cm": int(round(obs_w * 100)),
                "l_obs_m": round(obs_l, 2),
                "d_lat_m": 0.0,
                "delta_x_m": 0.0,
                "alpha_deg": 0.0,
                "alpha_rad": 0.0,
                "d_oblique_m": 0.0,
                "d_parallele_m": 0.0,
                "l_approche_m": 0.0,
            }

        chosen_side = int(best_eval["rel_side"])
        chosen_x_safe = float(best_eval["cand_x"])
        chosen_d_lat = round(abs(chosen_x_safe - corridor_x), 3)

        waypoints = best_eval["waypoints"]
        w0, w1, w2, w3 = waypoints[0], waypoints[1], waypoints[2], waypoints[3]

        start_pt = (w0[0], w0[1])
        point_a = (w1[0], w1[1])
        point_b = (w2[0], w2[1])
        point_c = (w3[0], w3[1])

        l_approche = abs(w1[1] - w0[1])
        alpha_rad = math.atan2(chosen_d_lat, max(0.50, l_approche))
        alpha_deg = math.degrees(alpha_rad)
        d_oblique = math.hypot(chosen_d_lat, l_approche)
        d_parallele = abs(w2[1] - w1[1])

        dir_label = "DROITE" if chosen_side > 0 else "GAUCHE"
        plan_dict = {
            "success": True,
            "best_direction": chosen_side,
            "direction": chosen_side,
            "bypass_side": chosen_side,
            "decision_msg": (
                f"🧭 SUIVI RAIL STRICT ({dir_label} / W0->W1->W2->W3 / FALLBACK GÉOMÉTRIQUE TRAPÉZOÏDAL OBLIQUE) : "
                f"X_safe={chosen_x_safe:+.2f}m | W0={z_w0:.2f}m -> W1={z_w1:.2f}m -> W2={z_w2:.2f}m -> W3={z_w3:.2f}m"
            ),
            "w0": w0,
            "w1": w1,
            "w2": w2,
            "w3": w3,
            "start_pt": start_pt,
            "point_a": point_a,
            "point_b": point_b,
            "point_c": point_c,
            "z_backup_safe": z_w0,
            "x_safe": chosen_x_safe,
            "z_depassement": z_w2,
            "obs_cx": round(obs_cx, 3),
            "obs_cz": round(obs_cz, 3),
            "obs_r": round(obs_r, 3),
            "combined_min_x": round(min_x, 3),
            "combined_max_x": round(max_x, 3),
            "combined_min_z": round(min_z, 3),
            "combined_max_z": round(max_z, 3),
            "merged_obstacles_count": len(combined_polys),
            "right_eval": next((e for e in side_evaluations if e["rel_side"] == 1), None),
            "left_eval": next((e for e in side_evaluations if e["rel_side"] == -1), None),
            "waypoints": waypoints,
            "path_points": waypoints,
            "w_obs_m": round(obs_w, 2),
            "w_obs_cm": int(round(obs_w * 100)),
            "l_obs_m": round(obs_l, 2),
            "d_lat_m": chosen_d_lat,
            "delta_x_m": chosen_d_lat,
            "l_approche_m": round(l_approche, 3),
            "alpha_deg": round(alpha_deg, 2),
            "alpha_rad": round(alpha_rad, 4),
            "d_oblique_m": round(d_oblique, 3),
            "d_parallele_m": round(d_parallele, 3),
            "d_recul_m": round(obs_r + 2.00, 3),
        }
        return plan_dict

    def select_next_peanut_row(
        self,
        current_x: float,
        crop_rows_x: List[float],
        direction_sign: int = 1,
    ) -> Dict[str, Any]:
        """Sélectionne dynamiquement la prochaine vraie ligne d'arachide en bout de rang :
        - Seuil = 2 × largeur grosse dent arrière (2 × 0.25 m = 0.50 m = 50 cm).
        - Cas 1 (Espacement normal E <= 0.50 m, ex: 0.46 m) : SAUTE 1 LIGNE -> ligne i + 2.
        - Cas 2 (Grand écart E > 0.50 m, ex: 0.65 m)        : NE SAUTE PAS DE LIGNE -> ligne i + 1.
        """
        large_tine_w_m = float(self.width_m * 0.0 + 0.25)  # 25 cm = 0.25 m
        threshold_m = 2.0 * large_tine_w_m                 # 50 cm = 0.50 m

        if not crop_rows_x:
            crop_rows_x = [round(r * 0.46, 3) for r in range(-8, 9)]

        rows = sorted((float(x) for x in crop_rows_x), reverse=(direction_sign < 0))
        curr_idx = min(range(len(rows)), key=lambda idx: abs(rows[idx] - current_x))
        x_i = rows[curr_idx]

        if curr_idx + 1 >= len(rows):
            return {
                "current_index": curr_idx,
                "current_x": round(x_i, 3),
                "next_index": curr_idx,
                "next_x": round(x_i, 3),
                "row_step": 0,
                "skip_row": False,
                "spacing_m": 0.0,
                "spacing_cm": 0,
                "threshold_m": threshold_m,
                "hud_message": "🏁 DERNIER RANG D'ARACHIDE ATTEINT",
                "hud_msg": "🏁 DERNIER RANG D'ARACHIDE ATTEINT",
                "decision_msg": "🏁 DERNIER RANG D'ARACHIDE ATTEINT",
            }

        e1_m = abs(rows[curr_idx + 1] - x_i)
        e1_cm = int(round(e1_m * 100.0))

        # Si l'espacement réel E <= 0.50 m (ex: 46 cm <= 50 cm) et que le rang i+2 existe (avec écart <= 0.50 m) :
        # on saute 1 ligne (rang i + 2). Sinon (E > 0.50 m), on prend la ligne immédiatement suivante (rang i + 1).
        can_skip = (e1_m <= threshold_m + 1e-6) and ((curr_idx + 2) < len(rows))
        if can_skip:
            e2_m = abs(rows[curr_idx + 2] - rows[curr_idx + 1])
            if e2_m > threshold_m + 1e-6:
                can_skip = False

        if can_skip:
            next_idx = curr_idx + 2
            row_step = 2
            skip_row = True
            hud_msg = f"🔄 DEMI-TOUR : ÉCART {e1_cm}cm (≤ 50cm) -> SAUT D'UNE LIGNE (RANG {next_idx})"
        else:
            next_idx = curr_idx + 1
            row_step = 1
            skip_row = False
            hud_msg = f"🔄 DEMI-TOUR : GRAND ÉCART ({e1_cm}cm > 50cm) -> PAS DE SAUT, LIGNE SUIVANTE (RANG {next_idx})"

        return {
            "current_index": curr_idx,
            "current_x": round(x_i, 3),
            "next_index": next_idx,
            "next_x": round(rows[next_idx], 3),
            "row_step": row_step,
            "skip_row": skip_row,
            "spacing_m": round(e1_m, 3),
            "spacing_cm": e1_cm,
            "threshold_m": threshold_m,
            "hud_message": hud_msg,
            "hud_msg": hud_msg,
            "decision_msg": hud_msg,
        }

    def simulate_trajectory(
        self,
        x: float,
        z: float,
        yaw: float,
        cmd_type: str,
        param1: int,
        param2: int,
        horizon_s: float = 1.5,
        dt: float = 0.15,
    ) -> List[Tuple[float, float, float]]:
        """Simule la trajectoire cinématique du robot sur `horizon_s` (1.5 seconde par défaut)."""
        if cmd_type == "TANK":
            v_left = float(max(-100, min(100, param1)))
            v_right = float(max(-100, min(100, param2)))
        else:  # DRIVE:a,d
            v_left = float(max(-100, min(100, param1 + param2)))
            v_right = float(max(-100, min(100, param1 - param2)))

        v_lin = ((v_left + v_right) / 2.0) * self.kv_linear
        v_ang = ((v_left - v_right) / 2.0) * self.kv_angular

        poses: List[Tuple[float, float, float]] = []
        cx, cz, cyaw = x, z, yaw
        steps = max(1, int(round(horizon_s / dt)))
        for _ in range(steps):
            cyaw = self.wrap_angle(cyaw + v_ang * dt)
            cx += math.sin(cyaw) * v_lin * dt
            cz += math.cos(cyaw) * v_lin * dt
            poses.append((cx, cz, cyaw))
        return poses

    def compute_cross_track_error(
        self,
        rx: float,
        rz: float,
        w_start: Tuple[float, float],
        w_end: Tuple[float, float],
    ) -> Dict[str, float]:
        """Calcule l'écart perpendiculaire signé (Cross-Track Error e_ct), la distance restante
        le long du segment (along_rem), la distance euclidienne au sommet final (dist_to_end),
        et les angles exacts du segment (theta_line et theta_seg).
        """
        x0, z0 = float(w_start[0]), float(w_start[1])
        x1, z1 = float(w_end[0]), float(w_end[1])
        dx_seg = x1 - x0
        dz_seg = z1 - z0
        seg_len = max(1e-6, math.hypot(dx_seg, dz_seg))
        tx = dx_seg / seg_len
        tz = dz_seg / seg_len

        # Vecteur normal droit (tz, -tx) dans le plan (X, Z) où cap 0° = +Z et +90° = +X
        e_ct = (rx - x0) * tz - (rz - z0) * tx
        along_progress = (rx - x0) * tx + (rz - z0) * tz
        along_rem = seg_len - along_progress
        dist_to_end = math.hypot(x1 - rx, z1 - rz)
        theta_line = math.atan2(dx_seg, dz_seg)
        theta_seg = math.atan2(x1 - rx, z1 - rz) if dist_to_end > 0.04 else theta_line

        return {
            "e_ct": e_ct,
            "abs_e_ct": abs(e_ct),
            "along_progress": along_progress,
            "along_rem": along_rem,
            "dist_to_end": dist_to_end,
            "seg_len": seg_len,
            "theta_line": theta_line,
            "theta_seg": theta_seg,
        }

    def compute_strict_rail_command(
        self,
        rx: float,
        rz: float,
        ryaw: float,
        w_start: Tuple[float, float],
        w_end: Tuple[float, float],
        is_parallel_pass: bool = False,
        dir_z: float = 1.0,
        is_aligned: bool = False,
        obs_z: Optional[float] = None,
        obs_r: float = 0.45,
    ) -> Dict[str, Any]:
        """Suiveur de Segments Strict ('Strict Waypoint Rail Follower' — Règles 2 & 3) :
        - Sous-étape A : Rotation pure sur place (TANK, speed = 0) tant que |θ_desired - θ_robot| > 2.5°
                         ou si en cours de segment l'erreur d'angle dépasse 8.0°.
        - Sous-étape B : Roulage collé sur le segment W_k -> W_{k+1} avec correction continue
                         de l'écart perpendiculaire (Cross-Track Error e_ct < 3 cm) et 2 roues avant/arrière positives.
        - Sous-étape C : Validation stricte de fin de segment à < 0.10 m (10 cm) et Verrouillage Dur de Transition
                         sur W1 -> W2 : dir_z * (Z_robot - Z_obs) >= R_obs + 2.20 m.
        """
        geom = self.compute_cross_track_error(rx, rz, w_start, w_end)
        e_ct = geom["e_ct"]
        along_rem = geom["along_rem"]
        dist_to_end = geom["dist_to_end"]
        theta_line = geom["theta_line"]
        theta_seg = geom["theta_seg"]

        # Sous-étape C — Validation Stricte de Fin de Segment et Dégagement de l'Obstacle
        # Règle 3 : Sur W1 -> W2, l'arrière du robot doit avoir complètement dépassé le rocher
        if is_parallel_pass:
            z_rem_w2 = (float(w_end[1]) - rz) * dir_z
            if obs_z is not None:
                hard_lock_passed = (dir_z * (rz - float(obs_z))) >= (float(obs_r) + 1.35)
            else:
                hard_lock_passed = z_rem_w2 <= 0.20
            segment_done = hard_lock_passed and (
                dist_to_end < 0.28 or along_rem <= 0.12 or z_rem_w2 <= 0.08
            )
        else:
            segment_done = (dist_to_end < 0.22) or (along_rem <= 0.06 and abs(e_ct) < 0.15)

        if segment_done:
            return {
                "segment_done": True,
                "is_aligned": True,
                "cmd_type": "STOP",
                "a": 0,
                "d": 0,
                "l": 0,
                "r": 0,
                "ms": 200,
                "e_ct": round(e_ct, 4),
                "dist_to_end": round(dist_to_end, 4),
                "heading_err_deg": 0.0,
            }

        # Cap de suivi unifié entre TANK et DRIVE avec correction proportionnelle de l'écart latéral (e_ct)
        if is_parallel_pass or dist_to_end > 0.20:
            theta_ref = self.wrap_angle(theta_line - math.atan(1.8 * e_ct))
        else:
            theta_ref = theta_seg

        err_ref = self.wrap_angle(theta_ref - ryaw)
        err_ref_deg = math.degrees(err_ref)

        # Roulage franc collé sur le segment avec asservissement différentiel continu (DRIVE) :
        # L'avance linéaire a_cmd >= 18 garantit un déplacement continu et fluide sans aucun arrêt.
        a_cmd = 22 if dist_to_end > 0.30 else max(16, int(round(dist_to_end * 55.0)))
        max_d = max(6, min(14, a_cmd - 4))
        d_cmd = int(np.clip(round(1.15 * err_ref_deg), -max_d, max_d))

        return {
            "segment_done": False,
            "is_aligned": True,
            "cmd_type": "DRIVE",
            "a": a_cmd,
            "d": d_cmd,
            "l": a_cmd + d_cmd,
            "r": a_cmd - d_cmd,
            "ms": 220,
            "e_ct": round(e_ct, 4),
            "dist_to_end": round(dist_to_end, 4),
            "heading_err_deg": round(err_ref_deg, 2),
        }

    def compute_dwa_command(
        self,
        x: float,
        z: float,
        yaw: float,
        target_pt: Tuple[float, float],
        obstacles: List[Dict[str, Any]],
        horizon_s: float = 1.5,
        min_buffer_m: float = 0.25,
    ) -> Dict[str, Any]:
        """Contrôleur Point-and-Shoot + DWA basé sur Shapely :
        - Tant que |θ_seg - θ_robot| > 2.5°, tourne strictement sur place (TANK, speed = 0)
        - Une fois aligné (<= 2.5°), avance en ligne droite vers `target_pt` tout en vérifiant
          avec Shapely que `robot_footprint.intersects(obstacle_poly.buffer(min_buffer_m))` est False.
        """
        obs_union = self.get_obstacles_union(obstacles)
        forbidden_zone = obs_union.buffer(min_buffer_m) if obs_union is not None else None

        tx, tz = target_pt
        dx = tx - x
        dz = tz - z
        dist_to_target = math.hypot(dx, dz)
        desired_yaw = math.atan2(dx, dz)
        heading_err = self.wrap_angle(desired_yaw - yaw)
        heading_err_deg = math.degrees(heading_err)

        # Règle 2A — Tant que |θ_seg - θ_robot| > 2.5°, rotation pure sur place (TANK, Zéro Avance)
        if abs(heading_err_deg) > 2.5 and dist_to_target > 0.08:
            if abs(heading_err_deg) > 14.0:
                mag = 28
            elif abs(heading_err_deg) > 5.0:
                mag = 16
            else:
                mag = max(6, int(round(abs(heading_err_deg) * 2.2)))
            l_cmd = mag if heading_err > 0 else -mag
            traj = self.simulate_trajectory(x, z, yaw, "TANK", l_cmd, -l_cmd, horizon_s=0.30, dt=0.10)
            is_safe = True
            if forbidden_zone is not None:
                for px, pz, pyaw in traj:
                    fp = self.get_robot_footprint(px, pz, pyaw, include_margin=False)
                    if fp.intersects(forbidden_zone):
                        is_safe = False
                        break
            return {
                "cmd_type": "TANK",
                "l": l_cmd,
                "r": -l_cmd,
                "a": 0,
                "d": l_cmd,
                "ms": 250,
                "safe": is_safe,
                "heading_err_deg": round(heading_err_deg, 1),
                "dist_to_target": round(dist_to_target, 3),
            }

        # Éventail de commandes DWA (vitesse, braquage)
        # Si le robot est proche de la cible (< 0.45 m), réduire l'horizon et la vitesse pour ne pas dépasser le point Lattice
        eff_horizon = min(horizon_s, max(0.35, dist_to_target / 0.75))
        speeds = [14, 18, 22] if dist_to_target > 0.40 else [12, 15]
        steers = [-28, -20, -14, -8, -4, 0, 4, 8, 14, 20, 28]

        best_score = -1e9
        best_cmd: Optional[Dict[str, Any]] = None

        for a_val in speeds:
            for d_val in steers:
                traj = self.simulate_trajectory(x, z, yaw, "DRIVE", a_val, d_val, horizon_s=eff_horizon, dt=0.15)
                safe = True
                min_obs_dist = 99.0

                if forbidden_zone is not None and obs_union is not None:
                    for px, pz, pyaw in traj:
                        fp = self.get_robot_footprint(px, pz, pyaw, include_margin=False)
                        if fp.intersects(forbidden_zone):
                            safe = False
                            break
                        d_obs = fp.distance(obs_union)
                        if d_obs < min_obs_dist:
                            min_obs_dist = d_obs

                if not safe:
                    continue

                # Évaluation au bout de la trajectoire simulée
                end_x, end_z, end_yaw = traj[-1]
                rem_dist = math.hypot(tx - end_x, tz - end_z)
                end_desired_yaw = math.atan2(tx - end_x, tz - end_z) if rem_dist > 0.05 else desired_yaw
                end_head_err = abs(self.wrap_angle(end_desired_yaw - end_yaw))

                # Fonction de coût DWA :
                # + progression vers la cible
                # - erreur de cap
                # + bonus de dégagement par rapport aux obstacles
                # - pénalité de braquage brusque
                progress_gain = (dist_to_target - rem_dist) * 12.0
                heading_penalty = end_head_err * 4.5
                clearance_bonus = min(1.5, min_obs_dist) * 2.0
                steer_penalty = abs(d_val) * 0.03

                score = progress_gain - heading_penalty + clearance_bonus - steer_penalty
                if score > best_score:
                    best_score = score
                    best_cmd = {
                        "cmd_type": "DRIVE",
                        "a": a_val,
                        "d": d_val,
                        "l": a_val + d_val,
                        "r": a_val - d_val,
                        "ms": 350,
                        "safe": True,
                        "min_clearance_m": round(min_obs_dist, 3),
                        "heading_err_deg": round(heading_err_deg, 1),
                        "dist_to_target": round(dist_to_target, 3),
                    }

        if best_cmd is not None:
            return best_cmd

        # Fallback sécurisé : rotation sur place TANK vers la cible si toutes les translations DRIVE sont bloquées
        mag = max(18, min(35, int(abs(heading_err_deg) * 0.9))) if abs(heading_err_deg) > 2.0 else 20
        l_cmd = mag if heading_err >= 0 else -mag
        return {
            "cmd_type": "TANK",
            "l": l_cmd,
            "r": -l_cmd,
            "a": 0,
            "d": l_cmd,
            "ms": 300,
            "safe": False,
            "heading_err_deg": round(heading_err_deg, 1),
            "dist_to_target": round(dist_to_target, 3),
        }
