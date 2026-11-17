"""
backend/navigation/reseeding_planner.py — Algorithme de Révision de la Semence (Resemence Ciblée).
Détection intelligente des trous de germination et des plants morts, calcul des points de resemence
chirurgicale à 20 cm, et déclenchement synchronisé du semoir M8 au passage des tuyaux arrière (-23 cm et +23 cm).
"""

import math
from typing import Any, Dict, List, Optional, Tuple


class ReseedingPlanner:
    """Planificateur et gestionnaire de Resemence Ciblée :
    - Scanne les rangs d'arachide sous les deux becs de semis (-23 cm et +23 cm).
    - Détecte les manques de germination (gaps > 35 cm) et les plants morts/desséchés.
    - Planifie les coordonnées précises des graines de remplacement (espacées de 20 cm).
    - Synchronise l'activation du semoir M8 avec la position réelle des tuyaux arrière.
    """

    def __init__(
        self,
        seeder_row_spacing_m: float = 0.46,
        seeder_rear_offset_m: float = 0.85,
        nominal_seed_gap_cm: float = 20.0,
        gap_detection_threshold_cm: float = 35.0,
        camera_lookahead_m: float = 1.80,
    ):
        self.seeder_row_spacing_m = seeder_row_spacing_m
        self.seeder_rear_offset_m = seeder_rear_offset_m  # Distance entre le centre du robot et les tuyaux M8
        self.nominal_seed_gap_cm = nominal_seed_gap_cm
        self.nominal_seed_gap_m = nominal_seed_gap_cm / 100.0
        self.gap_detection_threshold_m = gap_detection_threshold_cm / 100.0
        self.camera_lookahead_m = camera_lookahead_m

        # Compteurs statistiques de la mission
        self.detected_gaps_count: int = 0
        self.detected_dead_plants_count: int = 0
        self.reseeded_seeds_count: int = 0
        self.completed_rows_count: int = 0

        # Historique des graines resemées pour éviter tout doublon : Set[(round(x, 2), round(z, 2))]
        self._reseeded_positions: set = set()

    def reset_stats(self) -> None:
        """Réinitialise les métriques de la mission."""
        self.detected_gaps_count = 0
        self.detected_dead_plants_count = 0
        self.reseeded_seeds_count = 0
        self.completed_rows_count = 0
        self._reseeded_positions.clear()

    def get_seeder_nozzle_positions(
        self,
        robot_x: float,
        robot_z: float,
        robot_yaw_rad: float,
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """Calcule les positions exactes au sol des 2 tuyaux de semence arrière (M8).
        Tuyau gauche : -23 cm latéral.
        Tuyau droit : +23 cm latéral.
        Recul : -seeder_rear_offset_m par rapport au centre.
        """
        cos_y = math.cos(robot_yaw_rad)
        sin_y = math.sin(robot_yaw_rad)
        half_w = self.seeder_row_spacing_m / 2.0  # 0.23 m

        # Tuyau Gauche (lx = -0.23, lz = -offset)
        left_x = robot_x - half_w * cos_y - self.seeder_rear_offset_m * sin_y
        left_z = robot_z + half_w * sin_y - self.seeder_rear_offset_m * cos_y

        # Tuyau Droit (lx = +0.23, lz = -offset)
        right_x = robot_x + half_w * cos_y - self.seeder_rear_offset_m * sin_y
        right_z = robot_z - half_w * sin_y - self.seeder_rear_offset_m * cos_y

        return ((round(left_x, 3), round(left_z, 3)), (round(right_x, 3), round(right_z, 3)))

    def scan_row_plants(
        self,
        row_x: float,
        plants_on_row: List[Dict[str, Any]],
        dir_z: float = 1.0,
    ) -> Dict[str, Any]:
        """Analyse un rang d'arachide et identifie les trous et les plants morts.
        plants_on_row : liste de dict {'x': float, 'z': float, 'alive': bool, 'id': str}
        """
        # Trier selon la progression z
        sorted_plants = sorted(plants_on_row, key=lambda p: p["z"] * dir_z)
        gaps: List[Dict[str, Any]] = []
        dead_plants: List[Dict[str, Any]] = []
        planned_seeds: List[Dict[str, float]] = []

        for i, plant in enumerate(sorted_plants):
            if not plant.get("alive", True):
                dead_plants.append(plant)
                # Un plant mort nécessite une graine de remplacement à son emplacement exact
                planned_seeds.append({
                    "x": row_x,
                    "z": round(float(plant["z"]), 3),
                    "reason": "DEAD_PLANT",
                })

            if i < len(sorted_plants) - 1:
                next_p = sorted_plants[i + 1]
                dz = abs(float(next_p["z"]) - float(plant["z"]))
                if dz > self.gap_detection_threshold_m:
                    gap_start_z = float(plant["z"])
                    gap_end_z = float(next_p["z"])
                    gap_len = dz
                    gaps.append({
                        "row_x": row_x,
                        "start_z": round(gap_start_z, 3),
                        "end_z": round(gap_end_z, 3),
                        "length_m": round(gap_len, 3),
                    })
                    # Calculer les points de resemence espacés de 20 cm dans le trou
                    curr_z = gap_start_z + dir_z * self.nominal_seed_gap_m
                    while (dir_z * (gap_end_z - curr_z)) >= (self.nominal_seed_gap_m * 0.75):
                        planned_seeds.append({
                            "x": row_x,
                            "z": round(curr_z, 3),
                            "reason": "GAP_SEED",
                        })
                        curr_z += dir_z * self.nominal_seed_gap_m

        return {
            "row_x": row_x,
            "total_plants": len(sorted_plants),
            "dead_plants": dead_plants,
            "gaps": gaps,
            "planned_seeds": planned_seeds,
        }

    def evaluate_reseeding_at_pose(
        self,
        robot_x: float,
        robot_z: float,
        robot_yaw_rad: float,
        dir_z: float,
        nearby_plants: List[Dict[str, Any]],
        row_left_x: float,
        row_right_x: float,
    ) -> Dict[str, Any]:
        """Évalue l'état de resemence en temps réel à la position courante du robot.
        Retourne :
        - 'status': 'FULL_ROW' (vert), 'GAP_DETECTED' (magenta), 'RESEEDING_ACTIVE' (cyan)
        - 'recommended_speed': 32 (croisière) ou 18 (approche trou)
        - 'trigger_seeder': bool (True si les tuyaux arrière sont pile au-dessus d'un point à resemer)
        - 'seeds_to_drop': List[Dict[str, float]]
        - 'hud_message': str
        """
        left_nozzle, right_nozzle = self.get_seeder_nozzle_positions(robot_x, robot_z, robot_yaw_rad)

        # 1. Vérifier si un trou est visible devant par la caméra IA (lookahead 1.8 m)
        lookahead_z = robot_z + dir_z * self.camera_lookahead_m
        plants_ahead = [
            p for p in nearby_plants
            if 0.10 <= (dir_z * (float(p.get("z", 0.0)) - robot_z)) <= self.camera_lookahead_m
        ]

        has_dead_ahead = any(not p.get("alive", True) for p in plants_ahead)
        has_large_gap = False
        sorted_ahead = sorted(plants_ahead, key=lambda p: float(p.get("z", 0.0)) * dir_z)
        for i in range(len(sorted_ahead) - 1):
            if abs(float(sorted_ahead[i + 1]["z"]) - float(sorted_ahead[i]["z"])) > self.gap_detection_threshold_m:
                has_large_gap = True
                break

        # 2. Vérifier si les tuyaux arrière M8 passent actuellement au-dessus d'un emplacement à resemer
        seeds_to_drop: List[Dict[str, float]] = []
        trigger_seeder = False

        # Vérifier pour les 2 rangs (gauche et droite sous le châssis)
        for nozzle_x, nozzle_z in (left_nozzle, right_nozzle):
            # Chercher si une plante morte ou un trou se situe sous ce bec (tolérance longitudinale 12 cm)
            for p in nearby_plants:
                pz = float(p.get("z", 0.0))
                px = float(p.get("x", 0.0))
                if abs(px - nozzle_x) < 0.15 and abs(pz - nozzle_z) <= 0.12:
                    if not p.get("alive", True):
                        key = (round(px, 1), round(pz, 1))
                        if key not in self._reseeded_positions:
                            self._reseeded_positions.add(key)
                            self.reseeded_seeds_count += 1
                            trigger_seeder = True
                            seeds_to_drop.append({"x": px, "z": pz, "reason": "REPLACE_DEAD"})

        if trigger_seeder:
            status = "RESEEDING_ACTIVE"
            recommended_speed = 18
            hud_message = f"🎯 RESEMENCE CHIRURGICALE EN COURS ({len(seeds_to_drop)} graine(s))"
        elif has_dead_ahead or has_large_gap:
            status = "GAP_DETECTED"
            recommended_speed = 18
            hud_message = "🎯 ZONE VIDE / PLANTE MORTE DÉTECTÉE -> RESEMENCE"
        else:
            status = "FULL_ROW"
            recommended_speed = 32
            hud_message = "✅ RANG COMPLET"

        return {
            "status": status,
            "recommended_speed": recommended_speed,
            "trigger_seeder": trigger_seeder,
            "seeds_to_drop": seeds_to_drop,
            "hud_message": hud_message,
            "left_nozzle": left_nozzle,
            "right_nozzle": right_nozzle,
            "reseeded_count": self.reseeded_seeds_count,
            "gaps_count": self.detected_gaps_count,
            "dead_plants_count": self.detected_dead_plants_count,
        }

