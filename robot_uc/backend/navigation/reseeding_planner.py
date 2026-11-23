"""
backend/navigation/reseeding_planner.py — Algorithme de Révision de la Semence (Resemence Ciblée)
et Géo-Mémorisation des Poquets inspirée du FarmDroid FD20 (`SeedMapMemory`).

- Prise en charge de la plage intra-rang sahélienne 10,0 à 20,0 cm (préréglage Niger 15,0 cm selon
  Bakoye et al., 2020 ; profondeur cible 5,0 cm dans la plage 4,0–6,0 cm ; compatibilité 20,0 cm).
- Enregistrement des coordonnées exactes (X_seed, Z_seed) des graines déposées par les buses M8 (±23 cm)
  afin de protéger les poquets en désherbage pré/post-émergence et cibler la resemence (FarmDroid FD20, 2026).
"""

import json
import math
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


SAHEL_INTRA_ROW_PRESET_CM = 15.0
SEED_DEPTH_TARGET_CM = 5.0


class SeedMapMemory:
    """Carte mémoire géo-référencée des graines déposées au semis (architecture type FarmDroid FD20)."""

    def __init__(self, storage_path: Optional[Path] = None):
        self.storage_path: Optional[Path] = Path(storage_path) if storage_path else None
        self.seeds: List[Dict[str, Any]] = []
        self._dedup_keys: set = set()
        if self.storage_path and self.storage_path.is_file():
            self.load()

    def clear(self) -> None:
        self.seeds.clear()
        self._dedup_keys.clear()
        if self.storage_path:
            self.save()

    def record_seed(
        self,
        x_m: float,
        z_m: float,
        row_index: int = 0,
        nozzle: str = "center",
        depth_cm: float = SEED_DEPTH_TARGET_CM,
        spacing_cm: float = SAHEL_INTRA_ROW_PRESET_CM,
        alive: bool = True,
    ) -> Optional[Dict[str, Any]]:
        """Enregistre une graine semée (X_seed,i, Z_seed,i) avec déduplication à 4 cm près."""
        rx = round(float(x_m), 3)
        rz = round(float(z_m), 3)
        key = (round(rx * 25.0), round(rz * 25.0))  # Maille ~4 cm
        if key in self._dedup_keys:
            return None
        self._dedup_keys.add(key)
        entry = {
            "seed_id": len(self.seeds) + 1,
            "x": rx,
            "z": rz,
            "row_index": int(row_index),
            "nozzle": str(nozzle),
            "depth_cm": round(max(4.0, min(6.0, float(depth_cm))), 2),
            "spacing_cm": round(max(10.0, min(20.0, float(spacing_cm))), 1),
            "alive": bool(alive),
            "timestamp": round(time.time(), 3),
        }
        self.seeds.append(entry)
        return entry

    def record_nozzle_pass(
        self,
        robot_x: float,
        robot_z: float,
        robot_yaw_rad: float,
        seeder_row_spacing_m: float = 0.46,
        seeder_rear_offset_m: float = 0.85,
        pass_index: int = 0,
        depth_cm: float = SEED_DEPTH_TARGET_CM,
        spacing_cm: float = SAHEL_INTRA_ROW_PRESET_CM,
    ) -> List[Dict[str, Any]]:
        """Calcule (X_seed, Z_seed) des deux tuyaux arrière M8 (-23 cm et +23 cm) et mémorise les poquets."""
        cos_y = math.cos(robot_yaw_rad)
        sin_y = math.sin(robot_yaw_rad)
        half_w = seeder_row_spacing_m / 2.0

        left_x = robot_x - half_w * cos_y - seeder_rear_offset_m * sin_y
        left_z = robot_z + half_w * sin_y - seeder_rear_offset_m * cos_y
        right_x = robot_x + half_w * cos_y - seeder_rear_offset_m * sin_y
        right_z = robot_z - half_w * sin_y - seeder_rear_offset_m * cos_y

        added: List[Dict[str, Any]] = []
        e_l = self.record_seed(left_x, left_z, row_index=pass_index * 2, nozzle="left", depth_cm=depth_cm, spacing_cm=spacing_cm)
        e_r = self.record_seed(right_x, right_z, row_index=pass_index * 2 + 1, nozzle="right", depth_cm=depth_cm, spacing_cm=spacing_cm)
        if e_l:
            added.append(e_l)
        if e_r:
            added.append(e_r)
        return added

    def query_nearby_seeds(self, x_m: float, z_m: float, radius_m: float = 1.80) -> List[Dict[str, Any]]:
        """Retourne les graines mémorisées dans un rayon métrique autour d'une position."""
        r2 = float(radius_m) ** 2
        return [
            s for s in self.seeds
            if ((s["x"] - x_m) ** 2 + (s["z"] - z_m) ** 2) <= r2
        ]

    def is_near_memorized_seed(self, x_m: float, z_m: float, protection_radius_m: float = 0.08) -> bool:
        """Indique si un outil à (x_m, z_m) empiète sur un poquet géo-mémorisé (protection pré/post-émergence)."""
        r2 = float(protection_radius_m) ** 2
        return any(((s["x"] - x_m) ** 2 + (s["z"] - z_m) ** 2) <= r2 for s in self.seeds)

    def save(self, path: Optional[Path] = None) -> None:
        target = Path(path) if path else self.storage_path
        if not target:
            return
        target.parent.mkdir(parents=True, exist_ok=True)
        with open(target, "w", encoding="utf-8") as f:
            json.dump({"seeds": self.seeds}, f, ensure_ascii=False, indent=2)

    def load(self, path: Optional[Path] = None) -> None:
        target = Path(path) if path else self.storage_path
        if not target or not target.is_file():
            return
        try:
            with open(target, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.seeds = list(data.get("seeds", []))
            self._dedup_keys = {
                (round(s["x"] * 25.0), round(s["z"] * 25.0)) for s in self.seeds
            }
        except Exception:
            self.seeds = []
            self._dedup_keys = set()

    def summary(self) -> Dict[str, Any]:
        return {
            "total_memorized_seeds": len(self.seeds),
            "last_seed": self.seeds[-1] if self.seeds else None,
        }


class ReseedingPlanner:
    """Planificateur et gestionnaire de Resemence Ciblée :
    - Scanne les rangs d'arachide sous les deux becs de semis (-23 cm et +23 cm).
    - Détecte les manques de germination (gaps > 35 cm) et les plants morts/desséchés.
    - Planifie les coordonnées précises des graines de remplacement (plage 10–20 cm ; préréglage Sahel 15 cm).
    - Synchronise l'activation du semoir M8 avec la position réelle des tuyaux arrière et SeedMapMemory.
    """

    def __init__(
        self,
        seeder_row_spacing_m: float = 0.46,
        seeder_rear_offset_m: float = 0.85,
        nominal_seed_gap_cm: float = 20.0,
        gap_detection_threshold_cm: float = 35.0,
        camera_lookahead_m: float = 1.80,
        seed_depth_cm: float = SEED_DEPTH_TARGET_CM,
        seed_map: Optional[SeedMapMemory] = None,
    ):
        self.seeder_row_spacing_m = seeder_row_spacing_m
        self.seeder_rear_offset_m = seeder_rear_offset_m  # Distance entre le centre du robot et les tuyaux M8
        self.nominal_seed_gap_cm = max(10.0, min(20.0, float(nominal_seed_gap_cm)))
        self.nominal_seed_gap_m = self.nominal_seed_gap_cm / 100.0
        self.gap_detection_threshold_m = gap_detection_threshold_cm / 100.0
        self.camera_lookahead_m = camera_lookahead_m
        self.seed_depth_cm = max(4.0, min(6.0, float(seed_depth_cm)))
        self.seed_map: SeedMapMemory = seed_map if seed_map is not None else SeedMapMemory()

        # Compteurs statistiques de la mission
        self.detected_gaps_count: int = 0
        self.detected_dead_plants_count: int = 0
        self.reseeded_seeds_count: int = 0
        self.completed_rows_count: int = 0

        # Historique des graines resemées pour éviter tout doublon : Set[(round(x, 2), round(z, 2))]
        self._reseeded_positions: set = set()

    @classmethod
    def from_niger_sahel_preset(
        cls,
        seeder_row_spacing_m: float = 0.46,
        seed_map: Optional[SeedMapMemory] = None,
    ) -> "ReseedingPlanner":
        """Préréglage agronomique nigérien conforme à Bakoye et al. (2020) : 15 cm intra-rang, 5 cm profondeur."""
        return cls(
            seeder_row_spacing_m=seeder_row_spacing_m,
            seeder_rear_offset_m=0.85,
            nominal_seed_gap_cm=SAHEL_INTRA_ROW_PRESET_CM,
            gap_detection_threshold_cm=30.0,
            camera_lookahead_m=1.80,
            seed_depth_cm=SEED_DEPTH_TARGET_CM,
            seed_map=seed_map,
        )

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

