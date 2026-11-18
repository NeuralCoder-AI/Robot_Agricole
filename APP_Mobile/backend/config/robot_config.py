"""backend/config/robot_config.py
Gestionnaire de Configuration Dynamique & Persistant des Dimensions du Robot Agricole.

Stocke les paramètres mécaniques et optiques dans SQLite (`robot_specs.sqlite`) ET JSON (`robot_specs.json`)
et recalcule dynamiquement toutes les grandeurs dérivées :
- `tool_half_offset_m = tool_spacing_m / 2.0`
- `central_guide_spacing_m = tool_spacing_m * 2.0`
- `chassis_half_width_m = chassis_width_m / 2.0`
- Waypoints trapézoïdaux (WP0..WP3), couloir de collision et optique caméra (auto_pitch, distance sol).
"""

import json
import logging
import math
import os
import sqlite3
import threading
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

try:
    from dotenv import load_dotenv
    _APP_ROOT = Path(__file__).resolve().parent.parent.parent
    _WORKSPACE_ROOT = _APP_ROOT.parent
    load_dotenv(_WORKSPACE_ROOT / ".env", override=False)
    load_dotenv(_APP_ROOT / ".env", override=False)
except Exception:
    pass

logger = logging.getLogger("RobotConfig")

CONFIG_DIR = Path(__file__).resolve().parent
SQLITE_SPECS_PATH = CONFIG_DIR / "robot_specs.sqlite"
JSON_SPECS_PATH = CONFIG_DIR / "robot_specs.json"

# Constantes agronomiques validées pour le Niger (Bakoye et al., 2020)
SAHEL_THEORETICAL_INTER_ROW_CM_RANGE = (30.0, 40.0)
MECHANICAL_NOZZLE_SPACING_CM = 46.0
SEED_DEPTH_TARGET_CM = 5.0
SAHEL_INTRA_ROW_SPACING_CM_DEFAULT = 15.0
INTRA_ROW_SPACING_CM_RANGE = (10.0, 20.0)


def get_deployment_target() -> str:
    """Retourne le profil de déploiement actif : `ROBOT_UC` (embarqué 100 % réel) ou `DEV_FULL` (PC dev)."""
    raw = os.getenv("DEPLOYMENT_TARGET", "DEV_FULL").strip().upper()
    return "ROBOT_UC" if raw == "ROBOT_UC" else "DEV_FULL"


def is_robot_uc_mode() -> bool:
    """True lorsque le serveur tourne sur l'Unité Centrale embarquée du robot (Zéro Simulation)."""
    return get_deployment_target() == "ROBOT_UC"


DEFAULT_DYNAMIC_SPECS: Dict[str, Any] = {
    # Dimensions mécaniques (en mètres)
    "chassis_width_m": 1.21,
    "chassis_length_m": 2.15,
    "nose_offset_m": 0.85,
    "rear_teeth_offset_m": 1.15,
    "tool_spacing_m": 0.46,
    "lateral_safety_margin_m": 0.38,
    "teeth_rear_clearance_m": 0.25,
    "corridor_safety_margin_m": 0.08,
    "approach_length_m": 1.10,
    # Paramètres optiques Caméra IA
    "camera_height_m": 0.90,
    "camera_fov_deg": 78.0,
    "camera_pitch_deg": 32.0,
    "auto_pitch": True,
    # Paramètres agronomiques Niger / Sahel (Bakoye et al., 2020)
    "seed_depth_cm": SEED_DEPTH_TARGET_CM,
    "intra_row_seed_spacing_cm": SAHEL_INTRA_ROW_SPACING_CM_DEFAULT,
    "agronomic_preset": "NIGER_SAHEL_OPTIMAL",
    # Paramètres Obstacles Mobiles & Fin de Ligne Intelligente
    "moving_obstacle_wait_sec": 5.0,
    "stop_threshold_m": 1.20,
    "line_end_confirm_dist_m": 0.50,
    "field_boundary_mode": "FIXED_DIMENSIONS",  # "FIXED_DIMENSIONS" ou "SMART_LINE_END"
}


class RobotConfigManager:
    """Gestionnaire thread-safe et persistant (SQLite + JSON) des dimensions dynamiques du robot."""

    def __init__(
        self,
        sqlite_path: Optional[Path] = None,
        json_path: Optional[Path] = None,
    ):
        self._lock = threading.RLock()
        self.sqlite_path: Path = Path(sqlite_path) if sqlite_path else SQLITE_SPECS_PATH
        self.json_path: Path = Path(json_path) if json_path else JSON_SPECS_PATH
        self._listeners: List[Callable[[Dict[str, Any]], None]] = []
        self._specs: Dict[str, Any] = {}
        self._init_storage()
        self._specs = self._load_from_storage()

    def _init_storage(self) -> None:
        """Initialise la table SQLite et le fichier JSON si absents."""
        try:
            self.sqlite_path.parent.mkdir(parents=True, exist_ok=True)
            with sqlite3.connect(str(self.sqlite_path)) as conn:
                conn.execute(
                    """
                    CREATE TABLE IF NOT EXISTS robot_specs (
                        key TEXT PRIMARY KEY,
                        value_json TEXT NOT NULL
                    )
                    """
                )
                conn.commit()
        except Exception as e:
            logger.warning("Erreur initialisation SQLite %s : %s", self.sqlite_path, e)

    @staticmethod
    def compute_derived_specs(raw: Dict[str, Any]) -> Dict[str, Any]:
        """Fusionne avec les valeurs par défaut et calcule toutes les dimensions dérivées 100 % dynamiques."""
        merged: Dict[str, Any] = dict(DEFAULT_DYNAMIC_SPECS)
        for k, v in (raw or {}).items():
            if v is None:
                continue
            if k in DEFAULT_DYNAMIC_SPECS:
                default_val = DEFAULT_DYNAMIC_SPECS[k]
                if isinstance(default_val, bool):
                    if isinstance(v, str):
                        merged[k] = v.strip().lower() in ("true", "1", "yes", "on")
                    else:
                        merged[k] = bool(v)
                elif isinstance(default_val, float):
                    merged[k] = round(float(v), 4)
                elif isinstance(default_val, int):
                    merged[k] = int(round(float(v)))
                else:
                    merged[k] = str(v)
            else:
                merged[k] = v

        # Bornes de sécurité physiques
        chassis_width_m = max(0.30, float(merged.get("chassis_width_m", 1.21)))
        chassis_length_m = max(0.50, float(merged.get("chassis_length_m", 2.15)))
        nose_offset_m = max(0.20, float(merged.get("nose_offset_m", 0.85)))
        rear_teeth_offset_m = max(0.20, float(merged.get("rear_teeth_offset_m", 1.15)))
        tool_spacing_m = max(0.15, float(merged.get("tool_spacing_m", 0.46)))
        lateral_safety_margin_m = max(0.10, float(merged.get("lateral_safety_margin_m", 0.38)))
        teeth_rear_clearance_m = max(0.05, float(merged.get("teeth_rear_clearance_m", 0.25)))
        corridor_safety_margin_m = max(0.01, float(merged.get("corridor_safety_margin_m", 0.08)))
        approach_length_m = max(0.40, float(merged.get("approach_length_m", 1.10)))
        camera_height_m = max(0.15, float(merged.get("camera_height_m", 0.90)))
        camera_fov_deg = max(30.0, min(175.0, float(merged.get("camera_fov_deg", 78.0))))
        camera_pitch_deg = max(5.0, min(85.0, float(merged.get("camera_pitch_deg", 32.0))))
        moving_obstacle_wait_sec = max(0.5, float(merged.get("moving_obstacle_wait_sec", 5.0)))
        stop_threshold_m = max(0.30, float(merged.get("stop_threshold_m", 1.20)))
        line_end_confirm_dist_m = max(0.10, float(merged.get("line_end_confirm_dist_m", 0.50)))
        seed_depth_cm = max(4.0, min(6.0, float(merged.get("seed_depth_cm", SEED_DEPTH_TARGET_CM))))
        intra_row_seed_spacing_cm = max(
            10.0,
            min(20.0, float(merged.get("intra_row_seed_spacing_cm", SAHEL_INTRA_ROW_SPACING_CM_DEFAULT))),
        )

        merged["chassis_width_m"] = round(chassis_width_m, 4)
        merged["chassis_length_m"] = round(chassis_length_m, 4)
        merged["nose_offset_m"] = round(nose_offset_m, 4)
        merged["rear_teeth_offset_m"] = round(rear_teeth_offset_m, 4)
        merged["tool_spacing_m"] = round(tool_spacing_m, 4)
        merged["lateral_safety_margin_m"] = round(lateral_safety_margin_m, 4)
        merged["teeth_rear_clearance_m"] = round(teeth_rear_clearance_m, 4)
        merged["corridor_safety_margin_m"] = round(corridor_safety_margin_m, 4)
        merged["approach_length_m"] = round(approach_length_m, 4)
        merged["camera_height_m"] = round(camera_height_m, 4)
        merged["camera_fov_deg"] = round(camera_fov_deg, 2)
        merged["camera_pitch_deg"] = round(camera_pitch_deg, 2)
        merged["moving_obstacle_wait_sec"] = round(moving_obstacle_wait_sec, 2)
        merged["stop_threshold_m"] = round(stop_threshold_m, 3)
        merged["line_end_confirm_dist_m"] = round(line_end_confirm_dist_m, 3)
        merged["seed_depth_cm"] = round(seed_depth_cm, 2)
        merged["intra_row_seed_spacing_cm"] = round(intra_row_seed_spacing_cm, 2)
        merged["sahel_theoretical_inter_row_cm_range"] = list(SAHEL_THEORETICAL_INTER_ROW_CM_RANGE)
        merged["mechanical_nozzle_spacing_cm"] = MECHANICAL_NOZZLE_SPACING_CM
        merged["configurable_tool_spacing_m_range"] = [0.30, 0.50]

        # Grandeurs dérivées dynamiques obligatoires
        merged["chassis_half_width_m"] = round(chassis_width_m / 2.0, 4)
        merged["tool_half_offset_m"] = round(tool_spacing_m / 2.0, 4)
        merged["central_guide_spacing_m"] = round(tool_spacing_m * 2.0, 4)
        merged["wp0_backup_offset_m"] = round(nose_offset_m + approach_length_m, 4)
        merged["wp1_front_offset_m"] = round(nose_offset_m, 4)
        merged["wp2_rear_clearance_m"] = round(rear_teeth_offset_m + teeth_rear_clearance_m, 4)

        # Synchronisation en centimètres pour compatibilité ascendante avec robot_specs
        merged["total_width_with_wheels_cm"] = round(chassis_width_m * 100.0, 1)
        merged["seeder_row_spacing_cm"] = round(tool_spacing_m * 100.0, 1)
        merged["camera_height_cm"] = round(camera_height_m * 100.0, 1)
        merged["camera_tilt_deg"] = round(camera_pitch_deg, 1)

        return merged

    def _load_from_storage(self) -> Dict[str, Any]:
        """Charge depuis SQLite en priorité, sinon JSON, sinon DEFAULT_DYNAMIC_SPECS."""
        loaded: Dict[str, Any] = {}
        # 1. Lecture SQLite
        if self.sqlite_path.is_file():
            try:
                with sqlite3.connect(str(self.sqlite_path)) as conn:
                    rows = conn.execute("SELECT key, value_json FROM robot_specs").fetchall()
                    for k, v_json in rows:
                        loaded[k] = json.loads(v_json)
            except Exception as e:
                logger.warning("Erreur lecture SQLite %s : %s", self.sqlite_path, e)

        # 2. Fallback JSON si SQLite vide
        if not loaded and self.json_path.is_file():
            try:
                with open(self.json_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                if isinstance(data, dict):
                    loaded.update(data)
            except Exception as e:
                logger.warning("Erreur lecture JSON %s : %s", self.json_path, e)

        computed = self.compute_derived_specs(loaded)
        self._save_to_storage(computed)
        return computed

    def _save_to_storage(self, specs: Dict[str, Any]) -> None:
        """Persiste atomiquement dans SQLite et JSON."""
        try:
            with sqlite3.connect(str(self.sqlite_path)) as conn:
                for k, v in specs.items():
                    conn.execute(
                        "INSERT OR REPLACE INTO robot_specs (key, value_json) VALUES (?, ?)",
                        (str(k), json.dumps(v, ensure_ascii=False)),
                    )
                conn.commit()
        except Exception as e:
            logger.warning("Erreur écriture SQLite %s : %s", self.sqlite_path, e)

        try:
            self.json_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.json_path, "w", encoding="utf-8") as f:
                json.dump(specs, f, indent=2, ensure_ascii=False)
        except Exception as e:
            logger.warning("Erreur écriture JSON %s : %s", self.json_path, e)

    def get_specs(self) -> Dict[str, Any]:
        """Retourne une copie complète des spécifications dynamiques actuelles."""
        with self._lock:
            return dict(self._specs)

    def update_specs(self, new_values: Dict[str, Any]) -> Dict[str, Any]:
        """Met à jour les dimensions dynamiques, les persiste et notifie tous les moteurs."""
        with self._lock:
            combined = dict(self._specs)
            # Support de clés en cm ou en m
            if "total_width_with_wheels_cm" in new_values and "chassis_width_m" not in new_values:
                combined["chassis_width_m"] = float(new_values["total_width_with_wheels_cm"]) / 100.0
            if "seeder_row_spacing_cm" in new_values and "tool_spacing_m" not in new_values:
                combined["tool_spacing_m"] = float(new_values["seeder_row_spacing_cm"]) / 100.0
            if "camera_height_cm" in new_values and "camera_height_m" not in new_values:
                combined["camera_height_m"] = float(new_values["camera_height_cm"]) / 100.0
            if "camera_tilt_deg" in new_values and "camera_pitch_deg" not in new_values:
                combined["camera_pitch_deg"] = float(new_values["camera_tilt_deg"])

            for k, v in (new_values or {}).items():
                if v is not None:
                    combined[k] = v

            self._specs = self.compute_derived_specs(combined)
            self._save_to_storage(self._specs)
            snapshot = dict(self._specs)

        self._notify_listeners(snapshot)
        return snapshot

    def reset_specs(self) -> Dict[str, Any]:
        """Réinitialise toutes les dimensions aux valeurs par défaut (DEFAULT_DYNAMIC_SPECS)."""
        with self._lock:
            self._specs = self.compute_derived_specs(DEFAULT_DYNAMIC_SPECS)
            self._save_to_storage(self._specs)
            snapshot = dict(self._specs)

        self._notify_listeners(snapshot)
        return snapshot

    def register_listener(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        if callback not in self._listeners:
            self._listeners.append(callback)

    def _notify_listeners(self, snapshot: Dict[str, Any]) -> None:
        for cb in list(self._listeners):
            try:
                cb(snapshot)
            except Exception as e:
                logger.debug("Erreur listener RobotConfig : %s", e)

    # Propriétés d'accès direct rapide
    @property
    def chassis_width_m(self) -> float:
        return float(self._specs.get("chassis_width_m", 1.21))

    @property
    def chassis_half_width_m(self) -> float:
        return float(self._specs.get("chassis_half_width_m", 0.605))

    @property
    def chassis_length_m(self) -> float:
        return float(self._specs.get("chassis_length_m", 2.15))

    @property
    def nose_offset_m(self) -> float:
        return float(self._specs.get("nose_offset_m", 0.85))

    @property
    def rear_teeth_offset_m(self) -> float:
        return float(self._specs.get("rear_teeth_offset_m", 1.15))

    @property
    def tool_spacing_m(self) -> float:
        return float(self._specs.get("tool_spacing_m", 0.46))

    @property
    def tool_half_offset_m(self) -> float:
        return float(self._specs.get("tool_half_offset_m", 0.23))

    @property
    def central_guide_spacing_m(self) -> float:
        return float(self._specs.get("central_guide_spacing_m", 0.92))

    @property
    def lateral_safety_margin_m(self) -> float:
        return float(self._specs.get("lateral_safety_margin_m", 0.38))

    @property
    def teeth_rear_clearance_m(self) -> float:
        return float(self._specs.get("teeth_rear_clearance_m", 0.25))

    @property
    def corridor_safety_margin_m(self) -> float:
        return float(self._specs.get("corridor_safety_margin_m", 0.08))

    @property
    def approach_length_m(self) -> float:
        return float(self._specs.get("approach_length_m", 1.10))

    @property
    def camera_height_m(self) -> float:
        return float(self._specs.get("camera_height_m", 0.90))

    @property
    def camera_fov_deg(self) -> float:
        return float(self._specs.get("camera_fov_deg", 78.0))

    @property
    def camera_pitch_deg(self) -> float:
        return float(self._specs.get("camera_pitch_deg", 32.0))

    @property
    def auto_pitch(self) -> bool:
        return bool(self._specs.get("auto_pitch", True))

    @property
    def moving_obstacle_wait_sec(self) -> float:
        return float(self._specs.get("moving_obstacle_wait_sec", 5.0))

    @property
    def seed_depth_cm(self) -> float:
        return float(self._specs.get("seed_depth_cm", SEED_DEPTH_TARGET_CM))

    @property
    def intra_row_seed_spacing_cm(self) -> float:
        return float(self._specs.get("intra_row_seed_spacing_cm", SAHEL_INTRA_ROW_SPACING_CM_DEFAULT))


# Singleton global partagé
robot_config = RobotConfigManager()

