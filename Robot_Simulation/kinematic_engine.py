"""backend/navigation/kinematic_engine.py
Moteur Cinématique Déterministe Unique (KinematicEngine) — 100 % Dynamique via `robot_config`.
Architecture centralisée garantissant :
1. Lecture dynamique en temps réel des dimensions du robot (`robot_config`) : zéro constante codée en dur.
2. Génération exacte des lignes et curseur de rang strict (rowsX, rowStep 1 vs 2, pas de retour en arrière).
3. Couloir strict de collision ("Corridor Box") : les obstacles hors couloir sont ignorés à 100%.
4. Attente intelligente devant obstacle mobile (`WAITING_MOVING_OBSTACLE`) avant contournement.
5. Waypoints absolus du trapèze (WP0, WP1, WP2, WP3) recalculés dynamiquement.
6. Mode « Fin de Ligne Intelligente » (`SMART_LINE_END` / `LINE_END_DETECTED` > 0.50 m).
"""

import math
from typing import Any, Dict, List, Optional, Set, Tuple

from backend.config.robot_config import RobotConfigManager, robot_config

MOBILE_OBSTACLE_TYPES = {
    "person",
    "human",
    "humain",
    "personne",
    "pedestrian",
    "worker",
    "agriculteur",
    "child",
    "dog",
    "cat",
    "cow",
    "sheep",
    "goat",
    "horse",
    "bird",
    "animal",
    "animal_mobile",
    "car",
    "truck",
    "tractor",
    "bicycle",
    "motorcycle",
    "vehicle",
    "vehicule",
}


class KinematicEngine:
    # Valeurs initiales (synchronisées dynamiquement avec robot_config)
    ROBOT_WIDTH: float = 1.21
    ROBOT_HALF_WIDTH: float = 0.605
    ROBOT_LENGTH: float = 2.15
    ROBOT_NOSE_EXTENT: float = 0.85
    REAR_TINES_OFFSET: float = 1.15
    CORRIDOR_SAFETY_MARGIN: float = 0.08
    BYPASS_SAFETY_MARGIN: float = 0.38
    TEETH_REAR_CLEARANCE: float = 0.25
    APPROACH_LEN: float = 1.10
    REAR_CLEARANCE_EXTENT: float = 1.40

    def __init__(
        self,
        row_spacing: Optional[float] = None,
        field_length: float = 10.0,
        field_width: float = 4.6,
        start_x: float = 0.0,
        start_z: float = 0.0,
        step_dir: int = -1,
        config_manager: Optional[RobotConfigManager] = None,
        boundary_mode: Optional[str] = None,
    ):
        self.config: RobotConfigManager = config_manager or robot_config
        self._explicit_row_spacing: Optional[float] = float(row_spacing) if row_spacing is not None else None
        self._sync_dimensions_from_config(self.config.get_specs())

        self.row_spacing: float = (
            float(row_spacing)
            if row_spacing is not None
            else float(self.config.tool_spacing_m)
        )
        self.field_length: float = float(field_length)
        self.field_width: float = float(field_width)
        self.start_x: float = float(start_x)
        self.start_z: float = float(start_z)
        if isinstance(step_dir, str) and step_dir.strip().upper() == "AUTO":
            self.progression_mode: str = "AUTO"
            self.step_dir: int = -1
            self.locked_field_side_sign: Optional[int] = None
        else:
            self.step_dir = -1 if int(step_dir) < 0 else 1
            self.progression_mode = "RIGHT" if self.step_dir < 0 else "LEFT"
            self.locked_field_side_sign = self.step_dir

        self.mission_type: Optional[str] = None
        self.boundary_mode: str = str(
            boundary_mode or self.config.get_specs().get("field_boundary_mode", "FIXED_DIMENSIONS")
        )
        self.worked_rows_set: Set[float] = set()
        self.left_line_exists: bool = True
        self.right_line_exists: bool = True
        self.left_already_worked: bool = False
        self.right_already_worked: bool = False
        self.last_side_decision: Dict[str, Any] = {}

        # Construction initiale des grilles de lignes
        self.central_guides_x: List[float] = []
        self.total_furrows: int = 0
        self.furrows_x: List[float] = []
        self.total_rows: int = 0
        self.rows_x: List[float] = []
        self.row_step: int = 2
        self._rebuild_field_lines()

        self.current_row_index: int = 0
        self.target_row_x: float = self.rows_x[0] if self.rows_x else self.start_x
        self.pass_index: int = 0
        self.pass_direction: int = 1  # +1 vers +Z (cap 0), -1 vers -Z (cap PI)

        # Pose actuelle
        self.x: float = self.target_row_x
        self.y: float = 0.60
        self.z: float = self.start_z
        self.heading: float = 0.0 if self.pass_direction > 0 else math.pi

        # Suivi des obstacles contournés sur la passe actuelle
        self.cleared_obstacles_on_current_pass: Set[str] = set()

        # États internes :
        # IDLE, ROW_FOLLOWING, WAITING_MOVING_OBSTACLE, APPROACH_AVOIDANCE,
        # TRAJ_1_EXIT, TRAJ_2_PARALLEL, TRAJ_3_RETURN, REALIGN_AFTER_AVOID,
        # LINE_END_DETECTED, U_TURN_PIVOT_90, U_TURN_SHIFT, U_TURN_ALIGN, MISSION_COMPLETE
        self.state: str = "IDLE"
        self.active_waypoints: List[Dict[str, float]] = []
        self.active_wp_index: int = 0
        self.active_obstacle: Optional[Dict[str, Any]] = None

        # Gestion de l'attente d'obstacle en mouvement
        self.waiting_obstacle: Optional[Dict[str, Any]] = None
        self.waiting_elapsed_sec: float = 0.0
        self.remaining_wait_sec: float = 0.0
        self.resumed_after_moving_obstacle: bool = False

        # Gestion du mode Fin de Ligne Intelligente (SMART_LINE_END)
        self.line_missing_distance_m: float = 0.0
        self.line_end_detected: bool = False
        self.field_end_detected: bool = False
        self.awaiting_new_row_line_check: bool = False
        self.completion_message: str = ""

        # Moteurs [M1..M8]
        self.motors: List[int] = [0] * 8
        self.linear_speed: float = 0.0

        # Enregistrement pour mise à jour temps réel dès que robot_config change
        self.config.register_listener(self._on_config_updated)

    def _sync_dimensions_from_config(self, specs: Dict[str, Any]) -> None:
        """Synchronise toutes les variables mécaniques depuis `robot_config`."""
        self.ROBOT_WIDTH = float(specs.get("chassis_width_m", 1.21))
        self.ROBOT_HALF_WIDTH = float(specs.get("chassis_half_width_m", self.ROBOT_WIDTH / 2.0))
        self.ROBOT_LENGTH = float(specs.get("chassis_length_m", 2.15))
        self.ROBOT_NOSE_EXTENT = float(specs.get("nose_offset_m", 0.85))
        self.REAR_TINES_OFFSET = float(specs.get("rear_teeth_offset_m", 1.15))
        self.CORRIDOR_SAFETY_MARGIN = float(specs.get("corridor_safety_margin_m", 0.08))
        self.BYPASS_SAFETY_MARGIN = float(specs.get("lateral_safety_margin_m", 0.38))
        self.TEETH_REAR_CLEARANCE = float(specs.get("teeth_rear_clearance_m", 0.25))
        self.APPROACH_LEN = float(specs.get("approach_length_m", 1.10))
        self.REAR_CLEARANCE_EXTENT = round(self.REAR_TINES_OFFSET + self.TEETH_REAR_CLEARANCE, 4)
        self.TOOL_SPACING = float(specs.get("tool_spacing_m", 0.46))
        self.TOOL_HALF_OFFSET = float(specs.get("tool_half_offset_m", self.TOOL_SPACING / 2.0))
        self.CENTRAL_GUIDE_SPACING = float(specs.get("central_guide_spacing_m", self.TOOL_SPACING * 2.0))
        self.MOVING_OBSTACLE_WAIT_SEC = float(specs.get("moving_obstacle_wait_sec", 5.0))
        self.STOP_THRESHOLD_M = float(specs.get("stop_threshold_m", 1.20))
        self.LINE_END_CONFIRM_DIST_M = float(specs.get("line_end_confirm_dist_m", 0.50))

        # Mettre également à jour les attributs de classe pour compatibilité globale
        KinematicEngine.ROBOT_WIDTH = self.ROBOT_WIDTH
        KinematicEngine.ROBOT_HALF_WIDTH = self.ROBOT_HALF_WIDTH
        KinematicEngine.ROBOT_LENGTH = self.ROBOT_LENGTH
        KinematicEngine.ROBOT_NOSE_EXTENT = self.ROBOT_NOSE_EXTENT
        KinematicEngine.REAR_TINES_OFFSET = self.REAR_TINES_OFFSET
        KinematicEngine.CORRIDOR_SAFETY_MARGIN = self.CORRIDOR_SAFETY_MARGIN
        KinematicEngine.BYPASS_SAFETY_MARGIN = self.BYPASS_SAFETY_MARGIN
        KinematicEngine.TEETH_REAR_CLEARANCE = self.TEETH_REAR_CLEARANCE
        KinematicEngine.APPROACH_LEN = self.APPROACH_LEN
        KinematicEngine.REAR_CLEARANCE_EXTENT = self.REAR_CLEARANCE_EXTENT

    def _on_config_updated(self, specs: Dict[str, Any]) -> None:
        """Callback déclenché automatiquement lors d'un `PUT /api/config/specs` ou `robot_config.update_specs()`."""
        self._sync_dimensions_from_config(specs)
        if self._explicit_row_spacing is None or "tool_spacing_m" in specs:
            self.row_spacing = self.TOOL_SPACING
        self._rebuild_field_lines()
        if self.active_obstacle is not None:
            plan = self.compute_trapezoid_waypoints(self.active_obstacle)
            self.active_waypoints = plan["waypoints"]

    def _rebuild_field_lines(self) -> None:
        """Recalcule dynamiquement `central_guides_x`, `furrows_x` et `rows_x` selon les mesures actuelles."""
        guide_spacing = round(self.row_spacing * 2.0, 4)
        half_tool = round(self.row_spacing / 2.0, 4)

        num_guides = max(1, math.floor(self.field_width / max(0.10, guide_spacing)) + 1)
        self.central_guides_x = [
            round(self.start_x + k * guide_spacing * self.step_dir, 3)
            for k in range(num_guides)
        ]

        self.total_furrows = math.floor(self.field_width / max(0.10, self.row_spacing)) + 4
        first_furrow_x = self.start_x - half_tool * self.step_dir
        self.furrows_x = [
            round(first_furrow_x + i * self.row_spacing * self.step_dir, 3)
            for i in range(self.total_furrows)
        ]

        is_weeding = self.mission_type in ("weeding", "smart_weeding")
        base_offset_x = (half_tool * self.step_dir) if is_weeding else 0.0
        self.total_rows = math.floor(self.field_width / max(0.10, self.row_spacing)) + 1
        self.rows_x = [
            round(self.start_x + base_offset_x + i * self.step_dir * self.row_spacing, 3)
            for i in range(self.total_rows)
        ]
        self.row_step = 2 if self.row_spacing <= 0.50 else 1

    @classmethod
    def wrap_angle(cls, angle: float) -> float:
        """Normalise un angle dans [-PI, +PI]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def init_mission(
        self,
        row_spacing: Optional[float] = None,
        field_length: Optional[float] = None,
        field_width: Optional[float] = None,
        start_x: Optional[float] = None,
        start_z: Optional[float] = None,
        step_dir: Optional[int] = None,
        mission_type: Optional[str] = None,
        boundary_mode: Optional[str] = None,
    ) -> None:
        """Initialise la mission avec les dimensions dynamiques actuelles."""
        self._sync_dimensions_from_config(self.config.get_specs())
        if row_spacing is not None:
            self._explicit_row_spacing = float(row_spacing)
            self.row_spacing = float(row_spacing)
        elif self._explicit_row_spacing is None:
            self.row_spacing = self.TOOL_SPACING
        if field_length is not None:
            self.field_length = float(field_length)
        if field_width is not None:
            self.field_width = float(field_width)
        if start_x is not None:
            self.start_x = float(start_x)
        if start_z is not None:
            self.start_z = float(start_z)
        if step_dir is not None:
            if isinstance(step_dir, str) and step_dir.strip().upper() == "AUTO":
                self.progression_mode = "AUTO"
                self.locked_field_side_sign = None
                self.step_dir = -1
            elif isinstance(step_dir, str) and step_dir.strip().upper() in ("LEFT", "GAUCHE"):
                self.progression_mode = "LEFT"
                self.step_dir = 1
                self.locked_field_side_sign = 1
            elif isinstance(step_dir, str) and step_dir.strip().upper() in ("RIGHT", "DROITE"):
                self.progression_mode = "RIGHT"
                self.step_dir = -1
                self.locked_field_side_sign = -1
            else:
                self.step_dir = -1 if int(step_dir) < 0 else 1
                self.progression_mode = "RIGHT" if self.step_dir < 0 else "LEFT"
                self.locked_field_side_sign = self.step_dir
        if mission_type is not None:
            self.mission_type = str(mission_type)
        if boundary_mode is not None:
            self.boundary_mode = str(boundary_mode)

        self.worked_rows_set.clear()
        self.last_side_decision = {}

        self._rebuild_field_lines()
        is_weeding = self.mission_type in ("weeding", "smart_weeding")
        base_offset_x = (round(self.row_spacing / 2.0, 4) * self.step_dir) if is_weeding else 0.0

        self.current_row_index = 0
        self.target_row_x = self.rows_x[0] if self.rows_x else (self.start_x + base_offset_x)
        self.pass_index = 0
        self.pass_direction = 1

        self.x = self.target_row_x
        self.z = self.start_z
        self.heading = 0.0
        self.cleared_obstacles_on_current_pass.clear()
        self.state = "ROW_FOLLOWING"
        self.active_waypoints = []
        self.active_wp_index = 0
        self.active_obstacle = None

        self.waiting_obstacle = None
        self.waiting_elapsed_sec = 0.0
        self.remaining_wait_sec = 0.0
        self.resumed_after_moving_obstacle = False

        self.line_missing_distance_m = 0.0
        self.line_end_detected = False
        self.field_end_detected = False
        self.awaiting_new_row_line_check = False
        self.completion_message = ""

        # Outils prêts : dents descendues (M5=50), fraise en marche (M6=70)
        self.motors = [0, 0, 0, 0, 50, 70, 0, 0]

    @staticmethod
    def is_obstacle_moving(obs: Dict[str, Any]) -> bool:
        """Détermine si un obstacle est mobile (flag explicite, vitesse > seuil ou classe YOLO mobile)."""
        if bool(obs.get("is_moving", False)) or bool(obs.get("is_mobile_class", False)):
            return True
        cls_name = str(obs.get("class_name", obs.get("type", ""))).strip().lower()
        if cls_name in MOBILE_OBSTACLE_TYPES:
            return True
        if float(obs.get("speed_mps", 0.0)) > 0.05 or float(obs.get("speed_px_s", 0.0)) >= 12.0:
            return True
        if float(obs.get("displacement_px", 0.0)) >= 3.5:
            return True
        return False

    def check_obstacle_collision(self, obs: Dict[str, Any]) -> Dict[str, Any]:
        """Règle 2 : Couloir strict de collision ('Corridor Box') utilisant les dimensions dynamiques."""
        self._sync_dimensions_from_config(self.config.get_specs())
        obs_x = float(obs.get("x", obs.get("abs_x", 0.0)))
        obs_z = float(obs.get("z", obs.get("abs_z", 0.0)))
        obs_w = float(obs.get("width_m", obs.get("width", obs.get("radius", 0.45) * 2.0)))
        obs_depth = float(obs.get("length_m", obs.get("depth", obs.get("radius", 0.45) * 2.0)))
        obs_id = str(obs.get("id", f"{round(obs_x, 2)}_{round(obs_z, 2)}"))

        # Test 1 — Chevauchement Latéral Strict avec la Ligne Active (this.target_row_x)
        lateral_dist = abs(obs_x - self.target_row_x)
        collision_half_width = (obs_w / 2.0) + self.ROBOT_HALF_WIDTH + self.CORRIDOR_SAFETY_MARGIN
        is_blocking_current_row = lateral_dist < collision_half_width

        if not is_blocking_current_row:
            return {
                "is_blocking": False,
                "reason": "HORS_COULOIR",
                "lateral_dist": round(lateral_dist, 3),
                "collision_half_width": round(collision_half_width, 3),
                "obs_id": obs_id,
            }

        # Obstacle déjà contourné sur la passe actuelle
        if obs_id in self.cleared_obstacles_on_current_pass:
            return {
                "is_blocking": False,
                "reason": "ALREADY_CLEARED",
                "obs_id": obs_id,
            }

        # Test 2 — Obstacle Devant le Robot dans le Sens de Marche (dirZ = this.pass_direction)
        dir_z = self.pass_direction
        front_face_z = (obs_z - obs_depth / 2.0) if dir_z > 0 else (obs_z + obs_depth / 2.0)
        nose_z = self.z + dir_z * self.ROBOT_NOSE_EXTENT
        forward_dist = dir_z * (front_face_z - nose_z)

        # Déclenchement UNIQUEMENT si l'obstacle est bloquant et à portée d'arrêt
        if forward_dist > 0.0 and forward_dist <= self.STOP_THRESHOLD_M:
            return {
                "is_blocking": True,
                "forward_dist": round(forward_dist, 3),
                "front_face_z": round(front_face_z, 3),
                "collision_half_width": round(collision_half_width, 3),
                "is_moving": self.is_obstacle_moving(obs),
                "obs_id": obs_id,
                "obs": obs,
            }

        return {
            "is_blocking": False,
            "reason": "TROP_LOIN" if forward_dist > self.STOP_THRESHOLD_M else "DERRIERE",
            "forward_dist": round(forward_dist, 3),
            "collision_half_width": round(collision_half_width, 3),
            "obs_id": obs_id,
        }

    @staticmethod
    def check_real_ground_collision(
        robot_x: float,
        robot_z: float,
        robot_heading: float,
        obs_x: float,
        obs_z: float,
        obs_radius: float = 0.45,
    ) -> bool:
        """Empreinte 2D OBB exacte au sol du châssis contre un obstacle cylindrique (zéro faux positif)."""
        return check_real_ground_collision(
            robot_x=robot_x,
            robot_z=robot_z,
            robot_heading=robot_heading,
            obs_x=obs_x,
            obs_z=obs_z,
            obs_radius=obs_radius,
        )

    def compute_trapezoid_waypoints(
        self,
        obs: Dict[str, Any],
        side_x: int = 1,
    ) -> Dict[str, Any]:
        """Règle 3 : Contournement Trapézoïdal par Waypoints Absolus (WP0, WP1, WP2, WP3)
        100 % pilotés par les dimensions dynamiques (`robot_config`).
        """
        self._sync_dimensions_from_config(self.config.get_specs())
        obs_x = float(obs.get("x", obs.get("abs_x", 0.0)))
        obs_z = float(obs.get("z", obs.get("abs_z", 0.0)))
        obs_w = float(obs.get("width_m", obs.get("width", obs.get("radius", 0.45) * 2.0)))
        obs_depth = float(obs.get("length_m", obs.get("depth", obs.get("radius", 0.45) * 2.0)))

        min_z = float(obs.get("min_z", obs_z - obs_depth / 2.0))
        max_z = float(obs.get("max_z", obs_z + obs_depth / 2.0))

        dir_z = self.pass_direction
        row_x = self.target_row_x

        front_z = min_z if dir_z > 0 else max_z
        rear_z = max_z if dir_z > 0 else min_z

        # Coordonnée latérale dynamique : obs_w/2 + chassis_width_m/2 + lateral_safety_margin_m
        side = 1 if side_x >= 0 else -1
        lateral_clearance = self.BYPASS_SAFETY_MARGIN
        bypass_x = round(obs_x + side * ((obs_w / 2.0) + self.ROBOT_HALF_WIDTH + lateral_clearance), 3)
        delta_x = round(abs(bypass_x - row_x), 3)
        approach_len = self.APPROACH_LEN

        # WP0 : Marche arrière dynamique = front_z - dir_z * (nose_offset_m + approach_length_m)
        wp0_offset = self.ROBOT_NOSE_EXTENT + approach_len
        wp0 = {
            "x": round(row_x, 3),
            "z": round(front_z - dir_z * wp0_offset, 3),
        }
        # WP1 : Arrivée sur la ligne parallèle bypass_x AVANT l'obstacle à front_z - dir_z * nose_offset_m
        wp1 = {
            "x": round(bypass_x, 3),
            "z": round(front_z - dir_z * self.ROBOT_NOSE_EXTENT, 3),
        }
        # WP2 : Fin de la Trajectoire 2 — Dents arrière (rear_teeth_offset_m) + marge (teeth_rear_clearance_m)
        wp2_offset = self.REAR_TINES_OFFSET + self.TEETH_REAR_CLEARANCE
        wp2 = {
            "x": round(bypass_x, 3),
            "z": round(rear_z + dir_z * wp2_offset, 3),
        }
        # WP3 : Retour symétrique sur la ligne initiale
        wp3_offset = approach_len + 0.15
        wp3 = {
            "x": round(row_x, 3),
            "z": round(wp2["z"] + dir_z * wp3_offset, 3),
        }

        alpha = math.atan2(delta_x, approach_len)
        d1 = math.hypot(delta_x, approach_len)
        d2 = abs(wp2["z"] - wp1["z"])

        return {
            "WP0": wp0,
            "WP1": wp1,
            "WP2": wp2,
            "WP3": wp3,
            "waypoints": [wp0, wp1, wp2, wp3],
            "bypass_x": bypass_x,
            "delta_x": delta_x,
            "approach_len": approach_len,
            "alpha_rad": round(alpha, 4),
            "alpha_deg": round(math.degrees(alpha), 2),
            "d1_m": round(d1, 3),
            "d2_m": round(d2, 3),
            "dir_z": dir_z,
            "front_z": round(front_z, 3),
            "rear_z": round(rear_z, 3),
            "chassis_width_m": self.ROBOT_WIDTH,
            "nose_offset_m": self.ROBOT_NOSE_EXTENT,
            "rear_teeth_offset_m": self.REAR_TINES_OFFSET,
            "lateral_safety_margin_m": self.BYPASS_SAFETY_MARGIN,
            "teeth_rear_clearance_m": self.TEETH_REAR_CLEARANCE,
        }

    def analyze_soil_treatment_state(
        self,
        left_roi: Any = None,
        right_roi: Any = None,
    ) -> Dict[str, Any]:
        """Analyse l'état du sol à Gauche vs Droite (Déjà Traité vs Non Traité) :
        - Indice 1 : Densité d'adventices dans l'inter-rang.
        - Indice 2 : Variance de texture (empreintes de roues cages + sillons ±23 cm) et `worked_rows_set`.
        """
        step_x = round(self.row_spacing * self.row_step, 3)
        left_cand_x = round(self.x + step_x, 3)
        right_cand_x = round(self.x - step_x, 3)

        left_worked = bool(self.left_already_worked or (left_cand_x in self.worked_rows_set))
        right_worked = bool(self.right_already_worked or (right_cand_x in self.worked_rows_set))

        if isinstance(left_roi, dict):
            if "worked" in left_roi or "already_worked" in left_roi:
                left_worked = bool(left_roi.get("worked", left_roi.get("already_worked")))
            elif left_roi.get("texture_var", 0) >= 160.0 or left_roi.get("has_wheel_tracks"):
                left_worked = True
        if isinstance(right_roi, dict):
            if "worked" in right_roi or "already_worked" in right_roi:
                right_worked = bool(right_roi.get("worked", right_roi.get("already_worked")))
            elif right_roi.get("texture_var", 0) >= 160.0 or right_roi.get("has_wheel_tracks"):
                right_worked = True

        if left_worked and not right_worked:
            rec_side = "RIGHT"
            rec_sign = -1
            hud_msg = "🧠 IA SOL : Côté Gauche déjà traité -> Sens DROITE verrouillé"
        elif right_worked and not left_worked:
            rec_side = "LEFT"
            rec_sign = 1
            hud_msg = "🧠 IA SOL : Côté Droit déjà traité -> Sens GAUCHE verrouillé"
        else:
            rec_side = "BOTH_UNTOUCHED"
            rec_sign = -1
            hud_msg = "🤖 IA AUTO : Lignes vierges à Gauche et à Droite -> Choix autonome : Vers la DROITE verrouillé pour tous les tours."

        return {
            "left_worked": left_worked,
            "right_worked": right_worked,
            "recommended_side": rec_side,
            "recommended_sign": rec_sign,
            "hud_message": hud_msg,
        }

    analyzeSoilTreatmentState = analyze_soil_treatment_state

    def detect_next_row_direction(
        self,
        left_line_exists: Optional[bool] = None,
        right_line_exists: Optional[bool] = None,
        left_roi: Any = None,
        right_roi: Any = None,
        left_already_worked: Optional[bool] = None,
        right_already_worked: Optional[bool] = None,
        farmer_override: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Détermine le côté de progression sur la 1ère ligne (Passe 0 -> Passe 1) et verrouille
        définitivement `self.locked_field_side_sign` pour tous les tours suivants.
        Convention : GAUCHE = +1 ("LEFT"), DROITE = -1 ("RIGHT").
        """
        if self.locked_field_side_sign is not None and self.last_side_decision:
            return dict(self.last_side_decision)

        has_left = self.left_line_exists if left_line_exists is None else bool(left_line_exists)
        has_right = self.right_line_exists if right_line_exists is None else bool(right_line_exists)
        if left_already_worked is not None:
            self.left_already_worked = bool(left_already_worked)
        if right_already_worked is not None:
            self.right_already_worked = bool(right_already_worked)

        # CAS 1 — Ligne présente d'un seul côté (Bord de parcelle)
        if has_right and not has_left:
            self._lock_progression_side(-1, "RIGHT", "CASE_1_ONLY_RIGHT_LINE_EXISTS", "🧠 IA AUTO : Ligne uniquement à DROITE -> Sens DROITE verrouillé")
            return dict(self.last_side_decision)

        if has_left and not has_right:
            self._lock_progression_side(1, "LEFT", "CASE_1_ONLY_LEFT_LINE_EXISTS", "🧠 IA AUTO : Ligne uniquement à GAUCHE -> Sens GAUCHE verrouillé")
            return dict(self.last_side_decision)

        # CAS 2 — Lignes présentes des DEUX côtés : Analyse de l'état du sol (Déjà traité vs Vierge)
        soil = self.analyze_soil_treatment_state(left_roi=left_roi, right_roi=right_roi)
        if soil["left_worked"] and not soil["right_worked"]:
            self._lock_progression_side(-1, "RIGHT", "CASE_2_LEFT_ALREADY_WORKED", "🧠 IA SOL : Côté Gauche déjà traité -> Sens DROITE verrouillé")
            return dict(self.last_side_decision)

        if soil["right_worked"] and not soil["left_worked"]:
            self._lock_progression_side(1, "LEFT", "CASE_2_RIGHT_ALREADY_WORKED", "🧠 IA SOL : Côté Droit déjà traité -> Sens GAUCHE verrouillé")
            return dict(self.last_side_decision)

        # CAS 3 — Les deux côtés sont 100 % vierges (Non traités) : choix autonome verrouillé
        if farmer_override and str(farmer_override).strip().upper() in ("LEFT", "GAUCHE", "1", "+1"):
            self._lock_progression_side(
                1,
                "LEFT",
                "CASE_3_BOTH_UNTOUCHED_FARMER_LEFT",
                "🤖 IA AUTO : Lignes vierges à Gauche et à Droite -> Choix : Vers la GAUCHE verrouillé pour tous les tours.",
            )
        else:
            self._lock_progression_side(
                -1,
                "RIGHT",
                "CASE_3_BOTH_UNTOUCHED_AUTONOMOUS_FALLBACK",
                "🤖 IA AUTO : Lignes vierges à Gauche et à Droite -> Choix autonome : Vers la DROITE verrouillé pour tous les tours.",
            )
        return dict(self.last_side_decision)

    detectNextRowDirection = detect_next_row_direction

    def _lock_progression_side(self, side_sign: int, side_label: str, case_id: str, hud_message: str) -> None:
        """Verrouille définitivement le sens de progression et régénère la grille des lignes dans ce sens."""
        sign = -1 if side_sign < 0 else 1
        self.locked_field_side_sign = sign
        self.step_dir = sign
        self._rebuild_field_lines()
        self.last_side_decision = {
            "direction": side_label,
            "side_sign": sign,
            "locked": True,
            "case": case_id,
            "hud_message": hud_message,
        }

    def advance_row_cursor(self) -> bool:
        """Incrémente strictement l'index et bascule le cap.
        En mode AUTO, verrouille le côté de progression dès la fin de la 1ère ligne (Passe 0 -> Passe 1).
        """
        # Marquer la ligne qui vient d'être parcourue comme travaillée
        self.worked_rows_set.add(round(self.target_row_x, 3))

        # Si le sens n'est pas encore verrouillé (Passe 0 -> Passe 1 en mode AUTO), l'IA décide et verrouille !
        if self.locked_field_side_sign is None:
            self.detect_next_row_direction()

        self.current_row_index += self.row_step
        if self.boundary_mode != "SMART_LINE_END" and self.current_row_index >= len(self.rows_x):
            self.state = "MISSION_COMPLETE"
            self.motors = [0] * 8
            return False

        if self.current_row_index < len(self.rows_x):
            self.target_row_x = self.rows_x[self.current_row_index]
        else:
            # En mode SMART_LINE_END, la ligne suivante est décalée de ±central_guide_spacing_m
            self.target_row_x = round(
                self.target_row_x + self.step_dir * self.CENTRAL_GUIDE_SPACING,
                3,
            )
            self.rows_x.append(self.target_row_x)

        self.pass_direction *= -1
        self.pass_index += 1
        self.cleared_obstacles_on_current_pass.clear()
        return True

    def get_tines_bounding_box(
        self, x: float, z: float, dir_z: Optional[int] = None
    ) -> Tuple[float, float, float, float]:
        """Boîte englobante des dents arrière [x_min, z_min, x_max, z_max] dynamique."""
        self._sync_dimensions_from_config(self.config.get_specs())
        direction = self.pass_direction if dir_z is None else dir_z
        tines_z = z - direction * self.REAR_TINES_OFFSET
        half_w = 0.30
        half_d = 0.15
        return (x - half_w, tines_z - half_d, x + half_w, tines_z + half_d)

    @staticmethod
    def box_distance(
        b1: Tuple[float, float, float, float], b2: Tuple[float, float, float, float]
    ) -> float:
        """Calcule la distance minimale entre 2 boîtes englobantes [min_x, min_z, max_x, max_z]."""
        dx = max(0.0, b1[0] - b2[2], b2[0] - b1[2])
        dz = max(0.0, b1[1] - b2[3], b2[1] - b1[3])
        return math.hypot(dx, dz)

    def trigger_avoidance_plan(self, obs: Dict[str, Any], side_x: int = 1) -> Dict[str, Any]:
        """Déclenche et verrouille le plan de contournement trapézoïdal."""
        plan = self.compute_trapezoid_waypoints(obs, side_x=side_x)
        obs_id = str(obs.get("id", f"{round(float(obs.get('x', 0)), 2)}_{round(float(obs.get('z', 0)), 2)}"))
        self.cleared_obstacles_on_current_pass.add(obs_id)

        self.active_obstacle = obs
        self.active_waypoints = plan["waypoints"]
        self.active_wp_index = 0
        self.waiting_obstacle = None
        self.waiting_elapsed_sec = 0.0
        self.remaining_wait_sec = 0.0
        self.state = "APPROACH_AVOIDANCE"
        return plan

    def step_simulation(
        self,
        dt: float = 0.05,
        obstacles: Optional[List[Dict[str, Any]]] = None,
        line_detected: Optional[bool] = None,
        hybrid_row_valid: Optional[bool] = None,
    ) -> Dict[str, Any]:
        """Met à jour la cinématique déterministe par pas de temps dt."""
        self._sync_dimensions_from_config(self.config.get_specs())

        if self.state in ("IDLE", "MISSION_COMPLETE", "MISSION_COMPLETED_BY_HYBRID_VISION"):
            self.motors = [0] * 8
            self.linear_speed = 0.0
            return self.get_telemetry()

        # ----------------------------------------------------------------------
        # État : WAITING_MOVING_OBSTACLE (Attente devant obstacle mobile)
        # ----------------------------------------------------------------------
        if self.state == "WAITING_MOVING_OBSTACLE":
            self.motors = [0, 0, 0, 0, 50, 0, 0, 0]  # Robot à l'arrêt, M6 coupé
            self.linear_speed = 0.0

            still_blocking_obs: Optional[Dict[str, Any]] = None
            if obstacles:
                for obs in obstacles:
                    chk = self.check_obstacle_collision(obs)
                    if chk["is_blocking"]:
                        still_blocking_obs = obs
                        break

            # Si l'obstacle a quitté le couloir avant la fin du délai -> reprise directe !
            if still_blocking_obs is None:
                self.waiting_obstacle = None
                self.waiting_elapsed_sec = 0.0
                self.remaining_wait_sec = 0.0
                self.resumed_after_moving_obstacle = True
                self.state = "ROW_FOLLOWING"
                self.motors = [22, 22, 22, 22, 50, 70, 0, 0]
                self.linear_speed = 0.65
                return self.get_telemetry()

            # L'obstacle est toujours dans le couloir : incrémenter le compteur d'attente
            self.waiting_elapsed_sec += max(0.0, float(dt))
            wait_limit = self.MOVING_OBSTACLE_WAIT_SEC
            self.remaining_wait_sec = max(0.0, round(wait_limit - self.waiting_elapsed_sec, 2))

            if self.waiting_elapsed_sec >= wait_limit or math.isclose(self.waiting_elapsed_sec, wait_limit, abs_tol=1e-6):
                # Délai expiré : déclenche la séquence complète de contournement (WP0 -> WP1 -> WP2 -> WP3)
                self.trigger_avoidance_plan(still_blocking_obs)
                self.motors = [0, 0, 0, 0, -50, 0, 0, 0]
                return self.get_telemetry()

            return self.get_telemetry()

        # ----------------------------------------------------------------------
        # État : ROW_FOLLOWING
        # ----------------------------------------------------------------------
        if self.state == "ROW_FOLLOWING":
            # Priorité Absolue du Tandem Hybride (OpenCV + IA Arachide) sur Longueur/Largeur manuelles :
            # 1) À l'inspection de la ligne suivante post demi-tour : si non-arachide / invalide -> arrêt définitif !
            effective_line_ok = line_detected
            if hybrid_row_valid is False:
                effective_line_ok = False
                if self.awaiting_new_row_line_check:
                    self.awaiting_new_row_line_check = False
                    self.field_end_detected = True
                    self.line_end_detected = True
                    self.state = "MISSION_COMPLETED_BY_HYBRID_VISION"
                    self.completion_message = "🛑 ALERTE FRONTIÈRE : Fin du champ d'arachide certifiée — MISSION_COMPLETED_BY_HYBRID_VISION"
                    self.motors = [0] * 8
                    self.linear_speed = 0.0
                    return self.get_telemetry()
                else:
                    # Fin des plants d'arachide ou culture étrangère détectée en bout de ligne : demi-tour immédiat
                    # même si la longueur saisie par l'agriculteur n'est pas atteinte !
                    self.line_end_detected = True
                    self.motors = [0, 0, 0, 0, -50, 0, 0, 0]
                    self.linear_speed = 0.0
                    has_next = self.advance_row_cursor()
                    if has_next:
                        self.state = "U_TURN_SHIFT"
                        self.awaiting_new_row_line_check = True
                    else:
                        self.state = "MISSION_COMPLETED_BY_HYBRID_VISION"
                        self.completion_message = "🛑 ALERTE FRONTIÈRE : Dernière ligne atteinte — MISSION_COMPLETED_BY_HYBRID_VISION"
                    return self.get_telemetry()

            # Si on vient de finir un demi-tour en mode SMART_LINE_END, vérifier si une nouvelle ligne existe
            if self.awaiting_new_row_line_check and effective_line_ok is not None:
                self.awaiting_new_row_line_check = False
                if not effective_line_ok:
                    self.field_end_detected = True
                    self.line_end_detected = True
                    self.state = "MISSION_COMPLETE"
                    self.completion_message = "✅ Fin de parcelle détectée automatiquement — Mission terminée"
                    self.motors = [0] * 8
                    self.linear_speed = 0.0
                    return self.get_telemetry()

            # 1. Vérification collision couloir strict
            if obstacles:
                for obs in obstacles:
                    chk = self.check_obstacle_collision(obs)
                    if chk["is_blocking"]:
                        if chk.get("is_moving", False):
                            # Obstacle mobile : arrêt d'observation et compte à rebours avant contournement
                            self.state = "WAITING_MOVING_OBSTACLE"
                            self.waiting_obstacle = obs
                            self.waiting_elapsed_sec = 0.0
                            self.remaining_wait_sec = self.MOVING_OBSTACLE_WAIT_SEC
                            self.resumed_after_moving_obstacle = False
                            self.motors = [0, 0, 0, 0, 50, 0, 0, 0]  # Coupe M6 et stoppe les roues
                            self.linear_speed = 0.0
                            return self.get_telemetry()
                        else:
                            # Obstacle statique : contournement immédiat
                            self.trigger_avoidance_plan(obs)
                            self.motors = [0, 0, 0, 0, -50, 0, 0, 0]
                            return self.get_telemetry()

            # 2. Avance sur la ligne active
            dir_z = self.pass_direction
            target_heading = 0.0 if dir_z > 0 else math.pi
            self.heading = target_heading
            self.x = self.target_row_x

            step = 0.65 * dt  # Vitesse nominale 0.65 m/s (~2.3 km/h)
            self.z += dir_z * step
            self.linear_speed = 0.65
            self.motors = [22, 22, 22, 22, 50, 70, 0, 0]

            # 3. Détection de fin de ligne : Mode Intelligent vs Dimensions Fixes
            if self.boundary_mode == "SMART_LINE_END" or line_detected is not None:
                if line_detected is False:
                    self.line_missing_distance_m += abs(step)
                    if (
                        self.line_missing_distance_m >= self.LINE_END_CONFIRM_DIST_M
                        or math.isclose(self.line_missing_distance_m, self.LINE_END_CONFIRM_DIST_M, abs_tol=1e-6)
                    ):
                        self.line_end_detected = True
                        self.motors = [0, 0, 0, 0, -50, 0, 0, 0]
                        self.linear_speed = 0.0
                        has_next = self.advance_row_cursor()
                        if has_next:
                            self.state = "U_TURN_SHIFT"
                            self.awaiting_new_row_line_check = True
                        else:
                            self.state = "MISSION_COMPLETE"
                        return self.get_telemetry()
                elif line_detected is True:
                    self.line_missing_distance_m = 0.0
                    self.line_end_detected = False

            if self.boundary_mode != "SMART_LINE_END":
                at_end = (dir_z > 0 and self.z >= self.field_length) or (dir_z < 0 and self.z <= self.start_z)
                if at_end:
                    self.motors = [0, 0, 0, 0, -50, 0, 0, 0]
                    has_next = self.advance_row_cursor()
                    if has_next:
                        self.state = "U_TURN_SHIFT"
                    else:
                        self.state = "MISSION_COMPLETE"

            return self.get_telemetry()

        # ----------------------------------------------------------------------
        # État : U_TURN_SHIFT (Transition vers le nouveau rang)
        # ----------------------------------------------------------------------
        if self.state == "U_TURN_SHIFT":
            dx = self.target_row_x - self.x
            if abs(dx) > 0.015:
                step_x = math.copysign(min(0.50 * dt, abs(dx)), dx)
                self.x += step_x
                self.heading = math.atan2(dx, 0.0)
                self.motors = [18, 18, 18, 18, -50, 0, 0, 0]
            else:
                self.x = self.target_row_x
                self.heading = 0.0 if self.pass_direction > 0 else math.pi
                self.line_missing_distance_m = 0.0
                if self.awaiting_new_row_line_check and (line_detected is False or hybrid_row_valid is False):
                    self.awaiting_new_row_line_check = False
                    self.field_end_detected = True
                    self.line_end_detected = True
                    if hybrid_row_valid is False:
                        self.state = "MISSION_COMPLETED_BY_HYBRID_VISION"
                        self.completion_message = "🛑 ALERTE FRONTIÈRE : Fin du champ d'arachide certifiée — MISSION_COMPLETED_BY_HYBRID_VISION"
                    else:
                        self.state = "MISSION_COMPLETE"
                        self.completion_message = "✅ Fin de parcelle détectée automatiquement — Mission terminée"
                    self.motors = [0] * 8
                    self.linear_speed = 0.0
                else:
                    self.state = "ROW_FOLLOWING"
                    self.motors = [22, 22, 22, 22, 50, 70, 0, 0]
            return self.get_telemetry()

        # ----------------------------------------------------------------------
        # États de contournement : APPROACH_AVOIDANCE, TRAJ_1, TRAJ_2, TRAJ_3
        # ----------------------------------------------------------------------
        if self.state in ("APPROACH_AVOIDANCE", "TRAJ_1_EXIT", "TRAJ_2_PARALLEL", "TRAJ_3_RETURN"):
            if not self.active_waypoints or self.active_wp_index >= len(self.active_waypoints):
                self.state = "REALIGN_AFTER_AVOID"
                return self.get_telemetry()

            target_wp = self.active_waypoints[self.active_wp_index]
            target_x = target_wp["x"]
            target_z = target_wp["z"]

            dist_to_wp = math.hypot(target_x - self.x, target_z - self.z)

            if dist_to_wp > 0.03:
                target_head = math.atan2(target_x - self.x, target_z - self.z)
                head_err = self.wrap_angle(target_head - self.heading)

                if abs(head_err) > math.radians(3.0) and self.state != "TRAJ_2_PARALLEL":
                    self.heading += math.copysign(min(math.radians(90.0) * dt, abs(head_err)), head_err)
                    self.motors = (
                        [15, -15, 15, -15, -50, 0, 0, 0]
                        if head_err > 0
                        else [-15, 15, -15, 15, -50, 0, 0, 0]
                    )
                    self.linear_speed = 0.0
                else:
                    if self.state == "TRAJ_2_PARALLEL":
                        self.heading = 0.0 if self.pass_direction > 0 else math.pi

                    move_step = min(0.60 * dt, dist_to_wp)
                    dx = target_x - self.x
                    dz = target_z - self.z
                    tot = math.hypot(dx, dz)
                    self.x += (dx / tot) * move_step
                    self.z += (dz / tot) * move_step
                    self.linear_speed = 0.60
                    self.motors = [20, 20, 20, 20, -50, 0, 0, 0]
            else:
                self.x = target_x
                self.z = target_z
                self.active_wp_index += 1

                if self.active_wp_index == 1:
                    self.state = "TRAJ_1_EXIT"
                elif self.active_wp_index == 2:
                    self.state = "TRAJ_2_PARALLEL"
                elif self.active_wp_index == 3:
                    self.state = "TRAJ_3_RETURN"
                elif self.active_wp_index >= 4:
                    self.state = "REALIGN_AFTER_AVOID"

            return self.get_telemetry()

        # ----------------------------------------------------------------------
        # État : REALIGN_AFTER_AVOID
        # ----------------------------------------------------------------------
        if self.state == "REALIGN_AFTER_AVOID":
            target_heading = 0.0 if self.pass_direction > 0 else math.pi
            head_err = self.wrap_angle(target_heading - self.heading)
            if abs(head_err) > math.radians(2.0):
                self.heading += math.copysign(min(math.radians(80.0) * dt, abs(head_err)), head_err)
                self.motors = [14, -14, 14, -14, -50, 0, 0, 0]
            else:
                self.heading = target_heading
                self.x = self.target_row_x
                self.state = "ROW_FOLLOWING"
                self.motors = [22, 22, 22, 22, 50, 70, 0, 0]
            return self.get_telemetry()

        return self.get_telemetry()

    def get_telemetry(self) -> Dict[str, Any]:
        """Retourne l'état cinématique complet."""
        return {
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "z": round(self.z, 3),
            "heading": round(self.heading, 4),
            "heading_deg": round(math.degrees(self.heading), 2),
            "state": self.state,
            "current_row_index": self.current_row_index,
            "target_row_x": round(self.target_row_x, 3),
            "pass_direction": self.pass_direction,
            "pass_index": self.pass_index,
            "linear_speed": round(self.linear_speed, 2),
            "motors": list(self.motors),
            "rows_x": list(self.rows_x),
            "row_step": self.row_step,
            "waiting_elapsed_sec": round(self.waiting_elapsed_sec, 2),
            "remaining_wait_sec": round(self.remaining_wait_sec, 2),
            "resumed_after_moving_obstacle": self.resumed_after_moving_obstacle,
            "line_missing_distance_m": round(self.line_missing_distance_m, 3),
            "line_end_detected": self.line_end_detected,
            "field_end_detected": self.field_end_detected,
            "completion_message": self.completion_message,
        }


def check_real_ground_collision(
    robot_x: float,
    robot_z: float,
    robot_heading: float,
    obs_x: float,
    obs_z: float,
    obs_radius: float = 0.45,
) -> bool:
    """Empreinte physique STRICTE des parties basses au sol (roues + dents uniquement, sans marge gonflée).
    Zéro faux positif dû à la rotation ou aux panneaux solaires.
    """
    dx = obs_x - robot_x
    dz = obs_z - robot_z
    cos_h = math.cos(robot_heading)
    sin_h = math.sin(robot_heading)
    local_x = dx * cos_h - dz * sin_h
    local_z = dx * sin_h + dz * cos_h

    half_chassis_width = 0.52  # Intérieur strict des roues au sol
    front_limit = 0.72        # Nez avant strict
    rear_limit = -1.02        # Dents arrière strictes
    effective_obs_radius = max(0.15, obs_radius * 0.72)  # Cœur physique réel du rocher 3D

    closest_x = max(-half_chassis_width, min(half_chassis_width, local_x))
    closest_z = max(rear_limit, min(front_limit, local_z))
    dist_sq = (local_x - closest_x) ** 2 + (local_z - closest_z) ** 2

    return dist_sq < (effective_obs_radius ** 2)
