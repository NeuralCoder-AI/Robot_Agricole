"""backend/vision/yolo_pipeline.py
Pipeline Vision IA Avancé par YOLO (YOLOv8n / YOLO11n `model.track(persist=True)`) + Extracteur Géométrique :
1. Détection multi-missions (SEEDING, WEEDING, RESEEDING, OBSTACLE_AVOIDANCE).
2. Auto-calibration de l'angle d'inclinaison caméra (theta_pitch) par Point de Fuite (y_vp) :
   theta_pitch = arctan((y_vp - c_y) / f_y), avec f_y = (H / 2) / tan(FOV_v / 2).
3. Calcul exact de la distance longitudinale au sol (D_sol) et latérale (X_sol) par le pied de boîte (y_bottom) :
   D_sol = camera_height_m / tan(theta_pitch + arctan((y_bottom - c_y) / f_y))
   X_sol = D_sol * (x_center - c_x) / f_x
4. Logique « Vérifier si l'Obstacle est en Mouvement avant de Contourner » (WAITING_MOVING_OBSTACLE) :
   - Arrêt immédiat (STOP + coupure M6).
   - Analyse du mouvement du track_id (caméra fixe) ou de la classe mobile (personne, animal, véhicule).
   - Compte à rebours dynamique `moving_obstacle_wait_sec` (ex: 5.0 s) :
     * Si l'obstacle quitte le couloir avant la fin -> reprise directe de la ligne (WORKING) sans contournement !
     * Si le délai expire (ou obstacle statique) -> séquence complète de contournement (WP0 -> WP1 -> WP2 -> WP3).
5. Détection intelligente de Fin de Ligne (`LINE_END_DETECTED` après > 0.50 m sans ligne en bas d'image)
   et Fin de Champ (`FIELD_END_DETECTED` après décalage latéral sans nouvelle ligne).
"""

import logging
import math
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from backend.config.robot_config import RobotConfigManager, robot_config
from backend.vision.real_vision_pipeline import RealVisionPipeline

logger = logging.getLogger("YoloVisionPipeline")

# Import optionnel d'Ultralytics YOLO (bascule automatiquement sur le tracker hybride si absent)
try:
    from ultralytics import YOLO as UltralyticsYOLO  # type: ignore
    ULTRALYTICS_AVAILABLE = True
except Exception:
    UltralyticsYOLO = None
    ULTRALYTICS_AVAILABLE = False


MOBILE_YOLO_CLASSES = {
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


class MultiObjectCentroidTracker:
    """Suivi multi-objets déterministe compatible `model.track(persist=True)` (assigne et maintient `track_id`)."""

    def __init__(self, max_disappeared: int = 15, max_distance_px: float = 110.0):
        self._next_id: int = 1
        self._tracks: Dict[int, Dict[str, Any]] = {}
        self.max_disappeared = max_disappeared
        self.max_distance_px = max_distance_px

    def reset(self) -> None:
        self._tracks.clear()
        self._next_id = 1

    def update(self, raw_detections: List[Dict[str, Any]], now_ts: Optional[float] = None) -> List[Dict[str, Any]]:
        """Associe les boîtes détectées aux pistes existantes et calcule la vitesse en pixels."""
        ts = time.perf_counter() if now_ts is None else float(now_ts)
        updated_list: List[Dict[str, Any]] = []

        if not raw_detections:
            to_delete = []
            for tid, tr in self._tracks.items():
                tr["disappeared"] = tr.get("disappeared", 0) + 1
                if tr["disappeared"] > self.max_disappeared:
                    to_delete.append(tid)
            for tid in to_delete:
                del self._tracks[tid]
            return []

        used_track_ids = set()
        for det in raw_detections:
            bbox = det.get("bbox") or det.get("xyxy") or [0, 0, 10, 10]
            x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
            cx = float(det.get("cx", (x1 + x2) / 2.0))
            cy = float(det.get("cy", (y1 + y2) / 2.0))
            y_bottom = float(det.get("y_bottom", max(y1, y2)))

            explicit_tid = det.get("track_id")
            matched_tid: Optional[int] = None

            if explicit_tid is not None:
                matched_tid = int(explicit_tid)
            else:
                best_dist = self.max_distance_px
                for tid, tr in self._tracks.items():
                    if tid in used_track_ids:
                        continue
                    d = math.hypot(cx - tr["cx"], cy - tr["cy"])
                    if d < best_dist:
                        best_dist = d
                        matched_tid = tid

                if matched_tid is None:
                    matched_tid = self._next_id
                    self._next_id += 1

            used_track_ids.add(matched_tid)
            prev = self._tracks.get(matched_tid)
            if prev is not None:
                dt = max(1e-3, ts - prev["ts"])
                dx_px = cx - prev["cx"]
                dy_px = cy - prev["cy"]
                disp_px = math.hypot(dx_px, dy_px)
                speed_px_s = disp_px / dt
                frames_seen = prev.get("frames_seen", 1) + 1
            else:
                dx_px = 0.0
                dy_px = 0.0
                disp_px = float(det.get("displacement_px", 0.0))
                speed_px_s = float(det.get("speed_px_s", 0.0))
                frames_seen = 1

            cls_name = str(det.get("class_name", det.get("label", "obstacle"))).strip().lower()
            is_mobile_cls = cls_name in MOBILE_YOLO_CLASSES
            is_moving_flag = bool(
                det.get("is_moving", False)
                or is_mobile_cls
                or disp_px >= 3.5
                or speed_px_s >= 12.0
            )

            track_record = {
                "track_id": matched_tid,
                "class_name": cls_name,
                "conf": float(det.get("conf", 0.88)),
                "bbox": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
                "cx": round(cx, 2),
                "cy": round(cy, 2),
                "y_bottom": round(y_bottom, 2),
                "dx_px": round(dx_px, 2),
                "dy_px": round(dy_px, 2),
                "displacement_px": round(disp_px, 2),
                "speed_px_s": round(speed_px_s, 2),
                "is_mobile_class": is_mobile_cls,
                "is_moving": is_moving_flag,
                "frames_seen": frames_seen,
                "disappeared": 0,
                "ts": ts,
            }
            if "width_m" in det:
                track_record["width_m"] = float(det["width_m"])
            if "length_m" in det:
                track_record["length_m"] = float(det["length_m"])
            self._tracks[matched_tid] = track_record
            updated_list.append(dict(track_record))

        return updated_list


class YoloVisionPipeline:
    """Pipeline Vision IA unifié basé sur YOLO (`model.track(persist=True)`) + Géométrie Optique Dynamique."""

    def __init__(
        self,
        config_manager: Optional[RobotConfigManager] = None,
        model_path: str = "yolov8n.pt",
        pixel_motion_threshold: float = 3.5,
    ):
        self.config: RobotConfigManager = config_manager or robot_config
        self.model_path = model_path
        self.pixel_motion_threshold = float(pixel_motion_threshold)

        # Chargement optionnel du modèle Ultralytics YOLOv8n / YOLO11n si disponible localement
        self.yolo_model: Optional[Any] = None
        if ULTRALYTICS_AVAILABLE and UltralyticsYOLO is not None:
            try:
                import os
                if os.path.isfile(model_path):
                    self.yolo_model = UltralyticsYOLO(model_path)
            except Exception as e:
                logger.info("Modèle YOLO local non chargé (%s), utilisation du moteur hybride OpenCV+Tracker.", e)

        self.tracker = MultiObjectCentroidTracker()

        # Pipeline géométrique OpenCV synchronisé avec robot_config
        specs = self.config.get_specs()
        self.base_pipeline = RealVisionPipeline(
            chassis_width_m=float(specs.get("chassis_width_m", 1.21)),
            tool_spacing_m=float(specs.get("tool_spacing_m", 0.46)),
            camera_height_m=float(specs.get("camera_height_m", 0.90)),
            camera_tilt_deg=float(specs.get("camera_pitch_deg", 32.0)),
            k1=0.0,
            fov_h_deg=float(specs.get("camera_fov_deg", 78.0)),
        )

        # État d'auto-calibration de l'angle caméra
        self.calibrated_pitch_deg: float = float(specs.get("camera_pitch_deg", 32.0))
        self.last_vanishing_point_y: Optional[float] = None

        # Machine à états pour les obstacles mobiles vs statiques
        # États : "CLEAR", "WAITING_MOVING_OBSTACLE", "BYPASS_REQUIRED"
        self.obstacle_state: str = "CLEAR"
        self._wait_start_ts: Optional[float] = None
        self._waiting_track_id: Optional[int] = None
        self._elapsed_wait_sec: float = 0.0

        # Machine à états pour le Suivi Intelligent Jusqu'à Fin de Ligne
        self.line_missing_distance_m: float = 0.0
        self.line_state: str = "TRACKING_LINE"  # "TRACKING_LINE", "LINE_END_DETECTED", "FIELD_END_DETECTED"
        self.line_end_detected: bool = False
        self.field_end_detected: bool = False

        # Auto-Détection du Sens de Progression verrouillée dès la 1ère ligne
        # Convention : DROITE = -1 ("RIGHT"), GAUCHE = +1 ("LEFT")
        self.locked_field_side_sign: Optional[int] = None
        self.locked_field_side_label: Optional[str] = None
        self.last_side_detection_result: Optional[Dict[str, Any]] = None

        # Écoute des changements dynamiques de robot_config
        self.config.register_listener(self._on_config_updated)

    def _on_config_updated(self, specs: Dict[str, Any]) -> None:
        """Synchronise immédiatement toutes les dimensions avec `robot_config`."""
        self.base_pipeline.chassis_width_m = float(specs.get("chassis_width_m", 1.21))
        self.base_pipeline.tool_spacing_m = float(specs.get("tool_spacing_m", 0.46))
        self.base_pipeline.half_tool_m = float(specs.get("tool_half_offset_m", 0.23))
        self.base_pipeline.camera_height_m = float(specs.get("camera_height_m", 0.90))
        self.base_pipeline.camera_tilt_deg = float(specs.get("camera_pitch_deg", 32.0))
        self.base_pipeline.fov_h_deg = float(specs.get("camera_fov_deg", 78.0))
        if not bool(specs.get("auto_pitch", True)):
            self.calibrated_pitch_deg = float(specs.get("camera_pitch_deg", 32.0))

    # ==========================================================================
    # 3. AUTO-CALIBRATION DE L'ANGLE D'INCLINAISON CAMÉRA & DISTANCE AU SOL
    # ==========================================================================
    def compute_focal_lengths(
        self,
        img_width: int = 640,
        img_height: int = 480,
        fov_v_deg: Optional[float] = None,
        fov_h_deg: Optional[float] = None,
    ) -> Tuple[float, float]:
        """Calcule les focales (f_x, f_y) :
        f_y = (H_img / 2) / tan(FOV_v / 2)
        f_x = (W_img / 2) / tan(FOV_h / 2)
        """
        cfg_fov = float(self.config.camera_fov_deg)
        fv = float(fov_v_deg) if fov_v_deg is not None else cfg_fov
        fh = float(fov_h_deg) if fov_h_deg is not None else cfg_fov
        fy = (float(img_height) / 2.0) / math.tan(math.radians(max(5.0, min(170.0, fv)) / 2.0))
        fx = (float(img_width) / 2.0) / math.tan(math.radians(max(5.0, min(170.0, fh)) / 2.0))
        return fx, fy

    def calibrate_pitch_from_vanishing_point(
        self,
        y_vp: float,
        img_height: int = 480,
        fov_v_deg: Optional[float] = None,
    ) -> float:
        """Calcule automatiquement l'angle d'inclinaison de la caméra (theta_pitch en degrés)
        à partir de l'ordonnée du Point de Fuite (y_vp) où convergent les droites parallèles au sol :
            f_y = (H_img / 2) / tan(FOV_v / 2)
            theta_pitch = arctan((y_vp - c_y) / f_y)
        """
        _, fy = self.compute_focal_lengths(img_width=img_height, img_height=img_height, fov_v_deg=fov_v_deg)
        cy = float(img_height) / 2.0
        # Le point de fuite peut être exprimé en convention signée (y_vp - cy) ou absolue
        theta_rad = math.atan((float(y_vp) - cy) / fy)
        theta_deg = abs(math.degrees(theta_rad))
        self.last_vanishing_point_y = float(y_vp)
        if self.config.auto_pitch and theta_deg > 1.0:
            self.calibrated_pitch_deg = round(theta_deg, 2)
        return round(theta_deg, 4)

    def calibrate_pitch_from_reference_marker(
        self,
        y_marker: float,
        marker_distance_m: float = 1.00,
        img_height: int = 480,
        camera_height_m: Optional[float] = None,
        fov_v_deg: Optional[float] = None,
    ) -> float:
        """Calibre l'angle de la caméra à partir d'un repère posé au sol à `marker_distance_m` (défaut 1.00 m)."""
        cam_h = float(camera_height_m) if camera_height_m is not None else self.config.camera_height_m
        _, fy = self.compute_focal_lengths(img_width=img_height, img_height=img_height, fov_v_deg=fov_v_deg)
        cy = float(img_height) / 2.0
        total_depression_rad = math.atan2(cam_h, max(0.10, float(marker_distance_m)))
        ray_offset_rad = math.atan((float(y_marker) - cy) / fy)
        pitch_deg = math.degrees(total_depression_rad - ray_offset_rad)
        pitch_deg = float(np.clip(pitch_deg, 5.0, 80.0))
        if self.config.auto_pitch:
            self.calibrated_pitch_deg = round(pitch_deg, 2)
        return round(pitch_deg, 4)

    def compute_ground_distance_and_lateral(
        self,
        y_bottom: float,
        x_center: float = 320.0,
        img_width: int = 640,
        img_height: int = 480,
        camera_height_m: Optional[float] = None,
        pitch_deg: Optional[float] = None,
        fov_v_deg: Optional[float] = None,
        fov_h_deg: Optional[float] = None,
    ) -> Dict[str, float]:
        """Calcule la distance longitudinale exacte au sol (D_sol) et la position latérale (X_sol)
        en projetant le point de contact au sol `y_bottom` (bas de la boîte englobante YOLO) :
            D_sol = camera_height_m / tan(theta_pitch + arctan((y_bottom - c_y) / f_y))
            X_sol = D_sol * (x_center - c_x) / f_x
        """
        cam_h = float(camera_height_m) if camera_height_m is not None else self.config.camera_height_m
        theta_deg = (
            float(pitch_deg)
            if pitch_deg is not None
            else (self.calibrated_pitch_deg if self.config.auto_pitch else self.config.camera_pitch_deg)
        )
        theta_rad = math.radians(theta_deg)
        fx, fy = self.compute_focal_lengths(
            img_width=img_width,
            img_height=img_height,
            fov_v_deg=fov_v_deg,
            fov_h_deg=fov_h_deg,
        )
        cx = float(img_width) / 2.0
        cy = float(img_height) / 2.0

        alpha_y_rad = math.atan((float(y_bottom) - cy) / fy)
        total_angle_rad = max(math.radians(2.0), min(math.radians(88.0), theta_rad + alpha_y_rad))
        d_sol = cam_h / math.tan(total_angle_rad)
        x_sol = d_sol * ((float(x_center) - cx) / fx)

        return {
            "d_sol_m": round(float(d_sol), 4),
            "x_sol_m": round(float(x_sol), 4),
            "theta_pitch_deg": round(theta_deg, 2),
            "fx": round(fx, 2),
            "fy": round(fy, 2),
            "hud_pitch_label": f"📐 Angle Caméra Auto : {theta_deg:.1f}°",
        }

    # ==========================================================================
    # 2. GESTION INTELLIGENTE DES OBSTACLES EN MOUVEMENT VS STATIQUES
    # ==========================================================================
    def reset_obstacle_state(self) -> None:
        self.obstacle_state = "CLEAR"
        self._wait_start_ts = None
        self._waiting_track_id = None
        self._elapsed_wait_sec = 0.0
        self.tracker.reset()

    def evaluate_obstacle_behavior(
        self,
        detections: List[Dict[str, Any]],
        img_width: int = 640,
        img_height: int = 480,
        dt: float = 0.1,
        now_ts: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Évalue les obstacles détectés par YOLO dans le couloir dynamique du robot :
        1. Si aucun obstacle dans le couloir à `dist_m <= stop_threshold_m` :
           - Si on était en `WAITING_MOVING_OBSTACLE`, l'obstacle a quitté le couloir -> reprise directe (`WORKING`) sans contournement !
        2. Si un obstacle est dans le couloir à `dist_m <= stop_threshold_m` :
           - Arrêt immédiat (`STOP`, coupure `M6`).
           - Si l'obstacle est en mouvement (vitesse pixel > seuil ou classe mobile : personne, animal, véhicule) :
             * État `WAITING_MOVING_OBSTACLE` avec compte à rebours `moving_obstacle_wait_sec`.
             * Si le délai est dépassé et que l'obstacle est toujours là -> déclenche le contournement (`BYPASS_REQUIRED`).
           - Si l'obstacle est statique dès le départ -> déclenche le contournement (`BYPASS_REQUIRED`).
        """
        specs = self.config.get_specs()
        stop_threshold_m = float(specs.get("stop_threshold_m", 1.20))
        wait_limit_sec = float(specs.get("moving_obstacle_wait_sec", 5.0))
        half_robot_w = float(specs.get("chassis_half_width_m", 0.605))
        corridor_margin = float(specs.get("corridor_safety_margin_m", 0.08))

        tracked = self.tracker.update(detections, now_ts=now_ts)

        blocking_candidates: List[Dict[str, Any]] = []
        for tr in tracked:
            # Si dist_m et x_m sont déjà fournis dans la détection (ex: test direct), on les utilise ou on les calcule via y_bottom
            if "dist_m" in tr or "d_sol_m" in tr:
                d_sol = float(tr.get("dist_m", tr.get("d_sol_m")))
                x_sol = float(tr.get("x_m", tr.get("x_sol_m", 0.0)))
            else:
                proj = self.compute_ground_distance_and_lateral(
                    y_bottom=tr["y_bottom"],
                    x_center=tr["cx"],
                    img_width=img_width,
                    img_height=img_height,
                )
                d_sol = proj["d_sol_m"]
                x_sol = proj["x_sol_m"]

            obs_w_m = float(tr.get("width_m", 0.40))
            collision_half_w = (obs_w_m / 2.0) + half_robot_w + corridor_margin
            in_corridor = abs(x_sol) <= collision_half_w

            tr["d_sol_m"] = round(d_sol, 3)
            tr["x_sol_m"] = round(x_sol, 3)
            tr["in_corridor"] = in_corridor

            if in_corridor and 0.05 <= d_sol <= stop_threshold_m:
                blocking_candidates.append(tr)

        # Cas 1 : Le couloir est libre !
        if not blocking_candidates:
            was_waiting = self.obstacle_state == "WAITING_MOVING_OBSTACLE"
            self.obstacle_state = "CLEAR"
            self._wait_start_ts = None
            self._waiting_track_id = None
            self._elapsed_wait_sec = 0.0
            return {
                "state": "WORKING",
                "obstacle_state": "CLEAR",
                "stop_robot": False,
                "cut_m6": False,
                "bypass_triggered": False,
                "resumed_after_moving_obstacle": was_waiting,
                "remaining_wait_sec": 0.0,
                "hud_banner": (
                    "🟢 Obstacle mobile parti — Reprise directe de la ligne (WORKING)"
                    if was_waiting
                    else ""
                ),
                "blocking_obstacle": None,
                "tracked_objects": tracked,
            }

        # Sélectionner l'obstacle bloquant le plus proche
        primary = min(blocking_candidates, key=lambda o: o["d_sol_m"])
        is_moving = bool(primary.get("is_moving", False) or primary.get("is_mobile_class", False))

        # Cas 2 : Obstacle en mouvement -> Attente intelligente avant de contourner
        if is_moving or self.obstacle_state == "WAITING_MOVING_OBSTACLE":
            if self.obstacle_state != "WAITING_MOVING_OBSTACLE":
                self.obstacle_state = "WAITING_MOVING_OBSTACLE"
                self._waiting_track_id = primary["track_id"]
                self._elapsed_wait_sec = 0.0
                self._wait_start_ts = time.perf_counter() if now_ts is None else float(now_ts)
            else:
                if now_ts is not None and self._wait_start_ts is not None:
                    self._elapsed_wait_sec = max(0.0, float(now_ts) - self._wait_start_ts)
                else:
                    self._elapsed_wait_sec += max(0.0, float(dt))

            remaining = max(0.0, wait_limit_sec - self._elapsed_wait_sec)
            if self._elapsed_wait_sec < wait_limit_sec:
                return {
                    "state": "WAITING_MOVING_OBSTACLE",
                    "obstacle_state": "WAITING_MOVING_OBSTACLE",
                    "stop_robot": True,
                    "cut_m6": True,
                    "bypass_triggered": False,
                    "resumed_after_moving_obstacle": False,
                    "remaining_wait_sec": round(remaining, 2),
                    "elapsed_wait_sec": round(self._elapsed_wait_sec, 2),
                    "hud_banner": f"⏳ OBSTACLE EN MOUVEMENT — ATTENTE ({remaining:.1f}s)...",
                    "blocking_obstacle": primary,
                    "tracked_objects": tracked,
                }

            # Délai dépassé et l'obstacle est toujours dans le couloir -> Contournement !
            self.obstacle_state = "BYPASS_REQUIRED"
            return {
                "state": "BYPASS_REQUIRED",
                "obstacle_state": "BYPASS_REQUIRED",
                "stop_robot": True,
                "cut_m6": True,
                "bypass_triggered": True,
                "reason": "MOVING_OBSTACLE_TIMEOUT_EXCEEDED",
                "remaining_wait_sec": 0.0,
                "elapsed_wait_sec": round(self._elapsed_wait_sec, 2),
                "hud_banner": "🔄 Délai d'attente expiré — Contournement Trapézoïdal (WP0 -> WP1 -> WP2 -> WP3)",
                "blocking_obstacle": primary,
                "tracked_objects": tracked,
            }

        # Cas 3 : Obstacle totalement statique dès le départ -> Contournement immédiat
        self.obstacle_state = "BYPASS_REQUIRED"
        return {
            "state": "BYPASS_REQUIRED",
            "obstacle_state": "BYPASS_REQUIRED",
            "stop_robot": True,
            "cut_m6": True,
            "bypass_triggered": True,
            "reason": "STATIC_OBSTACLE_BLOCKING",
            "remaining_wait_sec": 0.0,
            "elapsed_wait_sec": 0.0,
            "hud_banner": "🛑 Obstacle statique bloquant — Contournement Trapézoïdal (WP0 -> WP1 -> WP2 -> WP3)",
            "blocking_obstacle": primary,
            "tracked_objects": tracked,
        }

    # ==========================================================================
    # 5. MODE « SUIVI INTELLIGENT JUSQU'À FIN DE LIGNE » (DÉTECTION AUTO)
    # ==========================================================================
    def reset_line_tracking(self) -> None:
        self.line_missing_distance_m = 0.0
        self.line_state = "TRACKING_LINE"
        self.line_end_detected = False
        self.field_end_detected = False

    def evaluate_smart_line_continuity(
        self,
        line_detected_in_bottom: bool,
        step_distance_m: float,
        after_uturn_shift: bool = False,
    ) -> Dict[str, Any]:
        """Évalue la continuité de la ligne/rang devant le robot en mode `SMART_LINE_END` :
        - Tant que la ligne est visible en bas d'image : `line_missing_distance_m = 0.0`, état `TRACKING_LINE`.
        - Si la ligne s'arrête dans le bas de l'image pendant > `line_end_confirm_dist_m` (0.50 m) :
          déclenche `LINE_END_DETECTED` (arrêt outils + demi-tour ±central_guide_spacing_m).
        - Après le décalage latéral du demi-tour (`after_uturn_shift=True`), si aucune nouvelle ligne n'est présente :
          déclenche `FIELD_END_DETECTED` (`✅ Fin de parcelle détectée automatiquement — Mission terminée`).
        """
        confirm_dist = float(self.config.get_specs().get("line_end_confirm_dist_m", 0.50))

        if after_uturn_shift:
            if not line_detected_in_bottom:
                self.field_end_detected = True
                self.line_state = "FIELD_END_DETECTED"
                return {
                    "line_state": "FIELD_END_DETECTED",
                    "line_end_detected": True,
                    "field_end_detected": True,
                    "line_missing_distance_m": round(self.line_missing_distance_m, 3),
                    "action": "FINISH_MISSION",
                    "message": "✅ Fin de parcelle détectée automatiquement — Mission terminée",
                }
            else:
                self.line_missing_distance_m = 0.0
                self.line_end_detected = False
                self.field_end_detected = False
                self.line_state = "TRACKING_LINE"
                return {
                    "line_state": "TRACKING_LINE",
                    "line_end_detected": False,
                    "field_end_detected": False,
                    "line_missing_distance_m": 0.0,
                    "action": "FOLLOW_NEW_ROW",
                    "message": "🌱 Nouvelle ligne détectée — Reprise du suivi",
                }

        if line_detected_in_bottom:
            self.line_missing_distance_m = 0.0
            self.line_end_detected = False
            self.line_state = "TRACKING_LINE"
            return {
                "line_state": "TRACKING_LINE",
                "line_end_detected": False,
                "field_end_detected": False,
                "line_missing_distance_m": 0.0,
                "action": "CONTINUE_ROW",
                "message": "Suivi de ligne actif",
            }

        self.line_missing_distance_m += max(0.0, float(step_distance_m))
        if self.line_missing_distance_m > confirm_dist or math.isclose(self.line_missing_distance_m, confirm_dist, abs_tol=1e-6):
            self.line_end_detected = True
            self.line_state = "LINE_END_DETECTED"
            return {
                "line_state": "LINE_END_DETECTED",
                "line_end_detected": True,
                "field_end_detected": False,
                "line_missing_distance_m": round(self.line_missing_distance_m, 3),
                "action": "TRIGGER_UTURN",
                "shift_m": self.config.central_guide_spacing_m,
                "message": f"🔄 Fin de ligne détectée ({self.line_missing_distance_m:.2f}m sans ligne) — Demi-tour ±{self.config.central_guide_spacing_m:.2f}m",
            }

        return {
            "line_state": "TRACKING_LINE",
            "line_end_detected": False,
            "field_end_detected": False,
            "line_missing_distance_m": round(self.line_missing_distance_m, 3),
            "action": "CONTINUE_ROW",
            "message": f"Vérification continuité ({self.line_missing_distance_m:.2f}m / {confirm_dist:.2f}m)",
        }

    # ==========================================================================
    # 6. AUTO-DÉTECTION INTELLIGENTE DU SENS DE PROGRESSION (GAUCHE / DROITE / SOL)
    # ==========================================================================
    def reset_side_detection_lock(self) -> None:
        """Réinitialise le verrouillage du sens de progression (appelé uniquement en début de nouvelle mission)."""
        self.locked_field_side_sign = None
        self.locked_field_side_label = None
        self.last_side_detection_result = None

    def _extract_roi_soil_metrics(self, roi: Any, mission: str = "WEEDING") -> Dict[str, Any]:
        """Extrait les 2 indices agronomiques d'une région latérale (image OpenCV ou dictionnaire d'état) :
        - Indice 1 : Ratio de densité d'adventices (`weeds_ratio`).
        - Indice 2 : Variance de texture Sobel/Laplacien et empreintes de roues/dents (`texture_var`, `has_tracks`).
        """
        if roi is None:
            return {"worked": False, "weeds_ratio": 0.05, "texture_var": 40.0, "has_tracks": False}

        if isinstance(roi, dict):
            has_tracks = bool(roi.get("has_wheel_tracks", roi.get("has_tracks", roi.get("has_furrow_marks", False))))
            tex_var = float(roi.get("texture_var", 260.0 if (roi.get("worked") or has_tracks) else 45.0))
            weeds_r = float(roi.get("weeds_ratio", roi.get("weeds_density", 0.0 if roi.get("worked") else 0.08)))
            if "weeds_count" in roi and "weeds_ratio" not in roi:
                weeds_r = min(1.0, float(roi["weeds_count"]) / 20.0)
            explicit_worked = roi.get("worked", roi.get("already_worked", None))
            if explicit_worked is not None:
                is_worked = bool(explicit_worked)
            else:
                if "WEED" in mission.upper():
                    is_worked = (weeds_r < 0.015 and tex_var >= 150.0) or has_tracks
                else:
                    is_worked = tex_var >= 160.0 or has_tracks
            return {
                "worked": is_worked,
                "weeds_ratio": round(weeds_r, 4),
                "texture_var": round(tex_var, 2),
                "has_tracks": has_tracks,
            }

        if isinstance(roi, np.ndarray) and roi.size > 0:
            if len(roi.shape) == 3:
                veg_mask = self.base_pipeline._compute_vegetation_mask(roi)
                weeds_ratio = float(np.count_nonzero(veg_mask)) / float(max(1, veg_mask.size))
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            else:
                weeds_ratio = 0.0
                gray = roi

            lap_var = float(np.var(cv2.Laplacian(gray, cv2.CV_64F)))
            sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
            sobel_energy = float(np.mean(np.abs(sobel_x)))
            texture_score = lap_var + sobel_energy * 2.5
            has_tracks = texture_score >= 165.0
            if "WEED" in mission.upper():
                is_worked = has_tracks and weeds_ratio < 0.02
            else:
                is_worked = has_tracks
            return {
                "worked": is_worked,
                "weeds_ratio": round(weeds_ratio, 4),
                "texture_var": round(texture_score, 2),
                "has_tracks": has_tracks,
            }

        return {"worked": False, "weeds_ratio": 0.05, "texture_var": 40.0, "has_tracks": False}

    def analyze_soil_treatment_state(
        self,
        left_roi: Any = None,
        right_roi: Any = None,
        mission: str = "WEEDING",
    ) -> Dict[str, Any]:
        """Analyse l'état du sol à Gauche vs Droite (Déjà Traité vs Non Traité) :
        - Indice 1 (Densité d'adventices en Désherbage) : Le côté avec adventices est NON TRAITÉ, le côté propre est DÉJÀ TRAITÉ.
        - Indice 2 (Texture et Traces de passage en Semis / Désherbage) : Empreintes de roues cages et stries des 2 dents/sillons
          à ±23 cm = SOL DÉJÀ TRAITÉ vs sable lisse avec uniquement la ligne directrice = SOL VIERGE NON TRAITÉ.
        """
        left_info = self._extract_roi_soil_metrics(left_roi, mission=mission)
        right_info = self._extract_roi_soil_metrics(right_roi, mission=mission)

        left_worked = bool(left_info["worked"])
        right_worked = bool(right_info["worked"])

        # Comparaison relative si aucun flag explicite n'a tranché mais que les densités/textures diffèrent nettement
        if not left_worked and not right_worked:
            if left_info["texture_var"] > right_info["texture_var"] * 1.8 and left_info["weeds_ratio"] <= right_info["weeds_ratio"] * 0.5:
                left_worked = True
            elif right_info["texture_var"] > left_info["texture_var"] * 1.8 and right_info["weeds_ratio"] <= left_info["weeds_ratio"] * 0.5:
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
            "left_weeds_ratio": left_info["weeds_ratio"],
            "right_weeds_ratio": right_info["weeds_ratio"],
            "left_texture_var": left_info["texture_var"],
            "right_texture_var": right_info["texture_var"],
            "recommended_side": rec_side,
            "recommended_sign": rec_sign,
            "hud_message": hud_msg,
        }

    # Alias camelCase pour compatibilité directe avec les appels JS/tests
    analyzeSoilTreatmentState = analyze_soil_treatment_state

    def detect_next_row_direction(
        self,
        left_line_exists: bool = True,
        right_line_exists: bool = True,
        left_roi: Any = None,
        right_roi: Any = None,
        left_already_worked: Optional[bool] = None,
        right_already_worked: Optional[bool] = None,
        mission: str = "SEEDING",
        farmer_override: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Arbre de décision en 3 cas sur la 1ère ligne avec verrouillage global (`locked_field_side_sign`) :
        - Règle d'or : Dès que le sens est choisi à la fin de la 1ère ligne, `self.locked_field_side_sign` est
          verrouillé définitivement pour tous les tours suivants (Passe 1 -> 2 -> 3 -> 4...).
        - CAS 1 : Ligne présente d'un seul côté (Bord de parcelle) -> verrouille ce côté.
        - CAS 2 : Lignes présentes des DEUX côtés -> analyse l'état du sol (Déjà traité vs Vierge) et verrouille le côté vierge.
        - CAS 3 : Les deux côtés sont 100 % vierges -> choix autonome (`RIGHT` / `-1` par défaut, ou `farmer_override`) verrouillé.
        """
        # Si déjà verrouillé lors du 1er tour, conserver strictement le même sens !
        if self.locked_field_side_sign is not None and self.last_side_detection_result is not None:
            return dict(self.last_side_detection_result)

        # CAS 1 — Ligne présente d'un seul côté (Bord de parcelle)
        if right_line_exists and not left_line_exists:
            self.locked_field_side_sign = -1
            self.locked_field_side_label = "RIGHT"
            res = {
                "direction": "RIGHT",
                "side_sign": -1,
                "locked": True,
                "case": "CASE_1_SINGLE_LINE_RIGHT",
                "hud_message": "🧠 IA AUTO : Ligne détectée uniquement à DROITE -> Sens DROITE verrouillé",
            }
            self.last_side_detection_result = res
            return dict(res)

        if left_line_exists and not right_line_exists:
            self.locked_field_side_sign = 1
            self.locked_field_side_label = "LEFT"
            res = {
                "direction": "LEFT",
                "side_sign": 1,
                "locked": True,
                "case": "CASE_1_SINGLE_LINE_LEFT",
                "hud_message": "🧠 IA AUTO : Ligne détectée uniquement à GAUCHE -> Sens GAUCHE verrouillé",
            }
            self.last_side_detection_result = res
            return dict(res)

        # CAS 2 — Lignes présentes des DEUX côtés (Analyse de l'État du Sol : Déjà Traité vs Non Traité)
        if left_already_worked is not None or right_already_worked is not None:
            if left_roi is None:
                left_roi = {"worked": bool(left_already_worked)}
            if right_roi is None:
                right_roi = {"worked": bool(right_already_worked)}

        soil_eval = self.analyze_soil_treatment_state(left_roi=left_roi, right_roi=right_roi, mission=mission)
        if soil_eval["left_worked"] and not soil_eval["right_worked"]:
            self.locked_field_side_sign = -1
            self.locked_field_side_label = "RIGHT"
            res = {
                "direction": "RIGHT",
                "side_sign": -1,
                "locked": True,
                "case": "CASE_2_LEFT_ALREADY_WORKED",
                "soil_analysis": soil_eval,
                "hud_message": "🧠 IA SOL : Côté Gauche déjà traité -> Sens DROITE verrouillé",
            }
            self.last_side_detection_result = res
            return dict(res)

        if soil_eval["right_worked"] and not soil_eval["left_worked"]:
            self.locked_field_side_sign = 1
            self.locked_field_side_label = "LEFT"
            res = {
                "direction": "LEFT",
                "side_sign": 1,
                "locked": True,
                "case": "CASE_2_RIGHT_ALREADY_WORKED",
                "soil_analysis": soil_eval,
                "hud_message": "🧠 IA SOL : Côté Droit déjà traité -> Sens GAUCHE verrouillé",
            }
            self.last_side_detection_result = res
            return dict(res)

        # CAS 3 — Les deux côtés sont 100 % vierges (Non traités) : choix autonome verrouillé
        if farmer_override and str(farmer_override).upper() in ("LEFT", "GAUCHE", "1", "+1"):
            chosen_sign = 1
            chosen_label = "LEFT"
            msg = "🤖 IA AUTO : Lignes vierges à Gauche et à Droite -> Choix : Vers la GAUCHE verrouillé pour tous les tours."
        else:
            chosen_sign = -1
            chosen_label = "RIGHT"
            msg = "🤖 IA AUTO : Lignes vierges à Gauche et à Droite -> Choix autonome : Vers la DROITE verrouillé pour tous les tours."

        self.locked_field_side_sign = chosen_sign
        self.locked_field_side_label = chosen_label
        res = {
            "direction": chosen_label,
            "side_sign": chosen_sign,
            "locked": True,
            "case": "CASE_3_BOTH_UNTOUCHED_AUTONOMOUS_FALLBACK",
            "soil_analysis": soil_eval,
            "notification_timeout_sec": 2.5,
            "hud_message": msg,
        }
        self.last_side_detection_result = res
        return dict(res)

    detectNextRowDirection = detect_next_row_direction

    def validate_peanut_row_hybrid(
        self,
        frame: np.ndarray,
        roi_box: Optional[Tuple[int, int, int, int]] = None,
        mission_type: str = "WEEDING",
    ) -> Dict[str, Any]:
        """Double Validation Hybride : OpenCV (Géométrie) + Modèle Pré-entraîné Arachis hypogaea (Botanique)."""
        return self.base_pipeline.validate_peanut_row_hybrid(frame=frame, roi_box=roi_box, mission_type=mission_type)

    # ==========================================================================
    # EXÉCUTION SUR IMAGE RÉELLE (CAMÉRA + YOLO + HUD)
    # ==========================================================================
    def process_frame(
        self,
        frame: np.ndarray,
        mission: str = "SEEDING",
        dt: float = 0.033,
        speed_mps: float = 0.60,
        injected_detections: Optional[List[Dict[str, Any]]] = None,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Exécute le pipeline complet (OpenCV + YOLO Tracking + Auto-Pitch + Fin de Ligne) sur une image."""
        annotated, telem = self.base_pipeline.process_frame(frame, mission=mission)
        h, w = annotated.shape[:2]

        # 1. Auto-calibration de l'angle caméra si un point de fuite des rangs est estimé
        y_vp = telem.get("vanishing_point_y", h * 0.16)
        pitch_deg = self.calibrate_pitch_from_vanishing_point(y_vp=float(y_vp), img_height=h)
        telem["auto_pitch_deg"] = round(pitch_deg, 2)
        telem["hud_pitch_label"] = f"📐 Angle Caméra Auto : {pitch_deg:.1f}°"

        # 2. Détections YOLO ou conversion des obstacles détectés en pistes suivies
        raw_dets: List[Dict[str, Any]] = []
        if injected_detections is not None:
            raw_dets = list(injected_detections)
        else:
            for obs in telem.get("obstacles", []):
                bx = float(obs.get("x", w // 2))
                by = float(obs.get("y", h // 2))
                bw = float(obs.get("w", 40))
                bh = float(obs.get("h", 40))
                raw_dets.append({
                    "class_name": obs.get("class_name", "obstacle"),
                    "bbox": [bx, by, bx + bw, by + bh],
                    "cx": bx + bw / 2.0,
                    "cy": by + bh / 2.0,
                    "y_bottom": by + bh,
                    "conf": 0.90,
                })

        obs_eval = self.evaluate_obstacle_behavior(raw_dets, img_width=w, img_height=h, dt=dt)
        telem["obstacle_evaluation"] = obs_eval
        telem["obstacle_state"] = obs_eval["obstacle_state"]
        telem["waiting_moving_obstacle"] = obs_eval["obstacle_state"] == "WAITING_MOVING_OBSTACLE"
        telem["remaining_wait_sec"] = obs_eval["remaining_wait_sec"]
        if obs_eval["hud_banner"]:
            telem["hud_obstacle_banner"] = obs_eval["hud_banner"]

        # 3. Continuité de ligne en bas d'image (pour le mode Fin de Ligne Intelligente)
        line_present = bool(telem.get("guideline_detected", True))
        line_eval = self.evaluate_smart_line_continuity(
            line_detected_in_bottom=line_present,
            step_distance_m=max(0.0, speed_mps * dt),
        )
        telem["line_continuity"] = line_eval
        telem["line_end_detected"] = line_eval["line_end_detected"]
        telem["line_state"] = line_eval["line_state"]

        # Affichage HUD de l'angle caméra auto-calibré et de la bannière obstacle mobile
        cv2.putText(
            annotated,
            f"Angle Camera Auto : {pitch_deg:.1f} deg",
            (14, h - 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 230, 0),
            2,
            cv2.LINE_AA,
        )
        if obs_eval["hud_banner"]:
            cv2.rectangle(annotated, (12, 52), (w - 12, 86), (15, 23, 42), -1)
            cv2.putText(
                annotated,
                obs_eval["hud_banner"].encode("ascii", "ignore").decode("ascii"),
                (20, 74),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 215, 255),
                2,
                cv2.LINE_AA,
            )

        return annotated, telem


# Instance partagée par défaut
yolo_pipeline = YoloVisionPipeline()

