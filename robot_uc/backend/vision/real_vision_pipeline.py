"""Pipeline de Vision IA Réelle (OpenCV) pour la Webcam USB Externe (T'nB INCAM1080A / 170°).

Ce module traite en temps réel les images issues de la caméra réelle branchée en USB (/dev/video2)
et exécute les 4 algorithmes de vision agricole sans aucune dépendance au moteur 3D :
1. SEEDING (Semence Initiale) : Extraction par filtre Morphologique Black-Hat + CLAHE + Histogramme par bandes
   horizontales + Régression robuste de Huber (cv2.fitLine DIST_HUBER) + lissage EMA temporel pour suivre
   la ligne directrice tracée au sol pendant que les 2 tuyaux M8 (-23 cm et +23 cm) sèment à 46 cm d'écart.
2. RESEEDING (Révision de Semence) : Segmentation végétale combinée Excess Green (ExG = 2G - R - B) + HSV,
   suivi actif des 2 rangs d'arachide (-23 cm et +23 cm) par régression linéaire et détection des trous
   de germination (> 25 cm) pour déclencher chirurgicalement le semoir M8 (PULSE:8:60:400).
3. WEEDING (Désherbage Intelligent) : Segmentation ExG + HSV, verrouillage et suivi actif de l'axe des rangs
   d'arachide (tunnel central protégé ±12 cm, rangs latéraux ±46 cm) avec calcul temps réel de (Δx, Δθ, steer d),
   et détection des adventices dans les couloirs des dents M5 / fraise M6 (-23 cm et +23 cm).
4. OBSTACLE_AVOIDANCE (Contournement d'Obstacles) : Détection multi-critères robuste (modèle adaptatif du sol,
   exclusion végétale ExG, exclusion des sillons fins longitudinaux, énergie de gradient Sobel/Canny et solidité)
   dans le couloir de 1.21 m du châssis avec choix automatique du côté libre de contournement (Gauche/Droite).
"""

import math
from typing import Any, Dict, List, Optional, Tuple
import cv2
import numpy as np

from backend.vision.peanut_classifier import PeanutPlantClassifier, SPECIES_PEANUT
from backend.vision.yolo_onnx_detector import (
    AGRI_OBSTACLE_COCO_CLASSES,
    MultiObjectCentroidTracker,
    YoloOnnxDetector,
    merge_yolo_and_opencv_obstacles,
)


class RealVisionPipeline:
    """Pipeline OpenCV industriel calibré pour la géométrie réelle du robot (Châssis 1.21 m, Outils ±23 cm)."""

    def __init__(
        self,
        chassis_width_m: float = 1.21,
        tool_spacing_m: float = 0.46,
        camera_height_m: float = 0.75,
        camera_tilt_deg: float = 35.0,
        k1: float = 0.0,  # 0.0 par défaut pour la T'nB INCAM1080A (optique rectilinéaire sans distorsion fisheye)
        fov_h_deg: float = 76.0,  # Angle de vue horizontal réel de la T'nB INCAM1080A en 16:9
        yolo_onnx_detector: Optional[YoloOnnxDetector] = None,
    ):
        self.chassis_width_m = chassis_width_m
        self.tool_spacing_m = tool_spacing_m
        self.half_tool_m = tool_spacing_m / 2.0  # 0.23 m (23 cm)
        self.camera_height_m = camera_height_m
        self.camera_tilt_deg = camera_tilt_deg
        self.k1 = k1
        self.fov_h_deg = fov_h_deg

        # Plages HSV pour le feuillage vert (arachide & adventices)
        self.lower_green = np.array([26, 38, 35], dtype=np.uint8)
        self.upper_green = np.array([92, 255, 255], dtype=np.uint8)

        # Cache des cartes de redressement (undistort map)
        self._map1: Optional[np.ndarray] = None
        self._map2: Optional[np.ndarray] = None
        self._cached_shape: Tuple[int, int, float] = (0, 0, 0.0)

        # Filtre temporel EMA (Exponential Moving Average) pour stabiliser la direction (anti-bruit caméra)
        self._ema_lat_cm: float = 0.0
        self._ema_head_deg: float = 0.0
        self._ema_initialized: bool = False
        self._clahe = cv2.createCLAHE(clipLimit=2.2, tileGridSize=(8, 8))
        self.peanut_classifier = PeanutPlantClassifier(confidence_threshold=0.65)
        self.yolo_onnx_detector = yolo_onnx_detector or YoloOnnxDetector()
        self.yolo_onnx_loaded: bool = bool(self.yolo_onnx_detector.yolo_onnx_loaded)
        self.yolo_model_name: str = str(self.yolo_onnx_detector.yolo_model_name)
        self.obstacle_tracker = MultiObjectCentroidTracker()
        self.consecutive_non_peanut_frames: int = 0
        self.end_of_row_consecutive_frames: int = 5

    def detect_objects_onnx(
        self, frame: np.ndarray, conf_thresh: float = 0.40, nms_thresh: float = 0.45
    ) -> List[Dict[str, Any]]:
        """Exécute `yolov8n.onnx` via OpenCV DNN (`cv2.dnn`) sur une image BGR."""
        return self.yolo_onnx_detector.detect_objects_onnx(
            frame, conf_thresh=conf_thresh, nms_thresh=nms_thresh
        )

    def set_optical_config(
        self,
        k1: Optional[float] = None,
        tilt_deg: Optional[float] = None,
        fov_h_deg: Optional[float] = None,
    ) -> None:
        """Met à jour les paramètres optiques de la caméra à chaud."""
        if k1 is not None:
            self.k1 = float(np.clip(k1, -0.55, 0.20))
        if tilt_deg is not None:
            self.camera_tilt_deg = float(np.clip(tilt_deg, 10.0, 75.0))
        if fov_h_deg is not None:
            self.fov_h_deg = float(np.clip(fov_h_deg, 45.0, 175.0))

    def undistort_170_frame(self, frame: np.ndarray) -> np.ndarray:
        """Applique la correction de distorsion radiale si k1 != 0 (désactivée à k1=0.0 pour T'nB INCAM1080A)."""
        if frame is None or frame.size == 0 or abs(self.k1) < 1e-3:
            return frame

        h, w = frame.shape[:2]
        shape_key = (w, h, round(self.k1, 4))

        if self._map1 is None or self._cached_shape != shape_key:
            fx = w * 0.72
            fy = h * 0.72
            cx = w / 2.0
            cy = h / 2.0
            cam_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
            dist_coeffs = np.array([self.k1, abs(self.k1) * 0.18, 0.0, 0.0, 0.0], dtype=np.float32)
            new_cam_matrix, _ = cv2.getOptimalNewCameraMatrix(cam_matrix, dist_coeffs, (w, h), 0.85, (w, h))
            self._map1, self._map2 = cv2.initUndistortRectifyMap(
                cam_matrix, dist_coeffs, None, new_cam_matrix, (w, h), cv2.CV_16SC2
            )
            self._cached_shape = shape_key

        return cv2.remap(frame, self._map1, self._map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    def _compute_vegetation_mask(self, bgr_roi: np.ndarray) -> np.ndarray:
        """Combine l'indice agronomique Excess Green (ExG = 2G - R - B) et la segmentation HSV
        pour détecter de façon ultra-fiable les feuilles d'arachide et adventices sous tout éclairage.
        """
        hsv = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2HSV)
        mask_hsv = cv2.inRange(hsv, self.lower_green, self.upper_green)

        # Indice Excess Green (ExG = 2*G - R - B) normalisé
        b = bgr_roi[:, :, 0].astype(np.int16)
        g = bgr_roi[:, :, 1].astype(np.int16)
        r = bgr_roi[:, :, 2].astype(np.int16)
        exg = (2 * g) - r - b
        mask_exg = np.where((exg > 24) & (g > 40), 255, 0).astype(np.uint8)

        combined = cv2.bitwise_or(mask_hsv, mask_exg)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
        return combined

    def _fit_robust_line(
        self,
        xs: np.ndarray,
        ys: np.ndarray,
        y_top: int,
        y_bot: int,
        w: int,
        default_x: float,
    ) -> Tuple[bool, float, float, float]:
        """Ajuste une droite robuste par M-estimateur de Huber (cv2.fitLine DIST_HUBER) sur un nuage (xs, ys).
        Retourne (found, x_at_bottom, x_at_top, angle_deg).
        """
        if len(xs) < 12:
            return False, default_x, default_x, 0.0

        pts = np.column_stack((xs, ys)).astype(np.float32)
        line_params = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01)
        vx, vy, x0, y0 = [float(v) for v in line_params.flatten()]
        if abs(vy) < 1e-4:
            return False, default_x, default_x, 0.0

        slope = vx / vy
        # Limiter l'inclinaison maximale d'un rang agricole valide (< 42°)
        angle_deg = math.degrees(math.atan(slope))
        if abs(angle_deg) > 42.0:
            return False, default_x, default_x, 0.0

        x_bot = float(np.clip(x0 + (y_bot - y0) * slope, 8.0, w - 8.0))
        x_top = float(np.clip(x0 + (y_top - y0) * slope, 8.0, w - 8.0))
        return True, x_bot, x_top, angle_deg

    def _apply_temporal_smoothing(self, raw_lat_cm: float, raw_head_deg: float, found: bool) -> Tuple[float, float]:
        """Applique un filtre passe-bas exponentiel (EMA) pour stabiliser la commande de direction."""
        if not found:
            self._ema_lat_cm *= 0.85
            self._ema_head_deg *= 0.85
            return round(self._ema_lat_cm, 1), round(self._ema_head_deg, 1)

        if not self._ema_initialized:
            self._ema_lat_cm = raw_lat_cm
            self._ema_head_deg = raw_head_deg
            self._ema_initialized = True
        else:
            alpha = 0.45
            self._ema_lat_cm = alpha * raw_lat_cm + (1.0 - alpha) * self._ema_lat_cm
            self._ema_head_deg = alpha * raw_head_deg + (1.0 - alpha) * self._ema_head_deg
        return round(self._ema_lat_cm, 1), round(self._ema_head_deg, 1)

    def _project_ground_corridor(
        self, w: int, h: int, offset_x_m: float
    ) -> Tuple[Tuple[int, int], Tuple[int, int]]:
        """Projette une ligne longitudinale du sol (en mètres par rapport au centre du robot) en pixels 2D.
        Adapté dynamiquement au ratio d'aspect (16:9 en 1280x720 / 1920x1080 ou 4:3 en 640x480) et au FOV réel (~76°).
        """
        cx = w / 2.0
        horizon_y = int(h * 0.32)
        bottom_y = h - 10

        aspect = float(w) / max(1.0, float(h))
        fov_scale = 76.0 / max(45.0, float(self.fov_h_deg))
        aspect_comp = (1.333 / aspect) if aspect > 1.4 else 1.0
        px_per_meter_bottom = (w * 0.56) * fov_scale * aspect_comp
        px_per_meter_top = (w * 0.17) * fov_scale * aspect_comp

        x_bottom = int(cx + offset_x_m * px_per_meter_bottom)
        x_top = int(cx + offset_x_m * px_per_meter_top)
        return (x_top, horizon_y), (x_bottom, bottom_y)

    def _estimate_distance_from_y(self, py: int, h: int, px: Optional[int] = None, w: int = 640) -> float:
        """Calcule la distance longitudinale exacte au sol (D_sol en mètres) par projection trigonométrique
        du point de contact au sol (y_bottom = py) avec la hauteur dynamique `camera_height_m` et `theta_pitch` :
            f_y = (H / 2) / tan(FOV_v / 2)
            D_sol = camera_height_m / tan(theta_pitch + arctan((y_bottom - c_y) / f_y))
        """
        fov_deg = max(30.0, min(170.0, float(self.fov_h_deg)))
        fy = (float(h) / 2.0) / math.tan(math.radians(fov_deg / 2.0))
        cy = float(h) / 2.0
        # Auto-calibration de l'angle caméra par point de fuite (horizon à ~0.16 * h)
        y_vp = float(getattr(self, "vanishing_point_y", h * 0.16))
        auto_pitch_rad = abs(math.atan((y_vp - cy) / fy))
        self.auto_pitch_deg = round(math.degrees(auto_pitch_rad), 2)
        theta_rad = auto_pitch_rad if getattr(self, "auto_pitch", True) else math.radians(float(self.camera_tilt_deg))

        alpha_y = math.atan((float(py) - cy) / fy)
        total_angle = max(math.radians(2.0), min(math.radians(88.0), theta_rad + alpha_y))
        d_sol = float(self.camera_height_m) / math.tan(total_angle)
        return round(float(np.clip(d_sol, 0.25, 12.0)), 2)

    def compute_ground_projection(self, y_bottom: float, x_center: float, w: int, h: int) -> Tuple[float, float]:
        """Retourne (D_sol, X_sol) en mètres à partir du bas de boîte (y_bottom, x_center)."""
        fov_deg = max(30.0, min(170.0, float(self.fov_h_deg)))
        fy = (float(h) / 2.0) / math.tan(math.radians(fov_deg / 2.0))
        fx = (float(w) / 2.0) / math.tan(math.radians(fov_deg / 2.0))
        cx = float(w) / 2.0
        cy = float(h) / 2.0
        d_sol = self._estimate_distance_from_y(int(round(y_bottom)), h, int(round(x_center)), w)
        x_sol = d_sol * ((float(x_center) - cx) / fx)
        return round(d_sol, 3), round(x_sol, 3)

    def process_frame(
        self, frame: np.ndarray, mission: Optional[str] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Traite une image caméra brute, dessine le HUD spécifique à la mission et retourne (annotated, telemetry)."""
        if frame is None or frame.size == 0:
            frame = self.generate_synthetic_farm_frame(640, 480, mission=mission)

        # 1. Redressement optique si nécessaire (k1=0.0 par défaut sur T'nB INCAM1080A)
        rectified = self.undistort_170_frame(frame)
        vis = rectified.copy()
        h, w = vis.shape[:2]
        aspect_ratio = round(w / max(1.0, float(h)), 2)
        aspect_str = "16:9" if abs(aspect_ratio - 1.78) < 0.15 else ("4:3" if abs(aspect_ratio - 1.33) < 0.15 else f"{w}:{h}")

        mission_norm = str(mission or "SEEDING").strip().upper()
        if "RESEED" in mission_norm:
            mode = "RESEEDING"
        elif "WEED" in mission_norm:
            mode = "WEEDING"
        elif "OBSTACLE" in mission_norm or "MULTI" in mission_norm or "AVOID" in mission_norm:
            mode = "OBSTACLE_AVOIDANCE"
        else:
            mode = "SEEDING"

        # 2. Dessin du gabarit physique réel en filigrane (Châssis 1.21 m et outils ±23 cm)
        self._draw_chassis_and_tool_grid(vis, w, h)

        # Auto-calibration de l'angle d'inclinaison caméra (theta_pitch)
        fov_deg = max(30.0, min(170.0, float(self.fov_h_deg)))
        fy_init = (float(h) / 2.0) / math.tan(math.radians(fov_deg / 2.0))
        cy_init = float(h) / 2.0
        y_vp_init = float(getattr(self, "vanishing_point_y", h * 0.16))
        self.auto_pitch_deg = round(math.degrees(abs(math.atan((y_vp_init - cy_init) / fy_init))), 2)

        telemetry: Dict[str, Any] = {
            "mode": mode,
            "resolution": f"{w}x{h}",
            "aspect_ratio": aspect_str,
            "fov_h_deg": self.fov_h_deg,
            "auto_pitch_deg": self.auto_pitch_deg,
            "hud_pitch_label": f"📐 Angle Caméra Auto : {self.auto_pitch_deg:.1f}°",
            "lateral_error_cm": 0.0,
            "heading_error_deg": 0.0,
            "is_aligned": True,
            "target_detected": False,
            "guideline_detected": False,
            "row_detected": False,
            "obstacle_detected": False,
            "obstacle_in_corridor": False,
            "obstacle_dist_m": 99.0,
            "min_distance_m": 99.0,
            "obstacle_stop": False,
            "emergency_stop": False,
            "recommended_bypass_side": 0,
            "reseed_required": False,
            "reseed_gaps_count": 0,
            "gaps_count": 0,
            "reseed_pulse_triggered": False,
            "weeds_detected_count": 0,
            "tines_m5_lowered": False,
            "cutter_m6_active": False,
            "seeder_m8_active": False,
            "recommended_steer": 0,
            "recommended_drive": {"a": 25, "d": 0, "ms": 250},
            "recommended_tools": {},
            "robot_order_str": "DRIVE:25,0:250",
            "hud_message": "",
            "yolo_onnx_loaded": self.yolo_onnx_loaded,
            "yolo_model_name": self.yolo_model_name,
            "yolo_detections": [],
            "obstacles": [],
        }

        # 3. Détection d'obstacles hybride (YOLOv8n ONNX + OpenCV Sobel/Anomalie Sol)
        self._detect_obstacles_170(vis, w, h, telemetry, raw_frame=rectified)

        # 4. Traitement spécialisé selon la mission
        if mode == "SEEDING":
            self._process_mission_seeding(vis, w, h, telemetry)
        elif mode == "RESEEDING":
            self._process_mission_reseeding(vis, w, h, telemetry)
        elif mode == "WEEDING":
            self._process_mission_weeding(vis, w, h, telemetry)
        elif mode == "OBSTACLE_AVOIDANCE":
            self._process_mission_obstacle_only(vis, w, h, telemetry)

        # 4-bis. Validation Hybride Binaire OpenCV + MobileNetV3 ONNX (Arachide vs Sol Nu)
        telemetry["onnx_loaded"] = bool(getattr(self.peanut_classifier, "onnx_loaded", False))
        telemetry["model_backend"] = getattr(self.peanut_classifier, "model_backend", "")
        if mode in ("WEEDING", "RESEEDING"):
            hybrid = self.validate_peanut_row_hybrid(rectified, roi_box=None, mission_type=mode)
            if hybrid["is_valid_peanut_row"]:
                self.consecutive_non_peanut_frames = 0
            else:
                self.consecutive_non_peanut_frames += 1

            confirmed_end = bool(self.consecutive_non_peanut_frames >= self.end_of_row_consecutive_frames)
            telemetry["hybrid_validation"] = hybrid
            telemetry["is_valid_peanut_row"] = hybrid["is_valid_peanut_row"]
            telemetry["opencv_line_aligned"] = hybrid["opencv_line_aligned"]
            telemetry["is_peanut"] = hybrid["is_peanut"]
            telemetry["peanut_confidence"] = hybrid["peanut_confidence"]
            telemetry["detected_species_label"] = hybrid["detected_species_label"]
            telemetry["hybrid_hud_badge"] = hybrid["hud_badge"]
            telemetry["hybrid_boundary_alert"] = hybrid["hud_alert"]
            telemetry["consecutive_non_peanut_frames"] = self.consecutive_non_peanut_frames
            telemetry["hybrid_line_end_confirmed"] = confirmed_end
            telemetry["hybrid_line_end_detected"] = confirmed_end

            if confirmed_end and not telemetry.get("obstacle_stop", False):
                telemetry["recommended_drive"] = {"a": 0, "d": 0, "ms": 250}
                telemetry["recommended_tools"] = {"tines_depth": 0.0, "cutter_m6": 0, "seeder_m8": 0}
                telemetry["cutter_m6_active"] = False
                telemetry["seeder_m8_active"] = False
                telemetry["tines_m5_lowered"] = False
                telemetry["hud_message"] = hybrid["hud_badge"]
        else:
            self.consecutive_non_peanut_frames = 0
            telemetry.setdefault("is_valid_peanut_row", True)
            telemetry.setdefault("hybrid_hud_badge", "🌱 Mode Semis Initial (Suivi Sillon Central)")
            telemetry.setdefault("hybrid_boundary_alert", False)
            telemetry["consecutive_non_peanut_frames"] = 0
            telemetry["hybrid_line_end_confirmed"] = False
            telemetry["hybrid_line_end_detected"] = False

        # 5. Synthèse de l'ordre matériel [VISION IA -> ORDRE ROBOT]
        rec = telemetry.get("recommended_drive") or {"a": 25, "d": 0, "ms": 250}
        if telemetry.get("obstacle_stop"):
            side_lbl = "GAUCHE" if telemetry.get("recommended_bypass_side", 1) > 0 else "DROITE"
            telemetry["robot_order_str"] = (
                f"STOP OBSTACLE ({telemetry.get('obstacle_dist_m', 0):.2f}m -> CONTOURNER {side_lbl}) | Dents M5: RELEVÉES | Fraise M6: OFF"
            )
        elif telemetry.get("hybrid_line_end_confirmed"):
            telemetry["robot_order_str"] = "STOP (FIN DE LIGNE ARACHIDE) | Fraise M6: OFF | Semoir M8: OFF | Dents M5: RELEVÉES"
        elif mode == "SEEDING":
            telemetry["robot_order_str"] = f"DRIVE:{rec['a']},{rec['d']}:{rec.get('ms', 250)} | Semoir M8: ACTIF | Dents M5: RELEVÉES | Fraise M6: OFF"
        elif mode == "RESEEDING":
            t_str = "Semoir M8: ON (PULSE:8:60:400)" if telemetry.get("reseed_required") else "Semoir M8: OFF (Veille)"
            telemetry["robot_order_str"] = f"DRIVE:{rec['a']},{rec['d']}:{rec.get('ms', 250)} | {t_str} | Dents M5: RELEVÉES"
        elif mode == "WEEDING":
            t_str = "Dents M5: ABAISSÉES | Fraise M6: ON" if telemetry.get("weeds_detected_count", 0) > 0 else "Dents M5: GARDE | Fraise M6: OFF"
            telemetry["robot_order_str"] = f"DRIVE:{rec['a']},{rec['d']}:{rec.get('ms', 250)} | {t_str} | Semoir M8: OFF"
        else:
            telemetry["robot_order_str"] = f"DRIVE:{rec['a']},{rec['d']}:{rec.get('ms', 250)} | VOIE LIBRE (0 Obstacle)"

        # 6. Bandeau HUD supérieur & bandeau d'ordre robot
        self._draw_hud_banner(vis, w, h, telemetry)

        return vis, telemetry

    def _draw_chassis_and_tool_grid(self, vis: np.ndarray, w: int, h: int) -> None:
        """Trace les repères géométriques au sol du robot (châssis 1.21m, outils ±23cm, axe central)."""
        overlay = vis.copy()
        half_w = self.chassis_width_m / 2.0  # ~0.605 m

        # Lignes extérieures du châssis (Cyan transparent)
        pt_tl, pt_bl = self._project_ground_corridor(w, h, -half_w)
        pt_tr, pt_br = self._project_ground_corridor(w, h, +half_w)
        cv2.line(overlay, pt_tl, pt_bl, (255, 200, 0), 2, cv2.LINE_AA)
        cv2.line(overlay, pt_tr, pt_br, (255, 200, 0), 2, cv2.LINE_AA)

        # Zone du tunnel intérieur sous châssis (polygone bleuté léger)
        corridor_poly = np.array([pt_tl, pt_tr, pt_br, pt_bl], dtype=np.int32)
        cv2.fillPoly(overlay, [corridor_poly], (30, 50, 70))

        # Lignes des deux outils à -23 cm (Gauche) et +23 cm (Droite) (Vert)
        pt_tool_l_top, pt_tool_l_bot = self._project_ground_corridor(w, h, -self.half_tool_m)
        pt_tool_r_top, pt_tool_r_bot = self._project_ground_corridor(w, h, +self.half_tool_m)
        cv2.line(overlay, pt_tool_l_top, pt_tool_l_bot, (0, 230, 100), 2, cv2.LINE_AA)
        cv2.line(overlay, pt_tool_r_top, pt_tool_r_bot, (0, 230, 100), 2, cv2.LINE_AA)

        # Ligne centrale directrice théorique (Jaune)
        pt_c_top, pt_c_bot = self._project_ground_corridor(w, h, 0.0)
        cv2.line(overlay, pt_c_top, pt_c_bot, (0, 230, 255), 1, cv2.LINE_AA)

        # Fusionner avec transparence
        cv2.addWeighted(overlay, 0.35, vis, 0.65, 0, vis)

        # Légendes au bas de l'image
        cv2.putText(vis, "OUTIL G (-23cm)", (pt_tool_l_bot[0] - 50, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 120), 1)
        cv2.putText(vis, "AXE (0.0)", (pt_c_bot[0] - 25, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
        cv2.putText(vis, "OUTIL D (+23cm)", (pt_tool_r_bot[0] - 50, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 120), 1)

    def _detect_obstacles_170(
        self,
        vis: np.ndarray,
        w: int,
        h: int,
        telemetry: Dict[str, Any],
        raw_frame: Optional[np.ndarray] = None,
    ) -> None:
        """Fusionne la détection sémantique YOLOv8n ONNX (entités mobiles & objets COCO) et
        OpenCV Sobel/Anomalie Sol (rochers, pierres, souches non-COCO), puis projette chaque
        piste suivie dans le couloir dynamique du châssis (1,21 m) via (D_sol, X_sol).
        """
        # 1. Inférence YOLOv8n ONNX (sur image propre avant surimpression HUD)
        src_frame = raw_frame if raw_frame is not None else vis
        raw_yolo_dets = self.yolo_onnx_detector.detect_objects_onnx(src_frame, conf_thresh=0.40, nms_thresh=0.45)
        yolo_dets = [
            d for d in raw_yolo_dets if str(d.get("class_name", "")).lower() in AGRI_OBSTACLE_COCO_CLASSES
        ]

        # 2. Détection géométrique complémentaire OpenCV Sobel + Anomalie Sol (rochers/souches)
        roi_top = int(h * 0.32)
        roi_bot = int(h * 0.94)
        roi = src_frame[roi_top:roi_bot, :]

        veg_mask = self._compute_vegetation_mask(roi)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)

        gx = cv2.Sobel(blurred, cv2.CV_16S, 1, 0, ksize=3)
        gy = cv2.Sobel(blurred, cv2.CV_16S, 0, 1, ksize=3)
        grad_mag = cv2.addWeighted(cv2.convertScaleAbs(gx), 0.5, cv2.convertScaleAbs(gy), 0.5, 0)

        non_veg = veg_mask == 0
        if np.count_nonzero(non_veg) > 200:
            med_v = float(np.median(hsv[:, :, 2][non_veg]))
            med_s = float(np.median(hsv[:, :, 1][non_veg]))
        else:
            med_v = float(np.median(hsv[:, :, 2]))
            med_s = float(np.median(hsv[:, :, 1]))

        val_diff = np.abs(hsv[:, :, 2].astype(np.float32) - med_v)
        sat_diff = np.abs(hsv[:, :, 1].astype(np.float32) - med_s)

        red_mask1 = cv2.inRange(hsv, np.array([0, 120, 60]), np.array([10, 255, 255]))
        red_mask2 = cv2.inRange(hsv, np.array([165, 120, 60]), np.array([180, 255, 255]))
        anomaly_mask = np.where(
            ((val_diff > 52.0) | ((val_diff > 35.0) & (sat_diff > 45.0)) | (red_mask1 > 0) | (red_mask2 > 0))
            & (veg_mask == 0),
            255,
            0,
        ).astype(np.uint8)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        anomaly_mask = cv2.morphologyEx(anomaly_mask, cv2.MORPH_OPEN, kernel)
        anomaly_mask = cv2.morphologyEx(anomaly_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(anomaly_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_obs_area = max(550, int(w * h * 0.0055))
        max_obs_area = int(w * h * 0.42)
        opencv_dets: List[Dict[str, Any]] = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_obs_area or area > max_obs_area:
                continue

            bx, by, bw, bh = cv2.boundingRect(cnt)
            aspect_box = float(bw) / max(1.0, float(bh))
            if bw < int(w * 0.085) and aspect_box < 0.45:
                continue
            if aspect_box > 4.2 or aspect_box < 0.22:
                continue

            solidity = float(area) / max(1.0, float(bw * bh))
            if solidity < 0.40:
                continue

            patch_grad = grad_mag[by : by + bh, bx : bx + bw]
            if patch_grad.size == 0 or float(np.mean(patch_grad)) < 14.0:
                continue

            abs_y_bot = roi_top + by + bh
            abs_y_top = roi_top + by
            opencv_dets.append({
                "source": "opencv_sobel",
                "class_id": -1,
                "class_name": "obstacle_sol",
                "conf": 0.85,
                "bbox": [float(bx), float(abs_y_top), float(bx + bw), float(abs_y_bot)],
                "x": int(bx),
                "y": int(abs_y_top),
                "w": int(bw),
                "h": int(bh),
                "cx": round(bx + bw / 2.0, 2),
                "cy": round(abs_y_top + bh / 2.0, 2),
                "y_bottom": float(abs_y_bot),
                "is_mobile_class": False,
                "is_moving": False,
            })

        # 3. Fusion YOLOv8n ONNX + OpenCV & suivi multi-objets par centroïde
        merged_dets = merge_yolo_and_opencv_obstacles(yolo_dets, opencv_dets, iou_thresh=0.35)
        tracked_dets = self.obstacle_tracker.update(merged_dets)

        fov_deg = max(30.0, min(170.0, float(self.fov_h_deg)))
        fx = (float(w) / 2.0) / math.tan(math.radians(fov_deg / 2.0))
        half_robot_w = self.chassis_width_m / 2.0
        corridor_margin = 0.08

        corridor_blockers: List[Dict[str, Any]] = []
        for tr in tracked_dets:
            x1, y1, x2, y2 = [int(round(v)) for v in tr["bbox"]]
            bw_px = max(1, x2 - x1)
            bh_px = max(1, y2 - y1)
            d_sol, x_sol = self.compute_ground_projection(tr["y_bottom"], tr["cx"], w, h)
            obs_w_m = round(max(0.25, min(2.5, d_sol * (bw_px / max(1.0, fx)))), 3)
            collision_half_w = (obs_w_m / 2.0) + half_robot_w + corridor_margin
            in_corridor = abs(x_sol) <= collision_half_w

            tr["x"] = x1
            tr["y"] = y1
            tr["w"] = bw_px
            tr["h"] = bh_px
            tr["d_sol_m"] = d_sol
            tr["x_sol_m"] = x_sol
            tr["dist_m"] = d_sol
            tr["x_m"] = x_sol
            tr["width_m"] = obs_w_m
            tr["in_corridor"] = in_corridor

            if d_sol <= 6.0:
                is_mob = bool(tr.get("is_moving") or tr.get("is_mobile_class"))
                state_tag = "MOBILE 5.0s" if is_mob else "STATIQUE"
                conf_pct = int(round(float(tr.get("conf", 0.0)) * 100))
                box_color = (
                    (0, 0, 255)
                    if (in_corridor and d_sol <= 1.45)
                    else ((0, 165, 255) if (in_corridor or is_mob) else (255, 210, 0))
                )
                cv2.rectangle(vis, (x1, y1), (x2, y2), box_color, 2 if not in_corridor else 3)
                hud_box_txt = f"#{tr['track_id']} {tr['class_name'].upper()} {conf_pct}% | D={d_sol:.2f}m | {state_tag}"
                cv2.putText(
                    vis,
                    hud_box_txt,
                    (max(8, x1), max(82, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.48,
                    box_color,
                    2,
                    cv2.LINE_AA,
                )

            if in_corridor and d_sol <= 4.5:
                corridor_blockers.append(tr)

        telemetry["yolo_detections"] = tracked_dets
        telemetry["obstacles"] = tracked_dets

        if corridor_blockers:
            primary = min(corridor_blockers, key=lambda o: o["d_sol_m"])
            min_obs_dist = float(primary["d_sol_m"])
            bx = int(primary["x"])
            bw = int(primary["w"])
            left_free_px = bx
            right_free_px = w - (bx + bw)
            bypass_side = 1 if left_free_px >= right_free_px else -1
            side_str = "GAUCHE" if bypass_side > 0 else "DROITE"

            telemetry["obstacle_detected"] = True
            telemetry["obstacle_in_corridor"] = True
            telemetry["obstacle_dist_m"] = round(min_obs_dist, 2)
            telemetry["min_distance_m"] = round(min_obs_dist, 2)
            telemetry["recommended_bypass_side"] = bypass_side
            telemetry["blocking_obstacle_class"] = primary["class_name"]
            telemetry["blocking_obstacle_conf"] = round(float(primary["conf"]), 3)
            telemetry["blocking_obstacle_track_id"] = int(primary["track_id"])
            telemetry["blocking_obstacle_mobile"] = bool(primary.get("is_moving") or primary.get("is_mobile_class"))

            is_critical = min_obs_dist <= 1.45
            telemetry["obstacle_stop"] = is_critical
            telemetry["emergency_stop"] = is_critical
            p_conf = int(round(float(primary["conf"]) * 100))
            p_state = "MOBILE 5.0s" if telemetry["blocking_obstacle_mobile"] else "STATIQUE"
            telemetry["hud_obstacle_summary"] = (
                f"#{primary['track_id']} {primary['class_name'].upper()} {p_conf}% | {min_obs_dist:.2f}m | {p_state}"
            )

            if is_critical:
                telemetry["hud_message"] = (
                    f"{primary['class_name'].upper()} #{primary['track_id']} ({p_conf}%) À {min_obs_dist:.2f}m [{p_state}] -> {side_str}"
                )
                telemetry["recommended_drive"] = {"a": 0, "d": 0, "ms": 500}
                telemetry["recommended_tools"] = {"tines_depth": 0.0, "cutter_m6": 0}
        else:
            telemetry["obstacle_detected"] = False
            telemetry["obstacle_in_corridor"] = False
            telemetry["min_distance_m"] = 99.0
            telemetry["obstacle_stop"] = False
            telemetry["emergency_stop"] = False
            telemetry["recommended_bypass_side"] = 0
            if tracked_dets:
                first_obj = min(tracked_dets, key=lambda o: o["d_sol_m"])
                f_conf = int(round(float(first_obj["conf"]) * 100))
                f_state = "MOBILE 5.0s" if (first_obj.get("is_moving") or first_obj.get("is_mobile_class")) else "STATIQUE"
                telemetry["hud_obstacle_summary"] = (
                    f"#{first_obj['track_id']} {first_obj['class_name'].upper()} {f_conf}% | {first_obj['d_sol_m']:.2f}m | {f_state}"
                )

    def _process_mission_seeding(self, vis: np.ndarray, w: int, h: int, telemetry: Dict[str, Any]) -> None:
        """Mission 1 (SEEDING) : Détection et suivi haute précision de la ligne directrice tracée au sol
        via Filtre Morphologique Black-Hat + CLAHE + Fenêtres Glissantes Horizontales + Régression de Huber.
        """
        roi_top = int(h * 0.35)
        roi_bot = int(h * 0.92)
        roi = vis[roi_top:roi_bot, :]
        roi_h, roi_w = roi.shape[:2]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        enhanced = self._clahe.apply(gray)
        blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)

        # 1. Filtre Morphologique Black-Hat : extrait spécifiquement les sillons/lignes sombres étroits
        #    (insensible aux ombres larges et aux variations globales de luminosité sur le sable !)
        kw = max(15, int(w * 0.045) | 1)
        kernel_bh = cv2.getStructuringElement(cv2.MORPH_RECT, (kw, 7))
        blackhat = cv2.morphologyEx(blurred, cv2.MORPH_BLACKHAT, kernel_bh)

        # Restreindre la recherche au couloir central autour de l'axe du robot (±35% de la largeur)
        cx = w / 2.0
        search_half = int(w * 0.34)
        x_min_s = max(0, int(cx - search_half))
        x_max_s = min(w, int(cx + search_half))

        # 2. Échantillonnage par 6 bandes horizontales (du bas près des roues vers l'horizon)
        num_bands = 6
        band_h = max(8, roi_h // num_bands)
        band_pts_x: List[float] = []
        band_pts_y: List[float] = []

        for b_idx in range(num_bands):
            by1 = b_idx * band_h
            by2 = min(roi_h, (b_idx + 1) * band_h)
            strip_bh = blackhat[by1:by2, x_min_s:x_max_s]
            strip_gray = blurred[by1:by2, x_min_s:x_max_s]
            if strip_bh.size == 0:
                continue

            # Profil horizontal combinant la réponse Black-Hat (creux sombre) et l'inversion de luminance locale
            col_profile = np.mean(strip_bh.astype(np.float32), axis=0) + 0.35 * np.maximum(
                0.0, float(np.median(strip_gray)) - np.mean(strip_gray.astype(np.float32), axis=0)
            )
            peak_local_x = int(np.argmax(col_profile))
            peak_val = float(col_profile[peak_local_x])
            median_val = float(np.median(col_profile))

            if peak_val > median_val + 4.5:
                abs_px = float(x_min_s + peak_local_x)
                abs_py = float(roi_top + (by1 + by2) / 2.0)
                band_pts_x.append(abs_px)
                band_pts_y.append(abs_py)
                cv2.circle(vis, (int(abs_px), int(abs_py)), 4, (0, 220, 255), -1)

        found_track = False
        detected_bot_x = cx
        detected_top_x = cx
        raw_heading_deg = 0.0

        if len(band_pts_x) >= 3:
            pts_np = np.column_stack((np.array(band_pts_x), np.array(band_pts_y))).astype(np.float32)
            line_params = cv2.fitLine(pts_np, cv2.DIST_HUBER, 0, 0.01, 0.01)
            vx, vy, x0, y0 = [float(v) for v in line_params.flatten()]
            if abs(vy) > 1e-4:
                slope = vx / vy
                ang = math.degrees(math.atan(slope))
                if abs(ang) < 42.0:
                    detected_bot_x = float(np.clip(x0 + (roi_bot - y0) * slope, 10.0, w - 10.0))
                    detected_top_x = float(np.clip(x0 + (roi_top - y0) * slope, 10.0, w - 10.0))
                    raw_heading_deg = ang
                    found_track = True
                    cv2.line(
                        vis,
                        (int(detected_bot_x), roi_bot),
                        (int(detected_top_x), roi_top),
                        (0, 255, 255),
                        3,
                        cv2.LINE_AA,
                    )

        # Calcul des écarts physiques (normalisé selon la résolution w) et lissage EMA
        scale_w = 640.0 / max(1.0, float(w))
        raw_err_cm = (detected_bot_x - cx) * 0.28 * scale_w
        err_cm, heading_err_deg = self._apply_temporal_smoothing(raw_err_cm, raw_heading_deg, found_track)

        telemetry["target_detected"] = found_track
        telemetry["guideline_detected"] = found_track
        telemetry["row_detected"] = found_track
        telemetry["lateral_error_cm"] = err_cm
        telemetry["heading_error_deg"] = heading_err_deg
        telemetry["is_aligned"] = bool(abs(err_cm) < 4.0 and abs(heading_err_deg) < 2.5)

        steer_cmd = 0
        if not telemetry.get("obstacle_stop", False):
            if telemetry["is_aligned"]:
                telemetry["recommended_drive"] = {"a": 25, "d": 0, "ms": 250}
                telemetry["hud_message"] = "SEMIS ACTIF — SILLON CENTRAL VERROUILLÉ (0cm, 0.0°)"
            else:
                steer_cmd = int(np.clip(-1.35 * heading_err_deg - 0.70 * err_cm, -30, 30))
                telemetry["recommended_drive"] = {"a": 22, "d": steer_cmd, "ms": 250}
                telemetry["hud_message"] = f"SUIVI SILLON : Écart {err_cm:+.1f}cm | Cap {heading_err_deg:+.1f}° -> Steer {steer_cmd:+d}"

            telemetry["recommended_steer"] = steer_cmd
            telemetry["seeder_m8_active"] = True
            telemetry["recommended_tools"] = {"seeder_m8": 60, "tines_depth": 0.0}

        # Dessin du réticule de centrage
        t_color = (0, 255, 0) if telemetry["is_aligned"] else (0, 165, 255)
        cv2.circle(vis, (int(detected_bot_x), int(h * 0.75)), 8, t_color, -1)
        cv2.putText(
            vis,
            f"Sillon: {err_cm:+.1f}cm ({heading_err_deg:+.1f}deg)",
            (max(10, int(detected_bot_x) - 65), int(h * 0.75) - 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            t_color,
            2,
        )

    def _process_mission_reseeding(self, vis: np.ndarray, w: int, h: int, telemetry: Dict[str, Any]) -> None:
        """Mission 2 (RESEEDING) : Suivi actif des 2 rangs d'arachide (-23 cm et +23 cm) par segmentation ExG+HSV
        ET détection des trous de germination (> 25 cm) pour déclencher PULSE:8:60:400.
        """
        mask_green = self._compute_vegetation_mask(vis)

        gaps_found = 0
        reseed_trigger = False
        cx = w / 2.0
        y_start = int(h * 0.90)
        y_end = int(h * 0.38)
        step_px = max(10, int(h * 0.025))
        scale_h = 480.0 / max(1.0, float(h))
        scale_w = 640.0 / max(1.0, float(w))

        row_bot_xs: List[float] = []
        row_angles: List[float] = []

        # Analyser et ajuster chacun des deux rangs attendus à -23 cm (Gauche) et +23 cm (Droite)
        for side_sign, label in ((-1, "RANG GAUCHE"), (1, "RANG DROIT")):
            top_pt, bot_pt = self._project_ground_corridor(w, h, side_sign * self.half_tool_m)
            expected_bot_x = float(bot_pt[0])
            rx = int((top_pt[0] + bot_pt[0]) / 2.0)
            strip_w = int(w * 0.14)
            x1 = max(0, rx - strip_w // 2)
            x2 = min(w, rx + strip_w // 2)

            # Ajustement robuste (Huber fitLine) du rang d'arachide dans sa bande
            strip_mask = mask_green[y_end:y_start, x1:x2]
            ys_s, xs_s = np.where(strip_mask > 0)
            if len(xs_s) >= 20:
                ok_fit, r_bot_x, r_top_x, r_ang = self._fit_robust_line(
                    xs_s + x1, ys_s + y_end, y_end, y_start, w, expected_bot_x
                )
                if ok_fit:
                    row_bot_xs.append(r_bot_x - (expected_bot_x - cx))
                    row_angles.append(r_ang)
                    cv2.line(vis, (int(r_bot_x), y_start), (int(r_top_x), y_end), (0, 255, 120), 2, cv2.LINE_AA)

            # Échantillonnage longitudinal des manques de germination du bas vers le haut
            consecutive_empty_px = 0
            max_empty_px = 0
            gap_y_center = y_start

            for y in range(y_start, y_end, -step_px):
                patch = mask_green[max(0, y - step_px) : y, x1:x2]
                green_pixels = cv2.countNonZero(patch)
                if green_pixels < max(12, int((x2 - x1) * step_px * 0.04)):
                    consecutive_empty_px += step_px
                    if consecutive_empty_px > max_empty_px:
                        max_empty_px = consecutive_empty_px
                        gap_y_center = y + consecutive_empty_px // 2
                else:
                    consecutive_empty_px = 0
                    cv2.rectangle(vis, (x1 + 4, y - step_px), (x2 - 4, y), (0, 200, 0), 1)

            gap_len_cm = max_empty_px * 0.35 * scale_h
            if gap_len_cm >= 25.0:
                gaps_found += 1
                reseed_trigger = True
                cv2.rectangle(
                    vis,
                    (x1, gap_y_center - max_empty_px // 2),
                    (x2, gap_y_center + max_empty_px // 2),
                    (0, 0, 255),
                    2,
                )
                cv2.putText(
                    vis,
                    f"MANQUE {gap_len_cm:.0f}cm",
                    (max(6, x1 - 10), gap_y_center),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (0, 0, 255),
                    2,
                )

        # Calcul de la correction de suivi des rangs d'arachide en mode Révision
        found_rows = len(row_bot_xs) > 0
        raw_center_x = float(np.mean(row_bot_xs)) if found_rows else cx
        raw_head_deg = float(np.mean(row_angles)) if found_rows else 0.0
        raw_err_cm = (raw_center_x - cx) * 0.28 * scale_w
        err_cm, heading_err_deg = self._apply_temporal_smoothing(raw_err_cm, raw_head_deg, found_rows)
        steer_cmd = int(np.clip(-1.3 * heading_err_deg - 0.65 * err_cm, -28, 28)) if not (abs(err_cm) < 3.5 and abs(heading_err_deg) < 2.0) else 0

        telemetry["row_detected"] = found_rows
        telemetry["target_detected"] = found_rows
        telemetry["guideline_detected"] = found_rows
        telemetry["lateral_error_cm"] = err_cm
        telemetry["heading_error_deg"] = heading_err_deg
        telemetry["is_aligned"] = bool(abs(err_cm) < 4.0 and abs(heading_err_deg) < 2.5)
        telemetry["recommended_steer"] = steer_cmd
        telemetry["reseed_required"] = reseed_trigger
        telemetry["reseed_gaps_count"] = gaps_found
        telemetry["gaps_count"] = gaps_found
        telemetry["reseed_pulse_triggered"] = reseed_trigger
        telemetry["seeder_m8_active"] = reseed_trigger

        if not telemetry.get("obstacle_stop", False):
            if reseed_trigger:
                telemetry["recommended_drive"] = {"a": 16, "d": steer_cmd, "ms": 250}
                telemetry["recommended_tools"] = {"seeder_m8": 60, "pulse_seed": True}
                telemetry["hud_message"] = f"TROU DÉTECTÉ ({gaps_found}) — RESEMIS M8 ACTIF | Steer {steer_cmd:+d}"
            else:
                telemetry["recommended_drive"] = {"a": 25, "d": steer_cmd, "ms": 250}
                telemetry["recommended_tools"] = {"seeder_m8": 0}
                telemetry["hud_message"] = f"RANGS D'ARACHIDE SUIVIS (dx:{err_cm:+.1f}cm) — SEMOIR EN VEILLE"

    def _process_mission_weeding(self, vis: np.ndarray, w: int, h: int, telemetry: Dict[str, Any]) -> None:
        """Mission 3 (WEEDING) : Détection et suivi actif des lignes d'arachide (ExG + HSV + Huber fitLine)
        pour maintenir le rang d'arachide sous le tunnel protégé et guider les dents M5 / fraise M6 (-23cm et +23cm)
        strictement dans les couloirs inter-rangs.
        """
        mask_green = self._compute_vegetation_mask(vis)

        cx = w // 2
        y_top = int(h * 0.36)
        y_bot = int(h * 0.92)
        scale_w = 640.0 / max(1.0, float(w))

        # 1. Détection et suivi de la ligne d'arachide (rang central enjambé sous le châssis OU couple de rangs latéraux)
        central_search_half = int(w * 0.14)
        c_x1 = max(0, cx - central_search_half)
        c_x2 = min(w, cx + central_search_half)
        central_crop_mask = mask_green[y_top:y_bot, c_x1:c_x2]
        ys_c, xs_c = np.where(central_crop_mask > 0)

        found_crop_row = False
        row_bot_x = float(cx)
        row_top_x = float(cx)
        row_angle_deg = 0.0

        if len(xs_c) >= 25:
            found_crop_row, row_bot_x, row_top_x, row_angle_deg = self._fit_robust_line(
                xs_c + c_x1, ys_c + y_top, y_top, y_bot, w, float(cx)
            )
        else:
            # Si le robot est entre 2 rangs d'arachide (à gauche et à droite), ajuster les 2 rangs et viser leur milieu !
            ys_l, xs_l = np.where(mask_green[y_top:y_bot, :cx] > 0)
            ys_r, xs_r = np.where(mask_green[y_top:y_bot, cx:] > 0)
            ok_l, bot_l, top_l, ang_l = self._fit_robust_line(xs_l, ys_l + y_top, y_top, y_bot, w, w * 0.25)
            ok_r, bot_r, top_r, ang_r = self._fit_robust_line(xs_r + cx, ys_r + y_top, y_top, y_bot, w, w * 0.75)
            if ok_l and ok_r:
                found_crop_row = True
                row_bot_x = (bot_l + bot_r) / 2.0
                row_top_x = (top_l + top_r) / 2.0
                row_angle_deg = (ang_l + ang_r) / 2.0
                cv2.line(vis, (int(bot_l), y_bot), (int(top_l), y_top), (0, 220, 90), 2, cv2.LINE_AA)
                cv2.line(vis, (int(bot_r), y_bot), (int(top_r), y_top), (0, 220, 90), 2, cv2.LINE_AA)

        if found_crop_row:
            cv2.line(vis, (int(row_bot_x), y_bot), (int(row_top_x), y_top), (0, 255, 100), 3, cv2.LINE_AA)

        # Calcul de l'erreur latérale et angulaire de suivi du rang d'arachide avec lissage EMA
        raw_err_cm = (row_bot_x - cx) * 0.28 * scale_w
        err_cm, heading_err_deg = self._apply_temporal_smoothing(raw_err_cm, row_angle_deg, found_crop_row)
        is_aligned = bool(abs(err_cm) < 4.0 and abs(heading_err_deg) < 2.5)
        steer_cmd = 0 if is_aligned else int(np.clip(-1.35 * heading_err_deg - 0.65 * err_cm, -30, 30))

        telemetry["row_detected"] = found_crop_row
        telemetry["target_detected"] = found_crop_row
        telemetry["guideline_detected"] = found_crop_row
        telemetry["lateral_error_cm"] = err_cm
        telemetry["heading_error_deg"] = heading_err_deg
        telemetry["is_aligned"] = is_aligned
        telemetry["recommended_steer"] = steer_cmd

        # 2. Zone protégée du rang central d'arachide (autour du rang détecté, entre -12cm et +12cm)
        safe_center_x = int(row_bot_x) if found_crop_row else cx
        safe_half_px = int(w * 0.085)
        sx1 = max(0, safe_center_x - safe_half_px)
        sx2 = min(w, safe_center_x + safe_half_px)
        cv2.rectangle(vis, (sx1, y_top), (sx2, y_bot), (0, 230, 80), 2)
        cv2.putText(vis, "RANG ARACHIDE PROTEGE", (max(8, sx1 - 20), y_top - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 120), 1)

        # 3. Masquer la zone protégée du rang d'arachide pour ne chercher que les adventices dans les couloirs M5/M6 (±23 cm)
        mask_weeds = mask_green.copy()
        mask_weeds[:, sx1:sx2] = 0
        # Ignorer les bords extérieurs hors gabarit du robot
        outer_margin = int(w * 0.14)
        mask_weeds[:, :outer_margin] = 0
        mask_weeds[:, w - outer_margin :] = 0

        contours, _ = cv2.findContours(mask_weeds, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        weed_count = 0
        min_weed_area = max(45, int(w * h * 0.00018))

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_weed_area:
                continue

            bx, by, bw, bh = cv2.boundingRect(cnt)
            if by + bh < y_top or by > y_bot:
                continue

            weed_count += 1
            cv2.rectangle(vis, (bx, by), (bx + bw, by + bh), (0, 140, 255), 2)
            cv2.putText(vis, "ADVENTICE", (bx, max(72, by - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 140, 255), 1)

        telemetry["weeds_detected_count"] = weed_count
        telemetry["tines_m5_lowered"] = bool(weed_count > 0)
        telemetry["cutter_m6_active"] = bool(weed_count > 0)

        if not telemetry.get("obstacle_stop", False):
            if weed_count > 0:
                telemetry["recommended_drive"] = {"a": 22, "d": steer_cmd, "ms": 250}
                telemetry["recommended_tools"] = {"tines_depth": 1.0, "cutter_m6": 70}
                telemetry["hud_message"] = (
                    f"{weed_count} ADVENTICES INTER-RANGS | dx:{err_cm:+.1f}cm Steer:{steer_cmd:+d} | M5+M6 ACTIFS"
                )
            else:
                telemetry["recommended_drive"] = {"a": 25, "d": steer_cmd, "ms": 250}
                telemetry["recommended_tools"] = {"tines_depth": 0.5, "cutter_m6": 0}
                telemetry["hud_message"] = f"SUIVI RANG ARACHIDE (dx:{err_cm:+.1f}cm, Steer:{steer_cmd:+d}) — COULOIRS PROPRES"

    def _process_mission_obstacle_only(self, vis: np.ndarray, w: int, h: int, telemetry: Dict[str, Any]) -> None:
        """Mission 4 (OBSTACLE_AVOIDANCE) : Surveillance du couloir 1.21 m et contournement actif."""
        if not telemetry.get("obstacle_detected", False):
            telemetry["recommended_drive"] = {"a": 25, "d": 0, "ms": 250}
            telemetry["hud_message"] = "VOIE DÉGAGÉE (CHÂSSIS 1.21m SANS OBSTACLE)"

    def validate_peanut_row_hybrid(
        self,
        frame: np.ndarray,
        roi_box: Optional[Tuple[int, int, int, int]] = None,
        mission_type: str = "WEEDING",
    ) -> Dict[str, Any]:
        """Double Validation Binaire Hybride : OpenCV (Alignement ExG) + MobileNetV3-Small ONNX (Arachide vs Sol Nu).
        Une ligne n'est validée (`is_valid_peanut_row == True`) QUE SI :
            `opencv_line_aligned == True` ET `classifier.is_peanut == True` (`peanut_confidence >= 0.65`).
        Dès que la ligne d'arachide laisse place au sol nu (`is_valid_peanut_row == False`), le robot s'arrête.
        """
        if frame is None or frame.size == 0:
            return {
                "is_valid_peanut_row": False,
                "opencv_line_aligned": False,
                "is_peanut": False,
                "peanut_confidence": 0.0,
                "detected_species_label": "Sol Nu — Fin de ligne d'arachide",
                "crops_count": 0,
                "onnx_loaded": bool(getattr(self.peanut_classifier, "onnx_loaded", False)),
                "hud_badge": "🛑 FIN DE LIGNE D'ARACHIDE : Sol nu détecté -> Arrêt / Demi-tour",
                "hud_alert": True,
                "boundary_action": "UTURN_LINE_END",
            }

        fh, fw = frame.shape[:2]
        if roi_box is not None and len(roi_box) >= 4:
            x1, y1, x2, y2 = [int(v) for v in roi_box[:4]]
            x1 = max(0, min(fw - 1, x1))
            y1 = max(0, min(fh - 1, y1))
            x2 = max(x1 + 1, min(fw, x2))
            y2 = max(y1 + 1, min(fh, y2))
        else:
            x1, y1, x2, y2 = int(fw * 0.12), int(fh * 0.25), int(fw * 0.88), int(fh * 0.96)

        roi = frame[y1:y2, x1:x2]
        rh, rw = roi.shape[:2]
        mask_veg = self._compute_vegetation_mask(roi)

        # Étape 1 : OpenCV (Filtre Géométrique Rapide — alignement colinéaire)
        # Sélectionner la bande de rang dominante (centrale ou latérale gauche/droite)
        cx_roi = rw // 2
        center_slice = mask_veg[:, max(0, cx_roi - int(rw * 0.22)) : min(rw, cx_roi + int(rw * 0.22))]
        if np.count_nonzero(center_slice) >= 35:
            row_mask = np.zeros_like(mask_veg)
            c1 = max(0, cx_roi - int(rw * 0.22))
            c2 = min(rw, cx_roi + int(rw * 0.22))
            row_mask[:, c1:c2] = mask_veg[:, c1:c2]
        else:
            left_nz = int(np.count_nonzero(mask_veg[:, :cx_roi]))
            right_nz = int(np.count_nonzero(mask_veg[:, cx_roi:]))
            row_mask = np.zeros_like(mask_veg)
            if left_nz >= right_nz:
                row_mask[:, :cx_roi] = mask_veg[:, :cx_roi]
            else:
                row_mask[:, cx_roi:] = mask_veg[:, cx_roi:]

        ys_r, xs_r = np.where(row_mask > 0)
        opencv_line_aligned = False
        line_angle_deg = 0.0
        bot_x = float(cx_roi)
        top_x = float(cx_roi)

        if len(xs_r) >= 25:
            ok_fit, bot_x, top_x, line_angle_deg = self._fit_robust_line(
                xs_r, ys_r, 0, rh - 1, rw, float(cx_roi)
            )
            if ok_fit:
                # Vérifier l'alignement colinéaire des centroïdes / pixels le long de la droite ajustée
                dx_line = top_x - bot_x
                dy_line = float(0 - (rh - 1))
                norm_l = max(1.0, math.hypot(dx_line, dy_line))
                perp_dists = np.abs(
                    (xs_r.astype(np.float32) - bot_x) * dy_line - (ys_r.astype(np.float32) - (rh - 1)) * dx_line
                ) / norm_l
                inliers_ratio = float(np.mean(perp_dists <= max(18.0, rw * 0.16)))
                if inliers_ratio >= 0.55:
                    opencv_line_aligned = True

        # Extraire les 3 meilleures vignettes (ROI crops) le long de la ligne pour la validation biologique
        crops: List[np.ndarray] = []
        contours, _ = cv2.findContours(row_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidate_cnts = sorted(
            [c for c in contours if cv2.contourArea(c) >= 25.0],
            key=cv2.contourArea,
            reverse=True,
        )

        # Préserver le pourtour sablonneux autour de chaque plante (padding ~35%) pour préserver sa silhouette foliaire
        for cnt in candidate_cnts[:3]:
            bx, by, bw, bh = cv2.boundingRect(cnt)
            pad = max(10, int(max(bw, bh) * 0.35))
            cx1 = max(0, bx - pad)
            cy1 = max(0, by - pad)
            cx2 = min(rw, bx + bw + pad)
            cy2 = min(rh, by + bh + pad)
            crop = roi[cy1:cy2, cx1:cx2]
            if crop.size > 0:
                crops.append(crop)

        # Si les plantes forment une haie continue (1 seul grand contour), découper 3 vignettes longitudinales
        if len(crops) < 2 and len(xs_r) >= 25:
            band_crops: List[np.ndarray] = []
            for k in range(3):
                by1 = int((rh * k) / 3.0)
                by2 = int((rh * (k + 1)) / 3.0)
                t_mid = (k + 0.5) / 3.0
                x_mid = int(bot_x + (top_x - bot_x) * (1.0 - t_mid))
                half_c = max(28, int(rw * 0.14))
                cx1 = max(0, x_mid - half_c)
                cx2 = min(rw, x_mid + half_c)
                band_mask = row_mask[by1:by2, cx1:cx2]
                if np.count_nonzero(band_mask) >= 12:
                    sub = roi[by1:by2, cx1:cx2]
                    if sub.size > 0:
                        band_crops.append(sub)
            if len(band_crops) >= len(crops):
                crops = band_crops[:3]

        # Étape 2 : Modèle Binaire MobileNetV3-Small ONNX / ExG (Arachide vs Sol Nu)
        clf_res = self.peanut_classifier.verify_peanut_crop(crops)
        is_peanut = bool(clf_res.get("is_peanut", False))
        peanut_conf = float(clf_res.get("peanut_confidence", 0.0))
        species_label = str(clf_res.get("detected_species_label", "Sol Nu — Fin de ligne d'arachide"))

        # Étape 3 : Décision Binaire Combinée (Alignement OpenCV + Arachide confirmée)
        conf_thresh = getattr(self.peanut_classifier, "confidence_threshold", 0.65)
        is_valid_peanut_row = bool(opencv_line_aligned and is_peanut and peanut_conf >= conf_thresh)

        conf_pct = int(round(peanut_conf * 100))
        if is_valid_peanut_row:
            hud_badge = f"🥜 IA Botanique : Arachide détectée ({conf_pct}%) + 📐 OpenCV : Rang aligné"
            hud_alert = False
            boundary_action = "CONTINUE"
        else:
            species_label = "Sol Nu — Fin de ligne d'arachide"
            hud_badge = "🛑 FIN DE LIGNE D'ARACHIDE : Sol nu détecté -> Arrêt / Demi-tour"
            hud_alert = True
            boundary_action = "UTURN_LINE_END"

        return {
            "is_valid_peanut_row": is_valid_peanut_row,
            "opencv_line_aligned": opencv_line_aligned,
            "is_peanut": is_peanut,
            "peanut_confidence": round(peanut_conf, 4),
            "detected_species_label": species_label,
            "crops_count": len(crops),
            "line_angle_deg": round(line_angle_deg, 2),
            "mission_type": mission_type,
            "model_backend": clf_res.get("model_backend", self.peanut_classifier.model_backend),
            "onnx_loaded": bool(getattr(self.peanut_classifier, "onnx_loaded", False)),
            "hud_badge": hud_badge,
            "hud_alert": hud_alert,
            "boundary_action": boundary_action,
        }

    def _draw_hud_banner(self, vis: np.ndarray, w: int, h: int, telemetry: Dict[str, Any]) -> None:
        """Affiche le bandeau supérieur translucide avec la télémétrie de vol et l'ordre robot en temps réel."""
        banner_h = 38
        overlay = vis.copy()
        cv2.rectangle(overlay, (0, 0), (w, banner_h), (12, 18, 30), -1)
        cv2.rectangle(overlay, (0, banner_h), (w, banner_h + 28), (18, 28, 45), -1)
        cv2.addWeighted(overlay, 0.85, vis, 0.15, 0, vis)

        mode_str = telemetry.get("mode", "SEEDING")
        msg = telemetry.get("hud_message", "")
        err_cm = telemetry.get("lateral_error_cm", 0.0)
        deg = telemetry.get("heading_error_deg", 0.0)
        aspect = telemetry.get("aspect_ratio", "16:9")
        order_str = telemetry.get("robot_order_str", "DRIVE:25,0:250")

        pitch_auto = telemetry.get("auto_pitch_deg", getattr(self, "auto_pitch_deg", 32.0))
        badge_color = (0, 220, 100) if not telemetry.get("obstacle_stop", False) else (0, 0, 255)
        cv2.putText(vis, f"CAM T'nB [{mode_str}] ({w}x{h} {aspect})", (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 2)
        cv2.putText(vis, f"dx: {err_cm:+.1f}cm | dTheta: {deg:+.1f}deg | Angle Cam Auto: {pitch_auto:.1f}deg", (305, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 230, 255), 1)

        if msg:
            text_size = cv2.getTextSize(msg, cv2.FONT_HERSHEY_SIMPLEX, 0.44, 1)[0]
            cv2.putText(vis, msg, (max(w - text_size[0] - 14, 520), 24), cv2.FONT_HERSHEY_SIMPLEX, 0.44, badge_color, 1)

        cv2.putText(
            vis,
            f"[VISION IA -> ORDRE ROBOT] {order_str}",
            (12, banner_h + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (80, 255, 160) if not telemetry.get("obstacle_stop", False) else (80, 80, 255),
            1,
        )
        obs_hud = telemetry.get("hud_obstacle_summary")
        yolo_tag = "YOLOv8n.onnx" if telemetry.get("yolo_onnx_loaded") else "OpenCV"
        chip_txt = f"[{yolo_tag}] {obs_hud}" if obs_hud else f"[{yolo_tag} + MobileNetV3]"
        chip_sz = cv2.getTextSize(chip_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.40, 1)[0]
        cv2.putText(
            vis,
            chip_txt,
            (max(12, w - chip_sz[0] - 12), banner_h + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (255, 220, 100),
            1,
        )

        # Badge Hybride Binaire (Arachide vs Sol Nu + OpenCV)
        hybrid_badge = telemetry.get("hybrid_hud_badge")
        if hybrid_badge and mode_str in ("WEEDING", "RESEEDING"):
            is_alert = bool(telemetry.get("hybrid_boundary_alert", False))
            conf_pct = int(round(float(telemetry.get("peanut_confidence", 0.0)) * 100))
            if not is_alert:
                ascii_badge = f"IA Botanique : Arachide detectee ({conf_pct}%) + OpenCV : Rang aligne"
                b_bg, b_fg = (12, 85, 35), (120, 255, 150)
            else:
                ascii_badge = "FIN DE LIGNE ARACHIDE : Sol nu detecte -> Arret / Demi-tour"
                b_bg, b_fg = (20, 20, 145), (150, 175, 255)
            cv2.rectangle(vis, (12, h - 42), (w - 12, h - 14), b_bg, -1)
            cv2.putText(vis, ascii_badge, (20, h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.45, b_fg, 2)

    def generate_synthetic_farm_frame(
        self, width: int = 640, height: int = 480, frame_idx: int = 0, mission: Optional[str] = None
    ) -> np.ndarray:
        """Génère une image réaliste d'un champ d'arachide au Niger pour tester à 100% sans caméra branchée."""
        frame = np.full((height, width, 3), (92, 168, 224), dtype=np.uint8)

        noise = np.random.randint(-12, 12, (height, width, 3), dtype=np.int16)
        frame = np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        horizon_y = int(height * 0.28)
        frame[:horizon_y, :] = (235, 215, 175)

        vib = int(math.sin(frame_idx * 0.15) * 4)
        cx = width // 2 + vib
        pt_c_top, pt_c_bot = (cx, horizon_y), (cx, height)
        cv2.line(frame, (pt_c_top[0], pt_c_top[1]), (pt_c_bot[0], pt_c_bot[1]), (35, 75, 115), max(6, int(width * 0.012)))

        # Rangs d'arachide verts réguliers à -23 cm et +23 cm
        for side_sign in (-1, 1):
            top_pt, bot_pt = self._project_ground_corridor(width, height, side_sign * self.half_tool_m)
            for step in range(12):
                t = step / 11.0
                px = int(top_pt[0] + (bot_pt[0] - top_pt[0]) * t) + vib
                py = int(top_pt[1] + (bot_pt[1] - top_pt[1]) * t)
                radius = int((8 + t * 14) * (width / 640.0))
                if step in (4, 5) and side_sign == 1:
                    continue
                cv2.circle(frame, (px, py), radius, (35, 160, 45), -1)
                cv2.circle(frame, (px + 3, py - 2), max(2, radius // 2), (55, 205, 75), -1)

        # Quelques adventices simulées dans les inter-rangs
        cv2.circle(frame, (cx - int(75 * width / 640.0) + vib, int(height * 0.65)), int(9 * width / 640.0), (25, 145, 35), -1)
        cv2.circle(frame, (cx + int(85 * width / 640.0) + vib, int(height * 0.78)), int(11 * width / 640.0), (30, 155, 40), -1)

        return frame


def validate_peanut_row_hybrid(
    frame: np.ndarray,
    roi_box: Optional[Tuple[int, int, int, int]] = None,
    mission_type: str = "WEEDING",
    classifier: Optional[PeanutPlantClassifier] = None,
) -> Dict[str, Any]:
    """Point d'entrée direct pour la validation hybride OpenCV + Classificateur Arachis hypogaea."""
    pipeline = RealVisionPipeline()
    if classifier is not None:
        pipeline.peanut_classifier = classifier
    return pipeline.validate_peanut_row_hybrid(frame=frame, roi_box=roi_box, mission_type=mission_type)
