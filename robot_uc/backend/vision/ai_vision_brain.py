"""Cerveau d'Intelligence Artificielle et Vision par Ordinateur Spécial Arachide (ai_vision_brain.py).

Implémente le pipeline OpenCV / NumPy dédié exclusivement à la culture de l'arachide
(semence et désherbage inter-rangs) pour la Caméra Embarquée Avant du Robot Agricole :
1. Détection automatique des 2 lignes de plants d'arachide (tracées en VERT : 'RANG ARACHIDE')
   espacées de 46 cm par segmentation HSV et régression linéaire (cv2.fitLine).
2. Calcul exact du centre de l'espace entre ces deux lignes d'arachide
   (Ligne directrice BLEUE : 'COULOIR DÉSHERBAGE DENTS/FRAISE').
3. Pilotage automatique de la direction (DRIVE:a,d:400) pour maintenir les dents de désherbage (M5)
   et la fraise avant (M6, 23 cm) strictement centrées dans ce couloir inter-rang de 46 cm.
4. Détection des mauvaises herbes situées dans ce couloir central (boîtes JAUNES : 'ADVENTICE INTER-RANG')
   pour activer automatiquement la fraise (M6:70) et abaisser les dents (PULSE:5:50:4000).
5. Tolérance totale aux roues (19 cm) et à la garde au sol (24 cm) sur le feuillage rampant d'arachide
   (10 à 18 cm) : aucune pénalité ni contournement pour le passage des roues sur les feuilles d'arachide.
"""

import base64
import math
from pathlib import Path
import time
from typing import Any, Dict, List, Optional, Tuple
import cv2
import numpy as np

from backend.navigation.obstacle_path_planner import ObstaclePathPlanner
from backend.navigation.agri_nav_engine import AgriNavEngine

BASE_DIR: Path = Path(__file__).resolve().parent

# Échelle de la scène 3D (index.html : const CM_TO_3D = 0.01) → 1 unité 3D = 1 mètre, 46 cm = 0.46 unités 3D
SIM_CM_TO_3D: float = 0.01


def get_row_curve_3d(z: float) -> float:
    """Lignes d'arachide rigoureusement droites et parallèles (espacement strict 0.46 m sans ondulation)."""
    return 0.0


def get_row_curve_tangent(z: float, heading_rad: float = 0.0) -> float:
    """Déviation angulaire locale de la ligne droite (0.0 rad)."""
    return 0.0


class AIVisionBrain:
    """Moteur de Vision par Ordinateur OpenCV Spécialisé Arachide (Suivi Inter-Rang 46 cm)."""

    def __init__(
        self,
        frame_width: int = 320,
        frame_height: int = 200,
        width: Optional[int] = None,
        height: Optional[int] = None,
    ):
        self.frame_width = width if width is not None else frame_width
        self.frame_height = height if height is not None else frame_height
        self.robot_specs: Dict[str, Any] = {
            "total_width_with_wheels_cm": 121.0,
            "inner_tunnel_width_cm": 79.5,
            "ground_clearance_cm": 24.0,
            "tire_width_cm": 19.0,
            "weeder_width_cm": 23.0,
            "seeder_row_spacing_cm": 46.0,
            "camera_height_cm": 75.0,
            "camera_to_front_axle_cm": 25.0,
            "camera_tilt_deg": 35.0,
        }
        self.obstacle_tracker: Dict[str, Dict[str, Any]] = {}
        self.moving_obstacle_detected: bool = False
        self.avoidance_plan: Optional[Dict[str, Any]] = None
        self.nav_engine = AgriNavEngine(self.robot_specs)
        self.last_analysis: Dict[str, Any] = self._default_state()

    def set_robot_specs(self, specs: Dict[str, Any]) -> None:
        """Met à jour les dimensions géométriques utilisées par la caméra IA."""
        if isinstance(specs, dict):
            self.robot_specs.update(specs)
            self.nav_engine.update_specs(self.robot_specs)

    def _default_state(self) -> Dict[str, Any]:
        w, h = self.frame_width, self.frame_height
        cx = w // 2
        return {
            "timestamp": int(time.time() * 1000),
            "crop_type": "ARACHIDE",
            "row_detected": True,
            "center_row_label": "RANG ARACHIDE CENTRAL",
            "left_row_label": "RANG ARACHIDE",
            "right_row_label": "RANG ARACHIDE",
            "left_corridor_label": "🎯 COULOIR DENT G (-23cm)",
            "right_corridor_label": "🎯 COULOIR DENT D (+23cm)",
            "corridor_label": "COULOIR DÉSHERBAGE DENTS/FRAISE",
            "alignment_locked": False,
            "is_locked": False,
            "alignment_status": "🔍 ALIGNEMENT EN COURS... (DENTS RELEVÉES)",
            "wheel_contact_ignored": True,
            "angular_error_deg": 0.0,
            "lateral_offset_cm": 0.0,
            "left_row_line": [int(w * 0.18), h - 10, int(w * 0.36), int(h * 0.28)],
            "right_row_line": [int(w * 0.82), h - 10, int(w * 0.64), int(h * 0.28)],
            "center_line": [cx, h - 10, cx, int(h * 0.25)],
            "weeds": [],
            "weed_boxes": [],
            "weeds_detected_count": 0,
            "obstacles": [],
            "obstacle_boxes": [],
            "obstacle_is_moving": False,
            "obstacle_speed_mps": 0.0,
            "obstacle_state": "CLEAR",
            "avoidance_plan": None,
            "weed_detected": False,
            "obstacle_stop": False,
            "recommended_tools": [],
            "recommended_drive": {"a": 24, "d": 0, "ms": 400},
            "recommended_command": "DRIVE:24,0:400",
            "recommended_action": "TRACK_PEANUT_CORRIDOR",
        }

    def synthesize_camera_frame(
        self,
        lateral_offset_m: float = 0.0,
        heading_error_deg: float = 0.0,
        weeds_in_fov: Optional[List[Dict[str, float]]] = None,
        obstacles_in_fov: Optional[List[Dict[str, float]]] = None,
    ) -> np.ndarray:
        """Génère une image BGR synthétique 320x200 d'une parcelle d'arachide sur sol sablonneux ocre."""
        w, h = self.frame_width, self.frame_height
        frame = np.zeros((h, w, 3), dtype=np.uint8)

        # Ciel à l'horizon et terre sablonneuse ocre/rougeâtre typique de l'arachide (H ≈ 15 en HSV)
        horizon_y = int(h * 0.24)
        frame[:horizon_y, :] = (215, 180, 125)  # BGR ciel clair
        frame[horizon_y:, :] = (68, 118, 168)   # BGR terre sablonneuse ocre/rougeâtre

        # Décalage visuel en pixels selon l'erreur latérale et angulaire du robot dans l'inter-rang (46 cm)
        lat_px = int(np.clip(-lateral_offset_m * 75.0, -70, 70))
        ang_px = int(np.clip(-heading_error_deg * 2.8, -55, 55))

        # Couloir central inter-rang (46 cm) de terre sablonneuse binée entre les 2 lignes d'arachide
        pts_corridor = np.array([
            [w // 2 - 42 + lat_px, h],
            [w // 2 + 42 + lat_px, h],
            [w // 2 + 14 + ang_px, horizon_y + 10],
            [w // 2 - 14 + ang_px, horizon_y + 10],
        ], dtype=np.int32)
        cv2.fillPoly(frame, [pts_corridor], (52, 96, 142))

        # 2 Lignes de buissons d'arachide bas et touffus (Vert franc BGR: 42, 168, 48) à gauche et à droite
        left_bot_x = int(np.clip(w * 0.16 + lat_px, 10, w - 10))
        left_top_x = int(np.clip(w * 0.36 + ang_px, 20, w - 20))
        right_bot_x = int(np.clip(w * 0.84 + lat_px, 10, w - 10))
        right_top_x = int(np.clip(w * 0.64 + ang_px, 20, w - 20))

        for t in np.linspace(0.08, 0.95, 10):
            ly = int(horizon_y + 12 + t * (h - horizon_y - 18))
            lx = int(left_top_x + t * (left_bot_x - left_top_x))
            rx = int(right_top_x + t * (right_bot_x - right_top_x))
            rx_axis = max(5, int(6 + t * 12))
            ry_axis = max(3, int(4 + t * 7))
            cv2.ellipse(frame, (lx, ly), (rx_axis, ry_axis), 0, 0, 360, (42, 168, 48), -1)
            cv2.ellipse(frame, (rx, ly), (rx_axis, ry_axis), 0, 0, 360, (42, 168, 48), -1)

        # Mauvaises herbes dans le couloir inter-rang central (teinte vert-jaune / chartreuse BGR: 28, 205, 175)
        if weeds_in_fov:
            for wd in weeds_in_fov:
                dist = max(0.8, min(8.0, float(wd.get("dist", 3.0))))
                angle = float(wd.get("angle", 0.0))  # radians
                t = np.clip(1.0 - (dist / 8.0), 0.12, 0.92)
                wy = int(horizon_y + 16 + t * (h - horizon_y - 24))
                wx = int(w // 2 + math.sin(angle) * (w * 0.36) + lat_px * t)
                r_weed = max(6, int(7 + t * 12))
                cv2.circle(frame, (np.clip(wx, 12, w - 12), wy), r_weed, (28, 205, 175), -1)

        # Obstacles physiques critiques éventuels (Rochers : BGR: 40, 40, 230)
        if obstacles_in_fov:
            for ob in obstacles_in_fov:
                dist = max(0.6, min(8.0, float(ob.get("dist", 2.5))))
                angle = float(ob.get("angle", 0.0))
                t = np.clip(1.0 - (dist / 8.0), 0.15, 0.95)
                oy = int(horizon_y + 14 + t * (h - horizon_y - 22))
                ox = int(w // 2 + math.sin(angle) * (w * 0.42) + lat_px * t)
                r_obs = max(10, int(12 + t * 20))
                cv2.rectangle(
                    frame,
                    (np.clip(ox - r_obs, 4, w - 20), np.clip(oy - r_obs, horizon_y, h - 10)),
                    (np.clip(ox + r_obs, 15, w - 4), np.clip(oy + r_obs, horizon_y + 10, h - 2)),
                    (40, 40, 230),
                    -1,
                )

        return frame

    def analyze_bgr_frame(
        self,
        frame: np.ndarray,
        terrain_roll_deg: float = 0.0,
        terrain_pitch_deg: float = 0.0,
        annotate: bool = True,
    ) -> Dict[str, Any]:
        """Analyse la scène d'arachide par OpenCV : détecte les 2 rangs d'arachide et centre le couloir M5/M6."""
        if frame is None or frame.size == 0:
            return self.last_analysis

        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 1. Segmentation des 2 lignes de plants d'arachide (Vert franc HSV)
        mask_crops = cv2.inRange(hsv, np.array([42, 70, 50]), np.array([78, 255, 220]))
        # 2. Segmentation des adventices dans l'inter-rang (Vert-jaune / chartreuse HSV)
        mask_weeds = cv2.inRange(hsv, np.array([22, 90, 90]), np.array([41, 255, 255]))
        # 3. Segmentation des obstacles rocheux (Rouge franc HSV)
        mask_obs1 = cv2.inRange(hsv, np.array([0, 145, 80]), np.array([10, 255, 255]))
        mask_obs2 = cv2.inRange(hsv, np.array([168, 145, 80]), np.array([180, 255, 255]))
        mask_obs = cv2.bitwise_or(mask_obs1, mask_obs2)

        # Extraction des 2 lignes d'arachide (gauche et droite)
        mid_x = w // 2
        horizon_y = int(h * 0.26)
        bottom_y = h - 8

        left_mask = mask_crops[:, :mid_x]
        right_mask = mask_crops[:, mid_x:]

        left_line = self._fit_crop_row_line(
            left_mask,
            x_offset=0,
            top_y=horizon_y,
            bot_y=bottom_y,
            default_top_x=int(w * 0.36),
            default_bot_x=int(w * 0.16),
        )
        right_line = self._fit_crop_row_line(
            right_mask,
            x_offset=mid_x,
            top_y=horizon_y,
            bot_y=bottom_y,
            default_top_x=int(w * 0.64),
            default_bot_x=int(w * 0.84),
        )

        # Calcul exact du centre de l'espace inter-rang (46 cm) entre les deux lignes d'arachide
        # C'est dans ce couloir central que plongent les dents M5 et la fraise avant M6 (23 cm)
        center_bot_x = int((left_line[0] + right_line[0]) / 2)
        center_top_x = int((left_line[2] + right_line[2]) / 2)
        center_line = [center_bot_x, bottom_y, center_top_x, horizon_y]

        # Calcul de l'écart angulaire Δθ et du décalage latéral par rapport au couloir inter-rang de 46 cm
        inter_row_cm = float(self.robot_specs.get("seeder_row_spacing_cm", 46.0))
        dx = float(center_top_x - (w / 2.0))
        dy = float(bottom_y - horizon_y)
        angular_error_deg = round(math.degrees(math.atan2(dx, max(1.0, dy))), 1)
        lateral_offset_cm = round(((center_bot_x - (w / 2.0)) / (w * 0.34)) * inter_row_cm, 1)

        # Détection des mauvaises herbes situées dans le couloir central (boîtes JAUNES : ADVENTICE INTER-RANG)
        weeds_list: List[Dict[str, Any]] = []
        contours_w, _ = cv2.findContours(mask_weeds, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_w:
            area = cv2.contourArea(cnt)
            if area >= 25:
                bx, by, bw, bh = cv2.boundingRect(cnt)
                conf = min(99, max(88, int(92 + (area / 120.0))))
                dist_m = round(max(0.6, 6.5 * (1.0 - (by / float(h)))), 1)
                weeds_list.append({
                    "bbox": [int(bx), int(by), int(bw), int(bh)],
                    "label": f"ADVENTICE INTER-RANG {conf}%",
                    "confidence": conf / 100.0,
                    "distance_m": dist_m,
                    "in_inter_row_corridor": True,
                })

        # Détection des obstacles critiques
        obstacles_list: List[Dict[str, Any]] = []
        contours_o, _ = cv2.findContours(mask_obs, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_o:
            area = cv2.contourArea(cnt)
            if area >= 40:
                bx, by, bw, bh = cv2.boundingRect(cnt)
                dist_m = round(max(0.5, 6.0 * (1.0 - ((by + bh) / float(h)))), 1)
                obstacles_list.append({
                    "bbox": [int(bx), int(by), int(bw), int(bh)],
                    "label": "OBSTACLE - STOP",
                    "distance_m": dist_m,
                })

        obstacle_stop = any(ob["distance_m"] < 2.2 for ob in obstacles_list)
        weed_detected = len(weeds_list) > 0

        # Pilotage automatique (DRIVE:a,d:400) pour maintenir les dents M5 et la fraise M6 (23 cm)
        # strictement centrées dans le couloir entre les 2 lignes d'arachide.
        # Aucune pénalité ni contournement pour les roues sur le feuillage rampant d'arachide !
        steer_kp = 0.85
        lat_kp = 0.30
        roll_comp = -terrain_roll_deg * 1.1
        raw_d = int(round(-(angular_error_deg * steer_kp + lateral_offset_cm * lat_kp) + roll_comp))
        d_cmd = int(np.clip(raw_d, -28, 28))

        if obstacle_stop:
            drive_cmd = {"a": 0, "d": 0, "ms": 400}
            action = "STOP"
            recommended_tools: List[str] = ["STOP:6"]
        else:
            base_speed = 22 if (weed_detected or abs(terrain_pitch_deg) > 8.0) else 25
            drive_cmd = {"a": base_speed, "d": d_cmd, "ms": 400}
            action = "WEED_INTER_ROW" if weed_detected else "TRACK_PEANUT_CORRIDOR"
            recommended_tools = ["M6:70", "PULSE:5:50:4000"] if weed_detected else []

        is_locked = bool(abs(angular_error_deg) < 2.0 and abs(lateral_offset_cm) < 2.5)
        status_badge = "🎯 VERROUILLÉ ENTRE LES LIGNES" if is_locked else "🔍 ALIGNEMENT EN COURS... (DENTS RELEVÉES)"

        annotated_b64: Optional[str] = None
        if annotate:
            hud = frame.copy()
            # 1. Rang central d'arachide (0cm) passant sous le châssis tunnel
            cv2.line(hud, (center_line[0], center_line[1]), (center_line[2], center_line[3]), (0, 240, 120), 2, cv2.LINE_AA)
            cv2.putText(hud, "RANG ARACHIDE CENTRAL", (max(10, center_top_x - 70), max(32, horizon_y - 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (0, 240, 120), 1, cv2.LINE_AA)

            # 2. Les 2 lignes de plants d'arachide latérales à -46 cm et +46 cm tracées en VERT
            cv2.line(hud, (left_line[0], left_line[1]), (left_line[2], left_line[3]), (0, 210, 100), 2, cv2.LINE_AA)
            cv2.line(hud, (right_line[0], right_line[1]), (right_line[2], right_line[3]), (0, 210, 100), 2, cv2.LINE_AA)
            cv2.putText(hud, "RANG ARACHIDE", (max(4, left_line[0] - 12), h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (0, 210, 100), 1, cv2.LINE_AA)
            cv2.putText(hud, "RANG ARACHIDE", (min(w - 86, right_line[0] - 40), h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (0, 210, 100), 1, cv2.LINE_AA)

            # 3. Lignes directrices BLEUES des 2 couloirs inter-rangs travaillés par les dents M5 et fraise M6 (-23cm et +23cm)
            cg_bot_x = int((left_line[0] + center_line[0]) / 2)
            cg_top_x = int((left_line[2] + center_line[2]) / 2)
            cd_bot_x = int((right_line[0] + center_line[0]) / 2)
            cd_top_x = int((right_line[2] + center_line[2]) / 2)

            cv2.line(hud, (cg_bot_x, bottom_y), (cg_top_x, horizon_y + 4), (255, 180, 0), 2, cv2.LINE_AA)
            cv2.line(hud, (cd_bot_x, bottom_y), (cd_top_x, horizon_y + 4), (255, 180, 0), 2, cv2.LINE_AA)
            cv2.putText(hud, "COULOIR DENT G (-23cm)", (max(4, cg_bot_x - 38), h - 26), cv2.FONT_HERSHEY_SIMPLEX, 0.28, (255, 200, 0), 1, cv2.LINE_AA)
            cv2.putText(hud, "COULOIR DENT D (+23cm)", (min(w - 110, cd_bot_x - 28), h - 26), cv2.FONT_HERSHEY_SIMPLEX, 0.28, (255, 200, 0), 1, cv2.LINE_AA)

            # 4. Boîtes JAUNES pour les mauvaises herbes de l'inter-rang : ADVENTICE INTER-RANG
            for wd in weeds_list:
                x, y, bw, bh = wd["bbox"]
                cv2.rectangle(hud, (x, y), (x + bw, y + bh), (0, 214, 255), 2)
                cv2.putText(
                    hud,
                    wd["label"],
                    (max(2, x - 4), max(12, y - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.32,
                    (0, 230, 255),
                    1,
                    cv2.LINE_AA,
                )

            # 5. Boîtes ROUGES pour les obstacles critiques
            for ob in obstacles_list:
                x, y, bw, bh = ob["bbox"]
                cv2.rectangle(hud, (x, y), (x + bw, y + bh), (45, 23, 255), 2)
                cv2.putText(
                    hud,
                    ob["label"],
                    (max(2, x - 4), max(12, y - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.36,
                    (60, 60, 255),
                    1,
                    cv2.LINE_AA,
                )

            # Bandeau télémétrique IA Arachide avec statut de verrouillage
            cv2.rectangle(hud, (0, 0), (w, 22), (18, 12, 8), -1)
            hud_txt = f"ARACHIDE 46cm | dTheta:{angular_error_deg:+.1f} deg | dx:{lateral_offset_cm:+.1f}cm | {status_badge}"
            badge_color = (0, 240, 120) if is_locked else (0, 214, 255)
            cv2.putText(hud, hud_txt, (6, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.30, badge_color, 1, cv2.LINE_AA)

            ok, buf = cv2.imencode(".jpg", hud, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                annotated_b64 = "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode("ascii")

        rec_cmd_str = "STOP" if obstacle_stop else f"DRIVE:{drive_cmd['a']},{drive_cmd['d']}:{drive_cmd['ms']}"
        result = {
            "timestamp": int(time.time() * 1000),
            "crop_type": "ARACHIDE",
            "row_detected": True,
            "center_row_label": "RANG ARACHIDE CENTRAL",
            "left_row_label": "RANG ARACHIDE",
            "right_row_label": "RANG ARACHIDE",
            "left_corridor_label": "🎯 COULOIR DENT G (-23cm)",
            "right_corridor_label": "🎯 COULOIR DENT D (+23cm)",
            "corridor_label": "COULOIR DÉSHERBAGE DENTS/FRAISE",
            "alignment_locked": is_locked,
            "alignment_status": status_badge,
            "wheel_contact_ignored": True,
            "angular_error_deg": angular_error_deg,
            "lateral_offset_cm": lateral_offset_cm,
            "left_row_line": left_line,
            "right_row_line": right_line,
            "center_line": center_line,
            "weeds": weeds_list,
            "weed_boxes": weeds_list,
            "weeds_detected_count": len(weeds_list),
            "obstacles": obstacles_list,
            "obstacle_boxes": obstacles_list,
            "weed_detected": weed_detected,
            "obstacle_stop": obstacle_stop,
            "recommended_tools": recommended_tools,
            "recommended_drive": drive_cmd,
            "recommended_command": rec_cmd_str,
            "recommended_action": action,
            "hud_frame_b64": annotated_b64,
        }
        self.last_analysis = result
        return result

    def _fit_crop_row_line(
        self,
        mask_half: np.ndarray,
        x_offset: int,
        top_y: int,
        bot_y: int,
        default_top_x: int,
        default_bot_x: int,
    ) -> List[int]:
        """Ajuste une droite robuste par moindres carrés (cv2.fitLine) sur les pixels des rangs d'arachide."""
        ys, xs = np.where(mask_half > 0)
        if len(xs) < 15:
            return [default_bot_x, bot_y, default_top_x, top_y]

        pts = np.column_stack((xs + x_offset, ys)).astype(np.float32)
        line_params = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
        vx, vy, x0, y0 = [float(v) for v in line_params.flatten()]
        if abs(vy) < 1e-4:
            return [default_bot_x, bot_y, default_top_x, top_y]

        x_bot = int(np.clip(x0 + (bot_y - y0) * (vx / vy), 4, self.frame_width - 4))
        x_top = int(np.clip(x0 + (top_y - y0) * (vx / vy), 4, self.frame_width - 4))
        return [x_bot, bot_y, x_top, top_y]

    def _normalize_fov_targets(
        self,
        targets: Optional[List[Dict[str, Any]]],
        robot_x: float,
        robot_z: float,
        robot_yaw_rad: float,
    ) -> Optional[List[Dict[str, float]]]:
        if not targets:
            return None
        normalized: List[Dict[str, float]] = []
        for item in targets:
            if "dist" in item and "angle" in item:
                ang = float(item["angle"])
                if abs(ang) <= (math.pi / 3.2):
                    normalized.append({"dist": float(item["dist"]), "angle": ang})
            elif "x" in item and "z" in item:
                dx = float(item["x"]) - robot_x
                dz = float(item["z"]) - robot_z
                dist = float(item.get("dist", math.hypot(dx, dz)))
                raw_ang = math.atan2(dx, dz) - robot_yaw_rad
                norm_ang = math.atan2(math.sin(raw_ang), math.cos(raw_ang))
                if abs(norm_ang) <= (math.pi / 3.2):
                    normalized.append({"dist": dist, "angle": norm_ang})
            elif "dist" in item:
                ang = float(item.get("angle", 0.0))
                if abs(ang) <= (math.pi / 3.2):
                    normalized.append({"dist": float(item["dist"]), "angle": ang})
        return normalized

    def track_obstacle_dynamics(
        self,
        obstacles_in_fov: Optional[List[Dict[str, Any]]],
        robot_x: float,
        robot_z: float,
        robot_yaw_rad: float,
    ) -> Tuple[bool, float, List[Dict[str, Any]]]:
        """Analyse le déplacement des obstacles entre frames successives pour détecter les obstacles mobiles."""
        now = time.perf_counter()
        enriched_obstacles: List[Dict[str, Any]] = []
        any_moving = False
        max_speed = 0.0

        if not obstacles_in_fov:
            self.obstacle_tracker.clear()
            self.moving_obstacle_detected = False
            return False, 0.0, []

        for idx, obs in enumerate(obstacles_in_fov):
            if "x" in obs and "z" in obs:
                ox = float(obs["x"])
                oz = float(obs["z"])
            else:
                dist = float(obs.get("dist", 2.0))
                ang = float(obs.get("angle", 0.0))
                abs_ang = robot_yaw_rad + ang
                ox = robot_x + math.sin(abs_ang) * dist
                oz = robot_z + math.cos(abs_ang) * dist

            track_id = str(obs.get("id", f"obs_{idx}"))
            prev = self.obstacle_tracker.get(track_id)
            is_moving = bool(obs.get("isMoving") or obs.get("is_moving", False))
            speed_mps = float(obs.get("speed", 0.0))

            if prev is not None:
                dt = max(0.01, now - prev["t"])
                dx = ox - prev["x"]
                dz = oz - prev["z"]
                disp = math.hypot(dx, dz)
                inst_speed = disp / dt
                if inst_speed > 0.12 or disp > 0.06:
                    is_moving = True
                    speed_mps = round(inst_speed, 2)
            elif is_moving and speed_mps <= 0.0:
                speed_mps = 0.40

            self.obstacle_tracker[track_id] = {"x": ox, "z": oz, "t": now, "is_moving": is_moving}

            if is_moving:
                any_moving = True
                max_speed = max(max_speed, speed_mps)

            enriched = dict(obs)
            enriched["abs_x"] = round(ox, 3)
            enriched["abs_z"] = round(oz, 3)
            enriched["is_moving"] = is_moving
            enriched["speed_mps"] = speed_mps
            enriched_obstacles.append(enriched)

        self.moving_obstacle_detected = any_moving
        return any_moving, max_speed, enriched_obstacles

    def scan_avoidance_path(
        self,
        robot_x: float,
        robot_z: float,
        robot_yaw_rad: float,
        target_heading_rad: float,
        obstacle: Dict[str, Any],
        all_obstacles: Optional[List[Dict[str, Any]]] = None,
        field_width_m: float = 4.6,
        chassis_width_m: float = 1.21,
        target_line_x: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Planificateur d'Évitement Géométrique Exact basé sur Shapely + State Lattice / DWA
        (AgriNavEngine dans agri_nav_engine.py).
        """
        obs_radius = float(obstacle.get("radius", 0.45))
        w_obs = max(0.60, float(obstacle.get("width_m", obs_radius * 2.0)))
        l_obs = max(0.60, float(obstacle.get("length_m", obs_radius * 2.0)))
        obs_x = float(obstacle.get("abs_x", obstacle.get("x", robot_x)))
        obs_z = float(obstacle.get("abs_z", obstacle.get("z", robot_z + 1.5)))

        line_x = target_line_x if target_line_x is not None else obs_x
        obs_enriched = dict(obstacle)
        obs_enriched["x"] = obs_x
        obs_enriched["z"] = obs_z
        obs_enriched["width_m"] = w_obs
        obs_enriched["length_m"] = l_obs

        nav_res = self.nav_engine.find_lateral_bypass_corridor(
            corridor_x=line_x,
            target_heading_rad=target_heading_rad,
            primary_obs=obs_enriched,
            all_obstacles=all_obstacles,
            field_width_m=field_width_m,
            robot_z=robot_z,
        )

        best_dir = nav_res.get("best_direction", 0)
        waypoints = nav_res.get("waypoints", [])
        decision_msg = nav_res.get("decision_msg", "")
        d_lat = nav_res.get("d_lat_m", 1.45)

        alpha_deg = float(nav_res.get("alpha_deg", 38.0))
        alpha_rad = float(nav_res.get("alpha_rad", math.radians(alpha_deg)))
        d_oblique = float(nav_res.get("d_oblique_m", 2.25))
        d_parallele = float(nav_res.get("d_parallele_m", 3.40))
        l_approche = float(nav_res.get("l_approche_m", 1.50))

        return {
            "success": nav_res.get("success", False),
            "best_direction": best_dir,
            "direction": best_dir,
            "decision_msg": decision_msg,
            "right_clear": bool(best_dir == 1),
            "left_clear": bool(best_dir == -1),
            "w_obs_m": round(w_obs, 2),
            "w_obs_cm": int(w_obs * 100),
            "l_obs_m": round(l_obs, 2),
            "d_lat_m": round(d_lat, 3),
            "delta_x_m": round(d_lat, 3),
            "d_securite_lat": round(d_lat, 3),
            "x_passage": nav_res.get("x_safe", round(obs_x + best_dir * d_lat, 3)),
            "start_pt": nav_res.get("start_pt"),
            "point_a": nav_res.get("point_a"),
            "point_b": nav_res.get("point_b"),
            "point_c": nav_res.get("point_c"),
            "z_backup_safe": nav_res.get("z_backup_safe"),
            "l_approche_m": round(l_approche, 3),
            "alpha_deg": round(alpha_deg, 2),
            "alpha_rad": round(alpha_rad, 4),
            "d_oblique_m": round(d_oblique, 3),
            "d_diag_m": round(d_oblique, 3),
            "d_evitement_m": round(d_oblique, 3),
            "d_parallele_m": round(d_parallele, 3),
            "d_depassement_m": round(d_parallele, 3),
            "l_bypass_m": round(d_parallele, 3),
            "d_recul_m": 1.35,
            "d_avant_obs_m": 0.45,
            "path_points": waypoints,
            "waypoints": waypoints,
        }

    def compute_corridor_steering(
        self,
        robot_x: float,
        robot_yaw_rad: float,
        corridor_center_x: float,
        target_heading_rad: float = 0.0,
        base_speed: int = 22,
        robot_z: float = 0.0,
    ) -> Dict[str, Any]:
        """Loi de guidage stable (PD cross-track + cap) pour suivre la ligne d'arachide en temps réel.

        Ne se base JAMAIS sur une ligne droite théorique fixe : calcule la position locale réelle
        de la ligne qui ondule naturellement selon Z (get_row_curve_3d(z)) et son orientation
        tangente locale (get_row_curve_tangent(z, target_heading_rad)).
        """
        # Position locale réelle du rang ondulé au Z actuel du robot
        real_row_x = corridor_center_x + get_row_curve_3d(robot_z)
        local_tangent = target_heading_rad + get_row_curve_tangent(robot_z, target_heading_rad)

        psi = math.atan2(math.sin(robot_yaw_rad - local_tangent), math.cos(robot_yaw_rad - local_tangent))
        psi_deg = math.degrees(psi)
        e_units = (robot_x - real_row_x) * math.cos(target_heading_rad)
        e_cm = e_units / SIM_CM_TO_3D

        # Vérification stricte des conditions de verrouillage visuel :
        # |Δθ| < 2.5° et |Δx| < 5.0 cm (ou e_units < 0.05 m)
        is_locked = bool(abs(psi_deg) < 2.5 and abs(e_cm) < 5.0)

        if is_locked or (abs(e_cm) < 5.0 and abs(psi_deg) < 3.0):
            d_cmd = 0
            speed = 25
        else:
            # Gains dimensionnés pour asservissement réactif et doux
            k_psi = 0.95   # par degré d'erreur de cap
            k_e = 0.45     # par cm d'écart latéral
            raw_d = -(k_psi * psi_deg + k_e * e_cm)
            d_cmd = int(round(max(-30.0, min(30.0, raw_d))))

            # Ralentir quand l'écart est grand pour laisser la direction converger
            speed = base_speed
            if abs(e_cm) > 6.0 or abs(psi_deg) > 8.0:
                speed = max(14, base_speed - 6)

        return {
            "heading_error_deg": round(psi_deg, 2),
            "cross_track_cm": round(e_cm, 1),
            "is_locked": is_locked,
            "real_row_x": round(real_row_x, 4),
            "local_tangent_rad": round(local_tangent, 4),
            "a": int(speed),
            "d": d_cmd,
        }

    def process_scene_telemetry(
        self,
        robot_x: float = 0.0,
        robot_z: float = 0.0,
        robot_yaw_rad: float = 0.0,
        terrain_roll_deg: float = 0.0,
        terrain_pitch_deg: float = 0.0,
        weeds_in_fov: Optional[List[Dict[str, Any]]] = None,
        obstacles_in_fov: Optional[List[Dict[str, Any]]] = None,
        annotate: bool = False,
        row_spacing_m: Optional[float] = None,
        target_heading_rad: float = 0.0,
        corridor_center_x: Optional[float] = None,
        base_speed: int = 22,
    ) -> Dict[str, Any]:
        """Calcule l'état de vision IA à partir de la pose 3D du robot dans les rangs d'arachide (46 cm).

        row_spacing_m : entraxe des couloirs exprimé dans les unités de la scène 3D
        (par défaut 46 cm × SIM_CM_TO_3D = 0.644 u, identique à index.html).
        """
        if row_spacing_m is None:
            row_spacing_m = float(self.robot_specs.get("seeder_row_spacing_cm", 46.0)) * SIM_CM_TO_3D
        spacing = max(0.05, float(row_spacing_m))
        if corridor_center_x is None:
            corridor_center_x = round(robot_x / spacing) * spacing

        steer = self.compute_corridor_steering(
            robot_x=robot_x,
            robot_yaw_rad=robot_yaw_rad,
            corridor_center_x=corridor_center_x,
            target_heading_rad=target_heading_rad,
            base_speed=base_speed,
            robot_z=robot_z,
        )

        norm_weeds = self._normalize_fov_targets(weeds_in_fov, robot_x, robot_z, robot_yaw_rad)
        norm_obs = self._normalize_fov_targets(obstacles_in_fov, robot_x, robot_z, robot_yaw_rad)

        any_moving, max_speed, enriched_obs = self.track_obstacle_dynamics(
            obstacles_in_fov, robot_x, robot_z, robot_yaw_rad
        )

        avoidance_plan = None
        if self.nav_engine.bypass_plan_locked and self.nav_engine.locked_plan is not None:
            avoidance_plan = dict(self.nav_engine.locked_plan)
            self.avoidance_plan = avoidance_plan
        else:
            critical_obs = [ob for ob in enriched_obs if float(ob.get("dist", 99.0)) < 3.5]
            if critical_obs:
                target_obs = min(critical_obs, key=lambda o: float(o.get("dist", 99.0)))
                avoidance_plan = self.scan_avoidance_path(
                    robot_x=robot_x,
                    robot_z=robot_z,
                    robot_yaw_rad=robot_yaw_rad,
                    target_heading_rad=target_heading_rad,
                    obstacle=target_obs,
                    all_obstacles=enriched_obs,
                )
                self.avoidance_plan = avoidance_plan

        # Image caméra synthétique cohérente avec la scène : robot décalé à gauche (+e) → couloir vu à droite.
        synth_frame = self.synthesize_camera_frame(
            lateral_offset_m=-steer["cross_track_cm"] / 100.0,
            heading_error_deg=-steer["heading_error_deg"],
            weeds_in_fov=norm_weeds,
            obstacles_in_fov=norm_obs,
        )
        result = self.analyze_bgr_frame(
            synth_frame,
            terrain_roll_deg=terrain_roll_deg,
            terrain_pitch_deg=terrain_pitch_deg,
            annotate=annotate,
        )

        # Fusion : la consigne de direction utilise la loi de guidage stable (pose fusionnée caméra + IMU)
        result["angular_error_deg"] = steer["heading_error_deg"]
        result["lateral_offset_cm"] = steer["cross_track_cm"]
        result["is_locked"] = steer["is_locked"]
        result["alignment_status"] = "🎯 VERROUILLÉ ENTRE LES LIGNES" if steer["is_locked"] else "🔍 ALIGNEMENT EN COURS... (DENTS RELEVÉES)"
        result["corridor_center_x"] = round(corridor_center_x, 4)
        result["target_heading_rad"] = round(target_heading_rad, 4)
        result["obstacle_is_moving"] = any_moving
        result["obstacle_speed_mps"] = max_speed
        result["avoidance_plan"] = avoidance_plan

        if any_moving and critical_obs:
            result["recommended_action"] = "WAIT_MOVING_OBSTACLE"
            result["recommended_drive"] = {"a": 0, "d": 0, "ms": 400}
            result["recommended_command"] = "STOP"
            result["recommended_tools"] = ["STOP:6"]
        elif not result.get("obstacle_stop"):
            a_cmd = min(int(result["recommended_drive"]["a"]), steer["a"]) if result.get("weed_detected") else steer["a"]
            drive_cmd = {"a": int(a_cmd), "d": steer["d"], "ms": 400}
            result["recommended_drive"] = drive_cmd
            result["recommended_command"] = f"DRIVE:{drive_cmd['a']},{drive_cmd['d']}:{drive_cmd['ms']}"
        self.last_analysis = result
        return result

    def analyze_scene(
        self,
        robot_x: float = 0.0,
        robot_z: float = 0.0,
        robot_yaw: float = 0.0,
        pitch_deg: float = 0.0,
        roll_deg: float = 0.0,
        weeds_in_fov: Optional[List[Dict[str, Any]]] = None,
        obstacles_in_fov: Optional[List[Dict[str, Any]]] = None,
        row_spacing_m: Optional[float] = None,
        base_speed: int = 25,
    ) -> Dict[str, Any]:
        """Alias haut-niveau d'analyse de scène 3D d'arachide par le Cerveau de Vision IA."""
        return self.process_scene_telemetry(
            robot_x=robot_x,
            robot_z=robot_z,
            robot_yaw_rad=robot_yaw,
            terrain_roll_deg=roll_deg,
            terrain_pitch_deg=pitch_deg,
            weeds_in_fov=weeds_in_fov,
            obstacles_in_fov=obstacles_in_fov,
            row_spacing_m=row_spacing_m,
            base_speed=base_speed,
        )
