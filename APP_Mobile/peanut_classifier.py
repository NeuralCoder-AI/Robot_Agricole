"""backend/vision/peanut_classifier.py
Classificateur IA Pré-entraîné Hors-Ligne de Reconnaissance Botanique de l'Arachide (Arachis hypogaea).

Chaîne de modèles embarquables hors-ligne sur l'Unité Centrale (`ROBOT_UC`) :
1. Niveau 1A (optionnel si poids présents dans `backend/models/`) :
   - YOLOv8n/v11 spécialisé Arachide/Adventices (`backend/models/yolo_peanut_pretrained.pt`)
   - Réseau CNN léger MobileNetV3 ONNX via OpenCV DNN (`backend/models/mobilenetv3_peanut.onnx`)
2. Niveau 1B (Modèle Botanique Morpho-Spectral & Gabor Pré-calibré Intégré — 0 ms réseau, 100% CPU) :
   Analyse chaque vignette (`ROI crop`) selon les traits distinctifs d'*Arachis hypogaea* vs graminées/mil/sorgho :
   - Signature spectrale HSV + Excess Green (ExG = 2G - R - B) normalisée du feuillage d'arachide ;
   - Morphologie foliaire : touffes basses composées de 4 folioles ovales arrondies (ratio d'aspect équilibré,
     circularité et solidité modérées-élevées) vs lames étroites dressées ou feuilles lancéolées de mil/sorgho/graminées ;
   - Banc de filtres de texture de Gabor multi-orientations (0°, 45°, 90°, 135°) : détecte l'isotropie
     des folioles ovales d'arachide par opposition aux stries nervurées directionnelles des céréales/graminées.
"""

import logging
import math
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import cv2
import numpy as np

logger = logging.getLogger("PeanutClassifier")

BACKEND_DIR = Path(__file__).resolve().parent.parent
MODELS_DIR = BACKEND_DIR / "models"

try:
    from ultralytics import YOLO as UltralyticsYOLO  # type: ignore
    ULTRALYTICS_AVAILABLE = True
except Exception:
    UltralyticsYOLO = None
    ULTRALYTICS_AVAILABLE = False


SPECIES_PEANUT = "Arachis hypogaea (Arachide)"
SPECIES_WEED = "Adventice / Graminée"
SPECIES_OTHER_CROP = "Autre Culture (Mil/Sorgho)"
SPECIES_BARE_SOIL = "Sol Nu"


class PeanutPlantClassifier:
    """Classificateur botanique hors-ligne dédié à la certification des plants d'Arachis hypogaea."""

    def __init__(
        self,
        models_dir: Optional[Path] = None,
        confidence_threshold: float = 0.65,
        custom_predictor: Optional[Callable[[np.ndarray], Dict[str, Any]]] = None,
    ):
        self.models_dir = Path(models_dir) if models_dir else MODELS_DIR
        self.confidence_threshold = float(confidence_threshold)
        self.custom_predictor = custom_predictor

        self.yolo_weights_path = self.models_dir / "yolo_peanut_pretrained.pt"
        self.onnx_weights_path = self.models_dir / "mobilenetv3_peanut.onnx"

        self.yolo_model: Optional[Any] = None
        self.onnx_net: Optional[Any] = None
        self.model_backend: str = "MorphoSpectralGabor-Arachis-v1"

        self._try_load_pretrained_weights()
        self._gabor_kernels = self._build_gabor_filter_bank()

    def _try_load_pretrained_weights(self) -> None:
        """Charge automatiquement les poids pré-entraînés YOLO / MobileNetV3 ONNX si présents dans `backend/models/`."""
        if ULTRALYTICS_AVAILABLE and UltralyticsYOLO is not None and self.yolo_weights_path.is_file():
            try:
                self.yolo_model = UltralyticsYOLO(str(self.yolo_weights_path))
                self.model_backend = f"YOLOv8-Peanut ({self.yolo_weights_path.name})"
                logger.info("✅ Poids YOLO Arachide chargés : %s", self.yolo_weights_path)
                return
            except Exception as e:
                logger.warning("Échec chargement YOLO Arachide (%s) : %s", self.yolo_weights_path, e)

        if self.onnx_weights_path.is_file():
            try:
                self.onnx_net = cv2.dnn.readNetFromONNX(str(self.onnx_weights_path))
                self.model_backend = f"MobileNetV3-ONNX ({self.onnx_weights_path.name})"
                logger.info("✅ Modèle MobileNetV3 Arachide chargé : %s", self.onnx_weights_path)
            except Exception as e:
                logger.warning("Échec chargement MobileNetV3 ONNX (%s) : %s", self.onnx_weights_path, e)

    @staticmethod
    def _build_gabor_filter_bank() -> List[np.ndarray]:
        """Construit un banc de 4 filtres de Gabor multi-orientations (0°, 45°, 90°, 135°)."""
        kernels: List[np.ndarray] = []
        for theta_deg in (0, 45, 90, 135):
            k = cv2.getGaborKernel(
                ksize=(11, 11),
                sigma=2.4,
                theta=math.radians(theta_deg),
                lambd=5.5,
                gamma=0.65,
                psi=0.0,
                ktype=cv2.CV_32F,
            )
            kernels.append(k)
        return kernels

    @staticmethod
    def _extract_vegetation_mask(crop_bgr: np.ndarray) -> np.ndarray:
        """Segmente le feuillage chlorophyllien par fusion Excess Green (ExG = 2G - R - B) + HSV."""
        hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
        mask_hsv = cv2.inRange(
            hsv,
            np.array([22, 32, 30], dtype=np.uint8),
            np.array([96, 255, 255], dtype=np.uint8),
        )
        b = crop_bgr[:, :, 0].astype(np.int16)
        g = crop_bgr[:, :, 1].astype(np.int16)
        r = crop_bgr[:, :, 2].astype(np.int16)
        exg = (2 * g) - r - b
        mask_exg = np.where((exg > 22) & (g > 38), 255, 0).astype(np.uint8)
        combined = cv2.bitwise_or(mask_hsv, mask_exg)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
        return combined

    def _score_single_crop(self, crop_bgr: np.ndarray) -> Dict[str, Any]:
        """Analyse botanique complète d'une vignette unique."""
        if crop_bgr is None or not isinstance(crop_bgr, np.ndarray) or crop_bgr.size == 0:
            return {
                "is_peanut": False,
                "peanut_confidence": 0.0,
                "label": SPECIES_BARE_SOIL,
                "vegetation_ratio": 0.0,
            }

        # Hook optionnel pour prédicteur externe injecté ou testé
        if self.custom_predictor is not None:
            pred = dict(self.custom_predictor(crop_bgr))
            conf = float(pred.get("peanut_confidence", 0.0))
            is_p = bool(pred.get("is_peanut", conf >= self.confidence_threshold))
            return {
                "is_peanut": is_p,
                "peanut_confidence": round(conf, 4),
                "label": str(pred.get("detected_species_label", SPECIES_PEANUT if is_p else SPECIES_OTHER_CROP)),
                "vegetation_ratio": float(pred.get("vegetation_ratio", 0.5)),
            }

        # Niveau 1A-1 : Ultralytics YOLO pré-entraîné si chargé
        if self.yolo_model is not None:
            try:
                results = self.yolo_model.predict(crop_bgr, verbose=False, imgsz=224)
                if results and len(results) > 0:
                    probs = getattr(results[0], "probs", None)
                    boxes = getattr(results[0], "boxes", None)
                    if probs is not None:
                        top_idx = int(probs.top1)
                        top_conf = float(probs.top1conf)
                        names = getattr(results[0], "names", {})
                        cls_name = str(names.get(top_idx, "")).lower()
                        is_p = ("peanut" in cls_name or "arachid" in cls_name) and top_conf >= self.confidence_threshold
                        lbl = SPECIES_PEANUT if is_p else (SPECIES_WEED if "weed" in cls_name else SPECIES_OTHER_CROP)
                        return {"is_peanut": is_p, "peanut_confidence": round(top_conf if is_p else 1.0 - top_conf, 4), "label": lbl, "vegetation_ratio": 0.5}
                    if boxes is not None and len(boxes) > 0:
                        names = getattr(results[0], "names", {})
                        best_p = 0.0
                        best_other = 0.0
                        for b in boxes:
                            c_idx = int(b.cls[0])
                            c_conf = float(b.conf[0])
                            cname = str(names.get(c_idx, "")).lower()
                            if "peanut" in cname or "arachid" in cname:
                                best_p = max(best_p, c_conf)
                            else:
                                best_other = max(best_other, c_conf)
                        if best_p > 0.0 or best_other > 0.0:
                            is_p = best_p >= self.confidence_threshold and best_p >= best_other
                            return {
                                "is_peanut": is_p,
                                "peanut_confidence": round(best_p, 4),
                                "label": SPECIES_PEANUT if is_p else SPECIES_OTHER_CROP,
                                "vegetation_ratio": 0.5,
                            }
            except Exception as e:
                logger.debug("Fallback vers descripteur botanique après erreur YOLO : %s", e)

        # Niveau 1A-2 : MobileNetV3 ONNX via OpenCV DNN si chargé
        if self.onnx_net is not None:
            try:
                blob = cv2.dnn.blobFromImage(
                    crop_bgr, scalefactor=1.0 / 255.0, size=(224, 224), mean=(0.485, 0.456, 0.406), swapRB=True
                )
                self.onnx_net.setInput(blob)
                logits = self.onnx_net.forward().flatten()
                exp_l = np.exp(logits - np.max(logits))
                probs = exp_l / np.sum(exp_l)
                if len(probs) >= 2:
                    p_peanut = float(probs[0])
                    is_p = p_peanut >= self.confidence_threshold
                    lbl = SPECIES_PEANUT if is_p else (SPECIES_BARE_SOIL if len(probs) > 2 and probs[2] > 0.5 else SPECIES_OTHER_CROP)
                    return {"is_peanut": is_p, "peanut_confidence": round(p_peanut, 4), "label": lbl, "vegetation_ratio": 0.5}
            except Exception as e:
                logger.debug("Fallback vers descripteur botanique après erreur ONNX : %s", e)

        # Niveau 1B : Modèle Botanique Morpho-Spectral & Texture Gabor pré-calibré sur Arachis hypogaea
        resized = cv2.resize(crop_bgr, (96, 96), interpolation=cv2.INTER_AREA)
        veg_mask = self._extract_vegetation_mask(resized)
        veg_pixels = int(np.count_nonzero(veg_mask))
        veg_ratio = float(veg_pixels) / float(96 * 96)

        if veg_ratio < 0.055:
            return {
                "is_peanut": False,
                "peanut_confidence": round(veg_ratio * 2.0, 4),
                "label": SPECIES_BARE_SOIL,
                "vegetation_ratio": round(veg_ratio, 4),
                "leaf_circularity": 0.0,
                "gabor_isotropy": 0.0,
            }

        hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
        veg_idx = veg_mask > 0
        hues = hsv[:, :, 0][veg_idx].astype(np.float32)
        sats = hsv[:, :, 1][veg_idx].astype(np.float32)
        vals = hsv[:, :, 2][veg_idx].astype(np.float32)

        b_ch = resized[:, :, 0][veg_idx].astype(np.float32)
        g_ch = resized[:, :, 1][veg_idx].astype(np.float32)
        r_ch = resized[:, :, 2][veg_idx].astype(np.float32)
        exg_mean = float(np.mean((2.0 * g_ch) - r_ch - b_ch))

        mean_hue = float(np.mean(hues))
        mean_sat = float(np.mean(sats))
        # Arachis hypogaea : vert tendre à franc HSV [34..84], faible composante jaune/brune ou bleutée sèche
        hue_in_peanut_band = float(np.mean((hues >= 33.0) & (hues <= 84.0)))
        spectral_score = (
            0.50 * hue_in_peanut_band
            + 0.25 * min(1.0, max(0.0, (mean_sat - 45.0) / 110.0))
            + 0.25 * min(1.0, max(0.0, exg_mean / 85.0))
        )

        # Morphologie des touffes & folioles ovales vs lames étroites / feuilles linéaires dressées
        contours, _ = cv2.findContours(veg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_cnts = [c for c in contours if cv2.contourArea(c) >= 18.0]
        if not valid_cnts:
            return {
                "is_peanut": False,
                "peanut_confidence": 0.12,
                "label": SPECIES_BARE_SOIL,
                "vegetation_ratio": round(veg_ratio, 4),
                "leaf_circularity": 0.0,
                "gabor_isotropy": 0.0,
            }

        circularities: List[float] = []
        solidities: List[float] = []
        aspects: List[float] = []
        total_cnt_area = 0.0
        for cnt in valid_cnts:
            area = float(cv2.contourArea(cnt))
            perim = float(cv2.arcLength(cnt, True))
            if perim <= 1e-3:
                continue
            circ = (4.0 * math.pi * area) / (perim * perim)
            hull = cv2.convexHull(cnt)
            hull_area = float(cv2.contourArea(hull))
            sol = area / max(1.0, hull_area)
            _, _, bw, bh = cv2.boundingRect(cnt)
            asp = float(bw) / max(1.0, float(bh))
            # Normaliser dans (0..1] pour mesurer si l'objet est équilibré (ovale/quadrifolié) ou très élancé
            balanced_asp = min(asp, 1.0 / max(1e-3, asp))
            circularities.append(min(1.0, circ))
            solidities.append(min(1.0, sol))
            aspects.append(balanced_asp)
            total_cnt_area += area

        mean_circ = float(np.mean(circularities)) if circularities else 0.0
        mean_sol = float(np.mean(solidities)) if solidities else 0.0
        mean_bal_asp = float(np.mean(aspects)) if aspects else 0.0

        # Texture Gabor 4 orientations (0°, 45°, 90°, 135°)
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0
        gabor_energies: List[float] = []
        for k in self._gabor_kernels:
            resp = cv2.filter2D(gray, cv2.CV_32F, k)
            gabor_energies.append(float(np.mean(np.abs(resp[veg_idx]))))

        max_g = max(gabor_energies) if gabor_energies else 1e-6
        min_g = min(gabor_energies) if gabor_energies else 0.0
        gabor_isotropy = float(min_g / max(1e-6, max_g))
        # Si une orientation domine largement (nervures parallèles verticales de mil/sorgho/herbe haute), pénalité !
        direction_bias = abs(gabor_energies[0] - gabor_energies[2]) / max(1e-6, gabor_energies[0] + gabor_energies[2])

        # Signature tétrafoliolée / touffe d'arachide : contours arrondis/ovales (bal_asp >= 0.45, circularité >= 0.34)
        # Les lames de mil/sorgho ou graminées ont aspect linéaire (< 0.38), silhouette étoilée (< 0.23) ou biais directionnel fort
        is_linear_monocot = (
            (mean_bal_asp < 0.38)
            or (mean_circ < 0.23)
            or (mean_circ < 0.29 and (mean_sol < 0.74 or direction_bias > 0.20))
        )
        morph_score = (
            0.40 * min(1.0, mean_circ / 0.56)
            + 0.30 * min(1.0, mean_sol / 0.80)
            + 0.30 * min(1.0, mean_bal_asp / 0.65)
        )

        # Score combiné calibré Arachis hypogaea [0..1]
        raw_confidence = (
            0.38 * spectral_score
            + 0.40 * morph_score
            + 0.22 * min(1.0, max(0.0, gabor_isotropy * 1.15 - direction_bias * 0.45))
        )
        if is_linear_monocot:
            raw_confidence *= 0.42
        if mean_hue < 27.0 or hue_in_peanut_band < 0.40:
            raw_confidence *= 0.48
        if direction_bias > 0.32:
            raw_confidence *= 0.58

        peanut_conf = round(float(np.clip(raw_confidence, 0.0, 0.99)), 4)
        is_peanut = bool(peanut_conf >= self.confidence_threshold and not is_linear_monocot)

        if is_peanut:
            label = SPECIES_PEANUT
        elif is_linear_monocot or mean_bal_asp < 0.42 or direction_bias > 0.32:
            # Distinguer grande culture céréalière (densité élevée / touffes hautes) vs petite graminée adventice
            label = SPECIES_OTHER_CROP if veg_ratio >= 0.18 else SPECIES_WEED
        elif hue_in_peanut_band < 0.45:
            label = SPECIES_OTHER_CROP
        else:
            label = SPECIES_WEED

        return {
            "is_peanut": is_peanut,
            "peanut_confidence": peanut_conf,
            "label": label,
            "vegetation_ratio": round(veg_ratio, 4),
            "spectral_score": round(spectral_score, 3),
            "morph_score": round(morph_score, 3),
            "leaf_circularity": round(mean_circ, 3),
            "leaf_solidity": round(mean_sol, 3),
            "balanced_aspect": round(mean_bal_asp, 3),
            "gabor_isotropy": round(gabor_isotropy, 3),
            "direction_bias": round(direction_bias, 3),
        }

    def verify_peanut_crop(self, image_crops: List[np.ndarray]) -> Dict[str, Any]:
        """Prend les vignettes (`ROI crops`) des plantes détectées sur une ligne par OpenCV
        et retourne la certification botanique agrégée :
        - `is_peanut`: bool
        - `peanut_confidence`: float (0.0 à 1.0)
        - `detected_species_label`: str
        """
        if not image_crops:
            return {
                "is_peanut": False,
                "peanut_confidence": 0.0,
                "detected_species_label": SPECIES_BARE_SOIL,
                "crops_analyzed": 0,
                "model_backend": self.model_backend,
                "per_crop": [],
            }

        crop_results: List[Dict[str, Any]] = []
        valid_veg_scores: List[float] = []
        non_soil_labels: List[str] = []

        for crop in image_crops:
            sc = self._score_single_crop(crop)
            crop_results.append(sc)
            if sc["label"] != SPECIES_BARE_SOIL:
                valid_veg_scores.append(sc["peanut_confidence"])
                non_soil_labels.append(sc["label"])

        if not valid_veg_scores:
            return {
                "is_peanut": False,
                "peanut_confidence": 0.0,
                "detected_species_label": SPECIES_BARE_SOIL,
                "crops_analyzed": len(crop_results),
                "model_backend": self.model_backend,
                "per_crop": crop_results,
            }

        avg_conf = round(float(np.mean(valid_veg_scores)), 4)
        peanut_votes = sum(1 for lbl in non_soil_labels if lbl == SPECIES_PEANUT)
        is_peanut = bool(avg_conf >= self.confidence_threshold and peanut_votes >= max(1, math.ceil(len(non_soil_labels) / 2.0)))

        if is_peanut:
            final_label = SPECIES_PEANUT
        else:
            # Vote majoritaire parmi les espèces non-arachide détectées
            other_crop_votes = sum(1 for lbl in non_soil_labels if lbl == SPECIES_OTHER_CROP)
            weed_votes = sum(1 for lbl in non_soil_labels if lbl == SPECIES_WEED)
            if other_crop_votes >= weed_votes:
                final_label = SPECIES_OTHER_CROP
            else:
                final_label = SPECIES_WEED

        return {
            "is_peanut": is_peanut,
            "peanut_confidence": avg_conf,
            "detected_species_label": final_label,
            "crops_analyzed": len(crop_results),
            "model_backend": self.model_backend,
            "per_crop": crop_results,
        }
