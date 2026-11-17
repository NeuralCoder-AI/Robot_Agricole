"""backend/vision/peanut_classifier.py
Classificateur IA Pré-entraîné Hors-Ligne — Détection Binaire Stricte :
  [0] Arachide (Arachis hypogaea)  vs  [1] Sol Nu — Fin de ligne d'arachide

Chaîne de modèles embarquables hors-ligne sur l'Unité Centrale (`ROBOT_UC`) :
1. Niveau 1A (poids pré-entraînés dans `backend/models/`) :
   - Réseau MobileNetV3-Small ONNX binaire via OpenCV DNN (`backend/models/mobilenetv3_peanut.onnx`)
   - YOLOv8n/v11 optionnel (`backend/models/yolo_peanut_pretrained.pt`) si Ultralytics disponible
2. Niveau 1B (Repli automatique 100% CPU OpenCV ExG + Contours foliaires si modèle absent) :
   Segmente le feuillage chlorophyllien (Excess Green + HSV) et valide la présence de
   plants d'arachide vs sol nu / fin de rang.
"""

import logging
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import cv2
import numpy as np

logger = logging.getLogger("PeanutClassifier")

_THIS_DIR = Path(__file__).resolve().parent
BACKEND_DIR = _THIS_DIR.parent if _THIS_DIR.name == "vision" else (_THIS_DIR / "backend")
MODELS_DIR = BACKEND_DIR / "models"

try:
    from ultralytics import YOLO as UltralyticsYOLO  # type: ignore
    ULTRALYTICS_AVAILABLE = True
except Exception:
    UltralyticsYOLO = None
    ULTRALYTICS_AVAILABLE = False


SPECIES_PEANUT = "Arachide détectée"
SPECIES_BARE_SOIL = "Sol Nu — Fin de ligne d'arachide"


class PeanutPlantClassifier:
    """Classificateur binaire hors-ligne : Arachide (Arachis hypogaea) vs Sol Nu."""

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
        self.onnx_loaded: bool = False
        self.model_backend: str = "BinaryPeanutExG-Arachis-v2"

        self._try_load_pretrained_weights()

    def _try_load_pretrained_weights(self) -> None:
        """Charge automatiquement le modèle MobileNetV3 ONNX (ou YOLO si présent) depuis `backend/models/`."""
        if self.onnx_weights_path.is_file():
            try:
                self.onnx_net = cv2.dnn.readNetFromONNX(str(self.onnx_weights_path))
                self.onnx_loaded = self.onnx_net is not None and not self.onnx_net.empty()
                if self.onnx_loaded:
                    self.model_backend = f"MobileNetV3-Small-ONNX ({self.onnx_weights_path.name})"
                    logger.info("✅ Modèle MobileNetV3-Small ONNX Arachide chargé : %s", self.onnx_weights_path)
            except Exception as e:
                self.onnx_net = None
                self.onnx_loaded = False
                logger.warning("Échec chargement MobileNetV3 ONNX (%s) : %s", self.onnx_weights_path, e)

        if ULTRALYTICS_AVAILABLE and UltralyticsYOLO is not None and self.yolo_weights_path.is_file():
            try:
                self.yolo_model = UltralyticsYOLO(str(self.yolo_weights_path))
                if not self.onnx_loaded:
                    self.model_backend = f"YOLOv8-Peanut ({self.yolo_weights_path.name})"
                logger.info("✅ Poids YOLO Arachide chargés : %s", self.yolo_weights_path)
            except Exception as e:
                logger.warning("Échec chargement YOLO Arachide (%s) : %s", self.yolo_weights_path, e)

    @staticmethod
    def _extract_vegetation_mask(crop_bgr: np.ndarray) -> np.ndarray:
        """Segmente le feuillage chlorophyllien par fusion Excess Green (ExG = 2G - R - B) + HSV."""
        hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
        mask_hsv = cv2.inRange(
            hsv,
            np.array([24, 34, 32], dtype=np.uint8),
            np.array([94, 255, 255], dtype=np.uint8),
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

    @staticmethod
    def _compute_exg_foliage_score(crop_bgr: np.ndarray) -> Dict[str, Any]:
        """Évalue la présence de feuillage chlorophyllien dense par OpenCV ExG + contours."""
        resized = cv2.resize(crop_bgr, (96, 96), interpolation=cv2.INTER_AREA)
        veg_mask = PeanutPlantClassifier._extract_vegetation_mask(resized)
        veg_pixels = int(np.count_nonzero(veg_mask))
        veg_ratio = float(veg_pixels) / float(96 * 96)

        if veg_ratio < 0.035:
            return {"veg_ratio": round(veg_ratio, 4), "exg_score": 0.0, "has_foliage": False}

        hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
        veg_idx = veg_mask > 0
        hues = hsv[:, :, 0][veg_idx].astype(np.float32)
        sats = hsv[:, :, 1][veg_idx].astype(np.float32)

        b_ch = resized[:, :, 0][veg_idx].astype(np.float32)
        g_ch = resized[:, :, 1][veg_idx].astype(np.float32)
        r_ch = resized[:, :, 2][veg_idx].astype(np.float32)
        exg_mean = float(np.mean((2.0 * g_ch) - r_ch - b_ch))

        hue_ok = float(np.mean((hues >= 30.0) & (hues <= 88.0)))
        mean_sat = float(np.mean(sats))

        contours, _ = cv2.findContours(veg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_area = sum(float(cv2.contourArea(c)) for c in contours if cv2.contourArea(c) >= 16.0)
        area_score = min(1.0, valid_area / float(96 * 96 * 0.14))

        exg_score = float(
            np.clip(
                0.35 * hue_ok
                + 0.25 * min(1.0, max(0.0, (mean_sat - 40.0) / 100.0))
                + 0.20 * min(1.0, max(0.0, exg_mean / 75.0))
                + 0.20 * area_score,
                0.0,
                0.99,
            )
        )
        return {"veg_ratio": round(veg_ratio, 4), "exg_score": round(exg_score, 4), "has_foliage": valid_area >= 16.0}

    def _score_single_crop(self, crop_bgr: np.ndarray) -> Dict[str, Any]:
        """Analyse binaire d'une vignette : Arachide détectée vs Sol Nu — Fin de ligne d'arachide."""
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
                "label": SPECIES_PEANUT if is_p else SPECIES_BARE_SOIL,
                "vegetation_ratio": float(pred.get("vegetation_ratio", 0.5 if is_p else 0.0)),
            }

        foliage = self._compute_exg_foliage_score(crop_bgr)
        veg_ratio = float(foliage["veg_ratio"])
        exg_score = float(foliage["exg_score"])

        # Niveau 1A-1 : Ultralytics YOLO binaire optionnel si chargé
        if self.yolo_model is not None and not self.onnx_loaded:
            try:
                results = self.yolo_model.predict(crop_bgr, verbose=False, imgsz=224)
                if results and len(results) > 0:
                    probs = getattr(results[0], "probs", None)
                    if probs is not None:
                        top_idx = int(probs.top1)
                        top_conf = float(probs.top1conf)
                        names = getattr(results[0], "names", {})
                        cls_name = str(names.get(top_idx, "")).lower()
                        is_p = ("peanut" in cls_name or "arachid" in cls_name) and top_conf >= self.confidence_threshold
                        conf = round(top_conf if is_p else 1.0 - top_conf, 4)
                        return {
                            "is_peanut": is_p,
                            "peanut_confidence": conf,
                            "label": SPECIES_PEANUT if is_p else SPECIES_BARE_SOIL,
                            "vegetation_ratio": veg_ratio,
                        }
            except Exception as e:
                logger.debug("Fallback vers ONNX/ExG après erreur YOLO : %s", e)

        # Niveau 1A-2 : MobileNetV3-Small ONNX via OpenCV DNN (`[0: Arachide, 1: Sol Nu]`)
        if self.onnx_net is not None and self.onnx_loaded:
            try:
                blob = cv2.dnn.blobFromImage(
                    crop_bgr,
                    scalefactor=1.0 / 255.0,
                    size=(224, 224),
                    mean=(0.0, 0.0, 0.0),
                    swapRB=True,
                )
                self.onnx_net.setInput(blob)
                logits = self.onnx_net.forward().flatten()
                exp_l = np.exp(logits - np.max(logits))
                probs = exp_l / np.sum(exp_l)
                if len(probs) >= 2:
                    p_onnx = float(probs[0])
                    if not foliage["has_foliage"]:
                        combined_conf = min(p_onnx * 0.25, 0.18)
                    else:
                        combined_conf = float(np.clip(0.75 * p_onnx + 0.25 * exg_score, 0.0, 0.99))
                    is_p = bool(combined_conf >= self.confidence_threshold and foliage["has_foliage"])
                    return {
                        "is_peanut": is_p,
                        "peanut_confidence": round(combined_conf, 4),
                        "label": SPECIES_PEANUT if is_p else SPECIES_BARE_SOIL,
                        "vegetation_ratio": veg_ratio,
                        "onnx_score": round(p_onnx, 4),
                        "exg_score": exg_score,
                    }
            except Exception as e:
                logger.debug("Fallback vers ExG après erreur MobileNetV3 ONNX : %s", e)

        # Niveau 1B : Repli OpenCV ExG + Contours pur
        is_p = bool(foliage["has_foliage"] and exg_score >= self.confidence_threshold)
        return {
            "is_peanut": is_p,
            "peanut_confidence": round(exg_score, 4),
            "label": SPECIES_PEANUT if is_p else SPECIES_BARE_SOIL,
            "vegetation_ratio": veg_ratio,
            "exg_score": exg_score,
        }

    def verify_peanut_crop(self, image_crops: List[np.ndarray]) -> Dict[str, Any]:
        """Prend les vignettes (`ROI crops`) extraites par OpenCV et répond binaire :
        - `is_peanut`: True si présence d'arachide confirmée, False si Sol Nu / fin de ligne
        - `peanut_confidence`: float (0.0 à 1.0)
        - `detected_species_label`: "Arachide détectée" | "Sol Nu — Fin de ligne d'arachide"
        """
        if not image_crops:
            return {
                "is_peanut": False,
                "peanut_confidence": 0.0,
                "detected_species_label": SPECIES_BARE_SOIL,
                "crops_analyzed": 0,
                "model_backend": self.model_backend,
                "onnx_loaded": self.onnx_loaded,
                "per_crop": [],
            }

        crop_results: List[Dict[str, Any]] = [self._score_single_crop(crop) for crop in image_crops]
        peanut_votes = sum(1 for sc in crop_results if sc["is_peanut"])
        veg_scores = [sc["peanut_confidence"] for sc in crop_results if sc["label"] == SPECIES_PEANUT]
        if veg_scores:
            avg_conf = round(float(np.mean(veg_scores)), 4)
        else:
            avg_conf = round(float(np.mean([sc["peanut_confidence"] for sc in crop_results])), 4)

        is_peanut = bool(avg_conf >= self.confidence_threshold and peanut_votes >= 1)
        final_label = SPECIES_PEANUT if is_peanut else SPECIES_BARE_SOIL

        return {
            "is_peanut": is_peanut,
            "peanut_confidence": avg_conf,
            "detected_species_label": final_label,
            "crops_analyzed": len(crop_results),
            "model_backend": self.model_backend,
            "onnx_loaded": self.onnx_loaded,
            "per_crop": crop_results,
        }
