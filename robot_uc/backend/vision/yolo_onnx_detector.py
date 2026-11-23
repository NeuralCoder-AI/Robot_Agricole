"""backend/vision/yolo_onnx_detector.py
Détecteur d'Obstacles Sémantiques YOLOv8n ONNX (80 classes COCO) via OpenCV DNN (`cv2.dnn`),
suivi multi-objets par centroïde (`MultiObjectCentroidTracker`) et fusion avec OpenCV Sobel/Contours.

Aucune dépendance lourde à `torch` ou `ultralytics` n'est requise sur l'Unité Centrale (`ROBOT_UC`).
Entrée réseau : blob BGR->RGB (1, 3, 640, 640), normalisation 1/255.0.
Sortie réseau : tenseur brut (1, 84, 8400) transposé en (8400, 84) -> NMS `cv2.dnn.NMSBoxes`.
"""

import logging
import math
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import cv2
import numpy as np

logger = logging.getLogger("YoloOnnxDetector")

_THIS_DIR = Path(__file__).resolve().parent
BACKEND_DIR = _THIS_DIR.parent if _THIS_DIR.name == "vision" else (_THIS_DIR / "backend")
DEFAULT_YOLOV8N_ONNX_PATH = BACKEND_DIR / "models" / "yolov8n.onnx"

COCO_CLASSES: Tuple[str, ...] = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
    "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
)

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
    "bus",
    "truck",
    "tractor",
    "bicycle",
    "motorcycle",
    "vehicle",
    "vehicule",
}

# Sous-ensemble COCO pertinent en champ agricole (évite les faux positifs végétaux/domestiques
# tels que `potted plant`, `broccoli` ou `toothbrush` sur rangs d'arachide synthétiques/réels).
AGRI_OBSTACLE_COCO_CLASSES = MOBILE_YOLO_CLASSES | {
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bench",
    "backpack",
    "umbrella",
    "handbag",
    "suitcase",
    "chair",
    "couch",
    "dining table",
    "bottle",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "boat",
    "train",
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
                "source": det.get("source", "hybrid"),
                "class_id": det.get("class_id"),
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


class YoloOnnxDetector:
    """Charge et infère `yolov8n.onnx` (~12,2 Mo) nativement via `cv2.dnn` sans PyTorch."""

    def __init__(self, model_path: Optional[Union[str, Path]] = None):
        self.model_path: Optional[Path] = (
            Path(model_path) if model_path is not None else DEFAULT_YOLOV8N_ONNX_PATH
        )
        self.net: Optional[Any] = None
        self.yolo_onnx_loaded: bool = False
        self.yolo_model_name: str = "OpenCV-Fallback"
        self.model_name: str = "OpenCV-Fallback"
        self._lock = threading.Lock()
        self._load_network()

    def _load_network(self) -> None:
        if self.model_path is None or not self.model_path.is_file():
            logger.info(
                "Fichier ONNX YOLOv8n introuvable (%s) — repli propre sur détecteur OpenCV Sobel/HSV.",
                self.model_path,
            )
            return

        if self.model_path.suffix.lower() != ".onnx":
            return

        try:
            net = cv2.dnn.readNetFromONNX(str(self.model_path))
            if net is not None and not net.empty():
                self.net = net
                self.yolo_onnx_loaded = True
                self.model_name = self.model_path.name
                self.yolo_model_name = self.model_path.name
                logger.info("✅ Modèle YOLOv8n ONNX chargé via cv2.dnn : %s", self.model_path)
        except Exception as e:
            self.net = None
            self.yolo_onnx_loaded = False
            logger.warning("Échec chargement YOLOv8n ONNX (%s) : %s", self.model_path, e)

    def detect_objects_onnx(
        self,
        frame: np.ndarray,
        conf_thresh: float = 0.40,
        nms_thresh: float = 0.45,
    ) -> List[Dict[str, Any]]:
        """Infère `yolov8n.onnx` sur `frame` et renvoie les détections filtrées par NMS."""
        if not self.yolo_onnx_loaded or self.net is None or frame is None or frame.size == 0:
            return []

        h, w = frame.shape[:2]
        scale_x = float(w) / 640.0
        scale_y = float(h) / 640.0

        blob = cv2.dnn.blobFromImage(
            frame,
            scalefactor=1.0 / 255.0,
            size=(640, 640),
            mean=(0.0, 0.0, 0.0),
            swapRB=True,
            crop=False,
        )

        with self._lock:
            self.net.setInput(blob)
            raw_out = self.net.forward()

        if raw_out is None or raw_out.size == 0:
            return []

        # Sortie standard YOLOv8 ONNX : (1, 84, 8400) -> transposition en (8400, 84)
        if raw_out.ndim == 3:
            outputs = raw_out[0].T if raw_out.shape[1] == 84 else raw_out[0]
        elif raw_out.ndim == 2:
            outputs = raw_out.T if raw_out.shape[0] == 84 else raw_out
        else:
            return []

        if outputs.shape[1] < 84:
            return []

        class_scores = outputs[:, 4:]
        class_ids = np.argmax(class_scores, axis=1)
        confidences = class_scores[np.arange(outputs.shape[0]), class_ids]

        keep_mask = confidences >= float(conf_thresh)
        if not np.any(keep_mask):
            return []

        kept_preds = outputs[keep_mask]
        kept_confs = confidences[keep_mask]
        kept_cids = class_ids[keep_mask]

        boxes_xywh: List[List[int]] = []
        scores_list: List[float] = []
        cids_list: List[int] = []

        for row, score, cid in zip(kept_preds, kept_confs, kept_cids):
            cx_640, cy_640, bw_640, bh_640 = float(row[0]), float(row[1]), float(row[2]), float(row[3])
            bw_px = max(1, int(round(bw_640 * scale_x)))
            bh_px = max(1, int(round(bh_640 * scale_y)))
            x1_px = max(0, int(round((cx_640 - bw_640 / 2.0) * scale_x)))
            y1_px = max(0, int(round((cy_640 - bh_640 / 2.0) * scale_y)))
            boxes_xywh.append([x1_px, y1_px, bw_px, bh_px])
            scores_list.append(float(score))
            cids_list.append(int(cid))

        if not boxes_xywh:
            return []

        indices = cv2.dnn.NMSBoxes(boxes_xywh, scores_list, float(conf_thresh), float(nms_thresh))
        if indices is None or len(indices) == 0:
            return []

        detections: List[Dict[str, Any]] = []
        for idx in np.array(indices).flatten():
            x1, y1, bw_px, bh_px = boxes_xywh[int(idx)]
            x2 = min(w, x1 + bw_px)
            y2 = min(h, y1 + bh_px)
            cid = cids_list[int(idx)]
            score = scores_list[int(idx)]
            cls_name = COCO_CLASSES[cid] if 0 <= cid < len(COCO_CLASSES) else f"coco_{cid}"
            is_mobile = cls_name in MOBILE_YOLO_CLASSES

            cx_px = (x1 + x2) / 2.0
            cy_px = (y1 + y2) / 2.0
            detections.append({
                "source": "yolov8n.onnx",
                "class_id": cid,
                "class_name": cls_name,
                "conf": round(score, 4),
                "bbox": [float(x1), float(y1), float(x2), float(y2)],
                "xyxy": [float(x1), float(y1), float(x2), float(y2)],
                "x": int(x1),
                "y": int(y1),
                "w": int(max(1, x2 - x1)),
                "h": int(max(1, y2 - y1)),
                "cx": round(cx_px, 2),
                "cy": round(cy_px, 2),
                "y_bottom": float(y2),
                "is_mobile_class": is_mobile,
                "is_moving": is_mobile,
            })

        return detections


def _bbox_iou(box_a: List[float], box_b: List[float]) -> float:
    ax1, ay1, ax2, ay2 = [float(v) for v in box_a[:4]]
    bx1, by1, bx2, by2 = [float(v) for v in box_b[:4]]
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = max(1e-6, (ax2 - ax1) * (ay2 - ay1))
    area_b = max(1e-6, (bx2 - bx1) * (by2 - by1))
    return inter / (area_a + area_b - inter)


def merge_yolo_and_opencv_obstacles(
    yolo_dets: List[Dict[str, Any]],
    opencv_dets: List[Dict[str, Any]],
    iou_thresh: float = 0.35,
) -> List[Dict[str, Any]]:
    """Fusionne les détections YOLOv8n ONNX (prioritaires) et OpenCV Sobel/Sol (rochers/souches)."""
    merged: List[Dict[str, Any]] = [dict(d) for d in yolo_dets]
    yolo_boxes = [d["bbox"] for d in merged if "bbox" in d]

    for ocv in opencv_dets:
        ocv_box = ocv.get("bbox")
        if not ocv_box:
            continue
        if any(_bbox_iou(ocv_box, ybox) >= iou_thresh for ybox in yolo_boxes):
            continue
        merged.append(dict(ocv))

    return merged
