"""Détection d'adventices par caméra (CNN NigerWeedNet) côté PC.

Deux moteurs d'inférence interchangeables :
  * ``DetecteurYOLO`` : modèle Ultralytics YOLO (fichier .pt, .onnx ou .engine
    TensorRT) entraîné sur NigerWeedNet-v1 (7 classes) ;
  * ``DetecteurSimule`` : renvoie des détections programmées (tests).

Le navigateur n'a besoin que d'une décision binaire par image : « adventice à
désherber dans la bande de travail ? » avec un score ≥ seuil (0,75 par défaut,
cohérent avec le seuil du sarcleur conditionnel de la thèse). La position
latérale de l'adventice (−1 = tout à gauche, +1 = tout à droite) permet une
petite correction de direction via DRIVE:a,d.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Protocol

log = logging.getLogger("robot_pc.vision")

CLASSES = ["arachide", "cyperus_rotundus", "digitaria_horizontalis",
           "commelina_benghalensis", "striga_gesnerioides",
           "eragrostis_tremula", "cassia_obtusifolia"]
CLASSE_CULTURE = "arachide"


@dataclass
class Detection:
    classe: str
    score: float
    x_centre: float      # 0..1 dans l'image (gauche → droite)
    y_centre: float      # 0..1 (haut → bas)
    largeur: float = 0.0
    hauteur: float = 0.0


@dataclass
class AnalyseImage:
    detections: list[Detection] = field(default_factory=list)
    horodatage: float = field(default_factory=time.monotonic)
    latence_ms: float = 0.0

    def adventices(self, seuil: float) -> list[Detection]:
        return [d for d in self.detections
                if d.classe != CLASSE_CULTURE and d.score >= seuil]

    def decision(self, seuil: float = 0.75, bande: tuple[float, float] = (0.25, 0.75)
                 ) -> tuple[bool, float]:
        """(désherber ?, décalage latéral −1..+1 de l'adventice la plus sûre
        dans la bande de travail). 0.0 si aucune."""
        cands = [d for d in self.adventices(seuil) if bande[0] <= d.x_centre <= bande[1]]
        if not cands:
            return False, 0.0
        meilleure = max(cands, key=lambda d: d.score)
        return True, (meilleure.x_centre - 0.5) * 2


class Detecteur(Protocol):
    def analyser(self) -> Optional[AnalyseImage]: ...
    def fermer(self) -> None: ...


# --------------------------------------------------------------------------- #
# YOLO (Ultralytics) — caméra USB ou CSI via OpenCV
# --------------------------------------------------------------------------- #
class DetecteurYOLO:
    """Capture + inférence dans un fil ; ``analyser()`` renvoie le dernier résultat.

    pip install ultralytics opencv-python   (et torch/CUDA ou un .engine TensorRT)
    """

    def __init__(self, modele: str, camera: int | str = 0, taille: int = 640,
                 seuil_conf: float = 0.25, device: Optional[str] = None,
                 fps_max: float = 10.0):
        import cv2
        from ultralytics import YOLO
        self._cv2 = cv2
        self._modele = YOLO(modele)
        self._cap = cv2.VideoCapture(camera)
        if not self._cap.isOpened():
            raise RuntimeError(f"caméra {camera!r} indisponible")
        self._taille, self._conf, self._device = taille, seuil_conf, device
        self._periode = 1.0 / fps_max
        self._dernier: Optional[AnalyseImage] = None
        self._stop = threading.Event()
        self._verrou = threading.Lock()
        self._fil = threading.Thread(target=self._boucle, daemon=True, name="vision")
        self._fil.start()

    def _boucle(self) -> None:
        while not self._stop.is_set():
            t0 = time.monotonic()
            ok, image = self._cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            h, w = image.shape[:2]
            res = self._modele.predict(image, imgsz=self._taille, conf=self._conf,
                                       device=self._device, verbose=False)[0]
            dets = []
            noms = res.names
            for b in res.boxes:
                x1, y1, x2, y2 = b.xyxy[0].tolist()
                dets.append(Detection(noms[int(b.cls)], float(b.conf),
                                      (x1 + x2) / 2 / w, (y1 + y2) / 2 / h,
                                      (x2 - x1) / w, (y2 - y1) / h))
            with self._verrou:
                self._dernier = AnalyseImage(dets, time.monotonic(),
                                             (time.monotonic() - t0) * 1000)
            reste = self._periode - (time.monotonic() - t0)
            if reste > 0:
                time.sleep(reste)

    def analyser(self) -> Optional[AnalyseImage]:
        with self._verrou:
            return self._dernier

    def fermer(self) -> None:
        self._stop.set()
        self._cap.release()


# --------------------------------------------------------------------------- #
# Simulation
# --------------------------------------------------------------------------- #
class DetecteurSimule:
    """Renvoie une adventice (score 0,9) au centre quand ``presence`` est vrai.
    ``carte`` : fonction (x, y) → bool pour simuler des taches d'adventices
    sur la parcelle (le navigateur lui passe sa position)."""

    def __init__(self, carte=None):
        self.carte = carte or (lambda x, y: False)
        self.x = self.y = 0.0

    def analyser(self) -> AnalyseImage:
        if self.carte(self.x, self.y):
            return AnalyseImage([Detection("cyperus_rotundus", 0.9, 0.5, 0.6)], latence_ms=18.4)
        return AnalyseImage([Detection("arachide", 0.95, 0.5, 0.5)], latence_ms=18.4)

    def fermer(self) -> None:
        pass
