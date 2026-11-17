"""Gestionnaire de Caméra USB Filaire 170° et Serveur de Streaming MJPEG OpenCV.

Permet le scan des périphériques vidéo Linux (/dev/video*), l'acquisition temps réel,
le redressement grand angle 170°, l'annotation par RealVisionPipeline et le streaming MJPEG.
"""

import glob
import logging
import os
import re
import threading
import time
from typing import Any, Dict, Generator, List, Optional
import cv2
import numpy as np

try:
    from backend.vision.real_vision_pipeline import RealVisionPipeline
except ImportError:
    from real_vision_pipeline import RealVisionPipeline

logger = logging.getLogger("CameraManager")


class CameraManager:
    """Singleton de gestion de la caméra automobile 170° USB."""

    _instance: Optional["CameraManager"] = None

    def __init__(self):
        devices = self.scan_camera_devices()
        default_dev = devices[0] if devices else {"index": 2, "path": "/dev/video2", "name": "Caméra Externe USB (T'nB INCAM1080A) (/dev/video2)"}
        self.device_index: int = int(default_dev.get("index", 2))
        self.device_path: str = str(default_dev.get("path", f"/dev/video{self.device_index}"))
        self.width: int = 1280
        self.height: int = 720
        self.fps: int = 30
        self.k1: float = 0.0
        self.camera_tilt_deg: float = 35.0
        self.current_mission: str = "SEEDING"

        self.connected: bool = False
        self.is_synthetic: bool = False
        self.last_error: str = ""
        self.active_device_name: str = "Caméra Non Connectée"

        self.pipeline = RealVisionPipeline(
            chassis_width_m=1.21,
            tool_spacing_m=0.46,
            camera_height_m=0.75,
            camera_tilt_deg=self.camera_tilt_deg,
            k1=self.k1,
            fov_h_deg=76.0,
        )

        self._cap: Optional[cv2.VideoCapture] = None
        self._lock = threading.RLock()
        self._running: bool = False
        self._last_frame: Optional[Any] = None
        self._last_annotated_frame: Optional[Any] = None
        self._last_telemetry: Dict[str, Any] = {}
        self._capture_thread: Optional[threading.Thread] = None
        self._frame_counter: int = 0
        self._last_logged_order: str = ""
        self._last_logged_order_ts: float = 0.0
        self.on_vision_order_cb: Optional[Any] = None

    @classmethod
    def get_instance(cls) -> "CameraManager":
        if cls._instance is None:
            cls._instance = CameraManager()
        return cls._instance

    @staticmethod
    def scan_camera_devices() -> List[Dict[str, Any]]:
        """Scanne les périphériques vidéo /dev/video* (en filtrant les noeuds V4L2 metadata) et identifie la T'nB INCAM1080A."""
        devices = []
        video_paths = sorted(glob.glob("/dev/video*"))

        for path in video_paths:
            m = re.search(r"video(\d+)", path)
            if not m:
                continue
            idx = int(m.group(1))

            # Sous Linux V4L2, index=0 est le flux vidéo de capture et index=1 est le noeud metadata
            sys_idx_path = f"/sys/class/video4linux/video{idx}/index"
            if os.path.exists(sys_idx_path):
                try:
                    with open(sys_idx_path, "r", encoding="utf-8") as f:
                        v4l_sub_index = f.read().strip()
                        if v4l_sub_index != "0":
                            continue
                except Exception:
                    pass

            dev_name = ""
            sys_name_path = f"/sys/class/video4linux/video{idx}/name"
            if os.path.exists(sys_name_path):
                try:
                    with open(sys_name_path, "r", encoding="utf-8") as f:
                        dev_name = f.read().strip()
                except Exception:
                    pass

            uevent_txt = ""
            sys_uevent_path = f"/sys/class/video4linux/video{idx}/device/uevent"
            if os.path.exists(sys_uevent_path):
                try:
                    with open(sys_uevent_path, "r", encoding="utf-8") as f:
                        uevent_txt = f.read().lower()
                except Exception:
                    pass

            is_integrated = ("integrated" in dev_name.lower()) or ("0bda/5538" in uevent_txt)
            is_tnb = ("usb camera" in dev_name.lower()) or ("c45/6366" in uevent_txt) or (not is_integrated and idx >= 2)

            if is_integrated:
                name = f"Webcam Interne PC ({path})"
            elif is_tnb:
                name = f"Caméra Externe USB (T'nB INCAM1080A) ({path})"
            elif dev_name:
                name = f"Caméra Externe USB ({dev_name}) ({path})"
            else:
                name = f"Caméra Externe USB ({path})"

            devices.append({
                "index": idx,
                "path": path,
                "name": name,
                "raw_kernel_name": dev_name or f"video{idx}",
                "is_external": not is_integrated,
                "is_tnb_incam1080a": is_tnb,
                "codec": "MJPG",
                "default_resolution": "1280x720",
                "resolutions": ["1280x720", "1920x1080", "640x480"],
            })

        # Prioriser la webcam USB externe T'nB INCAM1080A (/dev/video2) devant la webcam interne PC (/dev/video0)
        devices.sort(key=lambda d: (0 if d.get("is_external", True) else 1, d["index"]))

        if not devices:
            devices.append({
                "index": 2,
                "path": "/dev/video2",
                "name": "Caméra Externe USB (T'nB INCAM1080A) (/dev/video2)",
                "raw_kernel_name": "USB Camera2.0: USB Camera",
                "is_external": True,
                "is_tnb_incam1080a": True,
                "codec": "MJPG",
                "default_resolution": "1280x720",
                "resolutions": ["1280x720", "1920x1080", "640x480"],
            })

        return devices

    def _try_open_v4l2_device(self, idx: int) -> tuple[Optional[cv2.VideoCapture], Optional[np.ndarray]]:
        """Tente d'ouvrir un périphérique V4L2 en MJPG puis YUYV et vérifie la lecture d'une image réelle."""
        for use_mjpg in (True, False):
            cap = None
            try:
                cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                if not cap or not cap.isOpened():
                    if cap:
                        cap.release()
                    cap = cv2.VideoCapture(idx)
                if not cap or not cap.isOpened():
                    if cap:
                        cap.release()
                    continue

                try:
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except Exception:
                    pass
                if use_mjpg:
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                cap.set(cv2.CAP_PROP_FPS, self.fps)

                # Essai de lecture d'une vraie image (jusqu'à 5 tentatives pour le chauffe-capteur USB)
                for _ in range(5):
                    ret, frame = cap.read()
                    if ret and frame is not None and frame.size > 0:
                        return cap, frame
                    time.sleep(0.03)

                cap.release()
            except Exception as e:
                logger.warning("Échec test V4L2 index %d (mjpg=%s) : %s", idx, use_mjpg, e)
                if cap:
                    try:
                        cap.release()
                    except Exception:
                        pass
        return None, None

    def connect(
        self,
        device_index: Optional[int] = None,
        width: Any = None,
        height: Optional[int] = None,
        fps: Optional[int] = None,
        resolution: Optional[str] = None,
        k1: Optional[float] = None,
        tilt_deg: Optional[float] = None,
        mission: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Ouvre la caméra USB sélectionnée (T'nB INCAM1080A /dev/video2 par défaut) ou active le fallback synthétique."""
        # Arrêter proprement un éventuel thread précédent HORS du lock pour éviter tout deadlock
        self.disconnect()

        with self._lock:
            scanned = self.scan_camera_devices()
            if device_index is None or int(device_index) < 0:
                self.device_index = int(scanned[0]["index"]) if scanned else 2
            else:
                self.device_index = int(device_index)
            self.device_path = f"/dev/video{self.device_index}"

            # Compatibilité totale : connect(2, 1280, 720, 30) OU connect(2, "1280x720", k1=...)
            if isinstance(width, str) and resolution is None:
                resolution = width
                width = None

            if resolution is not None:
                res_str = str(resolution).lower()
                if "1920" in res_str or "1080" in res_str:
                    self.width = 1920
                    self.height = 1080
                elif "1280" in res_str or "720" in res_str:
                    self.width = 1280
                    self.height = 720
                else:
                    self.width = 640
                    self.height = 480
            elif width is not None and height is not None:
                self.width = int(width)
                self.height = int(height)

            if fps is not None:
                self.fps = int(fps)
            if k1 is not None:
                self.k1 = float(k1)
            if tilt_deg is not None:
                self.camera_tilt_deg = float(tilt_deg)
            if mission is not None:
                self.current_mission = str(mission)

            self.pipeline.set_optical_config(k1=self.k1, tilt_deg=self.camera_tilt_deg)

            cap, first_frame = self._try_open_v4l2_device(self.device_index)

            matched_dev = next((d for d in scanned if d["index"] == self.device_index), None)
            friendly_name = (
                matched_dev["name"]
                if matched_dev
                else (
                    f"Caméra Externe USB (T'nB INCAM1080A) (/dev/video{self.device_index})"
                    if self.device_index >= 2
                    else f"Caméra Réelle USB (/dev/video{self.device_index})"
                )
            )

            if cap is not None and first_frame is not None:
                self._cap = cap
                self.connected = True
                self.is_synthetic = False
                self.active_device_name = friendly_name
                self.last_error = ""
                # Mettre à jour la résolution réelle négociée avec le capteur V4L2
                actual_h, actual_w = first_frame.shape[:2]
                self.width = int(actual_w)
                self.height = int(actual_h)
                annotated, telem = self.pipeline.process_frame(first_frame, mission=self.current_mission)
                self._last_frame = first_frame
                self._last_annotated_frame = annotated
                self._last_telemetry = telem
                logger.info("✅ Caméra physique %s ouverte et vérifiée (%dx%d)", friendly_name, self.width, self.height)
            else:
                self._cap = None
                self.connected = True
                self.is_synthetic = True
                self.active_device_name = "Flux Virtuel 170° (Simulation Agricole Haute Fidélité)"
                self.last_error = "Aucune caméra matérielle USB n'a répondu — Mode flux synthétique 170° activé"
                synth = self.pipeline.generate_synthetic_farm_frame(
                    width=self.width, height=self.height, frame_idx=0, mission=self.current_mission
                )
                annotated, telem = self.pipeline.process_frame(synth, mission=self.current_mission)
                self._last_frame = synth
                self._last_annotated_frame = annotated
                self._last_telemetry = telem
                logger.info("ℹ️ Mode flux caméra synthétique 170° activé pour /dev/video%d", self.device_index)

            self._running = True
            self._capture_thread = threading.Thread(
                target=self._capture_loop, name="CameraCaptureThread", daemon=True
            )
            self._capture_thread.start()

            return self.get_status()

    def disconnect(self) -> Dict[str, Any]:
        """Arrête la capture et libère le périphérique sans deadlock."""
        self._running = False
        thread = self._capture_thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._capture_thread = None

        with self._lock:
            if self._cap:
                try:
                    self._cap.release()
                except Exception:
                    pass
                self._cap = None

            self.connected = False
            self.is_synthetic = False
            self.active_device_name = "Déconnectée"
            return self.get_status()

    def set_config(
        self,
        k1: Optional[float] = None,
        tilt_deg: Optional[float] = None,
        mission: Optional[str] = None,
        camera_tilt_deg: Optional[float] = None,
        current_mission: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Modifie les réglages optiques ou la mission active à chaud."""
        effective_tilt = camera_tilt_deg if camera_tilt_deg is not None else tilt_deg
        effective_mission = current_mission if current_mission is not None else mission

        if k1 is not None:
            self.k1 = float(k1)
        if effective_tilt is not None:
            self.camera_tilt_deg = float(effective_tilt)
        if effective_mission is not None:
            self.current_mission = effective_mission

        self.pipeline.set_optical_config(k1=self.k1, tilt_deg=self.camera_tilt_deg)
        return self.get_status()

    def _capture_loop(self) -> None:
        """Boucle permanente d'acquisition d'images et d'annotation Vision IA."""
        consecutive_failures = 0
        while self._running:
            raw_frame = None

            if self._cap and not self.is_synthetic:
                ret, frame = self._cap.read()
                if ret and frame is not None and frame.size > 0:
                    raw_frame = frame
                    consecutive_failures = 0
                else:
                    consecutive_failures += 1
                    if consecutive_failures > 30:
                        logger.warning("Perte temporaire flux /dev/video%d — tentative de réouverture...", self.device_index)
                        cap, recovered_frame = self._try_open_v4l2_device(self.device_index)
                        if cap is not None and recovered_frame is not None:
                            with self._lock:
                                if self._cap:
                                    try:
                                        self._cap.release()
                                    except Exception:
                                        pass
                                self._cap = cap
                            raw_frame = recovered_frame
                            consecutive_failures = 0
                        else:
                            # Bascule douce sur le flux de secours sans crasher le thread
                            self._frame_counter += 1
                            raw_frame = self.pipeline.generate_synthetic_farm_frame(
                                width=self.width, height=self.height, frame_idx=self._frame_counter, mission=self.current_mission
                            )
                    else:
                        time.sleep(0.02)
                        continue
            else:
                self._frame_counter += 1
                raw_frame = self.pipeline.generate_synthetic_farm_frame(
                    width=self.width, height=self.height, frame_idx=self._frame_counter, mission=self.current_mission
                )

            if raw_frame is not None:
                annotated, telem = self.pipeline.process_frame(
                    raw_frame, mission=self.current_mission
                )
                with self._lock:
                    self._last_frame = raw_frame
                    self._last_annotated_frame = annotated
                    self._last_telemetry = telem

                order_str = str(telem.get("robot_order_str", ""))
                now_ts = time.time()
                if self.on_vision_order_cb and order_str:
                    if order_str != self._last_logged_order or (now_ts - self._last_logged_order_ts) >= 2.5:
                        self._last_logged_order = order_str
                        self._last_logged_order_ts = now_ts
                        try:
                            self.on_vision_order_cb(order_str, telem)
                        except Exception:
                            pass

            time.sleep(0.033)  # ≈ 30 FPS

    def get_latest_annotated_frame(self) -> Optional[np.ndarray]:
        """Renvoie la dernière image annotée disponible."""
        with self._lock:
            if self._last_annotated_frame is not None:
                return self._last_annotated_frame.copy()
        # Si la caméra n'est pas encore connectée, générer une image d'aperçu annotée sans allumer la webcam physique
        self._frame_counter += 1
        synth = self.pipeline.generate_synthetic_farm_frame(
            self.width, self.height, self._frame_counter, mission=self.current_mission
        )
        annotated, telem = self.pipeline.process_frame(synth, mission=self.current_mission)
        with self._lock:
            self._last_telemetry = telem
        return annotated

    def get_latest_telemetry(self) -> Dict[str, Any]:
        """Renvoie les résultats de l'analyse Vision IA en cours."""
        with self._lock:
            if not self._last_telemetry:
                synth = self.pipeline.generate_synthetic_farm_frame(
                    self.width, self.height, self._frame_counter, mission=self.current_mission
                )
                _, telem = self.pipeline.process_frame(synth, mission=self.current_mission)
                self._last_telemetry = telem
            return dict(self._last_telemetry)

    def generate_mjpeg_stream(self) -> Generator[bytes, None, None]:
        """Générateur de flux vidéo multipart/x-mixed-replace pour affichage navigateur <img>."""
        while True:
            frame = self.get_latest_annotated_frame()
            if frame is None:
                frame = self.pipeline.generate_synthetic_farm_frame(
                    self.width, self.height, 0, mission=self.current_mission
                )

            ret, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ret:
                frame_bytes = jpeg.tobytes()
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
                )
            time.sleep(0.035)  # ~28 FPS

    def get_status(self) -> Dict[str, Any]:
        """Retourne l'état complet du gestionnaire caméra."""
        pc = getattr(self.pipeline, "peanut_classifier", None)
        onnx_loaded = bool(pc is not None and getattr(pc, "onnx_loaded", False))
        return {
            "connected": self.connected,
            "is_synthetic": self.is_synthetic,
            "device_index": self.device_index,
            "device_path": self.device_path,
            "device_name": self.active_device_name,
            "resolution": f"{self.width}x{self.height}",
            "fps": self.fps,
            "k1": self.k1,
            "camera_tilt_deg": self.camera_tilt_deg,
            "current_mission": self.current_mission,
            "last_error": self.last_error,
            "onnx_loaded": onnx_loaded,
            "model_backend": getattr(pc, "model_backend", "") if pc else "",
            "telemetry": self.get_latest_telemetry(),
        }


# Instance singleton globale
camera_manager = CameraManager.get_instance()

