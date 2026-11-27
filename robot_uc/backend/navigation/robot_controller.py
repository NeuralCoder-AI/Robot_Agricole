"""Contrôleur du Robot Agricole et Couche d'Abstraction Série / Simulateur.

Gère la communication (simulateur VirtualESP32FirmwareV2 ou port pyserial réel),
le tri des flux (OK/ERR, télémétrie #T, fins d'action #D n), le heartbeat PING (300 ms),
le gestionnaire de mode (MANUEL / AUTOMATIQUE), le suivi de progression des missions
(avec pause/reprise) et la synchronisation des métriques agronomiques 3D.
"""

from collections import deque
import enum
import json
import logging
import math
import os
import queue
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

from backend.hardware.esp32_simulator import VirtualESP32FirmwareV2
from backend.vision.ai_vision_brain import AIVisionBrain, SIM_CM_TO_3D
from backend.navigation.agri_nav_engine import AgriNavEngine, RtkImuEkfLocalizer
from backend.hardware.solar_bms import SolarBMS
from backend.navigation.obstacle_avoidance import SymmetricTrapezoidPlanner
from backend.navigation.kinematic_engine import KinematicEngine
from backend.navigation.reseeding_planner import (
    ReseedingPlanner,
    SeedMapMemory,
    SAHEL_INTRA_ROW_PRESET_CM,
    SEED_DEPTH_TARGET_CM,
)
from backend.config.robot_config import robot_config, is_robot_uc_mode, get_deployment_target
from backend.hardware.hardware_bridge import (
    ConnectionTransport,
    ESP32HardwareBridge,
    ExecutionMode,
    calculate_xor_checksum,
    format_command_with_optional_crc,
    is_console_noise_line,
)

# Import optionnel de pyserial si disponible
try:
    import serial
except ImportError:
    serial = None

logger = logging.getLogger("RobotController")
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

BASE_DIR: Path = Path(__file__).resolve().parent
ROBOT_SPECS_FILE: str = str(BASE_DIR / "robot_specs.json")

# Profil de dimensions physiques réelles par défaut (Relevé Terrain du 26/10/2026)
DEFAULT_ROBOT_SPECS: Dict[str, Any] = {
    # 1. Châssis & Gabarit Général (Forme Trapézoïdale avec pente en bas)
    "chassis_length_top_cm": 169.0,
    "chassis_length_bottom_cm": 130.0,
    "chassis_slope_length_cm": 30.0,
    "total_width_with_wheels_cm": 121.0,
    "chassis_body_width_cm": 81.0,
    "total_height_folded_cm": 88.0,
    "ground_clearance_cm": 24.0,
    "inner_tunnel_width_cm": 79.5,
    # Alias rétrocompatibles
    "chassis_length_cm": 169.0,
    "chassis_width_cm": 121.0,
    "chassis_height_cm": 88.0,
    # 2. Roues Métalliques à Crampons (M1 à M4)
    "tire_diameter_cm": 26.5,
    "tire_circumference_cm": 83.25,
    "tire_width_cm": 19.0,
    "track_width_cm": 103.0,
    "wheelbase_cm": 110.0,
    # 3. Système à 4 Dents en Tandem (2 Petites Devant Creuseuses + 2 Grosses Derrière Recouvreuses, M5) & Fraise (M6)
    "tines_bar_width_cm": 84.0,
    "tines_width_cm": 84.0,
    "tines_total_count": 4,
    "front_small_tine_width_cm": 13.0,
    "rear_large_tine_width_cm": 25.0,
    "front_to_rear_tines_distance_cm": 30.0,
    "rear_large_tines_gap_cm": 29.5,
    "front_small_tines_gap_cm": 41.5,
    "tines_raised_height_cm": 12.0,
    "tines_max_depth_cm": 8.0,
    "tines_depth_cm": 8.0,
    "tines_distance_from_front_cm": 170.0,
    "weeder_width_cm": 23.0,
    "weeder_rotor_diameter_cm": 5.0,
    "tines_to_weeder_distance_cm": 196.0,
    # 4. Semoir de Précision Double Rang (M8)
    "front_axle_to_seeder_cm": 165.0,
    "tines_to_seeder_cm": 29.0,
    "seed_drop_height_cm": 14.0,
    "seeder_nozzles_count": 2,
    "seeder_row_spacing_cm": 46.0,
    "mechanical_nozzle_spacing_cm": 46.0,
    "sahel_theoretical_inter_row_min_cm": 30.0,
    "sahel_theoretical_inter_row_max_cm": 40.0,
    "seed_spacing_cm": 15.0,  # Préréglage sahélien Niger 10–15 cm (configurable 10–20 cm)
    "seed_depth_cm": 5.0,     # Profondeur cible 4,0 à 6,0 cm (Bakoye et al., 2020)
    "seed_with_tines_down": True,
    # 5. Panneaux Solaires Double Vantail (M7) & Caméra IA
    "solar_closed_length_cm": 173.0,
    "solar_closed_width_cm": 106.0,
    "solar_deployed_span_cm": 99.0,
    "camera_height_cm": 90.0,
    "camera_to_front_axle_cm": 25.0,
    "camera_tilt_deg": 32.0,
}


class RobotMode(str, enum.Enum):
    MANUAL = "MANUAL"
    AUTO = "AUTO"


class RobotController:
    """Interface unifiée Série / Simulateur et Moteur d'Autonomie."""

    def __init__(
        self,
        use_simulator: Optional[bool] = None,
        port: Optional[str] = None,
        baudrate: int = 115200,
    ):
        if use_simulator is None:
            env_sim = os.getenv("USE_SIMULATOR", "true").strip().lower()
            self.use_simulator = env_sim not in ("false", "0", "no")
        else:
            self.use_simulator = use_simulator

        default_serial = "/dev/ttyUSB0" if os.name != "nt" else "COM3"
        self.port = port or os.getenv("SERIAL_PORT", default_serial)
        self.baudrate = baudrate

        # Connexion matérielle ou virtuelle
        self.device: Any = None
        self.simulator: Optional[VirtualESP32FirmwareV2] = None

        # Mode de fonctionnement et état de mission
        self.mode: RobotMode = RobotMode.MANUAL
        self.current_mission_name: Optional[str] = None
        self.mission_progress: int = 0
        self.mission_step_label: str = "En attente"
        self.mission_paused: bool = False
        self.last_mission_alert: Optional[Dict[str, Any]] = None
        self._last_corridor_ended_by_vision: bool = False
        self._recent_logs: deque[Dict[str, Any]] = deque(maxlen=30)
        self._mission_started_ts: Optional[float] = None
        self._mission_ended_ts: Optional[float] = None
        self._last_mission_status: str = "IDLE"
        self._last_mission_type: str = "manual"
        self.direction_mode: str = "AUTO"
        self.locked_side: Optional[str] = None
        self.done_history: List[Dict[str, Any]] = []
        self._mission_thread: Optional[threading.Thread] = None
        self._mission_stop_flag = threading.Event()
        self._mission_pause_event = threading.Event()
        self._mission_pause_event.set()  # Non en pause par défaut

        # État et Télémétrie #T
        self.armed: int = 0
        self.is_armed: bool = False
        self.estop_active: bool = True  # Priorité ESTOP active au démarrage
        self.mission_running: bool = False
        self.state: str = "ESTOP"
        self.last_telemetry: Dict[str, Any] = {
            "armed": 0,
            "motors": [0] * 8,
            "timestamp": 0,
        }

        # Dimensions mécaniques réelles (issues de robot_specs.json / DEFAULT_ROBOT_SPECS)
        self._specs_lock = threading.Lock()
        self.robot_specs: Dict[str, Any] = self._load_robot_specs()

        # Télémétrie terrain, cinématique & métriques agronomiques (synchronisées avec la scène 3D)
        self.terrain_pitch: float = 0.0
        self.terrain_roll: float = 0.0
        self.speed_kmh: float = 0.0
        self.weeds_destroyed: int = 0
        self.weeds_total: int = 180
        self.seeds_sown: int = 0
        self.seed_spacing_cm: int = int(self.robot_specs.get("seed_spacing_cm", 25))
        self.auto_seed_enabled: bool = False
        self.seeding_use_tines: bool = bool(self.robot_specs.get("seed_with_tines_down", True))
        self.field_length_m: float = 10.0
        self.field_width_m: float = 4.6
        self.corridors_total: int = 10
        self.current_corridor: int = 1
        self.distance_traveled_m: float = 0.0
        self._distance_since_last_seed_cm: float = 0.0
        self.tines_depth: float = 0.0  # 0.0 à 1.0
        self.solar_angle: float = 0.0  # 0.0 à 1.0
        self.obstacle_detected: bool = False
        self.obstacle_distance: float = 99.0
        self.obstacle_is_moving: bool = False
        self.obstacle_speed_mps: float = 0.0
        self.obstacle_state: str = "CLEAR"
        self.bypass_plan_locked: bool = False
        self.avoidance_memory: Optional[Dict[str, Any]] = None
        self.avoidance_plan: Optional[Dict[str, Any]] = None
        self.avoidance_path_points: List[List[float]] = []
        self._last_all_obstacles: List[Dict[str, Any]] = []

        # Contournement Trapézoïdal Symétrique en 3 Trajectoires Exactes (Mémorisation stricte)
        self.locked_alpha_rad: Optional[float] = None
        self.locked_dist_1_m: Optional[float] = None
        self.locked_d2_m: Optional[float] = None
        self.initial_line_x: Optional[float] = None
        self.initial_heading_rad: Optional[float] = None
        self.symmetric_planner: SymmetricTrapezoidPlanner = SymmetricTrapezoidPlanner(
            robot_width_m=float(self.robot_specs.get("total_width_with_wheels_cm", 121.0)) / 100.0,
            robot_length_m=float(self.robot_specs.get("chassis_length_top_cm", 169.0) + 46.0) / 100.0,
            front_overhang_m=1.05,
            rear_overhang_m=1.55,
            safety_margin_m=0.65,
            approach_l_m=1.40,
            extra_d2_clearance_m=1.15,
        )
        self.progression_direction: int = -1 # -1 = DROITE, +1 = GAUCHE
        self.robot_x: float = 0.0
        self.robot_z: float = 0.0
        self.robot_yaw: float = 0.0
        self.current_target_heading_rad: float = 0.0
        self.current_corridor_center_x: float = 0.0
        self.crop_rows_x: List[float] = [round(r * 0.46, 3) for r in range(-8, 9)]
        self.last_turn_decision: str = ""

        self.kinematic_engine: KinematicEngine = KinematicEngine(
            row_spacing=float(self.robot_specs.get("seeder_row_spacing_cm", 46.0)) / 100.0,
            field_length=float(self.field_length_m),
            field_width=float(self.field_width_m),
            start_x=float(self.robot_x),
            start_z=float(self.robot_z),
            step_dir=self.progression_direction,
        )
        self.seed_map: SeedMapMemory = SeedMapMemory()
        self.reseeding_planner: ReseedingPlanner = ReseedingPlanner(
            seeder_row_spacing_m=0.46,
            seeder_rear_offset_m=0.85,
            nominal_seed_gap_cm=max(10.0, min(20.0, float(self.seed_spacing_cm))),
            gap_detection_threshold_cm=35.0,
            camera_lookahead_m=1.80,
            seed_depth_cm=float(self.robot_specs.get("seed_depth_cm", SEED_DEPTH_TARGET_CM)),
            seed_map=self.seed_map,
        )
        self.solar_bms: SolarBMS = SolarBMS()
        self.reseeded_seeds_count: int = 0
        self.dead_plants_count: int = 0
        self.gaps_detected_count: int = 0
        self.reviewed_rows_count: int = 0
        self.dense_weed_detected: bool = False
        self.weeding_state: str = "IDLE"
        self.visual_lock_consecutive: int = 0
        self.vision_brain: AIVisionBrain = AIVisionBrain()
        self.vision_brain.set_robot_specs(self.robot_specs)
        self.nav_engine: AgriNavEngine = AgriNavEngine(self.robot_specs)
        self.ekf_localizer: RtkImuEkfLocalizer = self.nav_engine.ekf_localizer
        self._last_3d_feedback_time: float = 0.0
        self._last_telem_calc_time: float = time.perf_counter()

        # Événements de synchronisation pour les commandes et les fins d'action (#D n)
        self.done_events: Dict[int, threading.Event] = {i: threading.Event() for i in range(1, 9)}
        self._cmd_lock = threading.Lock()
        self._pending_response_queue: queue.Queue[str] = queue.Queue()

        # Callbacks pour WebSockets / UI
        self.telemetry_callbacks: List[Callable[[Dict[str, Any]], None]] = []
        self.event_callbacks: List[Callable[[str, Any], None]] = []
        self.log_callbacks: List[Callable[[str, str], None]] = []  # (direction, message)

        # Mode d'exécution : SIMULATION_ONLY, COUPLED ou REAL_ZERO_SIM (forcé si DEPLOYMENT_TARGET=ROBOT_UC)
        self.execution_mode: str = (
            ExecutionMode.REAL_ZERO_SIM.value
            if is_robot_uc_mode()
            else ExecutionMode.SIMULATION_ONLY.value
        )
        self.on_mission_stopped_callbacks: List[Callable[[], None]] = []
        self.use_xor_checksum: bool = False
        self.hardware_bridge: ESP32HardwareBridge = ESP32HardwareBridge(
            serial_port=self.port,
            baudrate=self.baudrate,
            use_xor_checksum=self.use_xor_checksum,
            on_line_callback=self._on_hardware_bridge_line,
            on_log_callback=self._on_hardware_bridge_log,
        )

        # Gestion des threads d'arrière-plan
        self._running: bool = True
        self._init_connection()

        # Thread de lecture permanente du port
        self.reader_thread = threading.Thread(target=self._reader_loop, name="SerialReader", daemon=True)
        self.reader_thread.start()

        # Thread Heartbeat (envoie PING toutes les 300 ms pour entretenir UWDT en silence)
        self.heartbeat_enabled: bool = True
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, name="Heartbeat", daemon=True)
        self.heartbeat_thread.start()

        # Synchronisation live avec la source unique de vérité `robot_config` (angle/hauteur caméra, dimensions)
        try:
            self.sync_dynamic_config(robot_config.get_specs())
            robot_config.register_listener(self.sync_dynamic_config)
        except Exception as e:
            logger.debug("Sync initiale robot_config ignorée : %s", e)

    # --------------------------------------------------------------------------
    # Initialisation de la connexion & Pont Matériel ESP32-S3 Firmware v2
    # --------------------------------------------------------------------------
    def _init_connection(self) -> None:
        if self.use_simulator:
            logger.info("Démarrage sur le Simulateur ESP32-S3 (VirtualESP32FirmwareV2)")
            self.simulator = VirtualESP32FirmwareV2()
            self.device = self.simulator
        else:
            if serial is None:
                raise RuntimeError("pyserial n'est pas installé sur le système")
            logger.info("Ouverture du port série physique %s à %d bauds", self.port, self.baudrate)
            self.device = serial.Serial(self.port, self.baudrate, timeout=0.1)

    def _emit_log(self, direction: str, message: str) -> None:
        """Émet un log console en filtrant à 100 % PING, PONG et #T, et l'archive dans le buffer circulaire (30 lignes)."""
        if is_console_noise_line(message):
            return
        self._recent_logs.append(
            {
                "direction": str(direction or "RX"),
                "line": str(message),
                "ts": int(time.time() * 1000),
            }
        )
        for cb in self.log_callbacks:
            try:
                cb(direction, message)
            except Exception:
                pass

    def get_recent_logs(self, limit: int = 30) -> List[Dict[str, Any]]:
        """Retourne les `limit` dernières lignes de journalisation console (sans PING/PONG/#T)."""
        lim = max(1, min(30, int(limit)))
        return list(self._recent_logs)[-lim:]

    @staticmethod
    def _normalize_mission_type(mission_name: Optional[str]) -> str:
        if not mission_name:
            return "manual"
        m = str(mission_name).strip().lower()
        if "reseed" in m:
            return "reseeding"
        if "seed" in m:
            return "seeding"
        if "weed" in m:
            return "weeding"
        if "manual" in m:
            return "manual"
        return m

    @staticmethod
    def _mission_label_for(mission_type: str) -> str:
        labels = {
            "seeding": "1. Semence Initiale",
            "reseeding": "2. Révision Semence",
            "weeding": "3. Désherbage IA",
            "manual": "4. Conduite Manuelle",
            "multi_row": "Parcours Multi-Rangs",
            "obstacle_avoidance": "Contournement Obstacles",
            "solar_deploy": "Déploiement Solaire M7",
            "solar_retract": "Rangement Solaire M7",
        }
        return labels.get(mission_type, f"Mission {mission_type}")

    def is_mission_locked(self) -> bool:
        """True tant qu'une mission est active ou en pause (verrouillage configuration/changement de mission)."""
        return bool(self.mission_running or self.mission_paused)

    def _on_hardware_bridge_log(self, direction: str, message: str) -> None:
        """Relaye les vrais échanges TX/RX du robot physique ESP32-S3 (sans PING/PONG/#T)."""
        if self.execution_mode in (ExecutionMode.COUPLED.value, ExecutionMode.REAL_ZERO_SIM.value):
            self._emit_log(direction, message)

    def _on_hardware_bridge_line(self, line: str) -> None:
        """Traite les trames temps réel reçues du robot physique ESP32-S3 (#T, #D n, OK, ERR)."""
        if not line:
            return
        clean = line.strip()
        if clean.startswith("#T "):
            parts = clean[3:].strip().split()
            if len(parts) >= 3:
                try:
                    armed_val = int(parts[0])
                    m_speeds = [int(v) for v in parts[1].split(",")]
                    ts_val = int(parts[2])
                    self.armed = armed_val
                    if armed_val == 1:
                        self.is_armed = True
                        self.estop_active = False
                    elif armed_val == 0 and self.estop_active:
                        self.is_armed = False
                    if self.simulator is not None:
                        self.simulator.armed = armed_val
                        for idx, spd in enumerate(m_speeds[:8]):
                            self.simulator.motors[idx].actual_speed = float(spd)
                            self.simulator.motors[idx].target_speed = float(spd)
                    self._update_fallback_agronomic_simulation(m_speeds)
                    self.last_telemetry = {
                        "armed": armed_val,
                        "motors": m_speeds,
                        "timestamp": ts_val,
                        "raw": clean,
                        "agronomics": self.get_agronomic_metrics(),
                        "mission_state": self.get_mission_state(),
                        "hardware": self.get_hardware_status(),
                    }
                    for cb in self.telemetry_callbacks:
                        try:
                            cb(self.last_telemetry)
                        except Exception:
                            pass
                except Exception as e:
                    logger.debug("Erreur parsing #T matériel : %s", e)
            return

        if clean.startswith("#D "):
            try:
                n = int(clean[3:].strip())
                if self.simulator is not None and 1 <= n <= 8:
                    self.simulator.motors[n - 1].target_speed = 0.0
                    self.simulator.motors[n - 1].actual_speed = 0.0
                    self.simulator.motors[n - 1].has_active_timer = False
                    self.simulator.motors[n - 1].timer_remaining_s = 0.0
                if n in self.done_events:
                    self.done_events[n].set()
                done_entry = {
                    "motor": n,
                    "raw": f"#D {n}",
                    "ts": int(time.time() * 1000),
                }
                self.done_history.append(done_entry)
                if len(self.done_history) > 50:
                    self.done_history = self.done_history[-50:]
                for cb in self.event_callbacks:
                    try:
                        cb("DONE", done_entry)
                    except Exception:
                        pass
            except ValueError:
                pass
            return

        if clean.upper().startswith("OK ARMED") or clean.upper() == "OK ARM":
            self.armed = 1
            self.is_armed = True
            self.estop_active = False
            self.state = "IDLE"
            self._mission_stop_flag.clear()
        elif clean.upper().startswith("OK ESTOP") or clean.upper() == "ERR: ESTOP":
            self.armed = 0
            self.is_armed = False
            self.estop_active = True
            self._force_stop_simulator(disarm=True)

    def _force_stop_simulator(self, disarm: bool = False) -> None:
        """Arrête immédiatement les 8 moteurs du simulateur 3D."""
        if self.simulator is not None:
            if disarm:
                self.simulator.armed = 0
                self.armed = 0
            for m in self.simulator.motors:
                m.target_speed = 0.0
                m.actual_speed = 0.0
                m.has_active_timer = False
                m.timer_remaining_s = 0.0

    def _mirror_command_to_simulator(self, wire_cmd: str) -> None:
        """Clone la commande acceptée par le robot physique vers le jumeau numérique 3D."""
        if self.simulator is not None:
            try:
                self.simulator.write((wire_cmd + "\r\n").encode("ascii"))
            except Exception as e:
                logger.debug("Erreur clonage vers simulateur : %s", e)

    def is_simulation_locked(self) -> bool:
        """Retourne True uniquement si le mode COUPLED est actif mais que le robot physique n'est pas connecté.
        En mode REAL_ZERO_SIM sans ESP32, le Mode Test Vision IA Seule (Dry-Run Hardware) prend le relais sans jamais bloquer.
        """
        return (
            self.execution_mode == ExecutionMode.COUPLED.value
            and not self.hardware_bridge.connected
        )

    def get_hardware_status(self) -> Dict[str, Any]:
        """Retourne l'état complet du mode d'exécution et du pont matériel ESP32-S3."""
        st = self.hardware_bridge.get_state()
        st["execution_mode"] = self.execution_mode
        st["simulation_locked"] = self.is_simulation_locked()
        is_vision_test_mode = (
            self.execution_mode == ExecutionMode.REAL_ZERO_SIM.value
            and not self.hardware_bridge.connected
        )
        st["vision_test_mode"] = is_vision_test_mode
        st["status_badge"] = (
            "🤖 Robot : Mode Test Vision (ESP32 non requis)"
            if is_vision_test_mode
            else (
                "🟢 Robot ESP32-S3 Connecté"
                if self.hardware_bridge.connected
                else "⚪ Simulation Virtuelle"
            )
        )
        if hasattr(self, "solar_bms") and self.solar_bms:
            st["solar_bms"] = self.solar_bms.get_status()
        return st

    def _on_camera_vision_order(self, order_str: str, telem: Dict[str, Any]) -> None:
        """Logue en console les décisions de la Vision IA en mode REAL_ZERO_SIM."""
        if self.execution_mode == ExecutionMode.REAL_ZERO_SIM.value:
            self._emit_log("RX", f"[VISION IA -> ORDRE ROBOT] {order_str}")

    def set_execution_mode(self, mode: str) -> Dict[str, Any]:
        """Bascule entre SIM_ONLY, COUPLED et REAL_ZERO_SIM (verrouillé sur REAL_ZERO_SIM si ROBOT_UC)."""
        if is_robot_uc_mode():
            norm = ExecutionMode.REAL_ZERO_SIM.value
        else:
            norm = str(mode).strip().upper()
            if norm in ("SIM_ONLY", "SIMULATION_ONLY", "SIM"):
                norm = ExecutionMode.SIMULATION_ONLY.value
            elif norm == ExecutionMode.COUPLED.value:
                norm = ExecutionMode.COUPLED.value
            elif norm in (ExecutionMode.REAL_ZERO_SIM.value, "REAL", "ZERO_SIM", "REAL_ZERO"):
                norm = ExecutionMode.REAL_ZERO_SIM.value
            else:
                norm = ExecutionMode.SIMULATION_ONLY.value
        self.execution_mode = norm
        if norm == ExecutionMode.REAL_ZERO_SIM.value:
            try:
                from backend.vision.camera_manager import camera_manager
                camera_manager.on_vision_order_cb = self._on_camera_vision_order
                if not camera_manager.connected:
                    camera_manager.connect()
            except Exception:
                pass
            # En mode Test Vision (ESP32 non branché), pré-armer le Dry-Run pour lancer les 4 missions sans blocage
            if not self.hardware_bridge.connected:
                self.armed = 1
                self.is_armed = True
                self.estop_active = False
                self.state = "IDLE"
                self._mission_stop_flag.clear()
                self._mirror_command_to_simulator("ARM")
        if self.is_simulation_locked():
            self.abort_mission()
            self._force_stop_simulator(disarm=False)
        for cb in self.event_callbacks:
            try:
                cb("HARDWARE_STATUS", self.get_hardware_status())
            except Exception:
                pass
        return self.get_hardware_status()

    def configure_and_connect_hardware(self, config: Dict[str, Any]) -> Dict[str, Any]:
        """Configure le transport (WIFI_HTTP, SOCKET_TCP_WS, USB_UART) et vérifie la connexion à l'ESP32-S3."""
        if "execution_mode" in config and config["execution_mode"]:
            mode_val = str(config["execution_mode"]).strip().upper()
            if mode_val in ("SIM_ONLY", "SIMULATION_ONLY"):
                self.execution_mode = ExecutionMode.SIMULATION_ONLY.value
            elif mode_val in (ExecutionMode.COUPLED.value, ExecutionMode.REAL_ZERO_SIM.value):
                self.execution_mode = mode_val
        if "use_xor_checksum" in config and config["use_xor_checksum"] is not None:
            self.use_xor_checksum = bool(config["use_xor_checksum"])

        target = str(config.get("target", "")).strip()
        serial_port = config.get("serial_port")
        host = config.get("host")
        port = config.get("port")
        if target:
            if target.startswith("/dev/") or target.upper().startswith("COM"):
                serial_port = target
            else:
                if ":" in target:
                    parts = target.split(":")
                    host = parts[0].strip()
                    try:
                        port = int(parts[1].strip())
                    except ValueError:
                        pass
                else:
                    host = target

        self.hardware_bridge.configure(
            transport=config.get("transport"),
            host=host,
            port=port,
            serial_port=serial_port,
            baudrate=config.get("baudrate"),
            use_xor_checksum=config.get("use_xor_checksum"),
            username=config.get("username"),
            password=config.get("password"),
        )
        timeout = float(config.get("timeout", 2.0))
        self.hardware_bridge.connect_and_verify(timeout=timeout)
        if self.hardware_bridge.connected:
            self.armed = self.hardware_bridge.armed
            if self.armed == 1:
                self.is_armed = True
                self.estop_active = False
            if self.simulator is not None:
                self.simulator.armed = self.armed
        elif self.is_simulation_locked():
            self._force_stop_simulator(disarm=False)

        status = self.get_hardware_status()
        for cb in self.event_callbacks:
            try:
                cb("HARDWARE_STATUS", status)
            except Exception:
                pass
        return status

    def auto_connect_uc_hardware(self) -> Dict[str, Any]:
        """En mode embarqué ROBOT_UC : connecte automatiquement la caméra USB T'nB INCAM1080A
        et tente la détection + connexion automatique du port série ESP32-S3 (/dev/ttyACM* / /dev/ttyUSB*).
        """
        cam_status: Dict[str, Any] = {}
        try:
            from backend.vision.camera_manager import camera_manager
            camera_manager.on_vision_order_cb = self._on_camera_vision_order
            if not camera_manager.connected:
                cam_status = camera_manager.connect()
        except Exception as e:
            logger.warning("Auto-connexion caméra UC échouée : %s", e)

        ports = [
            p["device"]
            for p in ESP32HardwareBridge.scan_serial_ports()
            if os.path.exists(p.get("device", "")) and ("ACM" in p["device"] or "USB" in p["device"])
        ]
        hw_status: Dict[str, Any] = self.get_hardware_status()
        for dev in ports:
            logger.info("Boot ROBOT_UC : tentative auto-connexion ESP32 sur %s...", dev)
            hw_status = self.configure_and_connect_hardware(
                {
                    "execution_mode": ExecutionMode.REAL_ZERO_SIM.value,
                    "transport": ConnectionTransport.USB_UART.value,
                    "serial_port": dev,
                    "baudrate": 115200,
                    "timeout": 1.5,
                }
            )
            if hw_status.get("connected"):
                logger.info("✅ Boot ROBOT_UC : ESP32-S3 connecté sur %s", dev)
                break

        return {"camera": cam_status, "hardware": hw_status, "serial_ports_detected": ports}

    def ping_hardware(self, config: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Teste directement la communication PING -> PONG avec le robot matériel ESP32-S3."""
        if config:
            self.hardware_bridge.configure(
                transport=config.get("transport"),
                host=config.get("host"),
                port=config.get("port"),
                serial_port=config.get("serial_port"),
                baudrate=config.get("baudrate"),
                use_xor_checksum=config.get("use_xor_checksum"),
                username=config.get("username"),
                password=config.get("password"),
            )
            if not self.hardware_bridge.connected:
                self.hardware_bridge.connect_and_verify(timeout=float(config.get("timeout", 1.5)))
        t0 = time.perf_counter()
        if self.hardware_bridge.connected:
            res = self.hardware_bridge.send_command("PING", timeout=1.2, silent=False)
            lat = round((time.perf_counter() - t0) * 1000.0, 1)
            ok = ("PONG" in res.upper() or res.upper().startswith("OK")) and not res.upper().startswith("ERR:")
            return {
                "ok": ok,
                "response": res,
                "latency_ms": lat,
                "connected": self.hardware_bridge.connected,
                "transport": self.hardware_bridge.transport,
                "last_error": "" if ok else (self.hardware_bridge.last_error or res),
            }
        return {
            "ok": False,
            "response": "ERR: NOT_CONNECTED",
            "latency_ms": 0.0,
            "connected": False,
            "transport": self.hardware_bridge.transport,
            "last_error": self.hardware_bridge.last_error or "Robot ESP32-S3 non connecté",
        }

    def disconnect_hardware(self) -> Dict[str, Any]:
        """Déconnecte le pont matériel ESP32-S3 et verrouille la simulation si en mode COUPLED."""
        self.hardware_bridge.disconnect()
        if self.is_simulation_locked():
            self.abort_mission()
            self._force_stop_simulator(disarm=False)
        status = self.get_hardware_status()
        for cb in self.event_callbacks:
            try:
                cb("HARDWARE_STATUS", status)
            except Exception:
                pass
        return status

    def read_all_hardware_configs(self) -> Dict[str, Any]:
        """Lit GCFG, STATUS, PERF, VER et CFG:1..8 sur le robot actif (ou simulateur)."""
        results: Dict[str, Any] = {}
        for cmd_key, cmd_str in (
            ("ver", "VER"),
            ("status", "STATUS"),
            ("gcfg", "GCFG"),
            ("perf", "PERF"),
        ):
            results[cmd_key] = self.send_command(cmd_str)
        motor_cfgs: Dict[int, str] = {}
        for m in range(1, 9):
            motor_cfgs[m] = self.send_command(f"CFG:{m}")
        results["motor_configs"] = motor_cfgs
        results["hardware"] = self.get_hardware_status()
        return results

    def calibrate_motor_nvs(
        self,
        motor_id: int,
        inv: Optional[int] = None,
        min_pwm: Optional[int] = None,
        max_pwm: Optional[int] = None,
        ramp: Optional[int] = None,
        tmax: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Applique les réglages matériels NVS Firmware v2 pour un moteur M1..M8."""
        m = max(1, min(8, int(motor_id)))
        responses: List[str] = []
        if inv is not None:
            responses.append(self.send_command(f"INV:{m}:{1 if int(inv) else 0}"))
        if min_pwm is not None:
            responses.append(self.send_command(f"MIN:{m}:{max(0, min(90, int(min_pwm)))}"))
        if max_pwm is not None:
            responses.append(self.send_command(f"MAX:{m}:{max(0, min(100, int(max_pwm)))}"))
        if ramp is not None:
            responses.append(self.send_command(f"RAMP:{m}:{max(1, min(1000, int(ramp)))}"))
        if tmax is not None:
            t_val = max(0.0, min(3600.0, float(tmax)))
            t_str = str(int(t_val)) if t_val.is_integer() else f"{t_val:.1f}"
            responses.append(self.send_command(f"TMAX:{m}:{t_str}"))
        cfg_line = self.send_command(f"CFG:{m}")
        return {
            "motor": m,
            "responses": responses,
            "cfg": cfg_line,
            "hardware": self.get_hardware_status(),
        }

    def close(self) -> None:
        """Arrêt propre du contrôleur."""
        self._running = False
        self.heartbeat_enabled = False
        self.abort_mission()
        if hasattr(self, "hardware_bridge") and self.hardware_bridge:
            try:
                self.hardware_bridge.close()
            except Exception:
                pass
        if self.device:
            try:
                self.device.close()
            except Exception as e:
                logger.error("Erreur à la fermeture du périphérique : %s", e)

    # --------------------------------------------------------------------------
    # Gestionnaire des Dimensions & Spécifications Mécaniques (robot_specs.json)
    # --------------------------------------------------------------------------
    def _normalize_specs_dict(self, specs: Dict[str, Any]) -> Dict[str, Any]:
        """Synchronise les alias et recalcule automatiquement la circonférence de roue et l'écart des dents."""
        merged = dict(DEFAULT_ROBOT_SPECS)
        merged.update(specs)

        # Synchronisation bidirectionnelle des alias rétrocompatibles
        if "chassis_length_top_cm" in specs:
            merged["chassis_length_cm"] = float(merged["chassis_length_top_cm"])
        elif "chassis_length_cm" in specs:
            merged["chassis_length_top_cm"] = float(merged["chassis_length_cm"])

        if "total_width_with_wheels_cm" in specs:
            merged["chassis_width_cm"] = float(merged["total_width_with_wheels_cm"])
        elif "chassis_width_cm" in specs:
            merged["total_width_with_wheels_cm"] = float(merged["chassis_width_cm"])

        if "total_height_folded_cm" in specs:
            merged["chassis_height_cm"] = float(merged["total_height_folded_cm"])
        elif "chassis_height_cm" in specs:
            merged["total_height_folded_cm"] = float(merged["chassis_height_cm"])

        if "tines_bar_width_cm" in specs:
            merged["tines_width_cm"] = float(merged["tines_bar_width_cm"])
        elif "tines_width_cm" in specs:
            merged["tines_bar_width_cm"] = float(merged["tines_width_cm"])

        if "tines_max_depth_cm" in specs:
            merged["tines_depth_cm"] = float(merged["tines_max_depth_cm"])
        elif "tines_depth_cm" in specs:
            merged["tines_max_depth_cm"] = float(merged["tines_depth_cm"])

        # Calcul automatique de la circonférence C = pi * D (ex: pi * 26.5 = 83.25 cm)
        diam = max(5.0, float(merged.get("tire_diameter_cm", 26.5)))
        merged["tire_diameter_cm"] = diam
        merged["tire_circumference_cm"] = round(math.pi * diam, 2)

        # Calcul de l'écart intérieur entre les 2 petites dents avant si non forcé explicitement
        if "front_small_tines_gap_cm" not in specs and (
            "rear_large_tines_gap_cm" in specs
            or "rear_large_tine_width_cm" in specs
            or "front_small_tine_width_cm" in specs
        ):
            rear_gap = float(merged.get("rear_large_tines_gap_cm", 29.5))
            rear_w = float(merged.get("rear_large_tine_width_cm", 25.0))
            front_w = float(merged.get("front_small_tine_width_cm", 13.0))
            merged["front_small_tines_gap_cm"] = round(max(5.0, rear_gap + (rear_w - front_w)), 1)

        return merged

    def _load_robot_specs(self) -> Dict[str, Any]:
        """Charge robot_specs.json ou le crée avec DEFAULT_ROBOT_SPECS s'il est absent."""
        if os.path.isfile(ROBOT_SPECS_FILE):
            try:
                with open(ROBOT_SPECS_FILE, "r", encoding="utf-8") as f:
                    data = json.load(f)
                if isinstance(data, dict):
                    norm = self._normalize_specs_dict(data)
                    self._save_robot_specs(norm)
                    return norm
            except Exception as e:
                logger.warning("Erreur lecture %s (%s), utilisation de DEFAULT_ROBOT_SPECS", ROBOT_SPECS_FILE, e)
        norm_default = self._normalize_specs_dict(DEFAULT_ROBOT_SPECS)
        self._save_robot_specs(norm_default)
        return norm_default

    def _save_robot_specs(self, specs: Dict[str, Any]) -> None:
        try:
            with open(ROBOT_SPECS_FILE, "w", encoding="utf-8") as f:
                json.dump(specs, f, indent=2, ensure_ascii=False)
        except Exception as e:
            logger.error("Impossible d'enregistrer %s : %s", ROBOT_SPECS_FILE, e)

    def get_robot_specs(self) -> Dict[str, Any]:
        """Retourne une copie des dimensions physiques actuelles du robot."""
        with self._specs_lock:
            return dict(self.robot_specs)

    def update_robot_specs(self, new_specs: Dict[str, Any]) -> Dict[str, Any]:
        """Met à jour les dimensions du robot à la volée, sauvegarde dans robot_specs.json et synchronise l'IA."""
        with self._specs_lock:
            combined = dict(self.robot_specs)
            for k, v in new_specs.items():
                if v is not None and k in DEFAULT_ROBOT_SPECS:
                    if isinstance(DEFAULT_ROBOT_SPECS[k], bool):
                        combined[k] = bool(v)
                    elif isinstance(DEFAULT_ROBOT_SPECS[k], int) and not isinstance(DEFAULT_ROBOT_SPECS[k], bool):
                        combined[k] = int(round(float(v)))
                    elif isinstance(DEFAULT_ROBOT_SPECS[k], float):
                        combined[k] = round(float(v), 2)
                    else:
                        combined[k] = v
            self.robot_specs = self._normalize_specs_dict(combined)
            self.seed_spacing_cm = max(10, min(20, int(round(float(self.robot_specs.get("seed_spacing_cm", 15))))))
            self.seeding_use_tines = bool(self.robot_specs.get("seed_with_tines_down", True))
            self._save_robot_specs(self.robot_specs)
            if hasattr(self, "vision_brain") and self.vision_brain:
                self.vision_brain.set_robot_specs(self.robot_specs)
            if hasattr(self, "nav_engine") and self.nav_engine:
                self.nav_engine.update_specs(self.robot_specs)
            return dict(self.robot_specs)

    def reset_robot_specs(self) -> Dict[str, Any]:
        """Réinitialise toutes les dimensions aux vraies mesures du relevé terrain (DEFAULT_ROBOT_SPECS)."""
        with self._specs_lock:
            self.robot_specs = self._normalize_specs_dict(DEFAULT_ROBOT_SPECS)
            self.seed_spacing_cm = int(round(float(self.robot_specs["seed_spacing_cm"])))
            self.seeding_use_tines = bool(self.robot_specs["seed_with_tines_down"])
            self._save_robot_specs(self.robot_specs)
            if hasattr(self, "vision_brain") and self.vision_brain:
                self.vision_brain.set_robot_specs(self.robot_specs)
            if hasattr(self, "nav_engine") and self.nav_engine:
                self.nav_engine.update_specs(self.robot_specs)
            return dict(self.robot_specs)

    def sync_dynamic_config(self, dyn_specs: Dict[str, Any]) -> Dict[str, Any]:
        """Synchronise `RobotController`, `SymmetricTrapezoidPlanner` et `KinematicEngine` avec `robot_config`."""
        cm_updates: Dict[str, Any] = {}
        if "chassis_width_m" in dyn_specs:
            cm_updates["total_width_with_wheels_cm"] = round(float(dyn_specs["chassis_width_m"]) * 100.0, 1)
            cm_updates["chassis_width_cm"] = cm_updates["total_width_with_wheels_cm"]
        if "tool_spacing_m" in dyn_specs:
            cm_updates["seeder_row_spacing_cm"] = round(float(dyn_specs["tool_spacing_m"]) * 100.0, 1)
        if "camera_height_m" in dyn_specs:
            cm_updates["camera_height_cm"] = round(float(dyn_specs["camera_height_m"]) * 100.0, 1)
        if "camera_pitch_deg" in dyn_specs:
            cm_updates["camera_tilt_deg"] = round(float(dyn_specs["camera_pitch_deg"]), 1)
        if cm_updates:
            self.update_robot_specs(cm_updates)

        if hasattr(self, "symmetric_planner") and self.symmetric_planner:
            self.symmetric_planner.robot_width_m = float(dyn_specs.get("chassis_width_m", 1.21))
            self.symmetric_planner.robot_length_m = float(dyn_specs.get("chassis_length_m", 2.15))
            self.symmetric_planner.front_overhang_m = float(dyn_specs.get("nose_offset_m", 0.85))
            self.symmetric_planner.rear_overhang_m = float(dyn_specs.get("rear_teeth_offset_m", 1.15))
            self.symmetric_planner.safety_margin_m = float(dyn_specs.get("lateral_safety_margin_m", 0.38))

        if hasattr(self, "kinematic_engine") and self.kinematic_engine:
            self.kinematic_engine._on_config_updated(dyn_specs)

        return self.get_robot_specs()

    def set_seeding_config(
        self,
        spacing_cm: Optional[int] = None,
        auto_seed: Optional[bool] = None,
        use_tines: Optional[bool] = None,
        field_length_m: Optional[float] = None,
        field_width_m: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Configure l'espacement entre 2 pieds d'arachide (cm), les dents M5 et les dimensions Longueur × Largeur (m)."""
        spec_updates: Dict[str, Any] = {}
        if spacing_cm is not None:
            self.seed_spacing_cm = max(10, min(20, int(spacing_cm)))
            spec_updates["seed_spacing_cm"] = float(self.seed_spacing_cm)
        if auto_seed is not None:
            self.auto_seed_enabled = bool(auto_seed)
            if not self.auto_seed_enabled:
                self._distance_since_last_seed_cm = 0.0
                self.stop_motor(8)
        if use_tines is not None:
            self.seeding_use_tines = bool(use_tines)
            spec_updates["seed_with_tines_down"] = self.seeding_use_tines
        if field_length_m is not None:
            self.field_length_m = round(max(1.0, min(1000.0, float(field_length_m))), 2)
        if field_width_m is not None:
            self.field_width_m = round(max(0.46, min(1000.0, float(field_width_m))), 2)
        row_spacing_m = max(0.20, float(self.robot_specs.get("seeder_row_spacing_cm", 46.0)) / 100.0)
        self.corridors_total = max(1, int(round(self.field_width_m / row_spacing_m)))
        if spec_updates:
            self.update_robot_specs(spec_updates)
        return {
            "seed_spacing_cm": self.seed_spacing_cm,
            "auto_seed_enabled": self.auto_seed_enabled,
            "seeding_use_tines": self.seeding_use_tines,
            "field_length_m": self.field_length_m,
            "field_width_m": self.field_width_m,
            "corridors_total": self.corridors_total,
        }

    def get_vision_state(self) -> Dict[str, Any]:
        """Retourne l'état courant du Cerveau de Vision IA (lignes de rangs d'arachide, couloir bleu, boîtes)."""
        v = dict(self.vision_brain.last_analysis)
        v.pop("hud_frame_b64", None)
        return v

    def get_agronomic_metrics(self) -> Dict[str, Any]:
        """Retourne le dictionnaire des métriques agronomiques, de parcelle d'arachide et de vision IA."""
        v_state = self.get_vision_state()
        row_spacing_m = max(0.20, float(self.robot_specs.get("seeder_row_spacing_cm", 46.0)) / 100.0)
        self.corridors_total = max(1, int(round(self.field_width_m / row_spacing_m)))
        return {
            "crop_type": "ARACHIDE",
            "speed_kmh": round(self.speed_kmh, 2),
            "pitch": round(self.terrain_pitch, 1),
            "roll": round(self.terrain_roll, 1),
            "weeds_destroyed": self.weeds_destroyed,
            "weeds_total": self.weeds_total,
            "seeds_sown": self.seeds_sown,
            "seed_spacing_cm": self.seed_spacing_cm,
            "auto_seed_enabled": self.auto_seed_enabled,
            "seeding_use_tines": self.seeding_use_tines,
            "field_length_m": self.field_length_m,
            "field_width_m": self.field_width_m,
            "corridors_total": self.corridors_total,
            "current_corridor": self.current_corridor,
            "distance_traveled_m": round(self.distance_traveled_m, 2),
            "tines_depth": round(self.tines_depth * 100),
            "solar_deploy": round(self.solar_angle * 100),
            "obstacle": self.obstacle_detected,
            "distance": round(self.obstacle_distance, 1),
            "obstacle_state": self.obstacle_state,
            "obstacle_is_moving": self.obstacle_is_moving,
            "obstacle_speed_mps": self.obstacle_speed_mps,
            "avoidance_memory": self.avoidance_memory,
            "avoidance_path_points": self.avoidance_path_points,
            "crop_rows_x": self.crop_rows_x,
            "last_turn_decision": self.last_turn_decision,
            "weeding_state": self.weeding_state,
            "vision_angle_deg": v_state.get("angular_error_deg", 0.0),
            "vision_offset_cm": v_state.get("lateral_offset_cm", 0.0),
            "vision_action": v_state.get("recommended_action", "TRACK_PEANUT_CORRIDOR"),
            "vision_locked": bool(self.weeding_state == "SYNCHRONOUS_ROW_WEEDING" or v_state.get("is_locked", False)),
            "vision": v_state,
            "progression_direction": self.progression_direction,
            "reseeded_seeds_count": self.reseeded_seeds_count,
            "dead_plants_count": self.dead_plants_count,
            "gaps_detected_count": self.gaps_detected_count,
            "reviewed_rows_count": self.reviewed_rows_count,
            "seed_depth_cm": round(float(self.robot_specs.get("seed_depth_cm", 5.0)), 1),
            "sahel_intra_row_preset_cm": 15.0,
            "intra_row_spacing_cm_range": [10.0, 20.0],
            "sahel_theoretical_inter_row_cm_range": [30.0, 40.0],
            "mechanical_nozzle_spacing_cm": 46.0,
            "seed_map_count": len(self.seed_map.seeds) if hasattr(self, "seed_map") else 0,
            "ekf_localization": self.ekf_localizer.get_state() if hasattr(self, "ekf_localizer") else {},
            "solar_bms": self.solar_bms.get_status() if hasattr(self, "solar_bms") else {},
            "robot_x": round(self.robot_x, 3),
            "robot_z": round(self.robot_z, 3),
            "robot_yaw_deg": round(math.degrees(self.robot_yaw), 1),
        }

    def get_mission_state(self) -> Dict[str, Any]:
        """Retourne l'état détaillé de la mission autonome en cours."""
        mission_active = bool(self.mission_running)
        mission_paused = bool(self.mission_paused)
        m_type = (
            self._normalize_mission_type(self.current_mission_name)
            if (mission_active or mission_paused)
            else self._last_mission_type
        )
        if mission_paused:
            m_status = "PAUSED"
        elif mission_active:
            if self.obstacle_state == "WAITING_MOVING_OBSTACLE":
                m_status = "WAITING_MOVING_OBSTACLE"
            elif self.bypass_plan_locked or self.obstacle_state in ("BYPASS_REQUIRED", "BYPASSING"):
                m_status = "BYPASSING"
            else:
                m_status = "RUNNING"
        else:
            m_status = self._last_mission_status

        if self._mission_started_ts is not None and (mission_active or mission_paused):
            elapsed_s = round(max(0.0, time.time() - self._mission_started_ts), 1)
        elif self._mission_started_ts is not None and self._mission_ended_ts is not None:
            elapsed_s = round(max(0.0, self._mission_ended_ts - self._mission_started_ts), 1)
        else:
            elapsed_s = 0.0

        return {
            "name": self.current_mission_name,
            "mission_active": mission_active,
            "mission_paused": mission_paused,
            "mission_status": m_status,
            "mission_type": m_type,
            "mission_label": self._mission_label_for(m_type),
            "current_action": self.mission_step_label,
            "phase_label": self.mission_step_label,
            "elapsed_s": elapsed_s,
            "distance_m": round(float(self.distance_traveled_m), 2),
            "progress": self.mission_progress,
            "step_label": self.mission_step_label,
            "paused": self.mission_paused,
            "armed": self.armed,
            "is_armed": self.is_armed,
            "estop": self.estop_active,
            "mission_running": self.mission_running,
            "state": self.state,
            "weeding_state": self.weeding_state,
            "progression_direction": self.progression_direction,
            "direction_mode": self.direction_mode,
            "locked_side": self.locked_side,
            "reseeded_seeds_count": self.reseeded_seeds_count,
            "dead_plants_count": self.dead_plants_count,
            "gaps_detected_count": self.gaps_detected_count,
            "reviewed_rows_count": self.reviewed_rows_count,
            "robot_x": round(self.robot_x, 3),
            "robot_z": round(self.robot_z, 3),
            "robot_yaw_deg": round(math.degrees(self.robot_yaw), 1),
            "obstacle_state": self.obstacle_state,
            "obstacle_is_moving": self.obstacle_is_moving,
            "avoidance_memory": self.avoidance_memory,
            "field_length_m": self.field_length_m,
            "field_width_m": self.field_width_m,
            "corridors_total": self.corridors_total,
            "current_corridor": self.current_corridor,
            "last_mission_alert": self.last_mission_alert,
            "done_history": self.done_history[-20:],
        }

    def get_mission_snapshot(self) -> Dict[str, Any]:
        """Snapshot complet de synchronisation multi-appareils & reprise après actualisation (F5)."""
        ms = self.get_mission_state()
        cam_connected = False
        cam_synth = False
        cam_dev = "/dev/video0"
        cam_pitch = float(robot_config.camera_pitch_deg)
        cam_height = float(robot_config.camera_height_m)
        cam_auto_pitch = bool(robot_config.auto_pitch)
        try:
            from backend.vision.camera_manager import camera_manager as _cm
            cam_connected = bool(_cm.connected)
            cam_synth = bool(_cm.is_synthetic)
            cam_dev = str(_cm.device_path or f"/dev/video{_cm.device_index}")
            cam_pitch = float(_cm.camera_tilt_deg)
            cam_height = float(_cm.camera_height_m)
            cam_auto_pitch = bool(_cm.auto_pitch)
        except Exception:
            pass

        hw_port = (
            self.hardware_bridge.serial_port
            if self.hardware_bridge.transport == ConnectionTransport.USB_UART.value
            else f"{self.hardware_bridge.host}:{self.hardware_bridge.port}"
        )

        return {
            "mission_active": ms["mission_active"],
            "mission_paused": ms["mission_paused"],
            "mission_status": ms["mission_status"],
            "mission_type": ms["mission_type"],
            "mission_name": ms["name"],
            "mission_label": ms["mission_label"],
            "current_action": ms["current_action"],
            "phase_label": ms["phase_label"],
            "progress": ms["progress"],
            "row_index": int(self.current_corridor),
            "total_rows_est": int(self.corridors_total),
            "distance_m": ms["distance_m"],
            "elapsed_s": ms["elapsed_s"],
            "parcel_length_m": round(float(self.field_length_m), 2),
            "parcel_width_m": round(float(self.field_width_m), 2),
            "boundary_mode": getattr(self.kinematic_engine, "boundary_mode", "FIXED_DIMENSIONS"),
            "progression_direction": int(self.progression_direction),
            "direction_mode": self.direction_mode,
            "locked_side": self.locked_side,
            "hardware_connected": bool(self.hardware_bridge.connected),
            "hardware_port": hw_port,
            "hardware_transport": self.hardware_bridge.transport,
            "armed": int(self.armed),
            "camera_connected": cam_connected,
            "camera_is_synthetic": cam_synth,
            "camera_device": cam_dev,
            "camera_pitch_deg": round(cam_pitch, 1),
            "camera_height_m": round(cam_height, 2),
            "auto_pitch": cam_auto_pitch,
            "last_mission_alert": self.last_mission_alert,
            "recent_logs": self.get_recent_logs(30),
        }

    # --------------------------------------------------------------------------
    # Fil de lecture permanent & Tri des trames
    # --------------------------------------------------------------------------
    def _reader_loop(self) -> None:
        """Lit continuellement le flux et trie les réponses, la télémétrie #T et les événements #D."""
        while self._running:
            try:
                raw_chunk = self.device.readline(timeout=0.05) if hasattr(self.device, "readline") else b""
                if not raw_chunk:
                    continue
                line_str = raw_chunk.decode("ascii", errors="replace").strip("\r\n")
                if not line_str:
                    continue

                self._dispatch_incoming_line(line_str)
            except Exception as e:
                if self._running:
                    logger.debug("Exception lecture série : %s", e)
                time.sleep(0.01)

    def _trigger_seed_pulse_nonblocking(self) -> None:
        """Déclenche une impulsion de semence PULSE:8:75:350 sans bloquer le thread de lecture."""
        if self.is_simulation_locked():
            return
        try:
            cmd = "PULSE:8:75:350"
            if self.execution_mode in (ExecutionMode.COUPLED.value, ExecutionMode.REAL_ZERO_SIM.value) and self.hardware_bridge.connected:
                threading.Thread(
                    target=lambda: self.send_command(cmd, timeout=0.8),
                    daemon=True,
                ).start()
                return
            crc = calculate_xor_checksum(cmd)
            wire = f"{cmd}*{crc}\r\n"
            if self.device:
                self.device.write(wire.encode("ascii"))
                self._emit_log("TX", f"{cmd}*{crc}")
        except Exception as e:
            logger.debug("Erreur impulsion semence auto-cadencée : %s", e)

    def _update_fallback_agronomic_simulation(self, m_speeds: List[int]) -> None:
        """Calcule la distance parcourue Δd (en cm) et déclenche la semence auto-cadencée tous les X cm."""
        now = time.perf_counter()
        dt = max(0.01, min(0.5, now - self._last_telem_calc_time))
        self._last_telem_calc_time = now

        has_3d_client = (now - self._last_3d_feedback_time) < 2.0

        if len(m_speeds) >= 8:
            avg_wheel = (m_speeds[0] + m_speeds[1] + m_speeds[2] + m_speeds[3]) / 4.0
            # Circonférence réelle de la roue (C = pi * D = 83.25 cm pour D = 26.5 cm)
            tire_diam_cm = float(self.robot_specs.get("tire_diameter_cm", 26.5))
            circumference_cm = math.pi * max(5.0, tire_diam_cm)
            scale_factor = circumference_cm / 83.25
            calc_speed_kmh = round(abs(avg_wheel) * 0.036 * 2.2 * scale_factor, 2)
            if not has_3d_client:
                self.speed_kmh = calc_speed_kmh

            # Distance parcourue Δd uniquement quand le robot avance/recule réellement (v > 0)
            active_speed_kmh = self.speed_kmh if self.speed_kmh > 0.05 else calc_speed_kmh
            if abs(avg_wheel) > 2 and active_speed_kmh > 0.05:
                delta_m = (active_speed_kmh / 3.6) * dt
                self.distance_traveled_m += delta_m
                delta_cm = delta_m * 100.0
                nozzles = max(1, int(self.robot_specs.get("seeder_nozzles_count", 2)))

                # Semence auto-cadencée tous les X cm sur les 2 rangs (si auto_seed_enabled ou M8 actif)
                if self.auto_seed_enabled:
                    self._distance_since_last_seed_cm += delta_cm
                    if self._distance_since_last_seed_cm >= self.seed_spacing_cm:
                        while self._distance_since_last_seed_cm >= self.seed_spacing_cm:
                            self._distance_since_last_seed_cm -= self.seed_spacing_cm
                            if not has_3d_client:
                                self.seeds_sown += nozzles
                            if hasattr(self, "seed_map"):
                                self.seed_map.record_nozzle_pass(
                                    robot_x=self.robot_x,
                                    robot_z=self.robot_z,
                                    robot_yaw_rad=self.robot_yaw,
                                    seeder_row_spacing_m=float(self.robot_specs.get("seeder_row_spacing_cm", 46.0)) / 100.0,
                                    depth_cm=float(self.robot_specs.get("seed_depth_cm", 5.0)),
                                    spacing_cm=float(self.seed_spacing_cm),
                                )
                        if m_speeds[7] == 0:
                            self._trigger_seed_pulse_nonblocking()
                elif m_speeds[7] > 10 and not has_3d_client:
                    self._distance_since_last_seed_cm += delta_cm
                    if self._distance_since_last_seed_cm >= self.seed_spacing_cm:
                        self._distance_since_last_seed_cm -= self.seed_spacing_cm
                        self.seeds_sown += nozzles
                        if hasattr(self, "seed_map"):
                            self.seed_map.record_nozzle_pass(
                                robot_x=self.robot_x,
                                robot_z=self.robot_z,
                                robot_yaw_rad=self.robot_yaw,
                                seeder_row_spacing_m=float(self.robot_specs.get("seeder_row_spacing_cm", 46.0)) / 100.0,
                                depth_cm=float(self.robot_specs.get("seed_depth_cm", 5.0)),
                                spacing_cm=float(self.seed_spacing_cm),
                            )

            if hasattr(self, "solar_bms") and self.solar_bms:
                self.solar_bms.step(dt=dt, motor_pwms=m_speeds, solar_deploy_ratio=self.solar_angle)

            if not has_3d_client:
                # Profondeur des dents M5
                if m_speeds[4] > 5:
                    self.tines_depth = min(1.0, self.tines_depth + 0.25 * dt)
                elif m_speeds[4] < -5:
                    self.tines_depth = max(0.0, self.tines_depth - 0.25 * dt)

                # Panneaux solaires M7
                if m_speeds[6] > 10:
                    self.solar_angle = min(1.0, self.solar_angle + 0.033 * dt)
                elif m_speeds[6] < -10:
                    self.solar_angle = max(0.0, self.solar_angle - 0.033 * dt)

                # Désherbage M5 + M6
                if self.tines_depth > 0.4 and abs(m_speeds[5]) > 15 and self.speed_kmh > 0.1:
                    if int(now * 2) != int((now - dt) * 2) and self.weeds_destroyed < self.weeds_total:
                        self.weeds_destroyed += 1

    def _dispatch_incoming_line(self, line: str) -> None:
        # 1. Télémétrie périodique : #T <armé> <m1>,...,<m8> <ms>
        # (Jamais envoyée dans la console !)
        if line.startswith("#T "):
            if self.is_simulation_locked():
                self._force_stop_simulator(disarm=False)
                m_speeds = [0] * 8
                armed_val = 0
                ts_val = int(time.time() * 1000)
            else:
                parts = line[3:].strip().split()
                if len(parts) < 3:
                    return
                try:
                    armed_val = int(parts[0])
                    m_speeds = [int(v) for v in parts[1].split(",")]
                    ts_val = int(parts[2])
                except Exception as e:
                    logger.warning("Format télémétrie invalide (%s): %s", line, e)
                    return

            self.armed = armed_val
            self._update_fallback_agronomic_simulation(m_speeds)
            self.last_telemetry = {
                "armed": armed_val,
                "motors": m_speeds,
                "timestamp": ts_val,
                "raw": line,
                "agronomics": self.get_agronomic_metrics(),
                "mission_state": self.get_mission_state(),
                "hardware": self.get_hardware_status(),
            }
            for cb in self.telemetry_callbacks:
                try:
                    cb(self.last_telemetry)
                except Exception:
                    pass
            return

        # En mode matériel (COUPLED ou REAL_ZERO_SIM), les logs RX et les réponses aux commandes proviennent du robot physique (hardware_bridge)
        if self.execution_mode in (ExecutionMode.COUPLED.value, ExecutionMode.REAL_ZERO_SIM.value):
            if line.startswith("#D "):
                try:
                    n = int(line[3:].strip())
                    if n in self.done_events:
                        self.done_events[n].set()
                except ValueError:
                    pass
                if self.hardware_bridge.transport == ConnectionTransport.WIFI_HTTP.value:
                    self._emit_log("RX", line)
            return

        # Notification Log RX (filtrée : JAMAIS de PING, PONG ou #T dans la console)
        self._emit_log("RX", line)

        # 2. Événement de fin d'action : #D n
        if line.startswith("#D "):
            try:
                n = int(line[3:].strip())
                if n in self.done_events:
                    self.done_events[n].set()
                done_entry = {
                    "motor": n,
                    "raw": f"#D {n}",
                    "ts": int(time.time() * 1000),
                }
                self.done_history.append(done_entry)
                if len(self.done_history) > 50:
                    self.done_history = self.done_history[-50:]
                for cb in self.event_callbacks:
                    try:
                        cb("DONE", done_entry)
                    except Exception:
                        pass
                logger.info("Fin d'action temporisée reçue : #D %d", n)
            except ValueError:
                logger.warning("Événement #D invalide : %s", line)
            return

        # 3. Réponses aux commandes (OK ..., ERR: ..., PONG, STATUS, etc.)
        if line.startswith("OK ARMED"):
            self.armed = 1
            self.is_armed = True
            self.estop_active = False
            self.state = "IDLE"
            self._mission_stop_flag.clear()
        elif line.startswith("OK ESTOP"):
            self.armed = 0
            self.is_armed = False
            self.estop_active = True
            self.mission_running = False
            self.state = "ESTOP"
            self.speed_kmh = 0.0
            self._mission_stop_flag.set()

        # Réveille l'émetteur de commande bloquant
        self._pending_response_queue.put(line)

    # --------------------------------------------------------------------------
    # Heartbeat silencieux (envoie PING toutes les 300 ms pour maintenir UWDT)
    # --------------------------------------------------------------------------
    def _heartbeat_loop(self) -> None:
        while self._running:
            time.sleep(0.3)  # 300 ms
            if not self._running or not self.heartbeat_enabled:
                continue
            try:
                if self.is_simulation_locked():
                    self._force_stop_simulator(disarm=False)
                # Envoi silencieux du PING au simulateur sans JAMAIS polluer les log_callbacks
                ping_str = "PING*10\r\n"
                if self.device:
                    self.device.write(ping_str.encode("ascii"))
            except Exception as e:
                logger.debug("Erreur Heartbeat PING : %s", e)

    # --------------------------------------------------------------------------
    # Envoi de commandes (Simulation Uniquement vs Couplé au Robot Physique ESP32-S3)
    # --------------------------------------------------------------------------
    def send_command(
        self,
        command: str,
        timeout: float = 1.0,
        with_checksum: Optional[bool] = None,
    ) -> str:
        """Envoie une commande au robot et attend la réponse correspondante."""
        clean_cmd = command.strip()
        if not clean_cmd:
            return ""

        clean_upper = clean_cmd.upper().split("*")[0].strip()
        base_cmd = clean_upper.split(":")[0].strip()

        # Priorité Absolue ESTOP : interruption immédiate frontend & backend
        if base_cmd == "ESTOP":
            self.armed = 0
            self.is_armed = False
            self.estop_active = True
            self.mission_running = False
            self.state = "ESTOP"
            self.speed_kmh = 0.0
            self._mission_stop_flag.set()
            for ev in self.done_events.values():
                ev.set()
            if self.execution_mode in (ExecutionMode.COUPLED.value, ExecutionMode.REAL_ZERO_SIM.value) and self.hardware_bridge.connected:
                self.hardware_bridge.send_command("ESTOP", timeout=timeout, with_checksum=self.use_xor_checksum, silent=False)
            self._force_stop_simulator(disarm=True)
            self._emit_log("TX", clean_cmd)
            self._emit_log("RX", "OK ESTOP")
            return "OK ESTOP"

        # Priorité Haute STOP global : coupure immédiate de la vitesse et envoi direct ESP32
        if clean_upper == "STOP":
            self.speed_kmh = 0.0
            for ev in self.done_events.values():
                ev.set()
            if self.execution_mode in (ExecutionMode.COUPLED.value, ExecutionMode.REAL_ZERO_SIM.value) and self.hardware_bridge.connected:
                res_stop = self.hardware_bridge.send_command(
                    "STOP", timeout=timeout, with_checksum=self.use_xor_checksum, silent=False
                )
                self._force_stop_simulator(disarm=False)
                return res_stop

        use_crc = self.use_xor_checksum if with_checksum is None else bool(with_checksum)
        wire_cmd = format_command_with_optional_crc(clean_cmd, use_xor_checksum=use_crc)

        # ----------------------------------------------------------------------
        # MODE B : SIMULATION COUPLÉE (COUPLED) — Nécessite l'ESP32 connecté
        # ----------------------------------------------------------------------
        if self.execution_mode == ExecutionMode.COUPLED.value and not self.hardware_bridge.connected:
            self._force_stop_simulator(disarm=False)
            self._emit_log("TX", wire_cmd)
            err_msg = "ERR: NOT_CONNECTED (En attente de connexion au robot physique ESP32-S3)"
            self._emit_log("RX", err_msg)
            return "ERR: NOT_CONNECTED"

        # ----------------------------------------------------------------------
        # MODE C SANS ESP32 : MODE TEST VISION IA SEULE (Dry-Run Hardware)
        # ----------------------------------------------------------------------
        is_vision_dry_run = (
            self.execution_mode == ExecutionMode.REAL_ZERO_SIM.value
            and not self.hardware_bridge.connected
        )
        if is_vision_dry_run and base_cmd == "ARM":
            self.armed = 1
            self.is_armed = True
            self.estop_active = False
            self.state = "IDLE"
            self._mission_stop_flag.clear()
            self._mirror_command_to_simulator("ARM")
            self._emit_log("TX", wire_cmd)
            self._emit_log("RX", "[VISION IA -> ORDRE ROBOT] ARM | Robot prêt en Mode Test Vision (ESP32 non requis)")
            return "OK ARMED"

        # Verrouillage dur : Si non armé ou ESTOP actif, tout mouvement est immédiatement rejeté
        if self.estop_active or not self.is_armed or self.armed == 0:
            if base_cmd in ("DRIVE", "TANK", "PULSE", "M1", "M2", "M3", "M4", "M5", "M6", "M7", "M8"):
                self._emit_log("TX", clean_cmd)
                self._emit_log("RX", "ERR: ESTOP")
                return "ERR: ESTOP"

        if is_vision_dry_run:
            m8_active = bool(self.simulator and self.simulator.motors[7].target_speed > 0) or base_cmd == "M8" or clean_upper.startswith("PULSE:8:")
            m6_active = bool(self.simulator and self.simulator.motors[5].target_speed > 0) or (base_cmd == "M6" and not clean_upper.endswith(":0"))
            m5_label = "ABAISSÉES" if self.tines_depth > 0.3 or clean_upper.startswith("PULSE:5:50") else "RELEVÉES"
            tool_summary = f"Semoir M8: {'ACTIF' if m8_active else 'OFF'} | Dents M5: {m5_label} | Fraise M6: {'ON' if m6_active else 'OFF'}"
            self._emit_log("TX", wire_cmd)
            self._emit_log("RX", f"[VISION IA -> ORDRE ROBOT] {clean_cmd} | {tool_summary}")
            self._mirror_command_to_simulator(wire_cmd)
            # Débloquer immédiatement les done_events en mode Dry-Run Vision pour une réactivité temps réel
            for ev in self.done_events.values():
                ev.set()
            return f"OK {base_cmd}"

        if self.execution_mode in (ExecutionMode.COUPLED.value, ExecutionMode.REAL_ZERO_SIM.value) and self.hardware_bridge.connected:
            # Envoi temps réel au vrai robot ESP32-S3
            res = self.hardware_bridge.send_command(
                clean_cmd,
                timeout=timeout,
                with_checksum=use_crc,
                silent=is_console_noise_line(wire_cmd),
            )
            res_upper = res.strip().upper()

            if res_upper.startswith("OK ARMED") or (base_cmd == "ARM" and not res_upper.startswith("ERR:")):
                self.armed = 1
                self.is_armed = True
                self.estop_active = False
                self.state = "IDLE"
                self._mission_stop_flag.clear()
                self._mirror_command_to_simulator("ARM")
            elif res_upper.startswith("OK ESTOP"):
                self.armed = 0
                self.is_armed = False
                self.estop_active = True
                self.mission_running = False
                self.state = "ESTOP"
                self.speed_kmh = 0.0
                self._mission_stop_flag.set()
                self._force_stop_simulator(disarm=True)
                self._mirror_command_to_simulator(wire_cmd)
            elif res_upper.replace(" ", "").startswith("ERR:ESTOP"):
                # Le robot physique a refusé la commande (non armé) : le simulateur 3D ne bouge pas !
                self.armed = 0
                self.is_armed = False
                self.estop_active = True
                self._force_stop_simulator(disarm=True)
            elif res_upper.startswith("ERR:"):
                # Toute erreur matérielle empêche l'action sur le jumeau 3D
                if "DISCONNECTED" in res_upper or "TIMEOUT" in res_upper:
                    self._force_stop_simulator(disarm=False)
            else:
                # Le robot physique a répondu OK ... (ou STATUS/GCFG/CFG/PERF/VER/PONG) :
                # le jumeau 3D exécute exactement la même commande en parallèle !
                self._mirror_command_to_simulator(wire_cmd)

            return res

        # ----------------------------------------------------------------------
        # MODE A : SIMULATION UNIQUEMENT (100 % Virtuel)
        # ----------------------------------------------------------------------
        with self._cmd_lock:
            # Purge les réponses précédentes
            while not self._pending_response_queue.empty():
                try:
                    self._pending_response_queue.get_nowait()
                except queue.Empty:
                    break

            # Envoi sur le simulateur / périphérique
            payload = (wire_cmd + "\r\n").encode("ascii")
            if self.device:
                self.device.write(payload)
                self._emit_log("TX", wire_cmd)

            # Attente de la réponse (OK, ERR, PONG, etc.)
            deadline = time.perf_counter() + timeout
            while time.perf_counter() < deadline:
                remaining = max(0.01, deadline - time.perf_counter())
                try:
                    res = self._pending_response_queue.get(timeout=remaining)
                    # Ignore les PONG spontanés issus du heartbeat s'il s'agissait d'une autre commande
                    if clean_cmd.upper().split("*")[0] != "PING" and res == "PONG":
                        continue
                    if res.startswith("OK ARMED"):
                        self.armed = 1
                        self.is_armed = True
                        self.estop_active = False
                        self.state = "IDLE"
                        self._mission_stop_flag.clear()
                    elif res.startswith("OK ESTOP"):
                        self.armed = 0
                        self.is_armed = False
                        self.estop_active = True
                        self.mission_running = False
                        self.state = "ESTOP"
                        self.speed_kmh = 0.0
                        self._mission_stop_flag.set()
                    return res
                except queue.Empty:
                    break

            logger.warning("Timeout sur la commande : %s", wire_cmd)
            return "ERR: TIMEOUT"

    # --------------------------------------------------------------------------
    # Commandes de base
    # --------------------------------------------------------------------------
    def arm(self) -> str:
        res = self.send_command("ARM")
        if "OK ARMED" in res or "ARMED" in res:
            self.armed = 1
            self.is_armed = True
            self.estop_active = False
            self.state = "IDLE"
            self._mission_stop_flag.clear()
        return res

    def estop(self) -> str:
        self.armed = 0
        self.is_armed = False
        self.estop_active = True
        self.mission_running = False
        self.state = "ESTOP"
        self.speed_kmh = 0.0
        self.mode = RobotMode.MANUAL
        self.abort_mission()
        res = self.send_command("ESTOP")
        self._force_stop_simulator(disarm=True)
        return res if res else "OK ESTOP"

    def stop(self) -> str:
        return self.send_command("STOP")

    def stop_motor(self, n: int) -> str:
        return self.send_command(f"STOP:{n}")

    def ping(self) -> str:
        return self.send_command("PING*10")

    def drive(
        self,
        a: int = 0,
        d: int = 0,
        ms: Optional[int] = None,
        speed: Optional[int] = None,
        steering: Optional[int] = None,
    ) -> str:
        if speed is not None:
            a = speed
        if steering is not None:
            d = steering
        if ms is not None:
            for i in (1, 2, 3, 4):
                self.done_events[i].clear()
            return self.send_command(f"DRIVE:{a},{d}:{ms}")
        return self.send_command(f"DRIVE:{a},{d}")

    def tank(self, l: int, r: int, ms: Optional[int] = None) -> str:
        if ms is not None:
            for i in (1, 2, 3, 4):
                self.done_events[i].clear()
            return self.send_command(f"TANK:{l},{r}:{ms}")
        return self.send_command(f"TANK:{l},{r}")

    def pulse(self, n: int, v: int, ms: int) -> str:
        if n in self.done_events:
            self.done_events[n].clear()
        return self.send_command(f"PULSE:{n}:{v}:{ms}")

    def motor(self, n: int, v: int, ms: Optional[int] = None) -> str:
        if ms is not None:
            if n in self.done_events:
                self.done_events[n].clear()
            return self.send_command(f"M{n}:{v}:{ms}")
        return self.send_command(f"M{n}:{v}")

    def enable_telemetry(self, ms: int = 100) -> str:
        return self.send_command(f"TELEM:{ms}")

    # --------------------------------------------------------------------------
    # Gestionnaire de Mode et Missions Autonomes
    # --------------------------------------------------------------------------
    def set_mode(self, mode: RobotMode) -> None:
        if self.mode == mode and not (mode == RobotMode.MANUAL and self.current_mission_name):
            return
        logger.info("Changement de mode : %s -> %s", self.mode.value, mode.value)
        if mode == RobotMode.MANUAL:
            # Passer de Auto à Manuel annule immédiatement la séquence et envoie STOP
            self.abort_mission()
            self.stop()
        self.mode = mode

    def _notify_mission_progress(self, progress: int, label: str) -> None:
        self.mission_progress = max(0, min(100, progress))
        self.mission_step_label = label
        for cb in self.event_callbacks:
            try:
                cb("MISSION_PROGRESS", self.get_mission_state())
            except Exception:
                pass

    def _emit_mission_alert(self, level: str, code: str, message: str, progress: Optional[int] = None) -> None:
        """Émet une alerte mission visible dans l'UI (bannière dédiée) et dans la console de commandes."""
        alert = {
            "level": level,  # "error", "warning", "success", "info"
            "code": code,
            "message": message,
            "ts": int(time.time() * 1000),
        }
        self.last_mission_alert = alert
        if progress is not None:
            self.mission_progress = max(0, min(100, int(progress)))
        self.mission_step_label = message
        if level in ("error", "warning"):
            logger.warning("[MISSION %s] %s", code, message)
        else:
            logger.info("[MISSION %s] %s", code, message)
        self._emit_log("RX", message)
        for cb in self.event_callbacks:
            try:
                cb("MISSION_ALERT", alert)
                cb("MISSION_PROGRESS", self.get_mission_state())
            except Exception:
                pass

    def _check_pause_or_stop(self) -> bool:
        """Vérifie si la mission est interrompue ou mise en pause. Retourne False si interrompue."""
        if self._mission_stop_flag.is_set() or self.estop_active or not self.is_armed or self.armed == 0:
            return False
        while not self._mission_pause_event.is_set():
            if self._mission_stop_flag.is_set() or self.estop_active or not self.is_armed or self.armed == 0:
                return False
            time.sleep(0.05)
        return not self._mission_stop_flag.is_set() and not self.estop_active and self.is_armed and self.armed == 1

    # --------------------------------------------------------------------------
    # Guidage en boucle fermée dans les couloirs d'arachide (pose 3D / caméra IA)
    # --------------------------------------------------------------------------
    GUIDE_PERIOD_S: float = 0.12   # Période de la boucle de guidage (≈ 8 Hz)
    GUIDE_CMD_MS: int = 500        # Durée de garde de chaque DRIVE (rafraîchi avant expiration → mouvement continu)

    def _has_live_pose(self) -> bool:
        """True si la scène 3D (navigateur) renvoie la pose du robot en temps réel (hors mode Zéro Simulation)."""
        if self.execution_mode == ExecutionMode.REAL_ZERO_SIM.value:
            return False
        return (time.perf_counter() - self._last_3d_feedback_time) < 1.0

    def _corridor_spacing_units(self) -> float:
        return max(0.05, float(self.robot_specs.get("seeder_row_spacing_cm", 46.0)) * SIM_CM_TO_3D)

    def find_nearest_peanut_row(self, x: float, crop_rows_x: Optional[List[float]] = None) -> float:
        """Retourne l'abscisse X exacte de la vraie ligne d'arachide détectée la plus proche."""
        rows = crop_rows_x if (crop_rows_x and len(crop_rows_x) > 0) else self.crop_rows_x
        if not rows:
            spacing_u = self._corridor_spacing_units()
            return round(x / spacing_u) * spacing_u
        return float(min(rows, key=lambda rx: abs(float(rx) - x)))

    def select_next_peanut_row(
        self,
        current_x: float,
        crop_rows_x: Optional[List[float]] = None,
        direction_sign: int = 1,
    ) -> Dict[str, Any]:
        """Sélectionne dynamiquement la prochaine vraie ligne d'arachide en bout de rang :
        - Seuil = 2 × grosse dent arrière (2 × 0.25 m = 0.50 m = 50 cm).
        - Si E <= 0.50 m : saute 1 ligne -> rang i + 2.
        - Si E >  0.50 m : ne saute pas de ligne -> rang i + 1.
        """
        rows = crop_rows_x if (crop_rows_x and len(crop_rows_x) > 0) else self.crop_rows_x
        res = self.nav_engine.select_next_peanut_row(current_x, rows, direction_sign=direction_sign)
        self.last_turn_decision = res.get("hud_message", "")
        return res

    @staticmethod
    def _wrap_angle(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    def _verify_initial_line_presence(
        self,
        mission_type: str = "SEEDING",
        sample_frames: int = 8,
        frame_delay_s: float = 0.14,
    ) -> bool:
        """Vérification obligatoire avant le premier DRIVE en mode REAL_ZERO_SIM :
        1. Si un vrai ESP32-S3 est connecté mais que la caméra est déconnectée ou synthétique -> bloque le mouvement.
        2. Échantillonne 5 à 10 frames (~1.0 à 1.5 s) pour confirmer qu'une ligne valide est visible.
           Si aucune ligne n'est présente : STOP immédiat, outils OFF, alerte UI/console explicite.
        """
        if self._has_live_pose() or self.execution_mode != ExecutionMode.REAL_ZERO_SIM.value:
            return True

        try:
            from backend.vision.camera_manager import camera_manager as _cm
        except Exception:
            return True

        # Garde-fou matériel : interdire tout mouvement physique ESP32 sur flux synthétique ou caméra déconnectée
        if self.hardware_bridge.connected and (not getattr(_cm, "connected", False) or getattr(_cm, "is_synthetic", False)):
            self.stop()
            self.stop_motor(6)
            self.stop_motor(8)
            if self.tines_depth > 0.05:
                self.pulse(5, -50, 4000)
                self.wait_for_done([5], timeout=5.0)
            self.state = "CAMERA_SAFETY_BLOCKED"
            self._emit_mission_alert(
                level="error",
                code="SYNTHETIC_CAMERA_BLOCKED",
                message="⚠️ Caméra USB réelle déconnectée ou en mode synthétique — Mouvement physique bloqué par sécurité",
                progress=0,
            )
            return False

        m_norm = str(mission_type or "SEEDING").strip().upper()
        try:
            _cm.pipeline.consecutive_no_line_frames = 0
            _cm.pipeline.consecutive_non_peanut_frames = 0
        except Exception:
            pass

        self._notify_mission_progress(6, "🔍 Vérification caméra : recherche d'une ligne valide avant départ...")
        total_samples = max(1, int(sample_frames))
        required_hits = 1 if total_samples <= 2 else 2
        valid_hits = 0

        for idx in range(total_samples):
            if not self._check_pause_or_stop():
                return False
            cam_telem = _cm.get_latest_telemetry()
            if m_norm == "SEEDING":
                line_ok = bool(cam_telem.get("guideline_detected", False))
            else:
                line_ok = bool(
                    cam_telem.get("row_detected", False)
                    and (cam_telem.get("is_valid_peanut_row") is not False)
                    and not cam_telem.get("hybrid_boundary_alert", False)
                )
            if line_ok:
                valid_hits += 1
                if valid_hits >= required_hits:
                    return True
            if idx < total_samples - 1 and frame_delay_s > 0:
                time.sleep(frame_delay_s)

        # Aucune ligne valide détectée -> arrêt complet, outils relevés/coupés, alerte explicite
        self.stop()
        self.stop_motor(6)
        self.stop_motor(8)
        if self.tines_depth > 0.05:
            self.pulse(5, -50, 4000)
            self.wait_for_done([5], timeout=5.0)
        self.state = "ABORTED_NO_LINE"
        self._emit_mission_alert(
            level="error",
            code="NO_LINE_DETECTED",
            message="⚠️ Aucune ligne détectée par le robot — Mission arrêtée",
            progress=0,
        )
        return False

    def _verify_peanut_row_present_after_turn(
        self,
        required_invalid_frames: int = 5,
        mission_type: str = "WEEDING",
    ) -> bool:
        """Après un demi-tour en mode REAL_ZERO_SIM, vérifie s'il existe une ligne suivante (5 frames).
        Si aucune ligne n'est présente sur 5 frames consécutives, arrête proprement la mission
        avec le message « ✅ Fin de parcelle — Aucune ligne suivante détectée ».
        """
        if self._has_live_pose() or self.execution_mode != ExecutionMode.REAL_ZERO_SIM.value:
            return True
        try:
            from backend.vision.camera_manager import camera_manager as _cm
        except Exception:
            return True

        m_norm = str(mission_type or "WEEDING").strip().upper()
        invalid_count = 0
        for _ in range(max(1, int(required_invalid_frames))):
            if not self._check_pause_or_stop():
                return False
            cam_telem = _cm.get_latest_telemetry()
            if m_norm == "SEEDING":
                is_missing = (cam_telem.get("guideline_detected") is False) or bool(cam_telem.get("line_end_confirmed"))
            else:
                is_missing = (
                    cam_telem.get("is_valid_peanut_row") is False
                    or bool(cam_telem.get("hybrid_boundary_alert"))
                    or (cam_telem.get("row_detected") is False)
                )
            if is_missing:
                invalid_count += 1
            else:
                return True
            time.sleep(0.05)

        if invalid_count >= required_invalid_frames:
            self.stop()
            self.stop_motor(6)
            self.stop_motor(8)
            self.state = "MISSION_COMPLETED_BY_HYBRID_VISION"
            msg = "✅ Fin de parcelle — Aucune ligne suivante détectée"
            self._emit_mission_alert(
                level="success",
                code="FIELD_END_NO_NEXT_ROW",
                message=msg,
                progress=100,
            )
            return False
        return True

    def _follow_corridor(
        self,
        corridor_center_x: float,
        heading_rad: float,
        length_units: float,
        label: str,
        pct_start: int,
        pct_end: int,
        weeding: bool = False,
        reseeding: bool = False,
        fallback_steps: int = 8,
        continuous_vision: bool = False,
    ) -> bool:
        """Avance dans le couloir en maintenant M5/M6 centrés (DRIVE rafraîchi en continu, sans à-coups)."""
        self._last_corridor_ended_by_vision = False
        if not self._has_live_pose():
            # Mode Zéro Simulation (100% Réel) piloté par la Caméra USB (78°) OpenCV OU fallback pas fixes
            use_real_cam = self.execution_mode == ExecutionMode.REAL_ZERO_SIM.value
            cam_mgr = None
            if use_real_cam:
                try:
                    from backend.vision.camera_manager import camera_manager as _cm
                    cam_mgr = _cm
                except Exception:
                    cam_mgr = None

            cutter_running_real = weeding
            consec_missing_line = 0
            # En mission réelle continue (sans encodeurs roues), ne pas s'arrêter arbitrairement après 3 s :
            # calculer un plafond réaliste depuis la longueur de parcelle (≈ 0.16 m / pas de 400 ms) ou 300 pas en mode SMART_LINE_END.
            if continuous_vision and cam_mgr is not None:
                is_smart_end = (
                    getattr(self.kinematic_engine, "boundary_mode", "FIXED_DIMENSIONS") == "SMART_LINE_END"
                )
                est_steps = max(int(fallback_steps), int(round(max(1.0, float(self.field_length_m)) / 0.16)) + 20)
                max_steps = max(est_steps, 300) if is_smart_end else est_steps
            else:
                max_steps = max(1, int(fallback_steps))

            for step_idx in range(max_steps):
                if not self._check_pause_or_stop():
                    return False

                if cam_mgr is not None:
                    cam_telem = cam_mgr.get_latest_telemetry()
                    rec = cam_telem.get("recommended_drive") or {
                        "a": 22 if weeding else (20 if reseeding else 25),
                        "d": int(cam_telem.get("recommended_steer", 0)),
                    }
                    lat_cm = float(cam_telem.get("lateral_error_cm", 0.0))
                    ang_deg = float(cam_telem.get("heading_error_deg", 0.0))
                    self.vision_brain.last_lateral_offset_cm = lat_cm
                    self.vision_brain.last_angular_error_deg = ang_deg

                    # Arrêt d'observation si obstacle en mouvement (WAITING_MOVING_OBSTACLE)
                    if cam_telem.get("waiting_moving_obstacle") or cam_telem.get("obstacle_state") == "WAITING_MOVING_OBSTACLE":
                        rem_s = float(cam_telem.get("remaining_wait_sec", 5.0))
                        self.stop()
                        self.stop_motor(6)
                        self.obstacle_state = "WAITING_MOVING_OBSTACLE"
                        self.obstacle_is_moving = True
                        self._notify_mission_progress(
                            pct_start,
                            f"⏳ OBSTACLE EN MOUVEMENT — ATTENTE ({rem_s:.1f}s)...",
                        )
                        time.sleep(0.25)
                        continue

                    # Arrêt et contournement prioritaire si obstacle détecté par la Caméra USB (78°)
                    if cam_telem.get("obstacle_stop") or cam_telem.get("emergency_stop"):
                        dist_obs = float(cam_telem.get("min_distance_m", cam_telem.get("obstacle_dist_m", 1.2)))
                        bypass_side = int(cam_telem.get("recommended_bypass_side", 1)) or 1
                        side_txt = "GAUCHE" if bypass_side > 0 else "DROITE"
                        self.stop()
                        self._notify_mission_progress(
                            pct_start,
                            f"⚠️ OBSTACLE CAMÉRA ({dist_obs:.2f}m) — Relevage M5 & Contournement {side_txt}",
                        )
                        if weeding or self.seeding_use_tines:
                            self.stop_motor(6)
                            self.pulse(5, -50, 4000)
                            self.wait_for_done([5], timeout=5.0)
                        self.drive(16, 28 * bypass_side, ms=500)
                        self.wait_for_done([1, 2, 3, 4], timeout=1.2)
                        self.drive(16, -28 * bypass_side, ms=500)
                        self.wait_for_done([1, 2, 3, 4], timeout=1.2)
                        if weeding or self.seeding_use_tines:
                            self.pulse(5, 50, 4000)
                            self.wait_for_done([5], timeout=5.0)
                            if weeding:
                                self.motor(6, 70)

                    # Détection fin de ligne (5 frames consécutives sans ligne) pour TOUTES les missions autonomes
                    if weeding or reseeding:
                        is_line_missing = (
                            cam_telem.get("is_valid_peanut_row") is False
                            or bool(cam_telem.get("hybrid_boundary_alert"))
                            or (cam_telem.get("row_detected") is False)
                        )
                    else:
                        is_line_missing = (
                            (cam_telem.get("guideline_detected") is False)
                            or bool(cam_telem.get("line_end_confirmed"))
                        )

                    if is_line_missing:
                        consec_missing_line += 1
                    else:
                        consec_missing_line = 0

                    confirmed_line_end = bool(
                        consec_missing_line >= 5
                        or (weeding or reseeding) and cam_telem.get("hybrid_line_end_confirmed")
                        or cam_telem.get("line_end_confirmed")
                    )
                    if confirmed_line_end:
                        self.stop()
                        self.stop_motor(6)
                        self.stop_motor(8)
                        if weeding or self.seeding_use_tines:
                            self.pulse(5, -50, 4000)
                            self.wait_for_done([5], timeout=5.0)
                        self._last_corridor_ended_by_vision = True
                        alert_msg = cam_telem.get("hybrid_hud_badge") if (weeding or reseeding) else None
                        if not alert_msg:
                            alert_msg = "🛑 FIN DE LIGNE DÉTECTÉE (5 frames) -> Arrêt outils & Demi-tour"
                        logger.warning("[VISION FIN DE LIGNE] %s", alert_msg)
                        self._notify_mission_progress(pct_end, alert_msg)
                        break

                    if weeding and int(cam_telem.get("weeds_detected_count", 0)) > 0:
                        if not cutter_running_real:
                            self.motor(6, 70)
                            cutter_running_real = True
                        self.weeds_destroyed += 1

                    if reseeding and (cam_telem.get("reseed_required") or cam_telem.get("reseed_pulse_triggered")):
                        self.pulse(8, 60, 400)
                        gaps_n = max(1, int(cam_telem.get("reseed_gaps_count", cam_telem.get("gaps_count", 1))))
                        self.reseeded_seeds_count += gaps_n
                        self.gaps_detected_count += 1
                    elif not weeding and not reseeding and not is_line_missing:
                        self.seeds_sown += 2

                    a_val = int(rec.get("a", 25))
                    d_val = int(rec.get("d", 0))
                    if a_val == 0 and d_val == 0 and is_line_missing:
                        # Frame manquante isolée (1..4) : pause courte sans avancer à l'aveugle
                        self.stop()
                        time.sleep(0.06)
                    else:
                        self.drive(a_val, d_val, ms=400)
                        self.wait_for_done([1, 2, 3, 4], timeout=1.2)
                    pct = pct_start + int((step_idx + 1) / max_steps * (pct_end - pct_start))
                    hud_str = cam_telem.get("hud_message") or f"écart {lat_cm:+.1f}cm Δθ {ang_deg:+.1f}°"
                    self._notify_mission_progress(pct, f"{label} [CAM 78°] : {hud_str} | DRIVE:{a_val},{d_val}:400")
                else:
                    res = self.vision_brain.process_scene_telemetry(
                        robot_x=self.robot_x, robot_z=self.robot_z, robot_yaw_rad=self.robot_yaw,
                        corridor_center_x=corridor_center_x, target_heading_rad=heading_rad,
                    )
                    rec = res["recommended_drive"]
                    self.drive(int(rec["a"]), int(rec["d"]), ms=400)
                    self.wait_for_done([1, 2, 3, 4], timeout=1.2)
                    pct = pct_start + int((step_idx + 1) / max_steps * (pct_end - pct_start))
                    self._notify_mission_progress(pct, f"{label} : DRIVE:{rec['a']},{rec['d']}:400")
            return True

        z0 = self.robot_z
        start_t = time.perf_counter()
        timeout_s = 8.0 + length_units / 0.5  # marge large (vitesse mini ≈ 0.6 u/s)
        cutter_running = weeding
        last_notify = 0.0

        while True:
            if not self._check_pause_or_stop():
                self.stop()
                return False

            progress = (self.robot_z - z0) * math.cos(heading_rad)
            if progress >= length_units:
                break
            if (time.perf_counter() - start_t) > timeout_s:
                logger.warning("Couloir non terminé dans le temps imparti (%.1f/%.1f u)", progress, length_units)
                break

            res = self.vision_brain.process_scene_telemetry(
                robot_x=self.robot_x,
                robot_z=self.robot_z,
                robot_yaw_rad=self.robot_yaw,
                terrain_roll_deg=self.terrain_roll,
                terrain_pitch_deg=self.terrain_pitch,
                weeds_in_fov=[{"dist": 2.2, "angle": 0.0}] if (weeding and self.dense_weed_detected) else None,
                obstacles_in_fov=self._last_all_obstacles
                if (self._last_all_obstacles and len(self._last_all_obstacles) > 0)
                else (
                    [{"dist": self.obstacle_distance, "angle": 0.0}]
                    if (self.obstacle_detected and self.obstacle_distance < 6.0)
                    else None
                ),
                corridor_center_x=corridor_center_x,
                target_heading_rad=heading_rad,
            )

            # Règle 2 : Interdiction formelle de déclencher la détection frontale de suivi de ligne
            # pendant que le robot est dans un état d'évitement ou que le plan est verrouillé (LOCK_BYPASS_PLAN)
            if not self.bypass_plan_locked and self.obstacle_state == "CLEAR":
                # Phase 1 Trigger : vérification géométrique Shapely sur tous les obstacles devant le robot
                dir_z_corr = 1.0 if math.cos(heading_rad) >= 0 else -1.0
                shapely_stop_triggered = False
                for obs_item in (self._last_all_obstacles or []):
                    ox_i = float(obs_item.get("x", obs_item.get("abs_x", 999.0)))
                    oz_i = float(obs_item.get("z", obs_item.get("abs_z", 999.0)))
                    if (oz_i - self.robot_z) * dir_z_corr > 0.0 and abs(ox_i - corridor_center_x) < 1.05:
                        obs_poly_i = self.nav_engine.get_obstacle_polygon(obs_item)
                        if self.nav_engine.should_trigger_stop_and_lift(
                            self.robot_x, self.robot_z, self.robot_yaw, obs_poly_i, threshold_m=0.45
                        ):
                            shapely_stop_triggered = True
                            break

                if shapely_stop_triggered or res["obstacle_stop"] or (self.obstacle_detected and self.obstacle_distance < 2.2):
                    avoid_res = self._handle_obstacle_avoidance(
                        corridor_center_x=corridor_center_x,
                        heading_rad=heading_rad,
                        pct=pct_start,
                    )
                    if avoid_res == "RESUMED_SAME_LINE":
                        cutter_running = True
                        start_t = time.perf_counter()
                        continue
                    elif avoid_res == "DEAD_END_SKIP_ROW":
                        self.stop()
                        return True
                    else:
                        self.stop()
                        return False

            if weeding and res["weed_detected"] and not cutter_running:
                self.motor(6, 70)
                cutter_running = True

            rec = res["recommended_drive"]
            a_cmd = int(rec["a"])
            d_cmd = int(rec["d"])

            if not weeding and not reseeding:
                # Asservissement strict de suivi de ligne en mode Semence (champ vierge rectiligne)
                # e_lat = X_robot - X_ligne_cible
                # Δθ = θ_robot - θ_cible
                # steer = Kp * e_lat + Kd * Δθ
                dir_z_sign = 1.0 if math.cos(heading_rad) >= 0 else -1.0
                e_lat = (self.robot_x - corridor_center_x) * dir_z_sign
                delta_theta = self._wrap_angle(self.robot_yaw - heading_rad)
                if abs(e_lat) < 0.05 and abs(math.degrees(delta_theta)) < 3.0:
                    d_cmd = 0
                    a_cmd = 25
                else:
                    kp = 40.0
                    kd = 28.0
                    raw_steer = -(kp * e_lat + kd * delta_theta)
                    d_cmd = int(max(-35, min(35, round(raw_steer))))
                    a_cmd = 25

            if reseeding:
                dir_z_reseed = 1.0 if math.cos(heading_rad) >= 0 else -1.0
                eval_reseed = self.reseeding_planner.evaluate_reseeding_at_pose(
                    robot_x=self.robot_x,
                    robot_z=self.robot_z,
                    robot_yaw_rad=self.robot_yaw,
                    dir_z=dir_z_reseed,
                    nearby_plants=getattr(self, "_last_all_plants", []),
                    row_left_x=corridor_center_x - 0.23,
                    row_right_x=corridor_center_x + 0.23,
                )
                a_cmd = int(eval_reseed["recommended_speed"])
                if eval_reseed["trigger_seeder"]:
                    self.pulse(8, 60, 400)
                    self.reseeded_seeds_count += len(eval_reseed["seeds_to_drop"])
                self.mission_step_label = eval_reseed["hud_message"]

            self.drive(a_cmd, d_cmd, ms=self.GUIDE_CMD_MS)

            now = time.perf_counter()
            if now - last_notify > 0.5:
                last_notify = now
                frac = max(0.0, min(1.0, progress / max(0.01, length_units)))
                pct = pct_start + int(frac * (pct_end - pct_start))
                self._notify_mission_progress(
                    pct,
                    f"{label} : {progress / (100 * SIM_CM_TO_3D):.1f} m | écart {res['lateral_offset_cm']:+.1f} cm | "
                    f"Δθ {res['angular_error_deg']:+.1f}° | DRIVE:{rec['a']},{rec['d']}",
                )
            time.sleep(self.GUIDE_PERIOD_S)

        self.stop()
        time.sleep(0.15)
        return True

    def _pivot_to_heading(self, target_heading: float, timeout_s: float = 8.0, tolerance_deg: float = 4.5) -> bool:
        """Pivot sur place (TANK) asservi sur le cap jusqu'à ±tolerance_deg."""
        if not self._has_live_pose():
            return True
        start_t = time.perf_counter()
        while (time.perf_counter() - start_t) < timeout_s:
            if not self._check_pause_or_stop():
                self.stop()
                return False
            err_deg = math.degrees(self._wrap_angle(target_heading - self.robot_yaw))
            if abs(err_deg) < tolerance_deg:
                break
            # TANK:l,-l → yaw += l·k : l > 0 fait croître le cap
            mag = max(18, min(42, int(abs(err_deg) * 1.1)))
            l_cmd = mag if err_deg > 0 else -mag
            self.tank(l_cmd, -l_cmd, ms=self.GUIDE_CMD_MS)
            time.sleep(self.GUIDE_PERIOD_S)
        self.stop()
        time.sleep(0.15)
        return True

    def _drive_to_x(self, target_x: float, heading: float, timeout_s: float = 10.0) -> bool:
        """Translation en tournière (cap fixe ±90°) jusqu'à l'abscisse du couloir suivant."""
        if not self._has_live_pose():
            return True
        start_t = time.perf_counter()
        direction = math.sin(heading)  # +1 si cap vers +X, -1 vers -X
        while (time.perf_counter() - start_t) < timeout_s:
            if not self._check_pause_or_stop():
                self.stop()
                return False
            remaining = (target_x - self.robot_x) * direction
            if remaining <= 0.01:
                break
            psi_deg = math.degrees(self._wrap_angle(self.robot_yaw - heading))
            d_cmd = int(max(-25, min(25, -1.0 * psi_deg)))
            a_cmd = 12 if remaining < 0.25 else 18
            self.drive(a_cmd, d_cmd, ms=self.GUIDE_CMD_MS)
            time.sleep(self.GUIDE_PERIOD_S)
        self.stop()
        time.sleep(0.2)
        return True

    def _compute_surface_distance_to_obstacles(self) -> float:
        """Calcule la distance géométrique exacte (Shapely) entre le polygone orienté réel du robot
        (1.21m x 2.20m hors-tout) et l'union de tous les obstacles de la scène.
        """
        obstacles = getattr(self, "_last_all_obstacles", None) or []
        if not obstacles:
            return 99.0

        obs_union = self.nav_engine.get_obstacles_union(obstacles)
        if obs_union is None:
            return 99.0

        fp_real = self.nav_engine.get_robot_footprint(
            self.robot_x, self.robot_z, self.robot_yaw, include_margin=False
        )
        return float(fp_real.distance(obs_union))

    def _drive_distance(self, distance_m: float, speed: int = 20, timeout_s: float = 8.0, check_guard: bool = True) -> bool:
        """Avance d'une distance précise en mètres avec garde-fou géométrique anti-collision temps réel (Règle 4)."""
        if not self._has_live_pose():
            steps = max(1, int(round(distance_m / 0.15)))
            for _ in range(steps):
                if not self._check_pause_or_stop():
                    return False
                self.drive(speed, 0, ms=250)
                self.wait_for_done([1, 2, 3, 4], timeout=1.0)
            return True

        start_x, start_z = self.robot_x, self.robot_z
        target_dist_u = distance_m * 100.0 * SIM_CM_TO_3D
        start_t = time.perf_counter()

        while (time.perf_counter() - start_t) < timeout_s:
            if not self._check_pause_or_stop():
                self.stop()
                return False

            # Règle 4 : Garde-fou temps réel anti-collision pendant le contournement (<20 cm)
            if check_guard and self.obstacle_state == "AVOIDING":
                surf_d = self._compute_surface_distance_to_obstacles()
                if surf_d < 0.20:
                    logger.warning(f"⚠️ Garde-fou anti-collision activé : distance bord-à-bord = {surf_d*100:.1f}cm < 20cm ! Arrêt avant impact.")
                    self.stop()
                    # Recule de 40 cm supplémentaires
                    self._drive_reverse_distance(distance_m=0.40, speed=-35)
                    # Élargit le passage latéral
                    if self.avoidance_memory:
                        direction = self.avoidance_memory.get("direction", 1)
                        extra_lat = 0.40 * direction
                        self.robot_x += extra_lat
                        logger.info(f"🛡️ Répulsion d'urgence : décalage latéral augmenté de {extra_lat:+.2f}m")
                    start_x, start_z = self.robot_x, self.robot_z

            traveled = math.hypot(self.robot_x - start_x, self.robot_z - start_z)
            if traveled >= target_dist_u:
                break
            self.drive(speed, 0, ms=self.GUIDE_CMD_MS)
            time.sleep(self.GUIDE_PERIOD_S)

        self.stop()
        time.sleep(0.1)
        return True

    def _follow_pure_pursuit_path(
        self,
        waypoints: List[List[float]],
        target_heading_rad: float,
        corridor_center_x: float,
        pct: int,
        obstacle_center: Optional[Tuple[float, float]] = None,
        obstacle_radius: float = 0.45,
        lookahead_distance_m: float = 0.65,
        timeout_s: float = 30.0,
    ) -> bool:
        """Suivi de trajectoire Pure Pursuit haute précision le long des waypoints A* (Règles 2 & 3)."""
        if not waypoints:
            return False

        if not self._has_live_pose():
            self.drive(18, 30, ms=800)
            self.wait_for_done([1, 2, 3, 4], timeout=1.5)
            self.drive(20, 0, ms=1200)
            self.wait_for_done([1, 2, 3, 4], timeout=2.0)
            self.drive(18, -30, ms=800)
            self.wait_for_done([1, 2, 3, 4], timeout=1.5)
            return True

        start_time = time.perf_counter()
        wp_idx = 0
        total_wps = len(waypoints)
        goal_wp = waypoints[-1]
        dir_z = 1.0 if math.cos(target_heading_rad) >= 0 else -1.0

        logger.info(f"🎯 Démarrage Pure Pursuit : {total_wps} waypoints, L_d={lookahead_distance_m}m")

        while (time.perf_counter() - start_time) < timeout_s:
            if not self._check_pause_or_stop():
                self.stop()
                return False

            rx, rz = self.robot_x, self.robot_z
            dist_to_goal = math.hypot(goal_wp[0] - rx, goal_wp[1] - rz)

            # Fin de trajectoire si le robot est à moins de 35 cm du dernier waypoint
            if dist_to_goal < 0.35:
                logger.info(f"🏁 Pure Pursuit terminé : objectif atteint (dist={dist_to_goal:.2f}m)")
                break

            # Trouver le point de visée (Lookahead Target) à une distance >= L_d
            target_pt = goal_wp
            for i in range(wp_idx, total_wps):
                d = math.hypot(waypoints[i][0] - rx, waypoints[i][1] - rz)
                if d >= lookahead_distance_m:
                    target_pt = waypoints[i]
                    wp_idx = i
                    break

            tx, tz = target_pt[0], target_pt[1]

            # Règle 3 — Verrouillage Anti-Rabattement Prématuré ("Tail-Swing Lock" pour protéger l'Arrière et les Dents)
            # Z_arriere_dents = Z_robot - dir_z * 1.15m
            # Tant que Z_arriere_dents n'a pas dépassé le bord arrière de l'obstacle :
            # (Z_arriere_dents - Z_obs) * dir_z < (R_obs + 0.45m)
            # il est STRICTEMENT INTERDIT au robot de se rabattre vers la ligne initiale X_ligne !
            if obstacle_center is not None:
                ox, oz = obstacle_center
                z_arriere_dents = rz - dir_z * 1.55
                tail_cleared = (z_arriere_dents - oz) * dir_z >= (obstacle_radius + 0.85)
                # Tail-Swing Lock : Pendant la phase de rabattement, tant que l'arrière n'a pas dépassé
                # l'obstacle, on empêche le retour prématuré vers l'axe de la ligne en maintenant tx au large
                if not tail_cleared:
                    max_dev_x = max(abs(wp[0] - corridor_center_x) for wp in waypoints)
                    side_sign = 1 if any((wp[0] - corridor_center_x) > 0.4 for wp in waypoints) else -1
                    bypass_x = corridor_center_x + side_sign * max_dev_x
                    if abs(tx - corridor_center_x) < abs(bypass_x - corridor_center_x) * 0.85:
                        tx = bypass_x

            dx = tx - rx
            dz = tz - rz
            target_bearing = math.atan2(dx, dz)
            delta_theta = self._wrap_angle(target_bearing - self.robot_yaw)
            delta_theta_deg = math.degrees(delta_theta)

            dist_target = max(0.20, math.hypot(dx, dz))
            self._notify_mission_progress(
                pct,
                f"🎯 PURE PURSUIT [{wp_idx+1}/{total_wps}] : Δθ={delta_theta_deg:+.1f}°, cible X={tx:.2f} Z={tz:.2f}",
            )

            # Si |Δθ| > 15°, pivot sur place (TANK) pour orienter le robot vers le waypoint avant d'avancer
            if abs(delta_theta_deg) > 15.0:
                turn_dir = 1 if delta_theta > 0 else -1
                self.tank(turn_dir * 32, -turn_dir * 32, ms=250)
            else:
                # Avance avec correction de courbure proportionnelle Pure Pursuit (DRIVE:a,d)
                curvature = (2.0 * math.sin(delta_theta)) / max(0.35, dist_target)
                steer_val = int(max(-35, min(35, curvature * 40.0)))
                self.drive(16, steer_val, ms=300)

            time.sleep(0.10)

        self.stop()
        time.sleep(0.15)
        return True

    def _drive_reverse_distance(self, distance_m: float, speed: int = -35, timeout_s: float = 5.0) -> bool:
        """Recul précis de dégagement (DRIVE:-35,0)."""
        if not self._has_live_pose():
            self.drive(speed, 0, ms=600)
            self.wait_for_done([1, 2, 3, 4], timeout=1.5)
            return True

        start_x, start_z = self.robot_x, self.robot_z
        target_dist_u = distance_m * 100.0 * SIM_CM_TO_3D
        start_t = time.perf_counter()

        while (time.perf_counter() - start_t) < timeout_s:
            if not self._check_pause_or_stop():
                self.stop()
                return False
            traveled = math.hypot(self.robot_x - start_x, self.robot_z - start_z)
            if traveled >= target_dist_u:
                break
            self.drive(speed, 0, ms=self.GUIDE_CMD_MS)
            time.sleep(self.GUIDE_PERIOD_S)

        self.stop()
        time.sleep(0.1)
        return True

    def _handle_obstacle_avoidance(
        self,
        corridor_center_x: float,
        heading_rad: float,
        pct: int,
    ) -> str:
        """Machine à états séquentielle stricte à 3 trajectoires symétriques :
        - Phase 1 (STOP & LIFT) : Arrêt net, coupure fraise M6, relevage dents M5, calcul & verrouillage de (alpha, D1, D2).
        - Trajectoire 1 (Sortie) : Pivot +alpha, ligne droite D1 jusqu'à (X1, Z1), arrêt net.
        - Trajectoire 2 (Dépassement) : Pivot -alpha au cap initial, ligne droite D2 selon profondeur obstacle, arrêt net.
        - Trajectoire 3 (Retour symétrique) : Pivot -alpha vers rang, ligne droite D1 ramenant exactement à X3 == X0.
        - Reprise : Réalignement strict cap initial (|Δθ| < 1.5°, |Δx| < 2 cm), descente M5, fraise M6, reprise désherbage.
        AUCUN RECALCUL en cours de manœuvre pour éliminer tout risque d'indécision ou de blocage.
        """
        dir_z = 1.0 if math.cos(heading_rad) >= 0 else -1.0

        # Identifier avec précision l'obstacle principal dans le couloir
        primary_obs = None
        min_dist_ahead = 999.0
        for obs_item in (self._last_all_obstacles or []):
            ox = float(obs_item.get("x", obs_item.get("abs_x", 999.0)))
            oz = float(obs_item.get("z", obs_item.get("abs_z", 999.0)))
            dist_ahead = (oz - self.robot_z) * dir_z
            if -0.40 < dist_ahead < 7.5 and abs(ox - corridor_center_x) < 1.45:
                if dist_ahead < min_dist_ahead:
                    min_dist_ahead = dist_ahead
                    primary_obs = dict(obs_item)
                    primary_obs["x"] = ox
                    primary_obs["z"] = oz
                    primary_obs["abs_x"] = ox
                    primary_obs["abs_z"] = oz

        if not primary_obs and self._last_all_obstacles:
            ahead_obs = [
                o for o in self._last_all_obstacles
                if ((float(o.get("z", o.get("abs_z", 999.0))) - self.robot_z) * dir_z) > -0.20
            ]
            if ahead_obs:
                ahead_obs.sort(key=lambda o: math.hypot(
                    float(o.get("x", o.get("abs_x", 0.0))) - self.robot_x,
                    float(o.get("z", o.get("abs_z", 0.0))) - self.robot_z,
                ))
                primary_obs = dict(ahead_obs[0])

        if not primary_obs:
            ox = corridor_center_x
            oz = self.robot_z + dir_z * 2.2
            primary_obs = {
                "x": ox, "z": oz, "abs_x": ox, "abs_z": oz,
                "width_m": 1.45, "length_m": 1.35, "radius": 0.725,
                "min_x": ox - 0.725, "max_x": ox + 0.725,
                "min_z": oz - 0.675, "max_z": oz + 0.675,
                "type": "ROCHER", "isMoving": False,
            }

        obs_poly = self.symmetric_planner.get_obstacle_polygon(primary_obs)
        min_x, min_z, max_x, max_z = obs_poly.bounds
        w_obs = max_x - min_x
        l_obs = max_z - min_z

        # Différenciation Mobile vs Statique (si l'obstacle devant le robot est en mouvement)
        is_moving = bool(primary_obs.get("isMoving") or primary_obs.get("is_moving"))
        if is_moving:
            self.obstacle_state = "WAITING_MOVING_OBSTACLE"
            self.stop()
            self.stop_motor(6)
            logger.info("⏳ Obstacle mobile détecté devant le robot — Attente passage 5.0s...")
            wait_start = time.perf_counter()

            while (time.perf_counter() - wait_start) < 5.0:
                if not self._check_pause_or_stop():
                    self.stop()
                    return "ABORTED"
                remaining = max(0.0, 5.0 - (time.perf_counter() - wait_start))
                self._notify_mission_progress(
                    pct,
                    f"⏳ OBSTACLE MOBILE DÉTECTÉ — ATTENTE PASSAGE ({remaining:.1f}s)...",
                )
                time.sleep(0.2)

                if not self.obstacle_detected or self.obstacle_distance > 2.5:
                    self._notify_mission_progress(pct, "✅ VOIE LIBÉRÉE — REPRISE IMMÉDIATE SANS DÉVIATION")
                    self.motor(6, 70)
                    self.obstacle_state = "CLEAR"
                    time.sleep(0.2)
                    return "RESUMED_SAME_LINE"

            logger.info("⏱️ Délai 5s expiré — Obstacle mobile toujours présent -> Bascule en mode contournement")
            self._notify_mission_progress(pct, "⏱️ DÉLAI 5s ÉCOULÉ — BASCULE EN MODE CONTOURNEMENT")

        # ======================================================================
        # PHASE 0A — Arrêt & Relevage à l'arrêt :
        # Le robot s'arrête net (STOP, speed = 0), coupe la fraise (STOP:6) et relève complètement les dents (PULSE:5:-50:4000).
        # ======================================================================
        self.obstacle_state = "STOP_AND_LIFT"
        self.stop()
        self.stop_motor(6)
        time.sleep(0.10)
        self._notify_mission_progress(
            pct,
            f"🛑 PHASE 0A (STOP & LIFT) : ROCHER {int(w_obs*100)}×{int(l_obs*100)}cm — ARRÊT NET & RELEVAGE DENTS M5",
        )
        self.pulse(5, -50, 4000)
        self.wait_for_done([5], timeout=6.0)

        # ======================================================================
        # PHASE 0B — Marche Arrière de Dégagement (BACKING_UP) :
        # Identifie la face qui fait face au robot selon le sens de marche dir_z :
        # - Si dir_z == +1 (Aller vers +Z) : front_face_z = min_z, rear_face_z = max_z
        # - Si dir_z == -1 (Retour vers -Z) : front_face_z = max_z, rear_face_z = min_z
        # Calcule dist_to_front = dir_z * (front_face_z - robot.z)
        # Tant que dist_to_front < 1.55m, le robot recule tout droit (DRIVE:-20,0)
        # jusqu'à dist_to_front >= 1.55m, puis s'arrête net en (X0, Z0) !
        # ======================================================================
        dir_z = 1.0 if math.cos(heading_rad) >= 0 else -1.0
        front_face_z = min_z if dir_z > 0 else max_z
        rear_face_z = max_z if dir_z > 0 else min_z

        self.obstacle_state = "BACKING_UP"
        self._notify_mission_progress(pct, "🔙 PHASE 0B : MARCHE ARRIÈRE DE DÉGAGEMENT (1.55m devant l'obstacle)")

        if not self._has_live_pose():
            self.drive(-20, 0, ms=600)
            self.wait_for_done([1, 2, 3, 4], timeout=1.5)
        else:
            t0 = time.perf_counter()
            while (time.perf_counter() - t0) < 15.0:
                if not self._check_pause_or_stop():
                    self.stop()
                    return "ABORTED"
                dist_to_front = dir_z * (front_face_z - self.robot_z)
                if dist_to_front >= 1.55:
                    break
                self.drive(-20, 0, ms=200)
                time.sleep(0.06)

        self.stop()
        time.sleep(0.12)
        x0 = self.robot_x if self._has_live_pose() else corridor_center_x
        z0 = self.robot_z if self._has_live_pose() else (front_face_z - dir_z * 1.55)
        logger.info(f"✅ Recul de dégagement terminé en (X0={x0:.2f}, Z0={z0:.2f}), dist_to_front={dir_z * (front_face_z - z0):.2f}m >= 1.55m")

        # Calcul du plan trapézoïdal symétrique strict en 3 trajectoires
        plan = self.symmetric_planner.plan_symmetric_trapezoid(
            x0=x0,
            z0=z0,
            initial_heading_rad=heading_rad,
            primary_obs=primary_obs,
            all_obstacles=self._last_all_obstacles,
            field_width_m=self.field_width_m,
        )

        if not plan.get("success", False):
            self.obstacle_state = "DEAD_END"
            self.bypass_plan_locked = False
            self.nav_engine.unlock_bypass_plan()
            self._notify_mission_progress(pct, "🚫 AUCUNE ISSUE (G/D BLOQUÉS) — DEMI-TOUR VERS LIGNE SUIVANTE")
            new_heading = self._wrap_angle(heading_rad + math.pi)
            self._pivot_to_heading(new_heading)
            self.current_target_heading_rad = new_heading
            self.obstacle_state = "CLEAR"
            return "DEAD_END_SKIP_ROW"

        # Verrouillage absolu en mémoire dès le départ (Règle d'or : AUCUN recalcul en vol)
        self.locked_alpha_rad = plan["locked_alpha_rad"]
        self.locked_dist_1_m = plan["locked_dist_1_m"]
        self.locked_d2_m = plan["locked_d2_m"]
        self.initial_line_x = plan["initial_line_x"]
        self.initial_heading_rad = plan["initial_heading_rad"]
        self.avoidance_plan = plan
        self.avoidance_memory = plan
        self.avoidance_path_points = plan["waypoints"]
        self.bypass_plan_locked = True
        self.nav_engine.lock_bypass_plan(plan)

        side = int(plan.get("best_direction", 1))
        side_label = "DROITE" if side > 0 else "GAUCHE"
        self._notify_mission_progress(pct, plan["decision_msg"])
        logger.info(plan["decision_msg"])

        # ======================================================================
        # TRAJECTOIRE 1 : Sortie de la portée de l'obstacle (+alpha, D1)
        # ======================================================================
        self.obstacle_state = "AVOIDING_TRAJ_1"
        self._notify_mission_progress(
            pct,
            f"🔄 TRAJ 1 : PIVOT SUR PLACE DE {plan['locked_alpha_deg']:+.1f}° ({side_label}) VERS CAP {math.degrees(plan['theta_seg_1']):+.1f}°",
        )
        self._pivot_to_heading(plan["theta_seg_1"], timeout_s=6.0, tolerance_deg=4.5)
        self.stop()
        time.sleep(0.10)

        self._notify_mission_progress(
            pct,
            f"🚀 TRAJ 1 : AVANCE OBLIQUE D1 = {self.locked_dist_1_m:.2f}m VERS (X1={plan['x1']:.2f}, Z1={plan['z1']:.2f})",
        )
        if not self._has_live_pose():
            self.drive(20, 0, ms=500)
            self.wait_for_done([1, 2, 3, 4], timeout=1.2)
        else:
            start_x, start_z = self.robot_x, self.robot_z
            t0 = time.perf_counter()
            while (time.perf_counter() - t0) < 14.0:
                if not self._check_pause_or_stop():
                    self.stop()
                    self.bypass_plan_locked = False
                    self.nav_engine.unlock_bypass_plan()
                    return "ABORTED"
                traveled = math.hypot(self.robot_x - start_x, self.robot_z - start_z)
                if traveled >= (self.locked_dist_1_m - 0.03):
                    break
                psi_deg = math.degrees(self._wrap_angle(self.robot_yaw - plan["theta_seg_1"]))
                d_cmd = 0 if abs(psi_deg) < 2.0 else int(np.clip(-1.2 * psi_deg, -25, 25))
                self.drive(20, d_cmd, ms=220)
                time.sleep(0.06)

        self.stop()
        time.sleep(0.12)
        logger.info(f"✅ Trajectoire 1 terminée en (X1={self.robot_x:.2f}, Z1={self.robot_z:.2f})")

        # ======================================================================
        # TRAJECTOIRE 2 : Ligne Droite Parallèle de Dépassement (D2)
        # ======================================================================
        self.obstacle_state = "AVOIDING_TRAJ_2"
        self._notify_mission_progress(
            pct,
            f"🔄 TRAJ 2 : PIVOT SUR PLACE DE {-plan['locked_alpha_deg']:+.1f}° AU CAP INITIAL {math.degrees(self.initial_heading_rad):.0f}°",
        )
        self._pivot_to_heading(self.initial_heading_rad, timeout_s=6.0, tolerance_deg=4.5)
        self.stop()
        time.sleep(0.10)

        # 2. Calcul Exact et Verrouillage de la Distance Droite D2 (Indépendant du Sens +Z ou -Z)
        seg2_start_x = self.robot_x if self._has_live_pose() else plan["x1"]
        seg2_start_z = self.robot_z if self._has_live_pose() else plan["z1"]

        obs_min_z = float(plan.get("obs_min_z", min_z))
        obs_max_z = float(plan.get("obs_max_z", max_z))
        obs_depth = abs(obs_max_z - obs_min_z)

        dir_z = 1.0 if math.cos(self.initial_heading_rad) >= 0 else -1.0
        if dir_z > 0:
            obs_rear_z = obs_max_z
            dist_to_rear_edge = max(0.0, obs_rear_z - seg2_start_z)
        else:
            obs_rear_z = obs_min_z
            dist_to_rear_edge = max(0.0, seg2_start_z - obs_rear_z)

        required_D2 = max(dist_to_rear_edge + 2.80, obs_depth + 3.20)
        self.locked_d2_m = round(required_D2, 3)

        # Trace console demandée :
        logger.info(
            f"Début Seg2 à Z1={seg2_start_z:.2f} | Bord arrière obstacle Z_rear={obs_rear_z:.2f} | Distance D2 requise={required_D2:.2f}m"
        )
        self._notify_mission_progress(
            pct,
            f"🏎️ TRAJ 2 : DÉPASSEMENT D2 = {required_D2:.2f}m (Z1={seg2_start_z:.2f}, Z_rear={obs_rear_z:.2f})",
        )

        if not self._has_live_pose():
            self.drive(22, 0, ms=600)
            self.wait_for_done([1, 2, 3, 4], timeout=1.2)
        else:
            target_x1 = plan["x1"]
            t0 = time.perf_counter()
            while (time.perf_counter() - t0) < 25.0:
                if not self._check_pause_or_stop():
                    self.stop()
                    self.bypass_plan_locked = False
                    self.nav_engine.unlock_bypass_plan()
                    return "ABORTED"

                # 3. Double Condition Obligatoire avant d'Autoriser la Trajectoire 3 (Retour -alpha)
                # Pendant toute la Trajectoire 2 (Ligne droite parallèle) :
                # Le robot maintient strictement son décalage latéral X = X1 et son cap heading = initial_heading (steer = 0 + correction proportionnelle sur X1).
                traveled_D2 = math.hypot(self.robot_x - seg2_start_x, self.robot_z - seg2_start_z)
                rear_teeth_z = self.robot_z - dir_z * 1.40
                teeth_cleared = (dir_z * (rear_teeth_z - obs_rear_z)) >= 0.50

                # VERROU ABSOLU : Tant que traveled_D2 < required_D2 OU que !teeth_cleared,
                # il est STRICTEMENT INTERDIT de passer à la Trajectoire 3 !
                if traveled_D2 >= (required_D2 - 0.02) and teeth_cleared:
                    logger.info(
                        f"Fin Seg2 à Z2={self.robot_z:.2f} | Dents arrière à Z_dents={rear_teeth_z:.2f} "
                        f"(dir_z={dir_z:+.0f}, Z_rear={obs_rear_z:.2f}, marge={(dir_z * (rear_teeth_z - obs_rear_z)):.2f}m >= 0.50m) -> OK Début Retour Seg3"
                    )
                    break

                psi_deg = math.degrees(self._wrap_angle(self.robot_yaw - self.initial_heading_rad))
                e_x = self.robot_x - target_x1
                if abs(e_x) < 0.05 and abs(psi_deg) < 2.5:
                    d_cmd = 0
                    a_cmd = 25
                else:
                    d_cmd = int(np.clip(-1.2 * psi_deg - 15.0 * e_x * dir_z, -25, 25))
                    a_cmd = 22
                self.drive(a_cmd, d_cmd, ms=220)
                time.sleep(0.06)

        self.stop()
        time.sleep(0.12)
        logger.info(f"✅ Trajectoire 2 terminée en (X2={self.robot_x:.2f}, Z2={self.robot_z:.2f})")

        # ======================================================================
        # TRAJECTOIRE 3 : Retour Symétrique Exact (-alpha, D1) vers le Rang
        # ======================================================================
        self.obstacle_state = "AVOIDING_TRAJ_3"
        heading_seg3 = math.atan2(self.initial_line_x - self.robot_x, dir_z * 1.50)
        self._notify_mission_progress(
            pct,
            f"🔄 TRAJ 3 : PIVOT VERS LE RANG VERS CAP {math.degrees(heading_seg3):+.1f}°",
        )
        self._pivot_to_heading(heading_seg3, timeout_s=6.0, tolerance_deg=4.5)
        self.stop()
        time.sleep(0.10)

        self._notify_mission_progress(
            pct,
            f"🛬 TRAJ 3 : RETOUR SYMÉTRIQUE D1 = {self.locked_dist_1_m:.2f}m VERS X={self.initial_line_x:.2f}m",
        )
        if not self._has_live_pose():
            self.drive(20, 0, ms=500)
            self.wait_for_done([1, 2, 3, 4], timeout=1.2)
        else:
            start_x, start_z = self.robot_x, self.robot_z
            world_sign = plan["world_sign"]
            t0 = time.perf_counter()
            while (time.perf_counter() - t0) < 14.0:
                if not self._check_pause_or_stop():
                    self.stop()
                    self.bypass_plan_locked = False
                    self.nav_engine.unlock_bypass_plan()
                    return "ABORTED"
                traveled = math.hypot(self.robot_x - start_x, self.robot_z - start_z)
                dx_to_init = (self.robot_x - self.initial_line_x) * world_sign
                if traveled >= (self.locked_dist_1_m - 0.03) or dx_to_init <= 0.02:
                    break
                psi_deg = math.degrees(self._wrap_angle(self.robot_yaw - heading_seg3))
                d_cmd = 0 if abs(psi_deg) < 2.0 else int(np.clip(-1.2 * psi_deg, -25, 25))
                self.drive(20, d_cmd, ms=220)
                time.sleep(0.06)

        self.stop()
        time.sleep(0.12)
        logger.info(f"✅ Trajectoire 3 terminée en (X3={self.robot_x:.2f}, Z3={self.robot_z:.2f})")

        # ======================================================================
        # PHASE 4 : RÉALIGNEMENT STRICT, DESCENTE DES DENTS & REPRISE
        # ======================================================================
        self.obstacle_state = "LINE_RELOCK"
        self._notify_mission_progress(
            pct,
            f"🔄 RÉALIGNEMENT FINAL AU CAP {math.degrees(self.initial_heading_rad):.0f}° (|Δθ| < 1.5°)",
        )
        self._pivot_to_heading(self.initial_heading_rad, timeout_s=6.0, tolerance_deg=1.5)
        self.stop()

        self.weeding_state = "VISUAL_LINE_LOCK"
        self._notify_mission_progress(pct, "🌱 VERROUILLAGE STRICT DENTS SUR RANG (|Δθ| < 1.5° & |Δx| < 2cm)")
        self._visual_alignment_lock(target_x=self.initial_line_x, heading_rad=self.initial_heading_rad, timeout_s=6.0)

        # Arrêt complet (speed = 0)
        self.stop()
        time.sleep(0.15)

        # Descente des dents M5 à l'arrêt
        self._notify_mission_progress(pct, "⬇️ DESCENTE DES DENTS M5 (PULSE:5:50:4000)")
        self.pulse(5, 50, 4000)
        self.wait_for_done([5], timeout=6.0)

        # Redémarrage de la fraise rotative M6
        self._notify_mission_progress(pct, "⚙️ REDÉMARRAGE FRAISE M6 (70% PWM)")
        self.motor(6, 70)
        time.sleep(0.2)

        # Reprise du désherbage synchrone sur le rang initial
        self.weeding_state = "SYNCHRONOUS_ROW_WEEDING"
        self.obstacle_state = "CLEAR"
        self.bypass_plan_locked = False
        self.nav_engine.unlock_bypass_plan()
        self.avoidance_memory = None
        self.avoidance_plan = None
        self.avoidance_path_points = []
        return "RESUMED_SAME_LINE"

    def _headland_turn(self, current_heading: float, next_center_x: float) -> Tuple[bool, float]:
        """Demi-tour en bout de ligne : pivot 90° → translation jusqu'au couloir suivant → pivot 90°.

        État 1 — TURN_AND_APPROACH : Les dents M5 sont strictement relevées et la fraise M6 coupée.
        Les roues peuvent rouler sur le feuillage bas de l'arachide sans dommage.
        Retourne (succès, nouveau cap).
        """
        self.weeding_state = "TURN_AND_APPROACH"
        self.stop()
        self.stop_motor(6)
        self.stop_motor(8)
        new_heading = self._wrap_angle(current_heading + math.pi)
        if not self._has_live_pose():
            # Convention terrain : progression_direction = -1 (DROITE) / +1 (GAUCHE).
            # En firmware DRIVE/TANK, TANK:+35,-35 pivote à DROITE, TANK:-35,+35 pivote à GAUCHE.
            dx_global = next_center_x - self.robot_x
            if abs(dx_global) < 1e-4:
                dx_global = float(self.progression_direction)
            facing_sign = 1 if math.cos(current_heading) >= 0 else -1
            turn_right = (dx_global * facing_sign) < 0
            turn_sign = 1 if turn_right else -1

            self.tank(35 * turn_sign, -35 * turn_sign, ms=750)
            self.wait_for_done([1, 2, 3, 4], timeout=1.6)
            self.drive(24, 0, ms=850)
            self.wait_for_done([1, 2, 3, 4], timeout=1.6)
            self.tank(35 * turn_sign, -35 * turn_sign, ms=750)
            self.wait_for_done([1, 2, 3, 4], timeout=1.6)
            self.stop()
            self.robot_x = float(next_center_x)
            self.robot_yaw = new_heading
            self.current_target_heading_rad = new_heading
            return True, new_heading

        lateral_heading = math.pi / 2 if next_center_x > self.robot_x else -math.pi / 2
        self.current_target_heading_rad = lateral_heading
        if not self._pivot_to_heading(lateral_heading):
            return False, current_heading
        if not self._drive_to_x(next_center_x, lateral_heading):
            return False, current_heading
        self.current_target_heading_rad = new_heading
        if not self._pivot_to_heading(new_heading):
            return False, current_heading
        return True, new_heading

    def _visual_alignment_lock(self, target_x: float, heading_rad: float, timeout_s: float = 8.0) -> bool:
        """État 2 — VISUAL_LINE_LOCK : Positionnement Synchrone entre les Lignes (Outils Relevés).

        Effectue des micro-ajustements d'approche avec outils strictement relevés jusqu'à ce que :
        - |Δθ| < 2.0° (parallélisme strict avec les rangs d'arachide)
        - |Δx| < 2.5 cm (centrage parfait des dents à ±23 cm dans les couloirs inter-rangs)
        soient validés simultanément pendant au moins 3 frames consécutives.
        """
        # Sécurité absolue : outils 100% relevés et fraise arrêtée
        self.stop_motor(6)
        self.pulse(5, -50, 4000)
        self.weeding_state = "VISUAL_LINE_LOCK"

        if not self._has_live_pose():
            if self.execution_mode == ExecutionMode.REAL_ZERO_SIM.value:
                try:
                    from backend.vision.camera_manager import camera_manager as _cm
                    cam_telem = _cm.get_latest_telemetry()
                    dx_cm = float(cam_telem.get("lateral_error_cm", 0.0))
                    heading_err_deg = float(cam_telem.get("heading_error_deg", 0.0))
                    if abs(dx_cm) > 3.0 or abs(heading_err_deg) > 3.0:
                        d_corr = int(max(-20, min(20, round(+dx_cm * 1.2 - heading_err_deg * 0.8))))
                        self.drive(12, d_corr, ms=180)
                        self.wait_for_done([1, 2, 3, 4], timeout=0.6)
                except Exception:
                    pass
            self.visual_lock_consecutive = 3
            self.stop()
            return True

        start_t = time.perf_counter()
        consecutive_locked = 0
        self.visual_lock_consecutive = 0

        while (time.perf_counter() - start_t) < timeout_s:
            if not self._check_pause_or_stop():
                self.stop()
                return False

            steer = self.vision_brain.compute_corridor_steering(
                robot_x=self.robot_x,
                robot_yaw_rad=self.robot_yaw,
                corridor_center_x=target_x,
                target_heading_rad=heading_rad,
                base_speed=12,
                robot_z=self.robot_z,
            )

            heading_err_deg = steer["heading_error_deg"]
            dx_cm = steer["cross_track_cm"]

            # Vérification stricte des conditions de verrouillage sur au moins 3 frames consécutives
            if abs(heading_err_deg) < 2.0 and abs(dx_cm) < 2.5:
                consecutive_locked += 1
                self.visual_lock_consecutive = consecutive_locked
                if consecutive_locked >= 3:
                    self.stop()
                    logger.info(
                        "Alignement IA 🎯 VERROUILLÉ ENTRE LES LIGNES (3 frames : Δθ=%.2f°, Δx=%.2fcm)",
                        heading_err_deg,
                        dx_cm,
                    )
                    return True
                self.stop()
                time.sleep(0.08)
                continue
            else:
                consecutive_locked = 0
                self.visual_lock_consecutive = 0

            # Micro-ajustements de trajectoire (outils relevés)
            if abs(heading_err_deg) > 3.5:
                # Pivot doux sur place
                mag = max(10, min(22, int(abs(heading_err_deg) * 0.8)))
                turn = mag if heading_err_deg < 0 else -mag
                self.tank(turn, -turn, ms=120)
            else:
                # Avance très lente asservie
                self.drive(steer["a"], steer["d"], ms=140)

            time.sleep(0.10)

        self.stop()
        logger.info("Fin phase alignement visuel (timeout %.1fs)", timeout_s)
        return True

    def pause_mission(self) -> bool:
        """Met en pause la mission autonome en cours et coupe momentanément la locomotion."""
        if not self.current_mission_name or self.mission_paused:
            return False
        self.mission_paused = True
        self._last_mission_status = "PAUSED"
        self._mission_pause_event.clear()
        self.stop()
        self._notify_mission_progress(self.mission_progress, f"⏸ PAUSE : {self.mission_step_label}")
        return True

    def resume_mission(self) -> bool:
        """Reprend la mission autonome mise en pause."""
        if not self.current_mission_name or not self.mission_paused:
            return False
        self.mission_paused = False
        self._last_mission_status = "RUNNING"
        clean_label = self.mission_step_label.replace("⏸ PAUSE : ", "")
        self.mission_step_label = clean_label
        self._mission_pause_event.set()
        self._notify_mission_progress(self.mission_progress, clean_label)
        return True

    def wait_for_done(self, motor_indices: List[int], timeout: float) -> bool:
        """Attend la réception de #D pour une liste de moteurs tout en gérant la pause et l'arrêt."""
        deadline = time.perf_counter() + timeout
        for n in motor_indices:
            while time.perf_counter() < deadline:
                if self._mission_stop_flag.is_set():
                    return False
                if not self._mission_pause_event.is_set():
                    # Décale la deadline pendant la pause
                    time.sleep(0.05)
                    deadline += 0.05
                    continue
                if self.done_events[n].wait(timeout=0.1):
                    break
            else:
                logger.warning("Timeout attente #D %d", n)
                return False
        return not self._mission_stop_flag.is_set()

    def abort_mission(self) -> None:
        """Interrompt la mission autonome en cours et arrête les moteurs."""
        self._mission_pause_event.set()
        self.mission_paused = False
        self.mission_running = False
        self._last_mission_status = "STOPPED"
        self._mission_ended_ts = time.time()
        self._mission_stop_flag.set()
        for ev in self.done_events.values():
            ev.set()  # Débloque les attentes
        if self._mission_thread and self._mission_thread.is_alive():
            logger.info("Interruption de la mission autonome '%s'", self.current_mission_name)
            if threading.current_thread() != self._mission_thread:
                self._mission_thread.join(timeout=0.6)
        self.current_mission_name = None
        self.mission_progress = 0
        self.mission_step_label = "Prêt"
        self.obstacle_state = "CLEAR"
        self.bypass_plan_locked = False
        if hasattr(self, "nav_engine") and self.nav_engine:
            self.nav_engine.unlock_bypass_plan()
        for cb in getattr(self, "on_mission_stopped_callbacks", []):
            try:
                cb()
            except Exception:
                pass

    def start_mission_async(self, mission_name: str, **kwargs: Any) -> bool:
        """Démarre une mission autonome dans un thread dédié."""
        # En mode REAL_ZERO_SIM sans ESP32 (Mode Test Vision IA Seule), auto-armer si nécessaire
        if (
            self.execution_mode == ExecutionMode.REAL_ZERO_SIM.value
            and not self.hardware_bridge.connected
            and (not self.is_armed or self.armed == 0)
        ):
            self.arm()

        if self.estop_active or not self.is_armed or self.armed == 0:
            self._emit_log(
                "RX",
                "ERR: ESTOP (Robot non armé — envoyez la commande ARM avant de lancer une mission)",
            )
            return False

        if self.is_simulation_locked():
            self._emit_log(
                "RX",
                "ERR: NOT_CONNECTED (Mode Couplé actif : connectez le robot physique ESP32-S3 avant de lancer une mission)",
            )
            return False

        if self.is_mission_locked() or (self._mission_thread and self._mission_thread.is_alive()):
            logger.warning("Une mission est déjà en cours d'exécution : %s", self.current_mission_name)
            return False

        # Mise à jour optionnelle des dimensions Longueur × Largeur et configuration semence Arachide
        if any(k in kwargs for k in ("field_length_m", "field_width_m", "spacing_cm", "seed_with_tines_down")):
            self.set_seeding_config(
                spacing_cm=kwargs.get("spacing_cm"),
                use_tines=kwargs.get("seed_with_tines_down"),
                field_length_m=kwargs.get("field_length_m"),
                field_width_m=kwargs.get("field_width_m"),
            )

        if "progression_direction" in kwargs and kwargs["progression_direction"] is not None:
            self.progression_direction = -1 if int(kwargs["progression_direction"]) < 0 else 1
        if "direction_mode" in kwargs and kwargs["direction_mode"]:
            self.direction_mode = str(kwargs["direction_mode"]).upper()
        else:
            self.direction_mode = "RIGHT" if self.progression_direction < 0 else "LEFT"
        self.locked_side = "RIGHT" if self.progression_direction < 0 else "LEFT"
        if "start_x" in kwargs and kwargs["start_x"] is not None:
            self.robot_x = float(kwargs["start_x"])
        if "start_z" in kwargs and kwargs["start_z"] is not None:
            self.robot_z = float(kwargs["start_z"])

        target_map: Dict[str, Callable[..., Any]] = {
            "solar_deploy": lambda: self.mission_solar_panels(deploy=True),
            "solar_retract": lambda: self.mission_solar_panels(deploy=False),
            "seeding": self.mission_autonomous_seeding,
            "weeding": self.mission_autonomous_weeding,
            "smart_weeding": self.mission_smart_terrain_weeding,
            "reseeding": self.mission_autonomous_reseeding,
            "multi_row": self.mission_multi_row_autonomous,
            "obstacle_avoidance": self.mission_multi_row_autonomous,
            "step_nav": lambda: self.mission_step_navigation(kwargs.get("steps", [])),
        }

        if mission_name not in target_map:
            logger.error("Mission inconnue : %s", mission_name)
            return False

        self.mode = RobotMode.AUTO
        self.current_mission_name = mission_name
        self._last_mission_type = self._normalize_mission_type(mission_name)
        self._mission_started_ts = time.time()
        self._mission_ended_ts = None
        self._last_mission_status = "RUNNING"
        self.mission_running = True
        self.last_mission_alert = None
        vision_mode_map = {
            "seeding": "SEEDING",
            "reseeding": "RESEEDING",
            "weeding": "WEEDING",
            "smart_weeding": "WEEDING",
            "multi_row": "OBSTACLE_AVOIDANCE",
            "obstacle_avoidance": "OBSTACLE_AVOIDANCE",
        }
        try:
            from backend.vision.camera_manager import camera_manager
            camera_manager.set_config(current_mission=vision_mode_map.get(mission_name, mission_name.upper()))
            if self.execution_mode == ExecutionMode.REAL_ZERO_SIM.value and not camera_manager.connected:
                camera_manager.connect()
        except Exception:
            pass
        self.mission_paused = False
        self._mission_pause_event.set()
        self._mission_stop_flag.clear()
        self._notify_mission_progress(
            5,
            f"Démarrage mission Arachide : {mission_name} ({self.field_length_m}m × {self.field_width_m}m — {self.corridors_total} couloirs 46 cm)",
        )

        self.kinematic_engine.init_mission(
            row_spacing=float(self.robot_specs.get("seeder_row_spacing_cm", 46.0)) / 100.0,
            field_length=float(self.field_length_m),
            field_width=float(self.field_width_m),
            start_x=float(self.robot_x),
            start_z=float(self.robot_z),
            step_dir=self.progression_direction,
        )
        self.crop_rows_x = list(self.kinematic_engine.rows_x)

        def runner():
            self.mission_running = True
            ok = False
            try:
                ok = target_map[mission_name]()
                if ok and not self._mission_stop_flag.is_set() and self.last_mission_alert is None:
                    self._last_mission_status = "COMPLETED"
                    self._notify_mission_progress(100, f"Mission {mission_name} terminée avec succès")
            except Exception as e:
                self._last_mission_status = "STOPPED"
                logger.error("Erreur durant la mission %s : %s", mission_name, e)
            finally:
                self.mission_running = False
                self._mission_ended_ts = time.time()
                if self._mission_stop_flag.is_set() or self.estop_active:
                    self._last_mission_status = "STOPPED"
                elif self.last_mission_alert is not None and self.last_mission_alert.get("level") == "error":
                    self._last_mission_status = "STOPPED"
                elif ok or (self.last_mission_alert is not None and self.last_mission_alert.get("level") == "success"):
                    self._last_mission_status = "COMPLETED"
                else:
                    self._last_mission_status = "STOPPED"
                self.current_mission_name = None
                self.mission_paused = False
                self._mission_pause_event.set()
                self.mode = RobotMode.MANUAL
                if self.last_mission_alert is not None:
                    pct_final = 100 if self.last_mission_alert.get("level") == "success" else 0
                    self._notify_mission_progress(pct_final, self.last_mission_alert["message"])
                else:
                    self._notify_mission_progress(0, "Prêt")
                logger.info("Mission %s terminée, retour en mode MANUAL", mission_name)
                for cb in getattr(self, "on_mission_stopped_callbacks", []):
                    try:
                        cb()
                    except Exception:
                        pass

        self._mission_thread = threading.Thread(target=runner, name=f"Mission_{mission_name}", daemon=True)
        self._mission_thread.start()
        return True

    # --------------------------------------------------------------------------
    # Séquences Autonomes Spécialisées Culture de l'Arachide (Inter-Rang 46 cm)
    # --------------------------------------------------------------------------
    def mission_solar_panels(self, deploy: bool = True) -> bool:
        """Mission 3 — Déploiement / Rangement Panneaux Solaires (M7) :

        ARM -> TMAX:7:35 -> PULSE:7:60:30000 (ou -60), attente de #D 7.
        """
        action_str = "Déploiement" if deploy else "Rangement"
        logger.info("Démarrage séquence Panneaux Solaires (deploy=%s)", deploy)
        self._notify_mission_progress(10, f"ARM & Sécurité TMAX:7:35 ({action_str})")
        self.arm()
        self.send_command("TMAX:7:35")
        if not self._check_pause_or_stop():
            return False

        v = 60 if deploy else -60
        self._notify_mission_progress(35, f"PULSE:7:{v}:30000 — Attente #D 7 (30 s)")
        self.pulse(7, v, 30000)
        ok = self.wait_for_done([7], timeout=35.0)
        return ok

    def mission_autonomous_seeding(self) -> bool:
        """🌱 Semence Arachide sur Champ 100 % Vide (Écartement Fixé par les 2 Tuyaux à 46 cm) :
        - Champ vierge de toute plante et mauvaise herbe.
        - Les 2 tuyaux de semence arrière (M8) à -23 cm et +23 cm déposent 2 rangées simultanées espacées de 46 cm.
        - En bout de ligne, demi-tour avec un pas latéral de 92 cm (2 × 46 cm) dans le sens choisi (Gauche ou Droite).
        """
        pass_width_m = 0.92  # 2 rangs de 46 cm semés simultanément par M8
        self.corridors_total = max(1, int(round(self.field_width_m / pass_width_m)))
        passes_to_exec = min(self.corridors_total, 8)
        steps_per_pass = max(4, min(12, int(round(self.field_length_m * 0.8))))
        length_u = self.field_length_m * 100.0 * SIM_CM_TO_3D
        initial_x = self.robot_x if self._has_live_pose() else 0.0
        corridor_x = initial_x
        heading = 0.0 if math.cos(self.robot_yaw) >= 0 else math.pi

        logger.info(
            "Démarrage Semence Arachide (parcelle=%.1fm×%.1fm, %d passes de 92 cm, espacement=%d cm, sens=%s)",
            self.field_length_m,
            self.field_width_m,
            self.corridors_total,
            self.seed_spacing_cm,
            "DROITE" if self.progression_direction < 0 else "GAUCHE",
        )
        self._notify_mission_progress(
            8,
            f"🌱 Semence Arachide Champ Vierge ({self.field_length_m}m × {self.field_width_m}m) — Espacement {self.seed_spacing_cm} cm (2 tuyaux 46 cm)",
        )
        self.arm()
        if not self._check_pause_or_stop():
            return False

        # Vérification obligatoire de présence de ligne avant tout mouvement / abaissement outil en mode réel
        if not self._verify_initial_line_presence("SEEDING"):
            self.stop()
            return False

        # Alignement initial sur le cap du couloir
        if not self._pivot_to_heading(heading):
            return False

        completed = True
        use_continuous = self.execution_mode == ExecutionMode.REAL_ZERO_SIM.value
        for pass_idx in range(1, passes_to_exec + 1):
            self.current_corridor = pass_idx
            if not self._check_pause_or_stop():
                completed = False
                break

            # 1. Descente optionnelle des dents M5 en début de ligne pour ouvrir le sillon
            if self.seeding_use_tines:
                self._notify_mission_progress(
                    12 + int(((pass_idx - 1) / passes_to_exec) * 75),
                    f"Passe {pass_idx}/{passes_to_exec} : Descente dents sillon (PULSE:5:50:4000)",
                )
                self.pulse(5, 50, 4000)
                if not self.wait_for_done([5], timeout=6.0):
                    return False

            # 2. Activation du semoir double rang (2 becs espacés de 46 cm)
            self.auto_seed_enabled = True
            self._distance_since_last_seed_cm = 0.0
            self.pulse(8, 75, 350)
            time.sleep(0.15)

            # 3. Avance guidée en boucle fermée sur toute la longueur saisie (ou jusqu'à fin de ligne caméra)
            pct_a = 20 + int(((pass_idx - 1) / passes_to_exec) * 65)
            pct_b = 20 + int((pass_idx / passes_to_exec) * 65)
            if not self._follow_corridor(
                corridor_center_x=corridor_x,
                heading_rad=heading,
                length_units=length_u,
                label=f"🌱 Semis Double Ligne {pass_idx}/{passes_to_exec}",
                pct_start=pct_a,
                pct_end=pct_b,
                weeding=False,
                reseeding=False,
                fallback_steps=steps_per_pass,
                continuous_vision=use_continuous,
            ):
                completed = False

            # 4. Bout de ligne : arrêt semoir STOP:8 et relevage dents M5 avant demi-tour
            self.auto_seed_enabled = False
            self.stop()
            self.stop_motor(8)
            time.sleep(0.15)

            if self.seeding_use_tines and not self._last_corridor_ended_by_vision:
                self._notify_mission_progress(88, f"Bout de passe {pass_idx}/{passes_to_exec} : Relevage dents (PULSE:5:-50:4000)")
                self.pulse(5, -50, 4000)
                self.wait_for_done([5], timeout=6.0)

            # 5. Demi-tour asservi vers les 2 lignes suivantes (+92 cm dans le sens choisi)
            if completed and pass_idx < passes_to_exec:
                corridor_x = round(initial_x + pass_idx * self.progression_direction * 0.92, 3)
                self._notify_mission_progress(
                    90,
                    f"Demi-tour vers les 2 lignes suivantes (Ligne {pass_idx + 1} à X={corridor_x:+.2f}m)",
                )
                ok, heading = self._headland_turn(heading, corridor_x)
                if not ok:
                    completed = False
                elif not self._verify_peanut_row_present_after_turn(mission_type="SEEDING"):
                    break
            if not completed:
                break

        self.stop()
        self.current_corridor = self.corridors_total
        return completed

    def mission_autonomous_reseeding(self) -> bool:
        """🔍 Révision de la Semence (Resemence Intelligente Ciblée) :
        - Parcourt les lignes d'arachide 2 par 2 sous les 2 tuyaux de semence arrière M8 (-23 cm et +23 cm).
        - La Caméra IA scanne devant le robot :
          * Si plein rang sain (delta_z <= 30 cm) : affiche RANG COMPLET, vitesse normale (DRIVE:32,d), semoir OFF (STOP:8).
          * Si trou de germination (> 35 cm) ou plant mort/desséché : encadre en MAGENTA, ralentit (DRIVE:18,d),
            et dès que les tuyaux arrière M8 arrivent au-dessus de la zone vide, active le semoir (PULSE:8:60:400)
            pour déposer chirurgicalement la graine de remplacement à 20 cm d'intervalle sans doublon.
        - En bout de ligne : relèvement, demi-tour de 92 cm dans le sens choisi (Gauche ou Droite), passe suivante.
        """
        pass_width_m = 0.92  # 2 rangs de 46 cm
        self.corridors_total = max(1, int(round(self.field_width_m / pass_width_m)))
        passes_to_exec = min(self.corridors_total, 8)
        steps_per_pass = max(4, min(12, int(round(self.field_length_m * 0.8))))
        length_u = self.field_length_m * 100.0 * SIM_CM_TO_3D
        corridor_x = self.find_nearest_peanut_row(self.robot_x)
        heading = 0.0 if math.cos(self.robot_yaw) >= 0 else math.pi

        self.reseeding_planner.reset_stats()
        self.reseeded_seeds_count = 0
        self.dead_plants_count = 0
        self.gaps_detected_count = 0
        self.reviewed_rows_count = 0

        logger.info(
            "Démarrage Révision Semence Arachide (parcelle=%.1fm×%.1fm, %d passes de 92 cm, sens=%s)",
            self.field_length_m,
            self.field_width_m,
            self.corridors_total,
            "DROITE" if self.progression_direction < 0 else "GAUCHE",
        )
        self._notify_mission_progress(
            8,
            f"🔍 Révision Semence Arachide ({self.field_length_m}m × {self.field_width_m}m) — Scan IA & Resemence Ciblée",
        )
        self.arm()
        if not self._check_pause_or_stop():
            return False

        # Sécurité outils : dents relevées, fraise coupée, semoir prêt
        self.stop_motor(6)
        self.stop_motor(8)

        # Vérification obligatoire de présence de ligne d'arachide avant tout mouvement en mode réel
        if not self._verify_initial_line_presence("RESEEDING"):
            self.stop()
            return False

        self.pulse(5, -50, 4000)

        if not self._pivot_to_heading(heading):
            return False

        completed = True
        use_continuous = self.execution_mode == ExecutionMode.REAL_ZERO_SIM.value
        for pass_idx in range(1, passes_to_exec + 1):
            self.current_corridor = pass_idx
            if not self._check_pause_or_stop():
                completed = False
                break

            pct_a = 15 + int(((pass_idx - 1) / passes_to_exec) * 70)
            pct_b = 15 + int((pass_idx / passes_to_exec) * 70)

            if not self._follow_corridor(
                corridor_center_x=corridor_x,
                heading_rad=heading,
                length_units=length_u,
                label=f"🔍 Révision Lignes {pass_idx}/{passes_to_exec}",
                pct_start=pct_a,
                pct_end=pct_b,
                weeding=False,
                reseeding=True,
                fallback_steps=steps_per_pass,
                continuous_vision=use_continuous,
            ):
                completed = False

            self.stop()
            self.stop_motor(8)
            self.reviewed_rows_count += 2

            if completed and pass_idx < passes_to_exec:
                step_x = self.progression_direction * 0.92
                corridor_x += step_x
                self._notify_mission_progress(
                    88,
                    f"Demi-tour vers lignes suivantes ({'+' if self.progression_direction > 0 else '-'}92 cm)",
                )
                ok, heading = self._headland_turn(heading, corridor_x)
                if not ok:
                    completed = False
                elif not self._verify_peanut_row_present_after_turn(mission_type="RESEEDING"):
                    break
            if not completed:
                break

        self.stop()
        self.stop_motor(8)
        self.current_corridor = self.corridors_total
        return completed

    def mission_autonomous_weeding(self) -> bool:
        """Mission 1 — Désherbage Arachide Inter-Rangs sur Toute la Parcelle."""
        return self.mission_smart_terrain_weeding()

    def mission_step_navigation(self, steps: List[Tuple[int, int, int]]) -> bool:
        """Navigation autonome par petits pas :

        Enchaîne des corrections courtes DRIVE:a,d:ms (300 à 1000 ms) simulant
        un guidage par vision / GPS RTK.
        """
        if not steps:
            steps = [
                (25, 4, 700),
                (25, -4, 700),
                (30, 0, 900),
                (25, 6, 600),
                (25, -6, 600),
                (30, 0, 800),
            ]
        logger.info("Démarrage Navigation par petits pas (%d étapes)", len(steps))
        self.arm()
        total = max(1, len(steps))
        for i, (a, d, ms) in enumerate(steps):
            if not self._check_pause_or_stop():
                self.stop()
                return False
            pct = int(10 + ((i + 1) / total) * 85)
            self._notify_mission_progress(pct, f"Pas RTK {i + 1}/{total} : DRIVE:{a},{d}:{ms}")
            self.drive(a, d, ms)
            timeout = (ms / 1000.0) + 1.0
            if not self.wait_for_done([1, 2, 3, 4], timeout=timeout):
                logger.warning("Étape %d non terminée à temps", i + 1)
        return True

    def update_terrain_sensor_data(
        self,
        pitch: float,
        roll: float,
        obstacle: bool = False,
        distance: float = 99.0,
        dense_weed: bool = False,
        speed_kmh: Optional[float] = None,
        weeds_destroyed: Optional[int] = None,
        weeds_total: Optional[int] = None,
        seeds_sown: Optional[int] = None,
        tines_depth: Optional[float] = None,
        solar_angle: Optional[float] = None,
        distance_traveled_m: Optional[float] = None,
        robot_x: Optional[float] = None,
        robot_z: Optional[float] = None,
        robot_yaw: Optional[float] = None,
        weeds_in_fov: Optional[List[Dict[str, float]]] = None,
        obstacles_in_fov: Optional[List[Dict[str, float]]] = None,
        crop_rows_x: Optional[List[float]] = None,
    ) -> None:
        """Met à jour les données télémétriques de pente, obstacles, vision IA et agronomie depuis la scène 3D."""
        if self.execution_mode == ExecutionMode.REAL_ZERO_SIM.value:
            # En mode Zéro Simulation (100% Réel), on ignore la télémétrie 3D du navigateur pour laisser la caméra USB piloter seule
            return
        self._last_3d_feedback_time = time.perf_counter()
        self.terrain_pitch = pitch
        self.terrain_roll = roll
        self.obstacle_detected = obstacle
        self.obstacle_distance = distance
        self.dense_weed_detected = dense_weed
        self._last_all_obstacles = obstacles_in_fov or []
        if crop_rows_x and isinstance(crop_rows_x, list) and len(crop_rows_x) > 0:
            self.crop_rows_x = sorted(round(float(rx), 3) for rx in crop_rows_x)
        nearest_crit_obs = [o for o in self._last_all_obstacles if float(o.get("dist", o.get("distance", 99.0))) < 3.0]
        self.obstacle_is_moving = any(bool(o.get("isMoving") or o.get("is_moving")) for o in nearest_crit_obs)
        if speed_kmh is not None:
            self.speed_kmh = speed_kmh
        if weeds_destroyed is not None:
            self.weeds_destroyed = weeds_destroyed
        if weeds_total is not None and weeds_total > 0:
            self.weeds_total = weeds_total
        if seeds_sown is not None:
            self.seeds_sown = seeds_sown
        if tines_depth is not None:
            self.tines_depth = max(0.0, min(1.0, tines_depth))
        if solar_angle is not None:
            self.solar_angle = max(0.0, min(1.0, solar_angle))
        if distance_traveled_m is not None and distance_traveled_m >= 0:
            self.distance_traveled_m = max(self.distance_traveled_m, distance_traveled_m)
        if robot_x is not None:
            self.robot_x = robot_x
        if robot_z is not None:
            self.robot_z = robot_z
        if robot_yaw is not None:
            self.robot_yaw = robot_yaw

        # Mettre à jour le cerveau de Vision IA en Python avec les cibles détectées
        synth_weeds = weeds_in_fov
        if synth_weeds is None and dense_weed:
            synth_weeds = [{"dist": min(3.5, max(1.0, distance)), "angle": 0.05}]
        synth_obs = obstacles_in_fov
        if synth_obs is None and (obstacle or distance < 5.0):
            synth_obs = [{"dist": max(0.8, distance), "angle": 0.0}]

        # Cap et axe de référence cohérents : évite le saut à 180° lors des retours ou demi-tours
        if self.mode == RobotMode.AUTO and self.current_mission_name:
            active_target_heading = self.current_target_heading_rad
            active_corridor_x = self.current_corridor_center_x
        else:
            active_target_heading = 0.0 if math.cos(self.robot_yaw) >= 0 else math.pi
            active_corridor_x = self.find_nearest_peanut_row(self.robot_x)

        self.vision_brain.process_scene_telemetry(
            robot_x=self.robot_x,
            robot_z=self.robot_z,
            robot_yaw_rad=self.robot_yaw,
            terrain_roll_deg=self.terrain_roll,
            terrain_pitch_deg=self.terrain_pitch,
            weeds_in_fov=synth_weeds,
            obstacles_in_fov=synth_obs,
            corridor_center_x=active_corridor_x,
            target_heading_rad=active_target_heading,
        )

    def mission_smart_terrain_weeding(self) -> bool:
        """🌿 Désherbage Intelligent d'Arachide sur Toute la Parcelle (Machine à 4 États Stricts) :

        État 1 — TURN_AND_APPROACH :
          Demi-tour et approche avec dents M5 strictement relevées et fraise M6 à 0.
        État 2 — VISUAL_LINE_LOCK :
          Micro-ajustements avec outils relevés jusqu'à verrouillage strict (|Δθ| < 2.0° et |Δx| < 2.5 cm)
          sur au moins 3 frames consécutives.
        État 3 — LOWERING_TINES_STATIONARY :
          Arrêt complet (vitesse = 0 km/h) -> descente des dents (PULSE:5:50:4000, attente #D 5) -> fraise ON (M6:70).
        État 4 — SYNCHRONOUS_ROW_WEEDING :
          Roulage et désherbage synchrone asservi sur les lignes d'arachide réelles (ondulation Z).
        En bout de passe :
          Arrêt complet (STOP) -> fraise OFF (STOP:6) -> relevage complet des dents (PULSE:5:-50:4000, attente #D 5)
          AVANT de braquer pour le demi-tour.
        """
        row_spacing_m = max(0.20, float(self.robot_specs.get("seeder_row_spacing_cm", 46.0)) / 100.0)
        self.corridors_total = max(1, int(round(self.field_width_m / row_spacing_m)))
        corridors_to_exec = min(self.corridors_total, 8)
        steps_per_corridor = max(4, min(11, int(round(self.field_length_m * 0.8))))
        length_u = self.field_length_m * 100.0 * SIM_CM_TO_3D
        # Centrage sur une VRAIE ligne d'arachide détectée dans le champ (jamais une formule aveugle)
        corridor_x = self.find_nearest_peanut_row(self.robot_x)
        heading = 0.0 if math.cos(self.robot_yaw) >= 0 else math.pi

        self.current_corridor_center_x = corridor_x
        self.current_target_heading_rad = heading
        self.weeding_state = "TURN_AND_APPROACH"

        logger.info(
            "Démarrage Désherbage Arachide Inter-Rangs (parcelle=%.1fm×%.1fm, %d couloirs de 46 cm)",
            self.field_length_m,
            self.field_width_m,
            self.corridors_total,
        )
        self._notify_mission_progress(
            8,
            f"Armement (ARM) — Parcelle Arachide {self.field_length_m}m × {self.field_width_m}m ({self.corridors_total} couloirs de 46 cm)",
        )
        self.arm()
        if not self._check_pause_or_stop():
            return False

        # Sécurité : outils strictement relevés et fraise coupée avant le premier engagement
        self.stop_motor(6)

        # Vérification obligatoire de présence de rang d'arachide avant tout mouvement / descente dents en mode réel
        if not self._verify_initial_line_presence("WEEDING"):
            self.stop()
            return False

        self.pulse(5, -50, 4000)

        # Alignement initial sur le cap du couloir avant de descendre les outils
        if not self._pivot_to_heading(heading):
            return False

        completed = True
        use_continuous = self.execution_mode == ExecutionMode.REAL_ZERO_SIM.value
        for corridor_idx in range(1, corridors_to_exec + 1):
            self.current_corridor = corridor_idx
            self.current_corridor_center_x = corridor_x
            self.current_target_heading_rad = heading
            if not self._check_pause_or_stop():
                completed = False
                break

            # ------------------------------------------------------------------
            # État 2 — VISUAL_LINE_LOCK (Positionnement Synchrone, Outils Relevés)
            # ------------------------------------------------------------------
            self.weeding_state = "VISUAL_LINE_LOCK"
            self._notify_mission_progress(
                10 + int(((corridor_idx - 1) / corridors_to_exec) * 75),
                f"Passe {corridor_idx}/{corridors_to_exec} : 🔍 ALIGNEMENT EN COURS... (DENTS RELEVÉES)",
            )
            if not self._visual_alignment_lock(target_x=corridor_x, heading_rad=heading):
                if not self._check_pause_or_stop():
                    completed = False
                    break

            # ------------------------------------------------------------------
            # État 3 — LOWERING_TINES_STATIONARY (Descente des Dents à l'Arrêt Complet)
            # ------------------------------------------------------------------
            self.weeding_state = "LOWERING_TINES_STATIONARY"
            self.stop()
            time.sleep(0.15)
            self._notify_mission_progress(
                15 + int(((corridor_idx - 1) / corridors_to_exec) * 75),
                f"Passe {corridor_idx}/{corridors_to_exec} : 🎯 VERROUILLÉ ENTRE LES LIGNES — Descente dents",
            )
            self.pulse(5, 50, 4000)
            if not self.wait_for_done([5], timeout=6.0):
                logger.warning("Échec descente des dents M5")
                return False

            if not self._check_pause_or_stop():
                self.stop()
                return False

            # Démarrage de la fraise avant M6:70 à l'arrêt complet
            self.motor(6, 70)
            time.sleep(0.2)

            # ------------------------------------------------------------------
            # État 4 — SYNCHRONOUS_ROW_WEEDING (Roulage + Suivi Synchrone des Courbes)
            # ------------------------------------------------------------------
            self.weeding_state = "SYNCHRONOUS_ROW_WEEDING"
            self._notify_mission_progress(
                20 + int(((corridor_idx - 1) / corridors_to_exec) * 75),
                f"Passe {corridor_idx}/{corridors_to_exec} : 🌿 DÉSHERBAGE SYNCHRONE",
            )
            pct_a = 24 + int(((corridor_idx - 1) / corridors_to_exec) * 60)
            pct_b = 24 + int((corridor_idx / corridors_to_exec) * 60)
            if not self._follow_corridor(
                corridor_center_x=corridor_x,
                heading_rad=heading,
                length_units=length_u,
                label=f"🌿 Passe {corridor_idx}/{corridors_to_exec}",
                pct_start=pct_a,
                pct_end=pct_b,
                weeding=True,
                fallback_steps=steps_per_corridor,
                continuous_vision=use_continuous,
            ):
                completed = False

            # ------------------------------------------------------------------
            # Séquence Propre en Bout de Ligne avant Demi-Tour :
            # 1. Arrêt complet (STOP)
            # 2. Coupe fraise (STOP:6)
            # 3. Relevage complet des dents (PULSE:5:-50:4000, attente #D 5)
            # AVANT d'engager le braquage des roues pour le demi-tour !
            # ------------------------------------------------------------------
            self.stop()
            time.sleep(0.15)
            self._notify_mission_progress(86, f"Bout de passe {corridor_idx}/{corridors_to_exec} : Arrêt fraise (STOP:6)")
            self.stop_motor(6)
            time.sleep(0.1)

            # Relevage complet des dents M5 AVANT demi-tour (si pas déjà relevées par la détection fin de ligne)
            if not self._last_corridor_ended_by_vision:
                self._notify_mission_progress(90, f"Bout de passe {corridor_idx}/{corridors_to_exec} : Relevage dents (PULSE:5:-50:4000)")
                self.pulse(5, -50, 4000)
                self.wait_for_done([5], timeout=6.0)

            # ------------------------------------------------------------------
            # État 1 — TURN_AND_APPROACH (Demi-tour, Dents 100% Relevées, Fraise OFF)
            # Sélection dynamique de la prochaine ligne d'arachide selon le seuil 2 × grosse dent (50 cm) :
            # - Si écart E <= 0.50 m : SAUT D'UNE LIGNE (RANG i+2)
            # - Si écart E >  0.50 m : PAS DE SAUT, LIGNE SUIVANTE (RANG i+1)
            # ------------------------------------------------------------------
            if completed and corridor_idx < corridors_to_exec:
                self.weeding_state = "TURN_AND_APPROACH"
                row_choice = self.select_next_peanut_row(corridor_x, self.crop_rows_x, direction_sign=self.progression_direction)
                if row_choice["row_step"] == 0:
                    break
                corridor_x = row_choice["next_x"]
                self.current_corridor_center_x = corridor_x
                self._notify_mission_progress(94, row_choice["hud_message"])
                ok, heading = self._headland_turn(heading, corridor_x)
                self.current_target_heading_rad = heading
                if not ok:
                    completed = False
                elif not self._verify_peanut_row_present_after_turn(mission_type="WEEDING"):
                    break
            if not completed:
                break

        self.stop()
        self.weeding_state = "IDLE"
        self.current_corridor = self.corridors_total
        return completed

    def mission_multi_row_autonomous(self) -> bool:
        """Mission 4 — Parcours Autonome Multi-Rangs (Vision / RTK par petits pas + Désherbage & Semis) :

        Le robot parcourt le champ, corrige sa trajectoire sur les pentes par petits pas
        DRIVE:a,d:ms (300 à 1000 ms), évite les obstacles et désherbe/sème automatiquement.
        """
        logger.info("Démarrage Mission 4 : Parcours Autonome Multi-Rangs (Vision / RTK)")
        self._notify_mission_progress(8, "Initialisation RTK & Armement (ARM)")
        self.arm()
        if not self._check_pause_or_stop():
            return False

        # Descente des dents M5
        self._notify_mission_progress(15, "Descente dents de terrage (PULSE:5:50:4000)")
        self.pulse(5, 50, 4000)
        if not self.wait_for_done([5], timeout=6.0):
            return False

        # Activation combinée Fraise M6 + Semoir M8
        self._notify_mission_progress(25, "Activation Fraise (M6:70) & Semoir (M8:60)")
        self.motor(6, 70)
        time.sleep(0.2)
        self.motor(8, 60)
        time.sleep(0.3)

        # Parcours guidé par petits pas (300 à 1000 ms) avec correction de pente
        rtk_steps = [
            (24, 0, 800),
            (22, 4, 650),
            (22, -4, 650),
            (25, 0, 900),
            (20, 18, 600),   # Virage bout de rang (demi-tour progressif)
            (20, 18, 600),
            (24, 0, 850),
            (22, -3, 600),
            (22, 3, 600),
            (25, 0, 800),
        ]

        total_steps = len(rtk_steps)
        for idx, (base_a, base_d, ms_step) in enumerate(rtk_steps):
            if not self._check_pause_or_stop():
                break

            # Évitement d'obstacle si détecté par le LiDAR
            if self.obstacle_detected and self.obstacle_distance < 2.0:
                self._notify_mission_progress(
                    30 + int((idx / total_steps) * 50),
                    f"⚠️ Évitement d'obstacle LiDAR ({self.obstacle_distance:.1f}m)",
                )
                self.drive(15, 25, ms=500)
                self.wait_for_done([1, 2, 3, 4], timeout=1.2)
                self.drive(15, -25, ms=500)
                self.wait_for_done([1, 2, 3, 4], timeout=1.2)

            slope_corr = int(max(-20, min(20, -self.terrain_roll * 1.5)))
            d_cmd = max(-100, min(100, base_d + slope_corr))
            pct = 30 + int(((idx + 1) / total_steps) * 50)
            self._notify_mission_progress(
                pct,
                f"Guidage RTK Pas {idx + 1}/{total_steps} : DRIVE:{base_a},{d_cmd}:{ms_step}",
            )
            self.drive(base_a, d_cmd, ms=ms_step)
            self.wait_for_done([1, 2, 3, 4], timeout=(ms_step / 1000.0) + 1.0)

        # Arrêt sécurisé des outils : STOP:8, STOP:6 AVANT relevage M5
        self._notify_mission_progress(86, "Arrêt Semoir (STOP:8) et Fraise (STOP:6)")
        self.stop_motor(8)
        self.stop_motor(6)
        time.sleep(0.3)

        self._notify_mission_progress(93, "Relevage des dents (PULSE:5:-50:4000)")
        self.pulse(5, -50, 4000)
        self.wait_for_done([5], timeout=6.0)
        return True
