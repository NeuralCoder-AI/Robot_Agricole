"""Simulateur ESP32-S3 Firmware v2 pour Robot Agricole.

Simule une boucle temps réel à 200 Hz (période 5 ms) et reproduit fidèlement
le protocole texte ASCII UART1 (115200 bauds, 8N1, RX=GPIO 1, TX=GPIO 2,
BTS7960 Enable=GPIO 38).
"""

import json
import logging
import math
import os
import queue
import re
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

logger = logging.getLogger("ESP32_Simulator")
BASE_DIR: Path = Path(__file__).resolve().parent


class MotorState:
    """État individuel d'un des 8 moteurs BTS7960."""

    def __init__(self, index: int):
        self.index: int = index  # 1 à 8
        self.target_speed: float = 0.0  # -100.0 à +100.0 %
        self.actual_speed: float = 0.0  # -100.0 à +100.0 %
        self.effective_pwm: float = 0.0  # PWM calculé selon MIN/MAX
        self.timer_remaining_s: float = 0.0  # Minuterie en cours (0 = permanent)
        self.has_active_timer: bool = False
        self.continuous_run_s: float = 0.0  # Temps continu de rotation
        self.dead_time_remaining_s: float = 0.0  # Temps mort (80 ms) lors d'inversion

        # Paramètres NVS
        self.inv: int = 0  # 0 ou 1
        self.min_p: int = 0  # 0 à 90
        self.max_p: int = 100  # MIN à 100
        self.ramp: int = 100  # 1 à 1000 %/s
        self.tmax: float = 0.0  # 0 à 86400 s (0 = désactivé)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "inv": self.inv,
            "min_p": self.min_p,
            "max_p": self.max_p,
            "ramp": self.ramp,
            "tmax": self.tmax,
        }

    def from_dict(self, data: Dict[str, Any]) -> None:
        self.inv = int(data.get("inv", 0))
        self.min_p = int(data.get("min_p", 0))
        self.max_p = int(data.get("max_p", 100))
        self.ramp = int(data.get("ramp", 100))
        self.tmax = float(data.get("tmax", 0.0))


class VirtualESP32FirmwareV2:
    """Jumeau numérique de l'ESP32-S3 Firmware v2.

    Gère 8 moteurs BTS7960, boucle 200 Hz, rampe, temps mort 80ms, NVS,
    watchdog UWDT, télémétrie #T, actions temporisées #D n et checksum XOR *HH.
    Fournit une interface compatible pyserial (write, readline, in_waiting, close).
    """

    NVS_FILENAME = "nvs_robot.json"

    def __init__(self, nvs_path: Optional[str] = None):
        self.nvs_path = nvs_path or str(BASE_DIR / self.NVS_FILENAME)
        self.is_open: bool = True
        self.armed: int = 0  # Robot démarre TOUJOURS en arrêt d'urgence

        # 8 Moteurs M1 à M8
        self.motors: List[MotorState] = [MotorState(i) for i in range(1, 9)]

        # Paramètres globaux NVS
        self.wdt_ms: int = 2000
        self.uwdt_ms: int = 1000
        self.autosave: int = 1

        # Horloges et Watchdogs
        self.start_time: float = time.perf_counter()
        self.last_cmd_time: float = time.perf_counter()
        self.last_cfg_change_time: float = 0.0
        self.autosave_pending: bool = False
        self.save_requested_pending: bool = False
        self.nvs_saved: int = 1

        # Télémétrie
        self.telem_interval_s: float = 0.0  # 0 = désactivé
        self.last_telem_time: float = 0.0

        # Files UART (compatible streaming série)
        self.rx_buffer: str = ""
        self.tx_queue: queue.Queue[bytes] = queue.Queue()
        self.lock = threading.RLock()

        # Statistiques boucle 200 Hz
        self.loop_count: int = 0
        self.last_loop_perf_time: float = time.perf_counter()
        self.loop_jitter_ms: float = 0.05

        # Chargement configuration NVS
        self.load_nvs()

        # Thread de simulation 200 Hz
        self._running: bool = True
        self.sim_thread = threading.Thread(target=self._realtime_loop, name="ESP32_200Hz_Loop", daemon=True)
        self.sim_thread.start()

    # --------------------------------------------------------------------------
    # Interface série compatible pyserial
    # --------------------------------------------------------------------------
    def write(self, data: bytes) -> int:
        """Écrit des octets sur le port série virtuel (UART1 RX)."""
        if not self.is_open:
            raise RuntimeError("Port série virtuel fermé")
        text = data.decode("ascii", errors="replace")
        with self.lock:
            self.rx_buffer += text
            while "\n" in self.rx_buffer:
                line, self.rx_buffer = self.rx_buffer.split("\n", 1)
                line = line.rstrip("\r")
                self._handle_incoming_raw_line(line)
        return len(data)

    def readline(self, timeout: Optional[float] = 0.1) -> bytes:
        """Lit une ligne complète (terminée par \r\n) depuis le buffer UART1 TX."""
        try:
            return self.tx_queue.get(timeout=timeout)
        except queue.Empty:
            return b""

    @property
    def in_waiting(self) -> int:
        """Nombre approximatif d'octets en attente dans le buffer TX."""
        return self.tx_queue.qsize() * 32

    def close(self) -> None:
        """Arrête la boucle de simulation."""
        self._running = False
        self.is_open = False
        if self.sim_thread.is_alive():
            self.sim_thread.join(timeout=0.5)

    def flush(self) -> None:
        pass

    # --------------------------------------------------------------------------
    # Transmission / Réception
    # --------------------------------------------------------------------------
    def _emit_tx(self, line: str) -> None:
        """Émet une ligne ASCII terminée par \r\n vers l'UART TX."""
        payload = (line + "\r\n").encode("ascii")
        self.tx_queue.put(payload)

    def send_line(self, raw_line: str) -> List[str]:
        """Méthode directe synchrone de test / simulation pour envoyer une commande."""
        with self.lock:
            return self._handle_incoming_raw_line(raw_line)

    # --------------------------------------------------------------------------
    # Checksum XOR *HH
    # --------------------------------------------------------------------------
    @staticmethod
    def calculate_checksum(cmd_without_star: str) -> str:
        """Calcule le XOR hexadécimal sur 2 caractères en majuscules."""
        crc = 0
        for ch in cmd_without_star:
            crc ^= ord(ch)
        return f"{crc:02X}"

    def _verify_and_strip_checksum(self, raw_line: str) -> Optional[str]:
        """Vérifie le checksum optionnel *HH.

        Retourne la commande nettoyée, ou None si erreur CRC.
        """
        if "*" not in raw_line:
            return raw_line

        parts = raw_line.split("*", 1)
        payload = parts[0]
        provided_crc = parts[1].strip()

        expected_crc = self.calculate_checksum(payload)
        if provided_crc.upper() != expected_crc:
            return None
        return payload

    # --------------------------------------------------------------------------
    # Traitement des lignes entrantes
    # --------------------------------------------------------------------------
    def _handle_incoming_raw_line(self, raw_line: str) -> List[str]:
        responses: List[str] = []

        # 1. Lignes vides ignorées
        clean_raw = raw_line.strip()
        if not clean_raw:
            return responses

        # 2. Longueur maximale de 96 caractères
        if len(clean_raw) > 96:
            err = "ERR: LONG"
            self._emit_tx(err)
            responses.append(err)
            return responses

        # 3. Checksum *HH
        cmd_payload = self._verify_and_strip_checksum(clean_raw)
        if cmd_payload is None:
            err = "ERR: CRC"
            self._emit_tx(err)
            responses.append(err)
            return responses

        # 4. Watchdog UART : toute commande valide rafraîchit l'horloge de réception
        self.last_cmd_time = time.perf_counter()

        # 5. Parsing insensible à la casse
        cmd = cmd_payload.strip().upper()
        if not cmd:
            return responses

        res = self._dispatch_command(cmd)
        if res:
            self._emit_tx(res)
            responses.append(res)
        return responses

    # --------------------------------------------------------------------------
    # Décodeur des commandes
    # --------------------------------------------------------------------------
    def _dispatch_command(self, cmd: str) -> str:
        # Commandes sans arguments
        if cmd == "ARM":
            self.armed = 1
            for m in self.motors:
                m.target_speed = 0.0
            return "OK ARMED"

        if cmd == "ESTOP":
            self.armed = 0
            for m in self.motors:
                m.target_speed = 0.0
                m.actual_speed = 0.0
                m.effective_pwm = 0.0
                m.timer_remaining_s = 0.0
                m.has_active_timer = False
                m.continuous_run_s = 0.0
                m.dead_time_remaining_s = 0.0
            return "OK ESTOP"

        if cmd == "PING":
            return "PONG"

        if cmd == "STOP":
            for m in self.motors:
                m.target_speed = 0.0
                m.actual_speed = 0.0
                m.effective_pwm = 0.0
                m.timer_remaining_s = 0.0
                m.has_active_timer = False
                m.continuous_run_s = 0.0
                m.dead_time_remaining_s = 0.0
            return "OK STOP"

        if cmd.startswith("STOP:"):
            parts = cmd.split(":")
            if len(parts) != 2:
                return "ERR: ARG"
            try:
                n = int(parts[1])
            except ValueError:
                return "ERR: ARG"
            if n < 1 or n > 8:
                return "ERR: MOTOR"
            m = self.motors[n - 1]
            m.target_speed = 0.0
            m.actual_speed = 0.0
            m.effective_pwm = 0.0
            m.timer_remaining_s = 0.0
            m.has_active_timer = False
            m.continuous_run_s = 0.0
            m.dead_time_remaining_s = 0.0
            return f"OK STOP M{n}"

        if cmd == "GCFG":
            return f"GCFG WDT:{self.wdt_ms} UWDT:{self.uwdt_ms} AUTOSAVE:{self.autosave} SAVED:{self.nvs_saved} saved={self.nvs_saved}"

        if cmd.startswith("CFG:"):
            parts = cmd.split(":")
            if len(parts) != 2:
                return "ERR: ARG"
            try:
                n = int(parts[1])
            except ValueError:
                return "ERR: ARG"
            if n < 1 or n > 8:
                return "ERR: MOTOR"
            m = self.motors[n - 1]
            return f"M{n} INV:{m.inv} MIN:{m.min_p} MAX:{m.max_p} RAMP:{m.ramp} TMAX:{m.tmax}"

        if cmd == "STATUS":
            speeds = ",".join(str(int(round(m.actual_speed))) for m in self.motors)
            return f"STATUS ARMED:{self.armed} M:{speeds}"

        if cmd == "PERF":
            return f"PERF LOOP:200Hz DT:5.0ms JITTER:{self.loop_jitter_ms:.2f}ms"

        if cmd == "VER":
            return "VER 2.1 ROBOT-AGRICOLE ESP32-S3 motors=8"

        if cmd == "HELP":
            return "HELP: ARM ESTOP PING STOP DRIVE TANK Mn PULSE INV MIN MAX RAMP TMAX WDT UWDT AUTOSAVE CFG GCFG STATUS PERF VER TELEM SAVE DEFAULTS REBOOT"

        if cmd == "SAVE":
            if self._is_any_motor_running():
                self.save_requested_pending = True
                return "OK SAVE_PENDING"
            self.save_nvs()
            return "OK SAVE"

        if cmd == "DEFAULTS":
            if self._is_any_motor_running():
                return "ERR: BUSY"
            self.reset_defaults()
            self.save_nvs()
            return "OK DEFAULTS"

        if cmd == "REBOOT":
            self.armed = 0
            for m in self.motors:
                m.target_speed = 0.0
                m.actual_speed = 0.0
                m.effective_pwm = 0.0
                m.timer_remaining_s = 0.0
                m.has_active_timer = False
            self.load_nvs()
            return "OK REBOOT"

        # Commandes avec paramètres ":"
        tokens = cmd.split(":")
        header = tokens[0]

        # DRIVE:a,d ou DRIVE:a,d:ms
        if header == "DRIVE":
            if len(tokens) < 2 or len(tokens) > 3:
                return "ERR: ARG"
            sub = tokens[1].split(",")
            if len(sub) != 2:
                return "ERR: ARG"
            try:
                a = float(sub[0])
                d = float(sub[1])
            except ValueError:
                return "ERR: ARG"
            # Entiers exigés pour les commandes moteurs directes
            if not sub[0].lstrip("-+").isdigit() or not sub[1].lstrip("-+").isdigit():
                return "ERR: RANGE"

            ms_val: Optional[int] = None
            if len(tokens) == 3:
                try:
                    ms_val = int(tokens[2])
                except ValueError:
                    return "ERR: ARG"
                if ms_val < 1 or ms_val > 3600000:
                    return "ERR: RANGE"

            if self.armed == 0:
                return "ERR: ESTOP"

            left_raw = a + d
            right_raw = a - d

            left = self._normalize_speed(left_raw)
            right = self._normalize_speed(right_raw)

            # M1, M3 = Roues gauches ; M2, M4 = Roues droites
            self._apply_motor_speed(1, left, ms_val)
            self._apply_motor_speed(3, left, ms_val)
            self._apply_motor_speed(2, right, ms_val)
            self._apply_motor_speed(4, right, ms_val)

            if ms_val is not None:
                return f"OK DRIVE {int(left)},{int(right)} {ms_val}"
            return f"OK DRIVE {int(left)},{int(right)}"

        # TANK:l,r ou TANK:l,r:ms
        if header == "TANK":
            if len(tokens) < 2 or len(tokens) > 3:
                return "ERR: ARG"
            sub = tokens[1].split(",")
            if len(sub) != 2:
                return "ERR: ARG"
            if not sub[0].lstrip("-+").isdigit() or not sub[1].lstrip("-+").isdigit():
                return "ERR: RANGE"
            try:
                l_val = float(sub[0])
                r_val = float(sub[1])
            except ValueError:
                return "ERR: ARG"

            ms_val = None
            if len(tokens) == 3:
                try:
                    ms_val = int(tokens[2])
                except ValueError:
                    return "ERR: ARG"
                if ms_val < 1 or ms_val > 3600000:
                    return "ERR: RANGE"

            if self.armed == 0:
                return "ERR: ESTOP"

            l_norm = self._normalize_speed(l_val)
            r_norm = self._normalize_speed(r_val)

            self._apply_motor_speed(1, l_norm, ms_val)
            self._apply_motor_speed(3, l_norm, ms_val)
            self._apply_motor_speed(2, r_norm, ms_val)
            self._apply_motor_speed(4, r_norm, ms_val)

            if ms_val is not None:
                return f"OK TANK {int(l_norm)},{int(r_norm)} {ms_val}"
            return f"OK TANK {int(l_norm)},{int(r_norm)}"

        # PULSE:n:v:ms -> durée obligatoire
        if header == "PULSE":
            if len(tokens) != 4:
                return "ERR: ARG"
            try:
                n = int(tokens[1])
            except ValueError:
                return "ERR: ARG"
            if n < 1 or n > 8:
                return "ERR: MOTOR"
            if not tokens[2].lstrip("-+").isdigit():
                return "ERR: RANGE"
            try:
                v = int(tokens[2])
                ms_val = int(tokens[3])
            except ValueError:
                return "ERR: ARG"
            if ms_val < 1 or ms_val > 3600000:
                return "ERR: RANGE"

            if self.armed == 0:
                return "ERR: ESTOP"

            v_norm = self._normalize_speed(v)
            self._apply_motor_speed(n, v_norm, ms_val)
            return f"OK M{n} {int(v_norm)} {ms_val}"

        # Mn:v ou Mn:v:ms
        m_match = re.match(r"^M([0-9]+)$", header)
        if m_match:
            n = int(m_match.group(1))
            if n < 1 or n > 8:
                return "ERR: MOTOR"
            if len(tokens) < 2 or len(tokens) > 3:
                return "ERR: ARG"
            if not tokens[1].lstrip("-+").isdigit():
                return "ERR: RANGE"
            try:
                v = int(tokens[1])
            except ValueError:
                return "ERR: ARG"

            ms_val = None
            if len(tokens) == 3:
                try:
                    ms_val = int(tokens[2])
                except ValueError:
                    return "ERR: ARG"
                if ms_val < 1 or ms_val > 3600000:
                    return "ERR: RANGE"

            if self.armed == 0:
                return "ERR: ESTOP"

            v_norm = self._normalize_speed(v)
            self._apply_motor_speed(n, v_norm, ms_val)
            if ms_val is not None:
                return f"OK M{n} {int(v_norm)} {ms_val}"
            return f"OK M{n} {int(v_norm)}"

        # INV:n:b (b=0 ou 1)
        if header == "INV":
            if len(tokens) != 3:
                return "ERR: ARG"
            try:
                n = int(tokens[1])
                b = int(tokens[2])
            except ValueError:
                return "ERR: ARG"
            if n < 1 or n > 8:
                return "ERR: MOTOR"
            if b not in (0, 1):
                return "ERR: RANGE"
            if self._is_motor_running(n):
                return "ERR: BUSY"
            self.motors[n - 1].inv = b
            self._mark_cfg_modified()
            return f"OK INV {n} {b}"

        # MIN:n:p (0 à 90, <= MAX)
        if header == "MIN":
            if len(tokens) != 3:
                return "ERR: ARG"
            try:
                n = int(tokens[1])
                p = int(tokens[2])
            except ValueError:
                return "ERR: ARG"
            if n < 1 or n > 8:
                return "ERR: MOTOR"
            if p < 0 or p > 90 or p > self.motors[n - 1].max_p:
                return "ERR: RANGE"
            self.motors[n - 1].min_p = p
            self._mark_cfg_modified()
            return f"OK MIN {n} {p}"

        # MAX:n:p (MIN à 100)
        if header == "MAX":
            if len(tokens) != 3:
                return "ERR: ARG"
            try:
                n = int(tokens[1])
                p = int(tokens[2])
            except ValueError:
                return "ERR: ARG"
            if n < 1 or n > 8:
                return "ERR: MOTOR"
            if p < self.motors[n - 1].min_p or p > 100:
                return "ERR: RANGE"
            self.motors[n - 1].max_p = p
            self._mark_cfg_modified()
            return f"OK MAX {n} {p}"

        # RAMP:n:r (1 à 1000 %/s)
        if header == "RAMP":
            if len(tokens) != 3:
                return "ERR: ARG"
            try:
                n = int(tokens[1])
                r = int(tokens[2])
            except ValueError:
                return "ERR: ARG"
            if n < 1 or n > 8:
                return "ERR: MOTOR"
            if r < 1 or r > 1000:
                return "ERR: RANGE"
            self.motors[n - 1].ramp = r
            self._mark_cfg_modified()
            return f"OK RAMP {n} {r}"

        # TMAX:n:s (0 à 86400 s, décimales permises ex: 2.5)
        if header == "TMAX":
            if len(tokens) != 3:
                return "ERR: ARG"
            try:
                n = int(tokens[1])
                s = float(tokens[2])
            except ValueError:
                return "ERR: ARG"
            if n < 1 or n > 8:
                return "ERR: MOTOR"
            if s < 0 or s > 86400:
                return "ERR: RANGE"
            self.motors[n - 1].tmax = s
            self._mark_cfg_modified()
            # Format entier si entier, sinon float
            s_str = f"{s:g}"
            return f"OK TMAX {n} {s_str}"

        # WDT:ms (0 ou 100 à 60000)
        if header == "WDT":
            if len(tokens) != 2:
                return "ERR: ARG"
            try:
                ms_val = int(tokens[1])
            except ValueError:
                return "ERR: ARG"
            if ms_val != 0 and (ms_val < 100 or ms_val > 60000):
                return "ERR: RANGE"
            self.wdt_ms = ms_val
            self._mark_cfg_modified()
            return f"OK WDT {ms_val}"

        # UWDT:ms (0 ou 100 à 60000)
        if header == "UWDT":
            if len(tokens) != 2:
                return "ERR: ARG"
            try:
                ms_val = int(tokens[1])
            except ValueError:
                return "ERR: ARG"
            if ms_val != 0 and (ms_val < 100 or ms_val > 60000):
                return "ERR: RANGE"
            self.uwdt_ms = ms_val
            self._mark_cfg_modified()
            return f"OK UWDT {ms_val}"

        # AUTOSAVE:b (0 ou 1)
        if header == "AUTOSAVE":
            if len(tokens) != 2:
                return "ERR: ARG"
            try:
                b = int(tokens[1])
            except ValueError:
                return "ERR: ARG"
            if b not in (0, 1):
                return "ERR: RANGE"
            self.autosave = b
            return f"OK AUTOSAVE {b}"

        # TELEM:ms (0 ou 20 à 60000)
        if header == "TELEM":
            if len(tokens) != 2:
                return "ERR: ARG"
            try:
                ms_val = int(tokens[1])
            except ValueError:
                return "ERR: ARG"
            if ms_val != 0 and (ms_val < 20 or ms_val > 60000):
                return "ERR: RANGE"
            self.telem_interval_s = (ms_val / 1000.0) if ms_val > 0 else 0.0
            return f"OK TELEM {ms_val}"

        return "ERR: UNKNOWN"

    # --------------------------------------------------------------------------
    # Utilitaires de commande moteur
    # --------------------------------------------------------------------------
    @staticmethod
    def _normalize_speed(val: float) -> float:
        """Toute valeur strictement comprise entre -1 et +1 vaut 0 ; au-delà de ±100, écrêtée à ±100."""
        if -1.0 < val < 1.0:
            return 0.0
        if val > 100.0:
            return 100.0
        if val < -100.0:
            return -100.0
        return float(val)

    def _apply_motor_speed(self, motor_idx: int, speed: float, duration_ms: Optional[int]) -> None:
        m = self.motors[motor_idx - 1]
        m.target_speed = speed
        if duration_ms is not None and duration_ms > 0:
            m.timer_remaining_s = duration_ms / 1000.0
            m.has_active_timer = True
        else:
            m.timer_remaining_s = 0.0
            m.has_active_timer = False

    def _is_motor_running(self, motor_idx: int) -> bool:
        m = self.motors[motor_idx - 1]
        return abs(m.actual_speed) > 0.01 or abs(m.target_speed) > 0.01

    def _is_any_motor_running(self) -> bool:
        return any(self._is_motor_running(i) for i in range(1, 9))

    def _mark_cfg_modified(self) -> None:
        self.last_cfg_change_time = time.perf_counter()
        self.nvs_saved = 0
        if self.autosave:
            self.autosave_pending = True

    # --------------------------------------------------------------------------
    # Boucle temps réel 200 Hz (période 5 ms)
    # --------------------------------------------------------------------------
    def _realtime_loop(self) -> None:
        period_s = 0.005  # 5 ms = 200 Hz
        next_tick = time.perf_counter()

        while self._running:
            now = time.perf_counter()
            dt = period_s

            with self.lock:
                self._update_physics_and_controllers(dt, now)

            next_tick += period_s
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # Rattrapage léger en cas de dépassement
                next_tick = time.perf_counter()

    def _update_physics_and_controllers(self, dt: float, now: float) -> None:
        # 1. Watchdog UWDT : coupe immédiatement les moteurs sans minuterie si aucune commande reçue
        if self.armed == 1 and self.uwdt_ms > 0:
            if (now - self.last_cmd_time) > (self.uwdt_ms / 1000.0):
                # Coupe les moteurs sans minuterie active
                for m in self.motors:
                    if not m.has_active_timer and (abs(m.target_speed) > 0.01 or abs(m.actual_speed) > 0.01):
                        m.target_speed = 0.0
                        m.actual_speed = 0.0
                        m.effective_pwm = 0.0

        # 2. Mise à jour de chaque moteur
        for m in self.motors:
            # Gestion de l'action temporisée (:ms)
            if m.has_active_timer:
                m.timer_remaining_s -= dt
                if m.timer_remaining_s <= 0.0:
                    m.timer_remaining_s = 0.0
                    m.has_active_timer = False
                    # À l'échéance, la consigne revient à 0 en suivant la rampe RAMP
                    m.target_speed = 0.0
                    # Émission asynchrone #D n
                    self._emit_tx(f"#D {m.index}")

            # Gestion de TMAX (temps continu de rotation maximal)
            if abs(m.actual_speed) > 0.01:
                m.continuous_run_s += dt
                if m.tmax > 0.0 and m.continuous_run_s >= m.tmax:
                    # Ramène la consigne à 0 avec la rampe
                    m.target_speed = 0.0
            else:
                m.continuous_run_s = 0.0

            # Gestion du temps mort de 80 ms lors d'un changement de sens
            # Sign inversion detected if actual and target have opposite signs (and neither is zero)
            sign_actual = 1 if m.actual_speed > 0.01 else (-1 if m.actual_speed < -0.01 else 0)
            sign_target = 1 if m.target_speed > 0.01 else (-1 if m.target_speed < -0.01 else 0)

            if sign_actual != 0 and sign_target != 0 and sign_actual != sign_target:
                # Doit d'abord revenir à 0, puis observer 80 ms de temps mort
                if abs(m.actual_speed) > 0.01:
                    # Décélération vers 0
                    step = m.ramp * dt
                    if abs(m.actual_speed) <= step:
                        m.actual_speed = 0.0
                        m.dead_time_remaining_s = 0.080  # 80 ms
                    else:
                        m.actual_speed -= math.copysign(step, m.actual_speed)
                else:
                    m.dead_time_remaining_s = 0.080
            elif m.dead_time_remaining_s > 0.0:
                m.dead_time_remaining_s -= dt
                if m.dead_time_remaining_s < 0.0:
                    m.dead_time_remaining_s = 0.0
                m.actual_speed = 0.0
            else:
                # Évolution normale suivant la rampe RAMP (%/s)
                step = m.ramp * dt
                diff = m.target_speed - m.actual_speed
                if abs(diff) <= step:
                    m.actual_speed = m.target_speed
                else:
                    m.actual_speed += math.copysign(step, diff)

            # Calcul du PWM effectif : PWM% = MIN + (MAX - MIN) * |v| / 100
            abs_v = abs(m.actual_speed)
            if abs_v < 0.01 or self.armed == 0:
                m.effective_pwm = 0.0
            else:
                m.effective_pwm = m.min_p + (m.max_p - m.min_p) * (abs_v / 100.0)

        # 3. Télémétrie périodique (#T <armé> <m1>,...,<m8> <ms>)
        if self.telem_interval_s > 0.0:
            if (now - self.last_telem_time) >= self.telem_interval_s:
                self.last_telem_time = now
                uptime_ms = int((now - self.start_time) * 1000)
                speeds_str = ",".join(str(int(round(m.actual_speed))) for m in self.motors)
                self._emit_tx(f"#T {self.armed} {speeds_str} {uptime_ms}")

        # 4. Sauvegarde automatique (AUTOSAVE: 3s après dernière modif dès que moteurs à l'arrêt)
        if (self.autosave_pending or self.save_requested_pending) and not self._is_any_motor_running():
            if self.save_requested_pending or (self.autosave and (now - self.last_cfg_change_time) >= 3.0):
                self.save_nvs()
                self.autosave_pending = False
                self.save_requested_pending = False

    # --------------------------------------------------------------------------
    # Persistance NVS (nvs_robot.json simulant ESP32 Preferences)
    # --------------------------------------------------------------------------
    def reset_defaults(self) -> None:
        """Réinitialise les paramètres par défaut."""
        self.wdt_ms = 2000
        self.uwdt_ms = 1000
        self.autosave = 1
        for m in self.motors:
            m.inv = 0
            m.min_p = 0
            m.max_p = 100
            m.ramp = 100
            m.tmax = 0.0

    def save_nvs(self) -> None:
        """Sauvegarde les paramètres dans le fichier JSON."""
        self.nvs_saved = 1
        data = {
            "global": {
                "wdt_ms": self.wdt_ms,
                "uwdt_ms": self.uwdt_ms,
                "autosave": self.autosave,
            },
            "motors": {f"M{m.index}": m.to_dict() for m in self.motors},
        }
        try:
            with open(self.nvs_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            logger.info("NVS sauvegardée dans %s", self.nvs_path)
        except Exception as e:
            logger.error("Erreur de sauvegarde NVS : %s", e)

    def load_nvs(self) -> None:
        """Charge les paramètres depuis le fichier JSON s'il existe."""
        if not os.path.isfile(self.nvs_path) or os.path.getsize(self.nvs_path) == 0:
            self.reset_defaults()
            self.save_nvs()
            return
        try:
            with open(self.nvs_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            g = data.get("global", {})
            self.wdt_ms = int(g.get("wdt_ms", 2000))
            self.uwdt_ms = int(g.get("uwdt_ms", 1000))
            self.autosave = int(g.get("autosave", 1))

            motors_data = data.get("motors", {})
            for m in self.motors:
                key = f"M{m.index}"
                if key in motors_data:
                    m.from_dict(motors_data[key])
            logger.info("NVS chargée depuis %s", self.nvs_path)
        except Exception as e:
            logger.error("Erreur de chargement NVS : %s", e)
            self.reset_defaults()
