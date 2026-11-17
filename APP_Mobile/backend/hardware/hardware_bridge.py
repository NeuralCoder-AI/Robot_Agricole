"""Module de Communication Matérielle Hardware-in-the-Loop (ESP32-S3 Firmware v2).

Supporte 3 modes de liaison temps réel avec le robot physique (8 × BTS7960 M1..M8) :
- WIFI_HTTP     : Serveur Web de l'ESP32-S3 (IP par défaut 192.168.4.1, port 80, maintien WDT 400 ms)
- SOCKET_TCP_WS : Socket TCP brute (socket.AF_INET, socket.SOCK_STREAM) ou WebSocket (ws://host:port)
                  avec fins de ligne \r\n (max 96 caractères)
- USB_UART      : Liaison Série UART1 (pyserial, /dev/ttyUSB0, /dev/ttyACM0, 115200..921600 bauds, 8N1,
                  sans redémarrage intempestif DTR/RTS, fins de ligne \r\n)

Respect strict du protocole Firmware v2 :
- Entretien silencieux du Watchdog (UWDT = 1000 ms / WDT = 2000 ms) toutes les 300 ms (PING -> PONG en silence)
- Option Checksum XOR (*HH) : PING*10, M1:50*43, ARM*5E, ESTOP*5D
- Fil de lecture dédié permanent pour :
  * Réponses directes : OK ..., ERR: UNKNOWN|MOTOR|ARG|RANGE|ESTOP|BUSY|CRC|LONG, STATUS, CFG, GCFG, PERF, VER
  * Fins d'action temporisée : #D 1 à #D 8
  * Télémétrie périodique : #T <armé> <m1>,...,<m8> <ms> (filtrée des logs console)
- Filtrage strict : PING, PONG et #T ne sont JAMAIS émis dans les journaux de la console !
"""

import enum
import glob
import logging
import os
import queue
import socket
import threading
import time
from typing import Any, Callable, Dict, List, Optional

try:
    import httpx
except ImportError:
    httpx = None  # type: ignore

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None  # type: ignore

logger = logging.getLogger("HardwareBridge")


class ExecutionMode(str, enum.Enum):
    SIMULATION_ONLY = "SIMULATION_ONLY"
    SIM_ONLY = "SIM_ONLY"
    COUPLED = "COUPLED"
    REAL_ZERO_SIM = "REAL_ZERO_SIM"


class ConnectionTransport(str, enum.Enum):
    WIFI_HTTP = "WIFI_HTTP"
    SOCKET_TCP_WS = "SOCKET_TCP_WS"
    USB_UART = "USB_UART"


def calculate_xor_checksum(cmd_without_star: str) -> str:
    """Calcule le checksum XOR hexadécimal sur 2 caractères majuscules du Firmware v2.

    Exemples vérifiés :
    - 'PING'  -> '10' ('PING*10')
    - 'M1:50' -> '43' ('M1:50*43')
    - 'ARM'   -> '5E' ('ARM*5E')
    - 'ESTOP' -> '5D' ('ESTOP*5D')
    """
    payload = cmd_without_star.split("*", 1)[0].strip()
    crc = 0
    for ch in payload:
        crc ^= ord(ch)
    return f"{crc:02X}"


def format_command_with_optional_crc(command: str, use_xor_checksum: bool = False) -> str:
    """Formate la commande (max 96 caractères) et ajoute *HH si demandé."""
    clean = command.strip()
    if not clean:
        return ""
    if use_xor_checksum and "*" not in clean:
        crc = calculate_xor_checksum(clean)
        clean = f"{clean}*{crc}"
    return clean[:96]


def is_console_noise_line(line: str) -> bool:
    """Retourne True si la ligne est un PING, PONG ou une trame #T à filtrer à 100 % de la console."""
    if not line:
        return True
    s = line.strip()
    if not s:
        return True
    upper_base = s.split("*", 1)[0].strip().upper()
    if upper_base in ("PING", "PONG"):
        return True
    if s.upper().startswith("PING*") or s.upper().startswith("PONG*"):
        return True
    if s.startswith("#T ") or s == "#T" or s.upper().startswith("#T "):
        return True
    return False


class ESP32HardwareBridge:
    """Gestionnaire matériel temps réel vers le vrai robot ESP32-S3 Firmware v2."""

    def __init__(
        self,
        transport: str = ConnectionTransport.WIFI_HTTP.value,
        host: str = "192.168.4.1",
        port: int = 80,
        serial_port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        use_xor_checksum: bool = False,
        username: str = "admin",
        password: str = "hyper1234",
        on_line_callback: Optional[Callable[[str], None]] = None,
        on_log_callback: Optional[Callable[[str, str], None]] = None,
    ):
        self.transport: str = transport
        self.host: str = host
        self.port: int = int(port)
        self.serial_port: str = serial_port
        self.baudrate: int = int(baudrate)
        self.use_xor_checksum: bool = bool(use_xor_checksum)
        self.username: str = username
        self.password: str = password

        self.on_line_callback = on_line_callback
        self.on_log_callback = on_log_callback

        # État de connexion vérifié (PING -> PONG + VER/STATUS)
        self.connected: bool = False
        self.connecting: bool = False
        self.last_error: str = ""
        self.firmware_version: str = ""
        self.status_line: str = ""
        self.gcfg_line: str = ""
        self.perf_line: str = ""
        self.motor_configs: Dict[int, str] = {}
        self.armed: int = 0
        self.latency_ms: float = 0.0
        self.last_pong_time: float = 0.0
        self.last_tx_time: float = 0.0

        # Objets bas-niveau
        self._serial_dev: Any = None
        self._tcp_sock: Optional[socket.socket] = None
        self._http_client: Any = None
        self._rx_buffer: str = ""

        # Synchronisation commande / réponse
        self._io_lock = threading.RLock()
        self._cmd_lock = threading.Lock()
        self._response_queue: queue.Queue[str] = queue.Queue()
        self._silence_next_pong: bool = False

        # Threads d'arrière-plan (lecture permanente + watchdog 300 ms)
        self._running: bool = True
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="ESP32BridgeReader", daemon=True
        )
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, name="ESP32BridgeWatchdog", daemon=True
        )
        self._reader_thread.start()
        self._watchdog_thread.start()

    # --------------------------------------------------------------------------
    # Scan automatique des ports série Linux / USB-UART
    # --------------------------------------------------------------------------
    @staticmethod
    def scan_serial_ports() -> List[Dict[str, str]]:
        """Scanne les ports série disponibles (/dev/ttyUSB*, /dev/ttyACM*, pyserial list_ports)."""
        ports_found: List[Dict[str, str]] = []
        seen = set()

        if serial is not None and hasattr(serial, "tools") and hasattr(serial.tools, "list_ports"):
            try:
                for p in serial.tools.list_ports.comports():
                    if p.device and p.device not in seen:
                        seen.add(p.device)
                        ports_found.append(
                            {
                                "device": p.device,
                                "description": p.description or "Port Série UART",
                            }
                        )
            except Exception:
                pass

        for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyS0"):
            for dev_path in sorted(glob.glob(pattern)):
                if dev_path not in seen:
                    seen.add(dev_path)
                    ports_found.append({"device": dev_path, "description": "Linux UART Device"})

        if not ports_found:
            default_dev = "/dev/ttyACM0" if os.path.exists("/dev/ttyACM0") else ("/dev/ttyUSB0" if os.name != "nt" else "COM3")
            ports_found.append({"device": default_dev, "description": "Port par défaut (ESP32-S3)"})

        # Priorité absolue aux ports USB réels (/dev/ttyACM*, /dev/ttyUSB*) par rapport aux ports ttyS intégrés
        def port_priority(item: Dict[str, str]) -> int:
            dev = item.get("device", "").lower()
            desc = item.get("description", "").lower()
            if "acm" in dev or "usb" in dev or "espressif" in desc or "ch340" in desc or "cp210" in desc or "serial" in desc:
                return 0
            if "ttys" in dev:
                return 2
            return 1

        ports_found.sort(key=port_priority)
        return ports_found

    # --------------------------------------------------------------------------
    # Configuration et Connexion Matérielle
    # --------------------------------------------------------------------------
    def configure(
        self,
        transport: Optional[str] = None,
        host: Optional[str] = None,
        port: Optional[int] = None,
        serial_port: Optional[str] = None,
        baudrate: Optional[int] = None,
        use_xor_checksum: Optional[bool] = None,
        username: Optional[str] = None,
        password: Optional[str] = None,
    ) -> None:
        if transport is not None:
            self.transport = str(transport).upper()
        if host is not None and str(host).strip():
            clean_h = str(host).strip().replace("http://", "").replace("https://", "").replace("ws://", "")
            clean_h = clean_h.split("/")[0].split(":")[0].strip()
            if clean_h:
                self.host = clean_h
        if port is not None:
            self.port = max(1, min(65535, int(port)))
        if serial_port is not None and str(serial_port).strip():
            self.serial_port = str(serial_port).strip()
        if baudrate is not None:
            self.baudrate = int(baudrate)
        if use_xor_checksum is not None:
            self.use_xor_checksum = bool(use_xor_checksum)
        if username is not None:
            self.username = str(username).strip()
        if password is not None:
            self.password = str(password).strip()

    def disconnect(self) -> None:
        """Ferme proprement la liaison active avec l'ESP32-S3."""
        self.connected = False
        self.connecting = False
        with self._io_lock:
            if self._serial_dev is not None:
                try:
                    self._serial_dev.close()
                except Exception:
                    pass
                self._serial_dev = None
            if self._tcp_sock is not None:
                try:
                    self._tcp_sock.close()
                except Exception:
                    pass
                self._tcp_sock = None
            if self._http_client is not None:
                try:
                    self._http_client.close()
                except Exception:
                    pass
                self._http_client = None
            self._rx_buffer = ""

    def close(self) -> None:
        self._running = False
        self.disconnect()

    def connect_and_verify(self, timeout: float = 2.0) -> Dict[str, Any]:
        """Établit la liaison (WIFI_HTTP, SOCKET_TCP_WS ou USB_UART) et vérifie l'ESP32-S3 :
        1. Échange PING -> PONG (mesure de latence ms)
        2. Lecture de VER et STATUS
        Déverrouille automatiquement la simulation couplée si connected == True.
        """
        target_port = self.serial_port
        if self.transport == ConnectionTransport.USB_UART.value and not os.path.exists(target_port):
            available = [p["device"] for p in self.scan_serial_ports() if os.path.exists(p.get("device", ""))]
            if available:
                target_port = available[0]
                self.serial_port = target_port

        # Réutilisation du port s'il est déjà ouvert pour éliminer la latence de reconnexion
        prev_ser = self._serial_dev
        can_reuse_serial = (
            self.transport == ConnectionTransport.USB_UART.value
            and prev_ser is not None
            and getattr(prev_ser, "is_open", False)
            and getattr(prev_ser, "port", None) == target_port
        )
        if not can_reuse_serial:
            self.disconnect()

        self.connecting = True
        self.last_error = ""
        t0 = time.perf_counter()

        try:
            if self.transport == ConnectionTransport.USB_UART.value:
                if not can_reuse_serial:
                    if serial is None:
                        raise RuntimeError("pyserial n'est pas installé")

                    # Configuration robuste compatible USB CDC natif ESP32-S3 (cdc_acm) et adaptateurs USB-UART
                    ser = serial.Serial()
                    ser.port = target_port
                    ser.baudrate = self.baudrate
                    ser.bytesize = serial.EIGHTBITS
                    ser.parity = serial.PARITY_NONE
                    ser.stopbits = serial.STOPBITS_ONE
                    ser.timeout = 0.10
                    ser.write_timeout = None  # None est impératif pour éviter SerialTimeoutException sur CDC-ACM Linux
                    ser.rtscts = False
                    ser.dsrdtr = False
                    ser.open()
                    ser.dtr = True   # Signale à l'ESP32-S3 CDC que l'hôte est prêt
                    ser.rts = False  # Évite tout reset intempestif
                    try:
                        ser.reset_input_buffer()
                        ser.reset_output_buffer()
                        ser.write(b"\r\n")
                    except Exception:
                        pass
                    self._serial_dev = ser
                    time.sleep(0.08)

            elif self.transport == ConnectionTransport.SOCKET_TCP_WS.value:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(timeout)
                sock.connect((self.host, self.port))
                sock.settimeout(0.08)
                self._tcp_sock = sock

            elif self.transport == ConnectionTransport.WIFI_HTTP.value:
                if httpx is not None:
                    auth_tuple = (self.username, self.password) if (self.username or self.password) else ("admin", "hyper1234")
                    default_headers = {"Connection": "close", "User-Agent": "RobotAgricole/2.2"}
                    self._http_client = httpx.Client(
                        base_url=f"http://{self.host}:{self.port}",
                        timeout=max(timeout, 3.5),
                        auth=auth_tuple,
                        trust_env=False,
                        headers=default_headers,
                    )

            # Vérification obligatoire par PING -> PONG silencieux (jusqu'à 3 tentatives pour absorber le boot UART)
            probe_timeout = max(timeout, 2.0) if self.transport == ConnectionTransport.WIFI_HTTP.value else max(min(timeout, 1.0), 0.5)
            pong_res = ""
            max_attempts = 1 if self.transport == ConnectionTransport.WIFI_HTTP.value else 3
            for attempt in range(max_attempts):
                pong_res = self.send_command("PING", timeout=probe_timeout, silent=True)
                if "PONG" in pong_res.upper() or pong_res.upper().startswith("OK"):
                    break
                time.sleep(0.10)

            if "PONG" not in pong_res.upper() and not pong_res.upper().startswith("OK"):
                raise RuntimeError(
                    f"Aucune réponse PONG de l'ESP32-S3 ({self.transport} : {pong_res or 'TIMEOUT'})"
                )

            self.latency_ms = round((time.perf_counter() - t0) * 1000.0, 1)
            self.last_pong_time = time.perf_counter()
            self.connected = True
            self.connecting = False

            # Lecture initiale de VER et STATUS (timeout selon transport)
            cmd_timeout = 2.0 if self.transport == ConnectionTransport.WIFI_HTTP.value else 0.35
            ver_res = self.send_command("VER", timeout=cmd_timeout, silent=False)
            if ver_res and not ver_res.startswith("ERR:"):
                self.firmware_version = ver_res.strip()
            else:
                self.firmware_version = "VER 2.2 ROBOT-AGRICOLE ESP32-S3"

            status_res = self.send_command("STATUS", timeout=cmd_timeout, silent=False)
            if status_res and not status_res.startswith("ERR:"):
                self.status_line = status_res.strip()
                if "ARMED:1" in status_res.upper():
                    self.armed = 1
                elif "ARMED:0" in status_res.upper():
                    self.armed = 0

            return self.get_state()

        except Exception as e:
            self.connected = False
            self.connecting = False
            self.last_error = str(e)
            self.disconnect()
            self.last_error = str(e)
            return self.get_state()

    # --------------------------------------------------------------------------
    # Envoi et Réception Matérielle (conforme Firmware v2)
    # --------------------------------------------------------------------------
    def _send_http_command(self, wire_cmd: str, timeout: float = 1.5) -> str:
        """Envoie une commande au serveur Web HTTP de l'ESP32-S3 (/api/cmd?c=..., avec Basic Auth admin:hyper1234)."""
        if httpx is None:
            raise RuntimeError("httpx non disponible")
        client = self._http_client
        auth_tuple = (self.username, self.password) if (self.username or self.password) else ("admin", "hyper1234")
        default_headers = {"Connection": "close", "User-Agent": "RobotAgricole/2.2"}
        effective_timeout = max(0.8, min(timeout, 2.0))
        if client is None:
            client = httpx.Client(
                base_url=f"http://{self.host}:{self.port}",
                timeout=effective_timeout,
                auth=auth_tuple,
                trust_env=False,
                headers=default_headers,
            )
            self._http_client = client

        # Endpoints conformes au Firmware v2 ESP32-S3 (g_server.on("/api/cmd", ...))
        endpoints = [
            ("GET", "/api/cmd", {"c": wire_cmd}),
            ("POST", "/api/cmd", {"c": wire_cmd}),
            ("GET", "/cmd", {"c": wire_cmd}),
        ]
        if wire_cmd.upper().startswith(("PING", "STATUS", "VER")):
            endpoints.append(("GET", "/api/state", {}))
        last_exc: Optional[Exception] = None
        for method, path, params in endpoints:
            try:
                if method == "GET":
                    resp = client.get(path, params=params, timeout=effective_timeout)
                elif method == "POST_RAW":
                    resp = client.post(
                        path,
                        content=str(params).encode("utf-8"),
                        headers={"Content-Type": "text/plain", "Connection": "close"},
                        timeout=effective_timeout,
                    )
                else:
                    resp = client.post(path, data=params, timeout=effective_timeout)

                # Si 401 Unauthorized, retenter automatiquement avec admin:hyper1234
                if resp.status_code == 401:
                    fallback_auth = ("admin", "hyper1234")
                    if method == "GET":
                        resp = client.get(path, params=params, timeout=effective_timeout, auth=fallback_auth)
                    else:
                        resp = client.post(path, data=params, timeout=effective_timeout, auth=fallback_auth)

                if resp.status_code == 200:
                    txt = resp.text.strip()
                    # Décodage JSON si le serveur renvoie l'état complet (/api/state)
                    if txt.startswith("{") and ("\"armed\"" in txt or "\"cur\"" in txt):
                        import json
                        try:
                            data = json.loads(txt)
                            if wire_cmd.upper().startswith("PING"):
                                return "PONG"
                            elif wire_cmd.upper().startswith("STATUS"):
                                armed_val = data.get("armed", 0)
                                cur_list = data.get("cur", [0] * 8)
                                mot_str = " ".join([f"M{i+1}={cur_list[i] if i < len(cur_list) else 0}" for i in range(8)])
                                return f"STATUS armed={armed_val} {mot_str}"
                            line = str(
                                data.get("response")
                                or data.get("status")
                                or data.get("result")
                                or "OK"
                            )
                            return line
                        except Exception:
                            pass
                    line = txt.splitlines()[0].strip() if txt else "OK"
                    return line
            except (httpx.ConnectError, httpx.ConnectTimeout) as exc:
                # L'hôte est injoignable : inutile de retenter 3 autres chemins sur la même IP morte
                raise exc
            except Exception as exc:
                last_exc = exc
                continue
        if last_exc:
            raise last_exc
        return "ERR: TIMEOUT"

    def send_command(
        self,
        command: str,
        timeout: float = 1.2,
        with_checksum: Optional[bool] = None,
        silent: bool = False,
    ) -> str:
        """Envoie une commande terminée par \\r\\n (max 96 caractères) au robot physique et attend sa réponse."""
        clean_raw = command.strip()
        if not clean_raw:
            return ""

        use_crc = self.use_xor_checksum if with_checksum is None else bool(with_checksum)
        wire_cmd = format_command_with_optional_crc(clean_raw, use_xor_checksum=use_crc)
        is_noise = silent or is_console_noise_line(wire_cmd)

        with self._cmd_lock:
            # Vider les réponses résiduelles
            while not self._response_queue.empty():
                try:
                    self._response_queue.get_nowait()
                except queue.Empty:
                    break

            t_send = time.perf_counter()
            self.last_tx_time = t_send

            # Journaliser TX uniquement si ce n'est PAS un PING/PONG/#T
            if not is_noise and self.on_log_callback:
                try:
                    self.on_log_callback("TX", wire_cmd)
                except Exception:
                    pass

            try:
                if self.transport == ConnectionTransport.WIFI_HTTP.value:
                    res = self._send_http_command(wire_cmd, timeout=timeout)
                    self.latency_ms = round((time.perf_counter() - t_send) * 1000.0, 1)
                    if "PONG" in res.upper():
                        self.last_pong_time = time.perf_counter()
                    for raw_l in res.splitlines():
                        l_clean = raw_l.strip()
                        if l_clean:
                            self._process_incoming_line(l_clean, from_http=True, force_silent=is_noise)
                    return res.strip()

                payload = (wire_cmd + "\r\n").encode("ascii", errors="replace")
                with self._io_lock:
                    if self.transport == ConnectionTransport.USB_UART.value and self._serial_dev is not None:
                        try:
                            fd = getattr(self._serial_dev, "fileno", lambda: None)()
                            if fd is not None:
                                os.write(fd, payload)
                            else:
                                self._serial_dev.write(payload)
                        except Exception as wr_err:
                            self.last_error = f"Write error: {wr_err}"
                            return "ERR: WRITE_FAIL"
                    elif self.transport == ConnectionTransport.SOCKET_TCP_WS.value and self._tcp_sock is not None:
                        self._tcp_sock.sendall(payload)
                    else:
                        return "ERR: DISCONNECTED"

                deadline = time.perf_counter() + timeout
                while time.perf_counter() < deadline:
                    rem = max(0.01, deadline - time.perf_counter())
                    try:
                        line = self._response_queue.get(timeout=rem)
                        if not is_noise and line.strip().upper().startswith("PONG"):
                            continue
                        self.latency_ms = round((time.perf_counter() - t_send) * 1000.0, 1)
                        return line
                    except queue.Empty:
                        break

                return "ERR: TIMEOUT"

            except Exception as e:
                self.last_error = str(e)
                self.connected = False
                return f"ERR: {e}"

    def _process_incoming_line(self, line: str, from_http: bool = False, force_silent: bool = False) -> None:
        """Traite chaque ligne reçue de l'ESP32-S3 (OK, ERR, #D n, #T, STATUS, CFG, GCFG, PERF, VER)."""
        clean = line.strip()
        if not clean:
            return

        upper = clean.upper()
        if upper.startswith("PONG"):
            self.last_pong_time = time.perf_counter()
            if not from_http:
                self._response_queue.put(clean)
            return

        # Mise à jour de l'état interne selon la réponse ESP32-S3
        if upper.startswith("OK ARMED") or upper == "OK ARM":
            self.armed = 1
        elif upper.startswith("OK ESTOP") or upper == "ERR: ESTOP":
            self.armed = 0
        elif upper.startswith("STATUS "):
            self.status_line = clean
            if "ARMED:1" in upper or "ARMED=1" in upper:
                self.armed = 1
            elif "ARMED:0" in upper or "ARMED=0" in upper:
                self.armed = 0
        elif upper.startswith("GCFG "):
            self.gcfg_line = clean
        elif upper.startswith("PERF "):
            self.perf_line = clean
        elif upper.startswith("VER "):
            self.firmware_version = clean
        elif len(upper) >= 3 and upper[0] == "M" and upper[1].isdigit() and "INV:" in upper:
            try:
                m_idx = int(upper.split()[0][1:])
                self.motor_configs[m_idx] = clean
            except Exception:
                pass

        # Notification au contrôleur (pour #T, #D n, etc.)
        if self.on_line_callback:
            try:
                self.on_line_callback(clean)
            except Exception:
                pass

        # Filtrage strict de la console : JAMAIS de PING, PONG ou #T dans les logs !
        if not force_silent and not is_console_noise_line(clean):
            if self.on_log_callback:
                try:
                    self.on_log_callback("RX", clean)
                except Exception:
                    pass

        # Les lignes directes (hors #T et #D asynchrones) réveillent send_command
        if not from_http and not clean.startswith("#T ") and not clean.startswith("#D "):
            self._response_queue.put(clean)

    def _reader_loop(self) -> None:
        """Fil de lecture dédié permanent pour USB_UART et SOCKET_TCP_WS."""
        while self._running:
            if (not self.connected and not self.connecting) or self.transport == ConnectionTransport.WIFI_HTTP.value:
                time.sleep(0.02)
                continue

            try:
                chunk = b""
                if self.transport == ConnectionTransport.USB_UART.value:
                    if self._serial_dev is not None:
                        waiting = getattr(self._serial_dev, "in_waiting", 0)
                        if waiting > 0:
                            with self._io_lock:
                                chunk = self._serial_dev.read(waiting)
                        else:
                            time.sleep(0.003)
                    else:
                        time.sleep(0.01)
                elif self.transport == ConnectionTransport.SOCKET_TCP_WS.value and self._tcp_sock is not None:
                    with self._io_lock:
                        try:
                            chunk = self._tcp_sock.recv(512)
                            if chunk == b"":
                                # Connexion socket fermée par l'ESP32
                                self.connected = False
                                self.last_error = "Socket fermée par l'ESP32-S3"
                                continue
                        except socket.timeout:
                            chunk = b""

                if chunk:
                    text = chunk.decode("ascii", errors="replace")
                    self._rx_buffer += text
                    while "\n" in self._rx_buffer:
                        raw_line, self._rx_buffer = self._rx_buffer.split("\n", 1)
                        clean_line = raw_line.rstrip("\r").strip()
                        if clean_line:
                            self._process_incoming_line(clean_line)
                else:
                    time.sleep(0.01)

            except Exception as e:
                if self.connected:
                    logger.debug("Erreur lecture matérielle ESP32 : %s", e)
                time.sleep(0.05)

    def _watchdog_loop(self) -> None:
        """Entretien silencieux du Watchdog (UWDT = 1000 ms / Web WDT = 2000 ms) :
        Envoie PING\\r\\n toutes les 300 ms lorsqu'aucune autre commande n'est envoyée,
        et lit le PONG en silence sans jamais polluer la console.
        """
        while self._running:
            time.sleep(0.30)  # 300 ms (< UWDT 1000 ms et Web WDT 400 ms)
            if not self._running or not self.connected or self.connecting:
                continue
            idle_s = time.perf_counter() - self.last_tx_time
            if idle_s >= 0.28:
                try:
                    res = self.send_command("PING", timeout=0.6, silent=True)
                    if res.startswith("ERR: DISCONNECTED"):
                        self.connected = False
                except Exception:
                    pass

    def get_state(self) -> Dict[str, Any]:
        return {
            "connected": self.connected,
            "connecting": self.connecting,
            "transport": self.transport,
            "host": self.host,
            "port": self.port,
            "username": self.username,
            "serial_port": self.serial_port,
            "baudrate": self.baudrate,
            "use_xor_checksum": self.use_xor_checksum,
            "firmware_version": self.firmware_version,
            "status_line": self.status_line,
            "gcfg_line": self.gcfg_line,
            "perf_line": self.perf_line,
            "motor_configs": self.motor_configs,
            "armed": self.armed,
            "latency_ms": self.latency_ms,
            "last_error": self.last_error,
            "available_serial_ports": self.scan_serial_ports(),
        }
