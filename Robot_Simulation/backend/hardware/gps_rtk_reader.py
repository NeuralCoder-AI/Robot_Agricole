"""backend/hardware/gps_rtk_reader.py
Lecteur et décodeur NMEA 0183 sans dépendance externe pour récepteur GPS-RTK u-blox ZED-F9P.

Aligné sur le document d'architecture « architecture_integration_GPS_RTK_vision.docx » :
- Interface : USB (CDC) ou UART sur HP t530 Ubuntu Server (/dev/ttyACM* / /dev/ttyUSB*) à 38 400 bauds (8N1),
  cadence 1 à 10 Hz (jusqu'à 20 Hz configurable via u-center).
- Trames NMEA exploitées (vérification stricte checksum XOR `$...*HH`) :
  * GGA ($GNGGA / $GPGGA) : latitude, longitude, altitude, qualité fix, nb satellites, HDOP.
  * RMC ($GNRMC / $GPRMC) : cap sol (COG) et vitesse sol (nœuds → m/s via × 0,514444).
  * VTG ($GNVTG / $GPVTG) : cap sol et vitesse sol (km/h → m/s via ÷ 3,6).
- Corrections RTCM3 (1 Hz) acheminées vers le ZED-F9P par radio locale 433/868 MHz (base fixe locale)
  ou par NTRIP (4G) pour obtenir la qualité GGA = 4 (RTK Fixe, précision 8 mm à 2 cm).
- Modes de qualité GGA :
  * 0 : NO_FIX
  * 1 : GPS_STANDALONE (~2–5 m)
  * 2 : DGPS (~0,5–1,5 m)
  * 4 : RTK_FIXED (~0,008–0,02 m — requis pour l'autonomie RTK stricte)
  * 5 : RTK_FLOAT (~0,20–0,50 m — bascule résiliente vers Vision Locale)
- Projection géodésique équirectangulaire WGS-84 → ENU local (R_TERRE = 6 378 137,0 m, erreur < 1 cm à 500 m).
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, Optional, Tuple

logger = logging.getLogger("GpsRtkReader")

WGS84_EARTH_RADIUS_M: float = 6_378_137.0
DEFAULT_ZED_F9P_BAUDRATE: int = 38400
DEFAULT_RTK_MAX_AGE_S: float = 1.5
MIN_GPS_HEADING_SPEED_MPS: float = 0.15

GGA_QUALITY_LABELS: Dict[int, str] = {
    0: "NO_FIX",
    1: "GPS_STANDALONE",
    2: "DGPS",
    3: "PPS_FIX",
    4: "RTK_FIXED",
    5: "RTK_FLOAT",
    6: "ESTIMATED",
}


def gga_quality_label(quality: int) -> str:
    return GGA_QUALITY_LABELS.get(int(quality), f"UNKNOWN_{quality}")


@dataclass
class GpsRtkPosition:
    """Instantané géodésique issu du récepteur u-blox ZED-F9P (GGA / RMC / VTG)."""

    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    quality: int = 0  # 4 = RTK fixe (~2 cm), 5 = RTK flottant (~20–50 cm)
    num_satellites: int = 0
    hdop: float = 99.0
    heading_deg: Optional[float] = None  # Cap sol (VTG/RMC), fiable seulement si speed_mps >= 0,15 m/s
    speed_mps: float = 0.0
    timestamp_monotonic: float = field(default_factory=time.monotonic)

    @property
    def is_rtk_fixed(self) -> bool:
        return self.quality == 4

    @property
    def is_rtk_float(self) -> bool:
        return self.quality == 5

    @property
    def quality_label(self) -> str:
        return gga_quality_label(self.quality)

    @property
    def age_s(self) -> float:
        return max(0.0, time.monotonic() - self.timestamp_monotonic)

    def has_reliable_heading(self, min_speed_mps: float = MIN_GPS_HEADING_SPEED_MPS) -> bool:
        return self.heading_deg is not None and self.speed_mps >= float(min_speed_mps)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "lat": round(self.lat, 8),
            "lon": round(self.lon, 8),
            "alt_m": round(self.alt, 3),
            "gga_quality": self.quality,
            "gga_quality_label": self.quality_label,
            "rtk_fixed": self.is_rtk_fixed,
            "num_satellites": self.num_satellites,
            "hdop": round(self.hdop, 2),
            "heading_deg": round(self.heading_deg, 2) if self.heading_deg is not None else None,
            "speed_mps": round(self.speed_mps, 3),
            "age_s": round(self.age_s, 3),
        }


@dataclass
class LocalEnuFrame:
    """Projection équirectangulaire WGS-84 ↔ repère local ENU (Est X, Nord Z) autour d'une origine parcelle.

    Erreur de projection inférieure à 1 cm dans un rayon de 500 m autour de (lat0, lon0).
    """

    lat0: float
    lon0: float

    def to_xy(self, lat: float, lon: float) -> Tuple[float, float]:
        dlat = math.radians(float(lat) - self.lat0)
        dlon = math.radians(float(lon) - self.lon0)
        x_east = WGS84_EARTH_RADIUS_M * dlon * math.cos(math.radians(self.lat0))
        y_north = WGS84_EARTH_RADIUS_M * dlat
        return x_east, y_north

    def to_latlon(self, x_east_m: float, y_north_m: float) -> Tuple[float, float]:
        lat = self.lat0 + math.degrees(float(y_north_m) / WGS84_EARTH_RADIUS_M)
        cos_lat0 = max(1e-6, math.cos(math.radians(self.lat0)))
        lon = self.lon0 + math.degrees(float(x_east_m) / (WGS84_EARTH_RADIUS_M * cos_lat0))
        return lat, lon


def compute_nmea_checksum(body: str) -> str:
    """Calcule le checksum XOR hexadécimal sur 2 caractères majuscules d'un corps NMEA (sans `$` ni `*`)."""
    xor_val = 0
    for ch in body:
        xor_val ^= ord(ch)
    return f"{xor_val:02X}"


def verify_nmea_checksum(sentence: str) -> bool:
    """Vérifie strictement l'intégrité d'une trame NMEA 0183 (`$...*HH`)."""
    line = sentence.strip()
    if not line.startswith("$") or "*" not in line:
        return False
    body, crc = line[1:].split("*", 1)
    if len(crc) < 2:
        return False
    try:
        expected = int(crc[:2], 16)
    except ValueError:
        return False
    xor_val = 0
    for ch in body:
        xor_val ^= ord(ch)
    return xor_val == expected


def parse_nmea_coord(field_val: str, hemisphere: str) -> Optional[float]:
    """Convertit un champ NMEA DDMM.MMMM (ou DDDMM.MMMM) + hémisphère en degrés décimaux."""
    if not field_val:
        return None
    try:
        raw = float(field_val)
    except ValueError:
        return None
    degrees = int(raw // 100)
    minutes = raw - degrees * 100.0
    decimal = degrees + (minutes / 60.0)
    return -decimal if hemisphere.upper() in ("S", "W") else decimal


def parse_nmea_sentence(
    sentence: str,
    current: Optional[GpsRtkPosition] = None,
    now_monotonic: Optional[float] = None,
) -> Optional[GpsRtkPosition]:
    """Décode une trame NMEA GGA, RMC ou VTG et retourne la position mise à jour (ou None si invalide/ignorée)."""
    line = sentence.strip()
    if not verify_nmea_checksum(line):
        return None

    cur = current if current is not None else GpsRtkPosition()
    ts = time.monotonic() if now_monotonic is None else float(now_monotonic)
    fields = line[1:].split("*", 1)[0].split(",")
    if not fields or len(fields[0]) < 5:
        return None

    msg_type = fields[0][2:].upper()  # GNGGA -> GGA, GPRMC -> RMC, GNVTG -> VTG
    try:
        if msg_type == "GGA" and len(fields) >= 10:
            lat = parse_nmea_coord(fields[2], fields[3])
            lon = parse_nmea_coord(fields[4], fields[5])
            qual = int(fields[6] or 0)
            sats = int(fields[7] or 0)
            hdop = float(fields[8] or 99.0)
            alt = float(fields[9] or 0.0)
            if lat is None or lon is None:
                return GpsRtkPosition(
                    lat=cur.lat,
                    lon=cur.lon,
                    alt=cur.alt,
                    quality=0,
                    num_satellites=sats,
                    hdop=99.0,
                    heading_deg=cur.heading_deg,
                    speed_mps=cur.speed_mps,
                    timestamp_monotonic=ts,
                )
            return GpsRtkPosition(
                lat=lat,
                lon=lon,
                alt=alt,
                quality=qual,
                num_satellites=sats,
                hdop=hdop,
                heading_deg=cur.heading_deg,
                speed_mps=cur.speed_mps,
                timestamp_monotonic=ts,
            )

        if msg_type == "RMC" and len(fields) >= 9 and fields[2].upper() == "A":
            lat = parse_nmea_coord(fields[3], fields[4])
            lon = parse_nmea_coord(fields[5], fields[6])
            speed_mps = float(fields[7] or 0.0) * 0.514444  # Nœuds -> m/s
            heading_deg = float(fields[8]) if fields[8] else cur.heading_deg
            return GpsRtkPosition(
                lat=lat if lat is not None else cur.lat,
                lon=lon if lon is not None else cur.lon,
                alt=cur.alt,
                quality=cur.quality,
                num_satellites=cur.num_satellites,
                hdop=cur.hdop,
                heading_deg=heading_deg,
                speed_mps=speed_mps,
                timestamp_monotonic=ts,
            )

        if msg_type == "VTG" and len(fields) >= 8:
            heading_deg = float(fields[1]) if fields[1] else cur.heading_deg
            speed_mps = float(fields[7] or 0.0) / 3.6  # km/h -> m/s
            return GpsRtkPosition(
                lat=cur.lat,
                lon=cur.lon,
                alt=cur.alt,
                quality=cur.quality,
                num_satellites=cur.num_satellites,
                hdop=cur.hdop,
                heading_deg=heading_deg,
                speed_mps=speed_mps,
                timestamp_monotonic=ts,
            )
    except ValueError:
        return None
    return None


class GpsRtkReader:
    """Lecteur asynchrone multi-thread pour u-blox ZED-F9P (38 400 bauds USB CDC / UART ou relecture NMEA)."""

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = DEFAULT_ZED_F9P_BAUDRATE,
        lines: Optional[Iterable[Any]] = None,
        rtcm3_source: str = "RADIO_433_868_OR_NTRIP",
    ):
        self.port = port
        self.baudrate = int(baudrate)
        self.rtcm3_source = str(rtcm3_source)
        self.position = GpsRtkPosition()
        self.frames_decoded: int = 0
        self.crc_errors: int = 0
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._ser = None
        self._lines = lines
        self._thread: Optional[threading.Thread] = None

        if port and str(port).upper() != "SIM":
            import serial  # type: ignore

            self._ser = serial.Serial(port, self.baudrate, timeout=0.5)

        if self._ser is not None or self._lines is not None:
            self._thread = threading.Thread(target=self._loop, daemon=True, name="gps_rtk_reader")
            self._thread.start()

    def _loop(self) -> None:
        source = self._lines if self._lines is not None else self._iter_serial()
        for raw_line in source:
            if self._stop.is_set():
                break
            line = raw_line.decode("ascii", errors="replace") if isinstance(raw_line, (bytes, bytearray)) else str(raw_line)
            self.feed_sentence(line)

    def _iter_serial(self):
        while not self._stop.is_set() and self._ser is not None:
            try:
                raw = self._ser.readline()
            except Exception as exc:
                logger.error("Erreur lecture série ZED-F9P (%s) : %s", self.port, exc)
                time.sleep(0.5)
                continue
            if raw:
                yield raw

    def feed_sentence(self, sentence: str, now_monotonic: Optional[float] = None) -> Optional[GpsRtkPosition]:
        """Injecte une trame NMEA (utile en flux série comme en simulation/rejeu)."""
        stripped = sentence.strip()
        if not stripped:
            return None
        if not verify_nmea_checksum(stripped):
            with self._lock:
                self.crc_errors += 1
            return None
        with self._lock:
            updated = parse_nmea_sentence(stripped, self.position, now_monotonic=now_monotonic)
            if updated is not None:
                self.position = updated
                self.frames_decoded += 1
            return updated

    def read(self, max_age_s: float = DEFAULT_RTK_MAX_AGE_S) -> Optional[GpsRtkPosition]:
        """Retourne la dernière position si elle a un fix (>0) et un âge <= max_age_s (1,5 s par défaut)."""
        with self._lock:
            pos = self.position
        return pos if (pos.quality > 0 and pos.age_s <= float(max_age_s)) else None

    def wait_for_fix(self, min_quality: int = 4, timeout_s: float = 120.0) -> bool:
        """Attend un fix de qualité minimale (4 = RTK Fixe requis avant mission autonome)."""
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            pos = self.read()
            if pos is not None:
                if min_quality == 4 and pos.is_rtk_fixed:
                    return True
                if min_quality != 4 and pos.quality >= min_quality:
                    return True
            time.sleep(0.05)
        return False

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            pos = self.position
            decoded = self.frames_decoded
            crc_err = self.crc_errors
        return {
            "module": "u-blox ZED-F9P",
            "port": self.port or "SIM/INJECT",
            "baudrate": self.baudrate,
            "rtcm3_source": self.rtcm3_source,
            "connected": self._ser is not None or decoded > 0,
            "frames_decoded": decoded,
            "crc_errors": crc_err,
            **pos.to_dict(),
        }

    def close(self) -> None:
        self._stop.set()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
