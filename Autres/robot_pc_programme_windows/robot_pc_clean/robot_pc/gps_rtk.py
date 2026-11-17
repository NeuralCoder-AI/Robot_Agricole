"""Lecture du récepteur GPS-RTK u-blox ZED-F9P (NMEA sur port série USB).

Le ZED-F9P sort par défaut des trames NMEA à 1 Hz (jusqu'à 20 Hz réglable avec
u-center) : on exploite GGA (position, qualité du fix, HDOP) et RMC/VTG (cap et
vitesse sol). Les corrections RTK (RTCM3) arrivent soit par NTRIP (Internet),
soit par radio depuis une base locale — ce module ne s'en occupe pas, il lit
seulement la solution déjà corrigée.

Qualité GGA : 0 = pas de fix, 1 = GPS, 2 = DGPS, 4 = RTK fixe (≈ 2 cm),
5 = RTK flottant (≈ 20–50 cm). Le navigateur n'avance en autonomie qu'en RTK
fixe (réglable).

Projection : repère local ENU (Est, Nord) en mètres autour d'un point d'origine,
par la formule équirectangulaire (largement suffisante à l'échelle d'une
parcelle de quelques hectares).
"""
from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

log = logging.getLogger("robot_pc.gps")

R_TERRE = 6_378_137.0  # m (WGS-84)


@dataclass
class PositionGPS:
    lat: float
    lon: float
    alt: float = 0.0
    qualite: int = 0          # 4 = RTK fixe, 5 = RTK flottant
    nb_sat: int = 0
    hdop: float = 99.0
    cap_deg: Optional[float] = None   # cap sol (VTG/RMC), None à l'arrêt
    vitesse_ms: float = 0.0
    horodatage: float = field(default_factory=time.monotonic)

    @property
    def rtk_fixe(self) -> bool:
        return self.qualite == 4

    @property
    def age_s(self) -> float:
        return time.monotonic() - self.horodatage


@dataclass
class RepereLocal:
    """Conversion lat/lon → (x Est, y Nord) en mètres autour d'une origine."""
    lat0: float
    lon0: float

    def vers_xy(self, lat: float, lon: float) -> tuple[float, float]:
        dlat = math.radians(lat - self.lat0)
        dlon = math.radians(lon - self.lon0)
        x = R_TERRE * dlon * math.cos(math.radians(self.lat0))
        y = R_TERRE * dlat
        return x, y

    def vers_latlon(self, x: float, y: float) -> tuple[float, float]:
        lat = self.lat0 + math.degrees(y / R_TERRE)
        lon = self.lon0 + math.degrees(x / (R_TERRE * math.cos(math.radians(self.lat0))))
        return lat, lon


# --------------------------------------------------------------------------- #
# Décodage NMEA (sans dépendance externe)
# --------------------------------------------------------------------------- #
def _nmea_valide(trame: str) -> bool:
    if not trame.startswith("$") or "*" not in trame:
        return False
    corps, crc = trame[1:].split("*", 1)
    x = 0
    for c in corps:
        x ^= ord(c)
    try:
        return x == int(crc[:2], 16)
    except ValueError:
        return False


def _deg(champ: str, hemi: str) -> Optional[float]:
    """'1330.1234' + 'N' → 13.50205…"""
    if not champ:
        return None
    v = float(champ)
    d = int(v // 100)
    m = v - d * 100
    r = d + m / 60
    return -r if hemi in ("S", "W") else r


def decoder_trame(trame: str, courant: PositionGPS) -> Optional[PositionGPS]:
    """Met à jour ``courant`` avec une trame GGA/RMC/VTG ; renvoie la nouvelle
    position ou None si la trame n'apporte rien."""
    trame = trame.strip()
    if not _nmea_valide(trame):
        return None
    champs = trame[1:].split("*")[0].split(",")
    typ = champs[0][2:]  # GNGGA → GGA
    try:
        if typ == "GGA" and len(champs) >= 10:
            lat, lon = _deg(champs[2], champs[3]), _deg(champs[4], champs[5])
            if lat is None or lon is None:
                return PositionGPS(courant.lat, courant.lon, courant.alt, 0, 0, 99.0)
            return PositionGPS(lat, lon,
                               float(champs[9] or 0.0),
                               int(champs[6] or 0),
                               int(champs[7] or 0),
                               float(champs[8] or 99.0),
                               courant.cap_deg, courant.vitesse_ms)
        if typ == "RMC" and len(champs) >= 9 and champs[2] == "A":
            lat, lon = _deg(champs[3], champs[4]), _deg(champs[5], champs[6])
            vit = float(champs[7] or 0.0) * 0.514444  # nœuds → m/s
            cap = float(champs[8]) if champs[8] else None
            return PositionGPS(lat if lat is not None else courant.lat,
                               lon if lon is not None else courant.lon,
                               courant.alt, courant.qualite, courant.nb_sat,
                               courant.hdop, cap, vit)
        if typ == "VTG" and len(champs) >= 8:
            cap = float(champs[1]) if champs[1] else None
            vit = float(champs[7] or 0.0) / 3.6  # km/h → m/s
            return PositionGPS(courant.lat, courant.lon, courant.alt, courant.qualite,
                               courant.nb_sat, courant.hdop, cap, vit)
    except ValueError:
        return None
    return None


# --------------------------------------------------------------------------- #
# Lecteur série
# --------------------------------------------------------------------------- #
class LecteurGPS:
    """Lit le ZED-F9P dans un fil et expose la dernière position.

    ``LecteurGPS("COM7")`` (Windows) ou ``LecteurGPS(lignes=iterable)`` pour
    rejouer un journal NMEA en test.
    """

    def __init__(self, port: Optional[str] = None, baud: int = 38400,
                 lignes=None):
        self.position = PositionGPS(0.0, 0.0)
        self._stop = threading.Event()
        self._verrou = threading.Lock()
        self._ser = None
        if port and port.upper() != "SIM":
            import serial
            self._ser = serial.Serial(port, baud, timeout=0.5)
        self._lignes = lignes
        self._fil = threading.Thread(target=self._boucle, daemon=True, name="gps")
        self._fil.start()

    def _boucle(self) -> None:
        source = self._lignes if self._lignes is not None else self._lire_serie()
        for ligne in source:
            if self._stop.is_set():
                break
            if isinstance(ligne, bytes):
                ligne = ligne.decode("ascii", errors="replace")
            maj = decoder_trame(ligne, self.position)
            if maj is not None:
                with self._verrou:
                    self.position = maj

    def _lire_serie(self):
        while not self._stop.is_set():
            try:
                l = self._ser.readline()
            except Exception as e:  # pragma: no cover
                log.error("GPS : %s", e)
                time.sleep(0.5)
                continue
            if l:
                yield l

    def lire(self, max_age_s: float = 1.5) -> Optional[PositionGPS]:
        """Dernière position si elle est assez récente, sinon None."""
        with self._verrou:
            p = self.position
        return p if p.age_s <= max_age_s and p.qualite > 0 else None

    def attendre_fix(self, qualite_min: int = 4, timeout_s: float = 120.0) -> bool:
        """Attend un fix (4 = RTK fixe) ; renvoie False à l'expiration."""
        fin = time.monotonic() + timeout_s
        while time.monotonic() < fin:
            p = self.lire()
            if p and (p.qualite == 4 or (qualite_min != 4 and p.qualite >= 1)):
                return True
            time.sleep(0.2)
        return False

    def fermer(self) -> None:
        self._stop.set()
        if self._ser:
            self._ser.close()


# --------------------------------------------------------------------------- #
# Simulation (tests et développement sans récepteur)
# --------------------------------------------------------------------------- #
class GPSSimule:
    """Modèle cinématique simple : le navigateur lui envoie les commandes
    DRIVE (a, d, durée) et il intègre la position. Vitesse max ~0,6 m/s à 100 %.
    Interface identique à LecteurGPS.lire()."""

    def __init__(self, lat0: float, lon0: float, cap_deg: float = 0.0,
                 vitesse_max_ms: float = 0.6, taux_rotation_max_degs: float = 60.0,
                 bruit_m: float = 0.0):
        self.repere = RepereLocal(lat0, lon0)
        self.x = self.y = 0.0
        self.cap = cap_deg
        self.vmax = vitesse_max_ms
        self.wmax = taux_rotation_max_degs
        self.bruit = bruit_m
        self.qualite = 4

    def appliquer(self, a: int, d: int, duree_s: float, pas: float = 0.05) -> None:
        """Intègre un DRIVE:a,d pendant duree_s (modèle différentiel)."""
        import random
        g, dr = max(-100, min(100, a + d)), max(-100, min(100, a - d))
        v = (g + dr) / 200 * self.vmax
        w = (g - dr) / 200 * self.wmax  # % différentiel → °/s, + = droite
        t = 0.0
        while t < duree_s:
            dt = min(pas, duree_s - t)
            self.cap = (self.cap + w * dt) % 360
            self.x += v * dt * math.sin(math.radians(self.cap))
            self.y += v * dt * math.cos(math.radians(self.cap))
            t += dt
        if self.bruit:
            self.x += random.gauss(0, self.bruit)
            self.y += random.gauss(0, self.bruit)

    def lire(self, max_age_s: float = 1.5) -> PositionGPS:
        lat, lon = self.repere.vers_latlon(self.x, self.y)
        return PositionGPS(lat, lon, 200.0, self.qualite, 18, 0.8, self.cap, 0.0)

    def attendre_fix(self, qualite_min: int = 4, timeout_s: float = 0) -> bool:
        return True

    def fermer(self) -> None:
        pass
