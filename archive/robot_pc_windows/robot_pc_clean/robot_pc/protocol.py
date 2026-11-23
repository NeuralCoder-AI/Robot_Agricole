"""Protocole texte du robot agricole (firmware ESP32-S3 v2).

Ce module ne dépend pas du port série : il sert à fabriquer les lignes de
commande, à calculer le checksum et à classer les lignes reçues. Il est donc
testable sans matériel.
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from enum import Enum
from typing import Optional

LINE_MAX = 96                     # au-delà : ERR:LONG côté ESP32
DUREE_MIN_MS = 1
DUREE_MAX_MS = 3_600_000          # 1 h
MOTEURS = range(1, 9)

ROLES = {
    1: "Roue avant gauche",
    2: "Roue avant droite",
    3: "Roue arrière gauche",
    4: "Roue arrière droite",
    5: "Descente des dents (+ = descendre)",
    6: "Rotation du désherbage",
    7: "Panneaux solaires (+ = déployer)",
    8: "Semoir (+ = semer)",
}


# --------------------------------------------------------------------------- #
# Erreurs
# --------------------------------------------------------------------------- #
class RobotError(Exception):
    """Erreur renvoyée par le robot (ERR:xxx) ou erreur de liaison."""


class RobotTimeout(RobotError):
    """Aucune réponse dans le délai imparti."""


class RobotEstop(RobotError):
    """Mouvement refusé : le robot est en arrêt d'urgence (envoyer ARM)."""


class RobotBusy(RobotError):
    """Commande refusée car un moteur tourne (INV, DEFAULTS)."""


def erreur_depuis_code(code: str) -> RobotError:
    """Transforme 'ERR:ESTOP' en exception typée."""
    if code == "ERR:ESTOP":
        return RobotEstop(code)
    if code == "ERR:BUSY":
        return RobotBusy(code)
    return RobotError(code)


# --------------------------------------------------------------------------- #
# Checksum
# --------------------------------------------------------------------------- #
def checksum(commande: str) -> str:
    """XOR de tous les octets de la commande (étoile exclue), en hexadécimal.

    >>> checksum("PING")
    '10'
    >>> checksum("M1:50")
    '43'
    """
    x = 0
    for octet in commande.encode("ascii"):
        x ^= octet
    return f"{x:02X}"


def avec_checksum(commande: str) -> str:
    return f"{commande}*{checksum(commande)}"


# --------------------------------------------------------------------------- #
# Fabrication des commandes
# --------------------------------------------------------------------------- #
def _verif_moteur(n: int) -> int:
    if n not in MOTEURS:
        raise ValueError(f"numéro de moteur invalide : {n} (attendu 1 à 8)")
    return n


def _verif_vitesse(v: int) -> int:
    v = int(v)
    return max(-100, min(100, v))


def _verif_duree(ms: Optional[int]) -> Optional[int]:
    if ms is None:
        return None
    ms = int(ms)
    if not DUREE_MIN_MS <= ms <= DUREE_MAX_MS:
        raise ValueError(f"durée hors plage : {ms} ms (1 à 3 600 000)")
    return ms


def cmd_moteur(n: int, v: int, ms: Optional[int] = None) -> str:
    """Mn:v ou Mn:v:ms"""
    n, v, ms = _verif_moteur(n), _verif_vitesse(v), _verif_duree(ms)
    return f"M{n}:{v}" if ms is None else f"M{n}:{v}:{ms}"


def cmd_pulse(n: int, v: int, ms: int) -> str:
    """PULSE:n:v:ms (durée obligatoire)"""
    n, v, ms = _verif_moteur(n), _verif_vitesse(v), _verif_duree(ms)
    return f"PULSE:{n}:{v}:{ms}"


def cmd_drive(a: int, d: int, ms: Optional[int] = None) -> str:
    """DRIVE:a,d ou DRIVE:a,d:ms — a = avance, d = direction (+ = droite)"""
    a, d, ms = _verif_vitesse(a), _verif_vitesse(d), _verif_duree(ms)
    return f"DRIVE:{a},{d}" if ms is None else f"DRIVE:{a},{d}:{ms}"


def cmd_tank(l: int, r: int, ms: Optional[int] = None) -> str:
    """TANK:l,r ou TANK:l,r:ms"""
    l, r, ms = _verif_vitesse(l), _verif_vitesse(r), _verif_duree(ms)
    return f"TANK:{l},{r}" if ms is None else f"TANK:{l},{r}:{ms}"


def cmd_stop(n: Optional[int] = None) -> str:
    return "STOP" if n is None else f"STOP:{_verif_moteur(n)}"


def drive_vers_gauche_droite(a: int, d: int) -> tuple[int, int]:
    """Reproduit le calcul de l'ESP32 : gauche = a+d, droite = a−d, bornés."""
    return (max(-100, min(100, a + d)), max(-100, min(100, a - d)))


# --------------------------------------------------------------------------- #
# Classement des lignes reçues
# --------------------------------------------------------------------------- #
class TypeLigne(Enum):
    OK = "ok"            # OK ..., PONG, STATUS, CFG, GCFG, PERF, VER, HELP
    ERR = "err"          # ERR:xxx
    TELEM = "telem"      # #T ...
    DONE = "done"        # #D n
    AUTRE = "autre"      # ligne vide ou inconnue (bruit de démarrage, etc.)


@dataclass
class Telemetrie:
    arme: bool
    moteurs: tuple[int, ...]   # 8 commandes M1..M8
    ms: int                    # temps depuis le démarrage de l'ESP32


@dataclass
class Ligne:
    type: TypeLigne
    brut: str
    telem: Optional[Telemetrie] = None
    moteur_fini: Optional[int] = None


_RE_TELEM = re.compile(r"^#T\s+([01])\s+([-\d,]+)\s+(\d+)\s*$")
_RE_DONE = re.compile(r"^#D\s+(\d)\s*$")
_MOTS_DONNEES = ("STATUS", "CFG", "GCFG", "PERF", "VER", "HELP")


def classer(ligne: str) -> Ligne:
    """Classe une ligne reçue (sans \\r\\n)."""
    s = ligne.strip()
    if not s:
        return Ligne(TypeLigne.AUTRE, s)
    if s.startswith("#"):
        m = _RE_TELEM.match(s)
        if m:
            try:
                moteurs = tuple(int(x) for x in m.group(2).split(","))
                if len(moteurs) == 8:
                    return Ligne(TypeLigne.TELEM, s,
                                 telem=Telemetrie(m.group(1) == "1", moteurs, int(m.group(3))))
            except ValueError:
                pass
        m = _RE_DONE.match(s)
        if m:
            return Ligne(TypeLigne.DONE, s, moteur_fini=int(m.group(1)))
        return Ligne(TypeLigne.AUTRE, s)
    if s.startswith("ERR:"):
        return Ligne(TypeLigne.ERR, s)
    if s.startswith("OK") or s == "PONG" or s.split(" ", 1)[0] in _MOTS_DONNEES:
        return Ligne(TypeLigne.OK, s)
    return Ligne(TypeLigne.AUTRE, s)


def parser_status(ligne: str) -> dict:
    """'STATUS armed=1 M1=0 ... M8=0' -> {'armed': 1, 'M1': 0, ...}"""
    d = {}
    for champ in ligne.split()[1:]:
        if "=" in champ:
            k, v = champ.split("=", 1)
            try:
                d[k] = int(v)
            except ValueError:
                d[k] = v
    return d


def parser_cles(ligne: str) -> dict:
    """Même chose pour GCFG, PERF et CFG (valeurs numériques converties)."""
    d = {}
    for champ in ligne.split()[1:]:
        if "=" in champ:
            k, v = champ.split("=", 1)
            try:
                d[k] = int(v)
            except ValueError:
                try:
                    d[k] = float(v)
                except ValueError:
                    d[k] = v
    return d
