"""Simulateur de l'ESP32 (firmware v2) utilisable comme Transport.

Il reproduit le protocole texte (commandes, codes d'erreur, checksum, actions
temporisées avec ``#D n``, télémétrie ``#T``, watchdog UART) pour développer et
tester le programme Windows sans carte. La rampe et le PWM ne sont pas simulés
finement : STATUS renvoie la commande cible.

Usage : ``RobotLink(SimulateurESP32())`` ou ``robot_pc --port SIM``.
"""
from __future__ import annotations

import queue
import re
import threading
import time
from dataclasses import dataclass, field

from .protocol import checksum

_RE_INT = re.compile(r"^-?\d+$")
_RE_FLOAT = re.compile(r"^-?\d+(\.\d+)?$")


@dataclass
class _Moteur:
    cmd: int = 0
    inv: int = 0
    mn: int = 0
    mx: int = 100
    ramp: int = 100
    tmax: float = 0.0
    deadline: float = 0.0        # 0 = pas d'action temporisée
    source: str = "uart"


@dataclass
class SimulateurESP32:
    uwdt_ms: int = 1000
    wdt_ms: int = 2000
    autosave: int = 1
    telem_ms: int = 0
    fw: str = "2.1"
    _m: list = field(default_factory=lambda: [_Moteur() for _ in range(8)])
    _arme: bool = False
    _saved: bool = True
    _rx: "queue.Queue[bytes]" = field(default_factory=queue.Queue)
    _entree: bytes = b""
    _t0: float = field(default_factory=time.monotonic)
    _derniere_ligne: float = field(default_factory=time.monotonic)
    _ferme: bool = False
    _verrou: threading.Lock = field(default_factory=threading.Lock)
    journal: list = field(default_factory=list)   # (t, sens, ligne) pour les tests

    def __post_init__(self):
        self._fil = threading.Thread(target=self._boucle, daemon=True)
        self._fil.start()

    # ---- Transport ------------------------------------------------------- #
    def write(self, data: bytes) -> int:
        self._entree += data
        while b"\n" in self._entree:
            ligne, self._entree = self._entree.split(b"\n", 1)
            texte = ligne.decode("ascii", errors="replace").rstrip("\r")
            with self._verrou:
                self._derniere_ligne = time.monotonic()
                if texte.strip() == "":
                    continue
                self.journal.append((time.monotonic(), ">", texte))
                self._repondre(self._executer(texte))
        return len(data)

    def readline(self) -> bytes:
        try:
            return self._rx.get(timeout=0.1)
        except queue.Empty:
            return b""

    def close(self) -> None:
        self._ferme = True

    def _repondre(self, texte: str) -> None:
        self.journal.append((time.monotonic(), "<", texte))
        self._rx.put((texte + "\r\n").encode("ascii"))

    # ---- boucle « temps réel » (200 Hz approximatif) ---------------------- #
    def _boucle(self) -> None:
        prochaine_telem = time.monotonic()
        while not self._ferme:
            time.sleep(0.005)
            now = time.monotonic()
            with self._verrou:
                # fins d'actions temporisées
                for i, m in enumerate(self._m):
                    if m.deadline and now >= m.deadline:
                        m.deadline = 0.0
                        m.cmd = 0
                        self._repondre(f"#D {i + 1}")
                # watchdog UART : moteurs sans durée commandés par l'UART
                if self.uwdt_ms and (now - self._derniere_ligne) * 1000 > self.uwdt_ms:
                    for m in self._m:
                        if m.source == "uart" and not m.deadline:
                            m.cmd = 0
                # télémétrie
                if self.telem_ms and now >= prochaine_telem:
                    prochaine_telem = now + self.telem_ms / 1000
                    self._repondre(self._ligne_telem())

    def _ligne_telem(self) -> str:
        ms = int((time.monotonic() - self._t0) * 1000)
        return f"#T {int(self._arme)} {','.join(str(m.cmd) for m in self._m)} {ms}"

    # ---- interpréteur ---------------------------------------------------- #
    def _executer(self, ligne: str) -> str:
        if len(ligne) > 96:
            return "ERR:LONG"
        if "*" in ligne:
            corps, _, crc = ligne.rpartition("*")
            if len(crc) != 2 or crc.upper() != checksum(corps):
                return "ERR:CRC"
            ligne = corps
        ligne = ligne.strip().upper()
        if not ligne:
            return ""
        parts = ligne.split(":")
        cmd = parts[0]
        args = parts[1:]

        # --- moteur Mn ---
        mm = re.match(r"^M([1-8])$", cmd)
        if mm:
            n = int(mm.group(1))
            return self._cmd_moteur(n, args)
        if re.match(r"^M\d+$", cmd):
            return "ERR:MOTOR"

        if cmd == "PING":
            return "PONG"
        if cmd == "VER":
            return f"VER {self.fw} ROBOT-AGRICOLE ESP32-S3 motors=8"
        if cmd == "HELP":
            return "HELP ARM ESTOP PING STOP[:n] Mn:v[:ms] PULSE:n:v:ms DRIVE:a,d[:ms] TANK:l,r[:ms] INV MIN MAX RAMP TMAX CFG:n WDT UWDT AUTOSAVE STATUS GCFG PERF VER TELEM:ms SAVE DEFAULTS REBOOT"
        if cmd == "ARM":
            self._arme = True
            for m in self._m:
                m.cmd = 0
                m.deadline = 0.0
            return "OK ARMED"
        if cmd == "ESTOP":
            self._arme = False
            for m in self._m:
                m.cmd = 0
                m.deadline = 0.0
            return "OK ESTOP"
        if cmd == "STOP":
            if args:
                n = self._num(args[0])
                if n is None:
                    return "ERR:MOTOR"
                self._m[n - 1].cmd = 0
                self._m[n - 1].deadline = 0.0
                return f"OK STOP M{n}"
            for m in self._m:
                m.cmd = 0
                m.deadline = 0.0
            return "OK STOP"
        if cmd == "STATUS":
            return "STATUS armed=%d " % self._arme + " ".join(
                f"M{i + 1}={m.cmd}" for i, m in enumerate(self._m))
        if cmd == "GCFG":
            return (f"GCFG wdt={self.wdt_ms} uwdt={self.uwdt_ms} autosave={self.autosave} "
                    f"telem={self.telem_ms} saved={int(self._saved)} baud=115200 fw=2.0")
        if cmd == "PERF":
            return ("PERF ctl_period_us=5000 loops=200 exec_avg_us=180 exec_max_us=420 "
                    "period_max_us=5100 heap=250000 heap_min=240000 core_ctl=1 core_net=0")
        if cmd == "SAVE":
            if any(m.cmd for m in self._m):
                return "OK SAVE_PENDING"
            self._saved = True
            return "OK SAVE"
        if cmd == "DEFAULTS":
            if any(m.cmd for m in self._m):
                return "ERR:BUSY"
            self._m = [_Moteur() for _ in range(8)]
            self.uwdt_ms, self.wdt_ms, self.autosave = 1000, 2000, 1
            self._saved = True
            return "OK DEFAULTS"
        if cmd == "REBOOT":
            self._arme = False
            for m in self._m:
                m.cmd = 0
                m.deadline = 0.0
            return "OK REBOOT"
        if cmd == "TELEM":
            v = self._entier(args)
            if v is None:
                return "ERR:ARG" if not args or not _RE_FLOAT.match(args[0]) else "ERR:RANGE"
            if v != 0 and not 20 <= v <= 60000:
                return "ERR:RANGE"
            self.telem_ms = v
            return f"OK TELEM {v}"
        if cmd in ("WDT", "UWDT"):
            v = self._entier(args)
            if v is None:
                return "ERR:ARG"
            if v != 0 and not 100 <= v <= 60000:
                return "ERR:RANGE"
            setattr(self, "wdt_ms" if cmd == "WDT" else "uwdt_ms", v)
            self._saved = False
            return f"OK {cmd} {v}"
        if cmd == "AUTOSAVE":
            v = self._entier(args)
            if v is None:
                return "ERR:ARG"
            if v not in (0, 1):
                return "ERR:RANGE"
            self.autosave = v
            self._saved = False
            return f"OK AUTOSAVE {v}"
        if cmd == "CFG":
            n = self._num(args[0]) if args else None
            if n is None:
                return "ERR:MOTOR"
            m = self._m[n - 1]
            tmax = int(m.tmax) if m.tmax == int(m.tmax) else m.tmax
            return f"CFG {n} inv={m.inv} min={m.mn} max={m.mx} ramp={m.ramp} tmax={tmax}"
        if cmd in ("INV", "MIN", "MAX", "RAMP", "TMAX"):
            return self._cmd_reglage(cmd, args)
        if cmd == "PULSE":
            if len(args) < 3:
                return "ERR:ARG"
            n = self._num(args[0])
            if n is None:
                return "ERR:MOTOR"
            return self._cmd_moteur(n, args[1:], pulse=True)
        if cmd in ("DRIVE", "TANK"):
            return self._cmd_drive(cmd, args)
        return "ERR:UNKNOWN"

    # ---- aides ----------------------------------------------------------- #
    @staticmethod
    def _num(s: str):
        if not _RE_INT.match(s):
            return None
        n = int(s)
        return n if 1 <= n <= 8 else None

    @staticmethod
    def _entier(args):
        if not args or not _RE_FLOAT.match(args[0]):
            return None
        if not _RE_INT.match(args[0]):
            return None  # décimale → ERR:RANGE chez l'appelant si besoin
        return int(args[0])

    def _vitesse(self, s: str):
        if not _RE_FLOAT.match(s):
            return "ERR:ARG"
        if not _RE_INT.match(s):
            return "ERR:RANGE"
        return max(-100, min(100, int(s)))

    def _duree(self, s: str):
        if not _RE_FLOAT.match(s):
            return "ERR:ARG"
        if not _RE_INT.match(s):
            return "ERR:RANGE"
        d = int(s)
        if not 1 <= d <= 3_600_000:
            return "ERR:RANGE"
        return d

    def _appliquer(self, n: int, v: int, ms) -> None:
        m = self._m[n - 1]
        m.cmd = v
        m.source = "uart"
        m.deadline = (time.monotonic() + ms / 1000) if ms else 0.0

    def _cmd_moteur(self, n: int, args, pulse: bool = False) -> str:
        if not args:
            return "ERR:ARG"
        v = self._vitesse(args[0])
        if isinstance(v, str):
            return v
        ms = None
        if len(args) > 1:
            ms = self._duree(args[1])
            if isinstance(ms, str):
                return ms
        elif pulse:
            return "ERR:ARG"
        if not self._arme:
            return "ERR:ESTOP"
        self._appliquer(n, v, ms)
        return f"OK M{n} {v}" + (f" {ms}" if ms else "")

    def _cmd_drive(self, cmd: str, args) -> str:
        if not args or "," not in args[0]:
            return "ERR:ARG"
        a_s, d_s = args[0].split(",", 1)
        a, d = self._vitesse(a_s), self._vitesse(d_s)
        if isinstance(a, str):
            return a
        if isinstance(d, str):
            return d
        ms = None
        if len(args) > 1:
            ms = self._duree(args[1])
            if isinstance(ms, str):
                return ms
        if not self._arme:
            return "ERR:ESTOP"
        if cmd == "DRIVE":
            g, dr = max(-100, min(100, a + d)), max(-100, min(100, a - d))
        else:
            g, dr = a, d
        for n in (1, 3):
            self._appliquer(n, g, ms)
        for n in (2, 4):
            self._appliquer(n, dr, ms)
        return f"OK {cmd} {g},{dr}" + (f" {ms}" if ms else "")

    def _cmd_reglage(self, cmd: str, args) -> str:
        if len(args) < 2:
            return "ERR:ARG" if len(args) == 1 else "ERR:MOTOR"
        n = self._num(args[0])
        if n is None:
            return "ERR:MOTOR"
        m = self._m[n - 1]
        s = args[1]
        if cmd == "TMAX":
            if not _RE_FLOAT.match(s):
                return "ERR:ARG"
            v = float(s)
            if not 0 <= v <= 86400:
                return "ERR:RANGE"
            m.tmax = v
            self._saved = False
            return f"OK TMAX {n} {int(v) if v == int(v) else v}"
        if not _RE_FLOAT.match(s):
            return "ERR:ARG"
        if not _RE_INT.match(s):
            return "ERR:RANGE"
        v = int(s)
        if cmd == "INV":
            if v not in (0, 1):
                return "ERR:RANGE"
            if m.cmd != 0 and v != m.inv:
                return "ERR:BUSY"
            m.inv = v
        elif cmd == "MAX":
            if not m.mn <= v <= 100:
                return "ERR:RANGE"
            m.mx = v
        elif cmd == "MIN":
            if not 0 <= v <= min(90, m.mx):
                return "ERR:RANGE"
            m.mn = v
        elif cmd == "RAMP":
            if not 1 <= v <= 1000:
                return "ERR:RANGE"
            m.ramp = v
        self._saved = False
        return f"OK {cmd} {n} {v}"
