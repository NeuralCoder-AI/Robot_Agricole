"""Liaison série avec le robot : fil de lecture, watchdog, attente des réponses.

Principes (référence firmware v2, section « Pilotage autonome depuis Windows ») :
  * un fil de lecture dédié lit en permanence les lignes reçues ;
  * les réponses (OK/ERR/PONG/données) vont dans une file consommée par
    ``envoyer()`` ; la télémétrie ``#T`` est transmise à un rappel ; les fins
    d'action ``#D n`` lèvent un évènement par moteur ;
  * un fil « watchdog » envoie ``PING`` toutes les 300 ms pour entretenir le
    watchdog UART (UWDT = 1000 ms par défaut) ;
  * une seule commande à la fois (verrou) : on attend la réponse avant la
    commande suivante ; toute réponse ``ERR:`` devient une exception.
"""
from __future__ import annotations

import logging
import queue
import threading
import time
from typing import Callable, Optional, Protocol

from . import protocol as P
from .protocol import (Ligne, RobotError, RobotTimeout, Telemetrie, TypeLigne,
                       erreur_depuis_code)

log = logging.getLogger("robot_pc.link")


# --------------------------------------------------------------------------- #
# Transport : port série réel ou simulateur (pour les tests)
# --------------------------------------------------------------------------- #
class Transport(Protocol):
    def write(self, data: bytes) -> int: ...
    def readline(self) -> bytes: ...
    def close(self) -> None: ...


class TransportSerie:
    """Port série Windows (COMx) ou Linux (/dev/ttyUSBx) via pyserial."""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.1,
                 reset_dtr: bool = False):
        import serial  # import tardif : pyserial n'est pas requis pour les tests
        self._ser = serial.Serial()
        self._ser.port = port
        self._ser.baudrate = baud
        self._ser.bytesize = serial.EIGHTBITS
        self._ser.parity = serial.PARITY_NONE
        self._ser.stopbits = serial.STOPBITS_ONE
        self._ser.timeout = timeout
        self._ser.write_timeout = 1.0
        # Sur le port USB natif de l'ESP32-S3, DTR/RTS peuvent redémarrer la
        # carte à l'ouverture : on les force à l'état inactif avant open().
        if not reset_dtr:
            self._ser.dtr = False
            self._ser.rts = False
        self._ser.open()
        self._ser.reset_input_buffer()

    def write(self, data: bytes) -> int:
        return self._ser.write(data)

    def readline(self) -> bytes:
        return self._ser.readline()

    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:  # pragma: no cover
            pass


# --------------------------------------------------------------------------- #
# Lien robot
# --------------------------------------------------------------------------- #
class RobotLink:
    """Pilote le robot par lignes de texte.

    Exemple ::

        with RobotLink.ouvrir("COM5") as robot:
            robot.armer()
            robot.drive(40, 0, ms=5000)
            robot.attendre_fin(1, 2, 3, 4)
    """

    INTERVALLE_PING_S = 0.3
    TIMEOUT_REPONSE_S = 1.0

    def __init__(self, transport: Transport, *, checksum: bool = False,
                 ping_auto: bool = True,
                 sur_telemetrie: Optional[Callable[[Telemetrie], None]] = None,
                 journal: Optional[Callable[[str, str], None]] = None):
        self._t = transport
        self.checksum = checksum
        self._journal = journal or (lambda sens, ligne: None)
        self._sur_telem = sur_telemetrie

        self._reponses: "queue.Queue[Ligne]" = queue.Queue()
        self._verrou_cmd = threading.Lock()
        self._fini: dict[int, threading.Event] = {n: threading.Event() for n in P.MOTEURS}
        self._derniere_telem: Optional[Telemetrie] = None
        self._derniere_emission = time.monotonic()
        self._arme = False
        self._stop = threading.Event()

        self._fil_lecture = threading.Thread(target=self._boucle_lecture,
                                             name="robot-lecture", daemon=True)
        self._fil_lecture.start()
        self._fil_ping: Optional[threading.Thread] = None
        if ping_auto:
            self._fil_ping = threading.Thread(target=self._boucle_ping,
                                              name="robot-ping", daemon=True)
            self._fil_ping.start()

    # ---- construction ---------------------------------------------------- #
    @classmethod
    def ouvrir(cls, port: str, baud: int = 115200, **kw) -> "RobotLink":
        return cls(TransportSerie(port, baud), **kw)

    def __enter__(self) -> "RobotLink":
        return self

    def __exit__(self, *exc) -> None:
        self.fermer()

    def fermer(self, estop: bool = True) -> None:
        """Arrête proprement : STOP (et ESTOP) puis ferme le port."""
        if self._stop.is_set():
            return
        try:
            if estop:
                self.estop()
            else:
                self.stop()
        except Exception:
            pass
        self._stop.set()
        self._t.close()

    # ---- fils internes --------------------------------------------------- #
    def _boucle_lecture(self) -> None:
        tampon = b""
        while not self._stop.is_set():
            try:
                data = self._t.readline()
            except Exception as e:  # port débranché
                if not self._stop.is_set():
                    log.error("lecture série : %s", e)
                    self._reponses.put(Ligne(TypeLigne.ERR, f"ERR:LINK {e}"))
                    time.sleep(0.2)
                continue
            if not data:
                continue
            tampon += data
            if not tampon.endswith(b"\n"):
                continue  # ligne incomplète (timeout de lecture) : on attend la suite
            texte = tampon.decode("ascii", errors="replace").rstrip("\r\n")
            tampon = b""
            self._traiter_ligne(texte)

    def _traiter_ligne(self, texte: str) -> None:
        self._journal("<", texte)
        ligne = P.classer(texte)
        if ligne.type is TypeLigne.TELEM:
            self._derniere_telem = ligne.telem
            if ligne.telem is not None:
                self._arme = ligne.telem.arme
            if self._sur_telem:
                try:
                    self._sur_telem(ligne.telem)
                except Exception:  # pragma: no cover
                    log.exception("rappel télémétrie")
        elif ligne.type is TypeLigne.DONE:
            log.debug("fin d'action moteur %s", ligne.moteur_fini)
            self._fini[ligne.moteur_fini].set()
        elif ligne.type is TypeLigne.AUTRE:
            log.debug("ligne ignorée : %r", texte)
        else:
            self._reponses.put(ligne)

    def _boucle_ping(self) -> None:
        """Entretient le watchdog UART : PING si rien n'a été émis depuis 300 ms."""
        while not self._stop.is_set():
            time.sleep(0.05)
            if time.monotonic() - self._derniere_emission >= self.INTERVALLE_PING_S:
                if self._verrou_cmd.acquire(blocking=False):
                    try:
                        self._envoyer_brut("PING", timeout=self.TIMEOUT_REPONSE_S,
                                           tolerer_erreur=True)
                    except Exception as e:
                        log.warning("PING sans réponse : %s", e)
                    finally:
                        self._verrou_cmd.release()

    # ---- émission -------------------------------------------------------- #
    def _envoyer_brut(self, commande: str, timeout: float,
                      tolerer_erreur: bool = False) -> str:
        if len(commande) > P.LINE_MAX - 3:
            raise ValueError("commande trop longue")
        ligne = P.avec_checksum(commande) if self.checksum else commande
        # purge des réponses orphelines (ex. réponse arrivée après un timeout)
        while True:
            try:
                self._reponses.get_nowait()
            except queue.Empty:
                break
        self._journal(">", ligne)
        self._t.write((ligne + "\n").encode("ascii"))
        self._derniere_emission = time.monotonic()
        try:
            rep = self._reponses.get(timeout=timeout)
        except queue.Empty:
            raise RobotTimeout(f"pas de réponse à {commande!r} en {timeout} s")
        if rep.type is TypeLigne.ERR and not tolerer_erreur:
            raise erreur_depuis_code(rep.brut.split(" ")[0]) from None
        return rep.brut

    def envoyer(self, commande: str, timeout: Optional[float] = None) -> str:
        """Envoie une ligne et renvoie la réponse ; lève RobotError sur ERR:."""
        with self._verrou_cmd:
            return self._envoyer_brut(commande, timeout or self.TIMEOUT_REPONSE_S)

    # ---- commandes de base ---------------------------------------------- #
    def ping(self) -> bool:
        return self.envoyer("PING") == "PONG"

    def version(self) -> str:
        return self.envoyer("VER")

    def armer(self) -> None:
        """ARM, puis vérifie armed=1 dans STATUS."""
        self.envoyer("ARM")
        st = self.status()
        if st.get("armed") != 1:
            raise RobotError("armement non confirmé par STATUS")
        self._arme = True

    def estop(self) -> str:
        self._arme = False
        for ev in self._fini.values():
            ev.clear()
        return self.envoyer("ESTOP")

    def stop(self, n: Optional[int] = None) -> str:
        if n is None:
            for ev in self._fini.values():
                ev.clear()
        else:
            self._fini[n].clear()
        return self.envoyer(P.cmd_stop(n))

    def status(self) -> dict:
        return P.parser_status(self.envoyer("STATUS"))

    def gcfg(self) -> dict:
        return P.parser_cles(self.envoyer("GCFG"))

    def perf(self) -> dict:
        return P.parser_cles(self.envoyer("PERF"))

    def cfg(self, n: int) -> dict:
        return P.parser_cles(self.envoyer(f"CFG:{n}"))

    def telemetrie(self, ms: int) -> str:
        """TELEM:ms (0 = arrêt). Les lignes #T arrivent au rappel sur_telemetrie."""
        return self.envoyer(f"TELEM:{ms}")

    @property
    def derniere_telemetrie(self) -> Optional[Telemetrie]:
        return self._derniere_telem

    @property
    def arme(self) -> bool:
        return self._arme

    # ---- mouvement ------------------------------------------------------- #
    def _lancer(self, commande: str, moteurs: tuple[int, ...]) -> str:
        for n in moteurs:
            self._fini[n].clear()
        return self.envoyer(commande)

    def moteur(self, n: int, v: int, ms: Optional[int] = None) -> str:
        return self._lancer(P.cmd_moteur(n, v, ms), (n,))

    def pulse(self, n: int, v: int, ms: int) -> str:
        return self._lancer(P.cmd_pulse(n, v, ms), (n,))

    def drive(self, a: int, d: int, ms: Optional[int] = None) -> str:
        return self._lancer(P.cmd_drive(a, d, ms), (1, 2, 3, 4))

    def tank(self, l: int, r: int, ms: Optional[int] = None) -> str:
        return self._lancer(P.cmd_tank(l, r, ms), (1, 2, 3, 4))

    def attendre_fin(self, *moteurs: int, timeout: float = 3700.0) -> bool:
        """Attend les lignes ``#D n`` de chaque moteur listé (actions temporisées).

        Renvoie False si le délai expire (ex. action remplacée par STOP : aucun
        ``#D`` n'est alors envoyé par l'ESP32).
        """
        fin = time.monotonic() + timeout
        for n in moteurs:
            reste = fin - time.monotonic()
            if reste <= 0 or not self._fini[n].wait(reste):
                return False
        return True

    def action_terminee(self, n: int) -> bool:
        return self._fini[n].is_set()

    # ---- réglages -------------------------------------------------------- #
    def regler(self, cle: str, n: int, valeur) -> str:
        """INV/MIN/MAX/RAMP/TMAX:n:valeur"""
        cle = cle.upper()
        if cle not in ("INV", "MIN", "MAX", "RAMP", "TMAX"):
            raise ValueError(cle)
        return self.envoyer(f"{cle}:{n}:{valeur}")

    def sauvegarder(self, attendre_s: float = 5.0) -> bool:
        """SAVE puis attend saved=1 dans GCFG."""
        self.envoyer("SAVE")
        fin = time.monotonic() + attendre_s
        while time.monotonic() < fin:
            if self.gcfg().get("saved") == 1:
                return True
            time.sleep(0.3)
        return False

    def reboot(self, attendre_s: float = 3.0) -> None:
        """SAVE, REBOOT, puis attend que l'ESP32 réponde de nouveau à PING."""
        self.sauvegarder()
        self._arme = False
        self.envoyer("REBOOT")
        time.sleep(1.0)
        fin = time.monotonic() + attendre_s + 5
        while time.monotonic() < fin:
            try:
                if self.ping():
                    return
            except RobotError:
                time.sleep(0.3)
        raise RobotTimeout("l'ESP32 ne répond plus après REBOOT")
