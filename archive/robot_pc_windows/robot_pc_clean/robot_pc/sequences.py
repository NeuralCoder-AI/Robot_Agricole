"""Séquences autonomes (semis, désherbage, panneaux) construites sur RobotLink.

Chaque séquence s'appuie sur les actions temporisées de l'ESP32 : c'est la
carte qui compte le temps et arrête le moteur, le PC n'attend que les ``#D n``.
Les durées/vitesses par défaut reprennent la référence firmware v2 ; elles se
règlent dans ``Reglages``.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field

from .link import RobotLink
from .protocol import RobotError

log = logging.getLogger("robot_pc.sequences")


@dataclass
class Reglages:
    # Panneaux solaires (M7) : mesurer la course réelle puis ajuster
    panneaux_vitesse: int = 60
    panneaux_duree_ms: int = 30_000
    # Dents de désherbage (M5)
    dents_vitesse: int = 50
    dents_duree_ms: int = 4_000
    # Rotation du désherbage (M6)
    rotation_vitesse: int = 70
    # Semoir (M8)
    semoir_vitesse: int = 60
    semoir_vidange_ms: int = 5_000
    # Déplacement
    vitesse_semis: int = 30
    vitesse_desherbage: int = 20
    vitesse_virage: int = 40
    # Marge d'attente au-delà de la durée demandée (s)
    marge_attente_s: float = 3.0
    # Valeurs TMAX de sécurité (s) à appliquer aux mouvements sans fin de course
    tmax_panneaux_s: float = 35.0
    tmax_dents_s: float = 6.0


@dataclass
class Sequences:
    robot: RobotLink
    r: Reglages = field(default_factory=Reglages)

    # ---- utilitaires ----------------------------------------------------- #
    def _attendre(self, duree_ms: int, *moteurs: int) -> None:
        ok = self.robot.attendre_fin(*moteurs,
                                     timeout=duree_ms / 1000 + self.r.marge_attente_s)
        if not ok:
            raise RobotError(f"fin d'action non reçue pour M{','.join(map(str, moteurs))}")

    def preparer(self) -> None:
        """Armement + garde-fous TMAX sur les mouvements sans fin de course."""
        if not self.robot.arme:
            self.robot.armer()
        self.robot.regler("TMAX", 7, self.r.tmax_panneaux_s)
        self.robot.regler("TMAX", 5, self.r.tmax_dents_s)

    # ---- panneaux solaires (M7) ----------------------------------------- #
    def deployer_panneaux(self) -> None:
        """ARM puis PULSE:7:60:30000, attente #D 7."""
        self.preparer()
        self.robot.pulse(7, self.r.panneaux_vitesse, self.r.panneaux_duree_ms)
        self._attendre(self.r.panneaux_duree_ms, 7)

    def ranger_panneaux(self) -> None:
        self.preparer()
        self.robot.pulse(7, -self.r.panneaux_vitesse, self.r.panneaux_duree_ms)
        self._attendre(self.r.panneaux_duree_ms, 7)

    # ---- semis (M8 + roues) --------------------------------------------- #
    def semer(self, duree_ms: int = 20_000, direction: int = 0) -> None:
        """M8:60 puis DRIVE:30,d:ms ; attend #D 1..#D 4 ; puis STOP:8."""
        self.preparer()
        self.robot.moteur(8, self.r.semoir_vitesse)
        try:
            self.robot.drive(self.r.vitesse_semis, direction, duree_ms)
            self._attendre(duree_ms, 1, 2, 3, 4)
        finally:
            self.robot.stop(8)

    def semer_rafale(self, duree_ms: int = 2_000) -> None:
        """PULSE:8:60:2000 → #D 8 (semis par rafale, robot à l'arrêt)."""
        self.preparer()
        self.robot.pulse(8, self.r.semoir_vitesse, duree_ms)
        self._attendre(duree_ms, 8)

    def vider_semoir(self) -> None:
        self.preparer()
        self.robot.pulse(8, -40, self.r.semoir_vidange_ms)
        self._attendre(self.r.semoir_vidange_ms, 8)

    # ---- désherbage (M5 dents, M6 rotation, roues) ----------------------- #
    def descendre_dents(self) -> None:
        self.robot.pulse(5, self.r.dents_vitesse, self.r.dents_duree_ms)
        self._attendre(self.r.dents_duree_ms, 5)

    def relever_dents(self) -> None:
        self.robot.pulse(5, -self.r.dents_vitesse, self.r.dents_duree_ms)
        self._attendre(self.r.dents_duree_ms, 5)

    def desherber(self, duree_ms: int = 15_000, direction: int = 0) -> None:
        """Passe de désherbage complète :
        1. PULSE:5:50:4000 (descendre) → #D 5
        2. M6:70 (rotation)
        3. DRIVE:20,d:ms → #D 1..4
        4. STOP:6 (toujours AVANT de relever)
        5. PULSE:5:-50:4000 (relever) → #D 5
        """
        self.preparer()
        self.descendre_dents()
        self.robot.moteur(6, self.r.rotation_vitesse)
        try:
            self.robot.drive(self.r.vitesse_desherbage, direction, duree_ms)
            self._attendre(duree_ms, 1, 2, 3, 4)
        finally:
            self.robot.stop(6)
            time.sleep(0.3)        # laisser la rotation s'arrêter
            self.relever_dents()

    def fin_de_passe(self) -> None:
        """Arrêt sûr en fin de passe (ou après anomalie) : STOP:6 puis relever."""
        try:
            self.robot.stop(6)
        finally:
            self.relever_dents()

    # ---- déplacements élémentaires -------------------------------------- #
    def avancer(self, vitesse: int, duree_ms: int) -> None:
        self.preparer()
        self.robot.drive(vitesse, 0, duree_ms)
        self._attendre(duree_ms, 1, 2, 3, 4)

    def reculer(self, vitesse: int, duree_ms: int) -> None:
        self.avancer(-abs(vitesse), duree_ms)

    def virer(self, direction: int, duree_ms: int = 2_000) -> None:
        """Virage en avançant : DRIVE:40,d:ms (d > 0 = droite)."""
        self.preparer()
        self.robot.drive(self.r.vitesse_virage, direction, duree_ms)
        self._attendre(duree_ms, 1, 2, 3, 4)

    def pivoter(self, sens: int, duree_ms: int = 1_500, vitesse: int = 40) -> None:
        """Pivot sur place : sens > 0 = droite (TANK:40,-40), < 0 = gauche."""
        self.preparer()
        if sens > 0:
            self.robot.tank(vitesse, -vitesse, duree_ms)
        else:
            self.robot.tank(-vitesse, vitesse, duree_ms)
        self._attendre(duree_ms, 1, 2, 3, 4)

    def demi_tour(self, duree_pivot_ms: int = 3_000, sens: int = 1) -> None:
        """Demi-tour en bout de ligne : la durée est à étalonner sur le terrain
        (elle dépend du sol et de la charge) ; voir navigator pour une version
        asservie au cap GPS."""
        self.pivoter(sens, duree_pivot_ms)

    # ---- rangement ------------------------------------------------------- #
    def mise_en_securite(self) -> None:
        """Fin de journée : STOP, dents relevées, semoir arrêté, ESTOP."""
        try:
            self.robot.stop()
            self.relever_dents()
        finally:
            self.robot.estop()


# --------------------------------------------------------------------------- #
# Assistant de premier démarrage (étapes 4 à 10 de la référence)
# --------------------------------------------------------------------------- #
def premier_demarrage(robot: RobotLink, afficher=print, demander=input) -> dict:
    """Assistant interactif : test de sens moteur par moteur, INV, MIN, SAVE.

    À exécuter ROUES SOULEVÉES, dents relevées, panneaux et semoir dégagés.
    Renvoie les réglages appliqués.
    """
    from .protocol import ROLES
    resultat: dict[int, dict] = {}
    afficher("PING → " + robot.envoyer("PING"))
    afficher(robot.version())
    afficher(robot.envoyer("GCFG"))
    robot.armer()
    afficher("Robot armé. Test de chaque moteur à 20 % pendant 2 s.")
    for n in range(1, 9):
        attendu = ROLES[n]
        afficher(f"\nM{n} — {attendu}")
        robot.moteur(n, 20, 2000)
        robot.attendre_fin(n, timeout=5)
        rep = demander("  Le sens observé est-il le sens logique indiqué ? [O/n] ").strip().lower()
        inv = 0 if rep in ("", "o", "oui", "y") else 1
        if inv:
            afficher("  → " + robot.regler("INV", n, 1))
        # seuil de démarrage
        mn = 0
        while True:
            robot.moteur(n, 1, 1500)
            robot.attendre_fin(n, timeout=4)
            rep = demander(f"  MIN={mn} : le moteur démarre-t-il franchement avec M{n}:1 ? [o/N] ").strip().lower()
            if rep in ("o", "oui", "y") or mn >= 90:
                break
            mn += 5
            afficher("  → " + robot.regler("MIN", n, mn))
        resultat[n] = {"inv": inv, "min": mn}
    afficher("\nVérification roues : DRIVE:30,0 (les 4 roues doivent avancer) 2 s")
    robot.drive(30, 0, 2000)
    robot.attendre_fin(1, 2, 3, 4, timeout=5)
    afficher("DRIVE:0,30 (le robot doit pivoter à droite) 2 s")
    robot.drive(0, 30, 2000)
    robot.attendre_fin(1, 2, 3, 4, timeout=5)
    afficher("SAVE → " + ("saved=1" if robot.sauvegarder() else "ÉCHEC saved=0"))
    for n in range(1, 9):
        afficher(robot.envoyer(f"CFG:{n}"))
    return resultat
