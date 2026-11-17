"""Navigation autonome par GPS-RTK et vision, par petits pas.

Il n'y a ni codeur ni capteur de position sur le robot : une durée n'est pas
une distance (charge, batterie, sol, patinage). La référence firmware
recommande donc de répéter des ``DRIVE:a,d:ms`` courts (300 à 1000 ms) et de
recalculer à chaque pas depuis le GPS-RTK et la vision. C'est ce que fait
``Navigateur`` :

    boucle (chaque pas) :
        1. lire la position RTK (refus si pas de fix fixe ou donnée trop vieille)
        2. estimer le cap : cap sol du récepteur si le robot avance, sinon
           direction entre les deux dernières positions
        3. calculer l'écart latéral et l'écart de cap par rapport au segment
           (loi de commande type Stanley : d = k_cap·Δcap + k_lat·atan(k·e/v))
        4. si vision active : décider le désherbage (dents/rotation) et
           ajouter une petite correction latérale vers l'adventice
        5. envoyer DRIVE:a,d:ms et attendre #D 1..#D 4
        6. arrêt quand la distance au point d'arrivée < tolérance

Les outils (M5 dents, M6 rotation, M8 semoir) sont pilotés en mode
« conditionnel » : rotation enclenchée seulement quand le CNN voit une
adventice dans la bande de travail (seuil 0,75), comme le sarcleur de la thèse.
"""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from .gps_rtk import PositionGPS, RepereLocal
from .link import RobotLink
from .protocol import RobotError
from .sequences import Reglages, Sequences
from .vision import Detecteur

log = logging.getLogger("robot_pc.nav")


def _angle_normalise(deg: float) -> float:
    """Ramène dans ]−180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0


def cap_vers(x0: float, y0: float, x1: float, y1: float) -> float:
    """Cap (° depuis le Nord, sens horaire) de (x0,y0) vers (x1,y1) en repère ENU."""
    return math.degrees(math.atan2(x1 - x0, y1 - y0)) % 360.0


@dataclass
class ParametresNavigation:
    # Pas élémentaire
    pas_ms: int = 600                 # durée d'un DRIVE (300–1000 ms conseillés)
    vitesse_transit: int = 40         # % en déplacement libre
    vitesse_travail: int = 20         # % en semis / désherbage
    # Loi de commande (Stanley simplifiée, sortie en % de direction)
    k_cap: float = 0.8                # % par degré d'écart de cap
    k_lat: float = 40.0               # % pour atan(k·e) à écart latéral 1 m
    k_e: float = 2.0                  # gain sur l'écart latéral (1/m)
    direction_max: int = 35           # % de direction maximum en roulant
    # Arrivée
    tolerance_arrivee_m: float = 0.30
    # Pivot en bout de ligne
    pivot_pas_ms: int = 400
    pivot_vitesse: int = 40
    tolerance_cap_deg: float = 8.0
    # Sécurité GPS
    exiger_rtk_fixe: bool = True
    age_max_gps_s: float = 1.5
    vitesse_min_cap_ms: float = 0.15  # en dessous, le cap GPS n'est pas fiable
    # Vision
    seuil_adventice: float = 0.75
    correction_vision: float = 10.0   # % de direction max ajouté vers l'adventice
    # Limites
    pas_max_par_segment: int = 2000   # garde-fou (≈ 20 min à 600 ms)


@dataclass
class EtatNavigation:
    x: float = 0.0
    y: float = 0.0
    cap: Optional[float] = None
    ecart_lateral: float = 0.0
    ecart_cap: float = 0.0
    distance_restante: float = 0.0
    desherbage_actif: bool = False
    pas: int = 0
    message: str = ""


class ArretNavigation(Exception):
    """Navigation interrompue (GPS perdu, demande d'arrêt, erreur robot)."""


class Navigateur:
    def __init__(self, robot: RobotLink, gps, repere: RepereLocal,
                 vision: Optional[Detecteur] = None,
                 params: Optional[ParametresNavigation] = None,
                 reglages: Optional[Reglages] = None,
                 sur_etat: Optional[Callable[[EtatNavigation], None]] = None,
                 gps_simule=None):
        self.robot = robot
        self.gps = gps
        self.repere = repere
        self.vision = vision
        self.p = params or ParametresNavigation()
        self.seq = Sequences(robot, reglages or Reglages())
        self.sur_etat = sur_etat or (lambda e: None)
        self.etat = EtatNavigation()
        self._stop = False
        self._prec: Optional[tuple[float, float]] = None
        self._gps_simule = gps_simule   # si fourni, intégration cinématique en test

    # ---- arrêt externe --------------------------------------------------- #
    def demander_arret(self) -> None:
        self._stop = True

    # ---- lecture capteurs ------------------------------------------------ #
    def _position(self) -> tuple[float, float, Optional[float]]:
        pos: Optional[PositionGPS] = self.gps.lire(self.p.age_max_gps_s)
        if pos is None:
            raise ArretNavigation("GPS : aucune position récente")
        if self.p.exiger_rtk_fixe and not pos.rtk_fixe:
            raise ArretNavigation(f"GPS : pas de fix RTK fixe (qualité={pos.qualite})")
        x, y = self.repere.vers_xy(pos.lat, pos.lon)
        cap = None
        if pos.cap_deg is not None and pos.vitesse_ms >= self.p.vitesse_min_cap_ms:
            cap = pos.cap_deg
        elif self._prec is not None:
            dx, dy = x - self._prec[0], y - self._prec[1]
            if math.hypot(dx, dy) > 0.05:
                cap = math.degrees(math.atan2(dx, dy)) % 360.0
        if cap is None and self._gps_simule is not None:
            cap = self._gps_simule.cap
        if cap is None:
            cap = self.etat.cap
        self._prec = (x, y)
        return x, y, cap

    # ---- envoi d'un pas -------------------------------------------------- #
    def _pas(self, a: int, d: int, ms: Optional[int] = None) -> None:
        if self._stop:
            raise ArretNavigation("arrêt demandé")
        ms = ms or self.p.pas_ms
        d = int(max(-self.p.direction_max, min(self.p.direction_max, d)))
        self.robot.drive(int(a), d, ms)
        if self._gps_simule is not None:
            self._gps_simule.appliquer(int(a), d, ms / 1000)
        if not self.robot.attendre_fin(1, 2, 3, 4, timeout=ms / 1000 + 2.0):
            raise ArretNavigation("fin de pas non reçue (#D)")
        self.etat.pas += 1

    # ---- cap : pivot sur place ------------------------------------------ #
    def orienter(self, cap_cible: float, max_pas: int = 60) -> None:
        """Pivote (TANK) jusqu'à ce que le cap soit à ±tolérance du cap cible.
        À l'arrêt, le cap GPS n'existe pas : on avance d'un petit pas entre
        deux pivots pour mesurer le cap par déplacement."""
        for _ in range(max_pas):
            x, y, cap = self._position()
            if cap is None:
                self._pas(self.p.vitesse_travail, 0, 500)
                continue
            err = _angle_normalise(cap_cible - cap)
            self.etat.cap, self.etat.ecart_cap = cap, err
            self.sur_etat(self.etat)
            if abs(err) <= self.p.tolerance_cap_deg:
                return
            v = self.p.pivot_vitesse
            ms = self.p.pivot_pas_ms if abs(err) > 30 else self.p.pivot_pas_ms // 2
            if err > 0:
                self.robot.tank(v, -v, ms)
                sens = 1
            else:
                self.robot.tank(-v, v, ms)
                sens = -1
            if self._gps_simule is not None:
                self._gps_simule.appliquer(0, sens * v, ms / 1000)
            self.robot.attendre_fin(1, 2, 3, 4, timeout=ms / 1000 + 2)
            # petit pas avant pour rafraîchir le cap mesuré
            self._pas(self.p.vitesse_travail, 0, 400)
        raise ArretNavigation("orientation impossible (cap non atteint)")

    # ---- suivi d'un segment --------------------------------------------- #
    def suivre_segment(self, x0: float, y0: float, x1: float, y1: float,
                       vitesse: Optional[int] = None,
                       outil: str = "aucun") -> None:
        """Suit la droite (x0,y0)→(x1,y1) jusqu'à l'arrivée.

        outil : 'aucun' | 'semis' | 'desherbage'
        """
        v = vitesse or (self.p.vitesse_transit if outil == "aucun" else self.p.vitesse_travail)
        cap_seg = cap_vers(x0, y0, x1, y1)
        ux, uy = math.sin(math.radians(cap_seg)), math.cos(math.radians(cap_seg))
        longueur = math.hypot(x1 - x0, y1 - y0)

        self._outil_debut(outil)
        try:
            for _ in range(self.p.pas_max_par_segment):
                x, y, cap = self._position()
                # projection sur le segment
                s = (x - x0) * ux + (y - y0) * uy           # abscisse curviligne
                e = (x - x0) * uy - (y - y0) * ux           # écart latéral (+ = à droite de la ligne)
                reste = longueur - s
                self.etat.x, self.etat.y, self.etat.cap = x, y, cap
                self.etat.ecart_lateral, self.etat.distance_restante = e, reste
                if reste <= self.p.tolerance_arrivee_m:
                    self.etat.message = "arrivé"
                    self.sur_etat(self.etat)
                    break
                # loi de commande
                cap_ref = cap if cap is not None else cap_seg
                d_cap = _angle_normalise(cap_seg - cap_ref)
                self.etat.ecart_cap = d_cap
                v_ms = max(0.1, v / 100 * 0.6)
                direction = (self.p.k_cap * d_cap
                             - self.p.k_lat * math.atan(self.p.k_e * e / v_ms) / (math.pi / 2))
                # vision
                if self.vision is not None and outil == "desherbage":
                    direction += self._desherbage_conditionnel(x, y)
                self.sur_etat(self.etat)
                # ralentir à l'approche
                vit = v if reste > 1.0 else max(15, v // 2)
                self._pas(vit, round(direction))
            else:
                raise ArretNavigation("nombre de pas maximal atteint")
        finally:
            self._outil_fin(outil)

    # ---- outils ---------------------------------------------------------- #
    def _outil_debut(self, outil: str) -> None:
        if outil == "semis":
            self.robot.moteur(8, self.seq.r.semoir_vitesse)
        elif outil == "desherbage":
            self.seq.descendre_dents()
            if self.vision is None:            # pas de caméra : rotation continue
                self.robot.moteur(6, self.seq.r.rotation_vitesse)
                self.etat.desherbage_actif = True

    def _outil_fin(self, outil: str) -> None:
        if outil == "semis":
            self.robot.stop(8)
        elif outil == "desherbage":
            self.robot.stop(6)
            self.etat.desherbage_actif = False
            time.sleep(0.3)
            self.seq.relever_dents()

    def _desherbage_conditionnel(self, x: float, y: float) -> float:
        """Enclenche/arrête M6 selon la vision ; renvoie une correction de direction."""
        if hasattr(self.vision, "x"):          # DetecteurSimule : lui donner la position
            self.vision.x, self.vision.y = x, y
        analyse = self.vision.analyser()
        if analyse is None or analyse.horodatage < time.monotonic() - 1.0:
            # image trop vieille : par sécurité on désherbe (comportement conservateur)
            presence, decalage = True, 0.0
        else:
            presence, decalage = analyse.decision(self.p.seuil_adventice)
        if presence and not self.etat.desherbage_actif:
            self.robot.moteur(6, self.seq.r.rotation_vitesse)
            self.etat.desherbage_actif = True
        elif not presence and self.etat.desherbage_actif:
            self.robot.stop(6)
            self.etat.desherbage_actif = False
        return decalage * self.p.correction_vision if presence else 0.0

    # ---- mission complète ------------------------------------------------ #
    def executer_mission(self, lignes: list[tuple[tuple[float, float], tuple[float, float]]],
                         outil: str = "aucun") -> None:
        """Enchaîne des lignes (x0,y0)→(x1,y1) en repère local, avec
        orientation en tête de ligne (pivot) entre deux lignes."""
        if not self.robot.arme:
            self.robot.armer()
        self.seq.preparer()
        try:
            for i, ((x0, y0), (x1, y1)) in enumerate(lignes):
                log.info("ligne %d/%d", i + 1, len(lignes))
                self.etat.message = f"ligne {i + 1}/{len(lignes)} : orientation"
                self.orienter(cap_vers(x0, y0, x1, y1))
                self.etat.message = f"ligne {i + 1}/{len(lignes)} : {outil}"
                self.suivre_segment(x0, y0, x1, y1, outil=outil)
        except (ArretNavigation, RobotError) as e:
            log.error("navigation interrompue : %s", e)
            self.robot.stop()
            if outil == "desherbage":
                self.seq.fin_de_passe()
            raise
        finally:
            self.robot.stop()


# --------------------------------------------------------------------------- #
# Génération des lignes de travail (aller-retour en lacets)
# --------------------------------------------------------------------------- #
def lignes_parcelle(a: tuple[float, float], b: tuple[float, float],
                    largeur_m: float, ecartement_m: float = 0.5,
                    cote: int = 1) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """Lignes parallèles à AB, espacées de ``ecartement_m`` (inter-rang
    arachide : 40–60 cm), sur ``largeur_m`` vers la droite (cote=1) ou la gauche
    (cote=−1) de AB, en lacets (A→B, puis B'→A', ...). Coordonnées locales (m)."""
    ax, ay = a
    bx, by = b
    L = math.hypot(bx - ax, by - ay)
    ux, uy = (bx - ax) / L, (by - ay) / L
    nx, ny = uy * cote, -ux * cote            # normale (vers la droite si cote=1)
    n = int(largeur_m // ecartement_m) + 1
    lignes = []
    for i in range(n):
        ox, oy = nx * i * ecartement_m, ny * i * ecartement_m
        p0, p1 = (ax + ox, ay + oy), (bx + ox, by + oy)
        lignes.append((p0, p1) if i % 2 == 0 else (p1, p0))
    return lignes
