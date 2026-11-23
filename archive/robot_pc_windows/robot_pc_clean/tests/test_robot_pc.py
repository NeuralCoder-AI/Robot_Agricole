"""Tests sans matériel : protocole, lien + simulateur, séquences, GPS, navigation."""
import math
import time

import pytest

from robot_pc import protocol as P
from robot_pc.gps_rtk import GPSSimule, RepereLocal, decoder_trame, PositionGPS
from robot_pc.link import RobotLink
from robot_pc.navigator import Navigateur, lignes_parcelle, cap_vers
from robot_pc.protocol import RobotBusy, RobotError, RobotEstop
from robot_pc.sequences import Sequences, Reglages
from robot_pc.simulator import SimulateurESP32
from robot_pc.vision import AnalyseImage, Detection, DetecteurSimule


# ---- protocole ----------------------------------------------------------- #
def test_checksum_reference():
    assert P.checksum("PING") == "10"
    assert P.checksum("M1:50") == "43"
    assert P.checksum("ARM") == "5E"
    assert P.checksum("ESTOP") == "5D"


def test_commandes():
    assert P.cmd_moteur(5, 60) == "M5:60"
    assert P.cmd_moteur(7, 60, 30000) == "M7:60:30000"
    assert P.cmd_pulse(7, -60, 30000) == "PULSE:7:-60:30000"
    assert P.cmd_drive(40, 0, 5000) == "DRIVE:40,0:5000"
    assert P.cmd_tank(-40, 40, 1500) == "TANK:-40,40:1500"
    assert P.cmd_moteur(1, 250) == "M1:100"          # borné
    assert P.drive_vers_gauche_droite(50, 20) == (70, 30)
    with pytest.raises(ValueError):
        P.cmd_moteur(9, 10)
    with pytest.raises(ValueError):
        P.cmd_moteur(1, 10, 0)


def test_classer():
    assert P.classer("PONG").type is P.TypeLigne.OK
    assert P.classer("OK M5 60").type is P.TypeLigne.OK
    assert P.classer("STATUS armed=1 M1=0").type is P.TypeLigne.OK
    assert P.classer("ERR:ESTOP").type is P.TypeLigne.ERR
    t = P.classer("#T 1 0,0,60,60,0,0,0,0 123456")
    assert t.type is P.TypeLigne.TELEM and t.telem.moteurs[2] == 60 and t.telem.arme
    d = P.classer("#D 7")
    assert d.type is P.TypeLigne.DONE and d.moteur_fini == 7
    assert P.parser_status("STATUS armed=1 M1=0 M3=60") == {"armed": 1, "M1": 0, "M3": 60}


# ---- lien + simulateur --------------------------------------------------- #
@pytest.fixture
def robot():
    sim = SimulateurESP32()
    r = RobotLink(sim, ping_auto=False)
    r.sim = sim
    yield r
    r.fermer()


def test_ping_version_estop(robot):
    assert robot.ping()
    assert robot.version().startswith("VER 2.1")
    with pytest.raises(RobotEstop):
        robot.drive(40, 0)
    robot.armer()
    assert robot.status()["armed"] == 1
    assert robot.drive(50, 20) == "OK DRIVE 70,30"
    assert robot.status()["M1"] == 70 and robot.status()["M2"] == 30


def test_checksum_sur_le_lien():
    sim = SimulateurESP32()
    r = RobotLink(sim, checksum=True, ping_auto=False)
    assert r.ping()
    r.fermer()
    assert any(l == "PING*10" for _, s, l in sim.journal if s == ">")


def test_erreurs(robot):
    robot.armer()
    with pytest.raises(RobotError, match="ERR:RANGE"):
        robot.envoyer("MAX:1:50.5")
    with pytest.raises(RobotError, match="ERR:ARG"):
        robot.envoyer("PULSE:7:60")
    with pytest.raises(RobotError, match="ERR:MOTOR"):
        robot.envoyer("M9:10")
    with pytest.raises(RobotError, match="ERR:UNKNOWN"):
        robot.envoyer("FOO")
    with pytest.raises(ValueError):            # refusé côté PC avant envoi (ERR:LONG côté ESP32)
        robot.envoyer("X" * 97)
    robot.sim.write(b"X" * 97 + b"\n")           # mais le simulateur, lui, répond ERR:LONG
    time.sleep(0.1)
    assert ("<", "ERR:LONG") == robot.sim.journal[-1][1:]
    robot.moteur(3, 50)
    with pytest.raises(RobotBusy):
        robot.regler("INV", 3, 1)
    robot.stop(3)
    assert robot.regler("INV", 3, 1) == "OK INV 3 1"


def test_action_temporisee_et_done(robot):
    robot.armer()
    t0 = time.monotonic()
    assert robot.pulse(7, 60, 300) == "OK M7 60 300"
    assert robot.attendre_fin(7, timeout=2)
    assert 0.25 <= time.monotonic() - t0 <= 1.0
    assert robot.status()["M7"] == 0
    robot.drive(40, 0, 200)
    assert robot.attendre_fin(1, 2, 3, 4, timeout=2)


def test_stop_annule_sans_done(robot):
    robot.armer()
    robot.pulse(5, 50, 400)
    robot.stop(5)
    assert not robot.attendre_fin(5, timeout=0.8)


def test_watchdog_uart_sans_ping():
    sim = SimulateurESP32(uwdt_ms=200)
    r = RobotLink(sim, ping_auto=False)
    r.armer()
    r.moteur(6, 70)
    time.sleep(0.5)
    assert r.status()["M6"] == 0          # arrêté par le watchdog
    r.fermer()


def test_watchdog_uart_entretenu_par_ping_auto():
    sim = SimulateurESP32(uwdt_ms=500)
    r = RobotLink(sim, ping_auto=True)
    r.armer()
    r.moteur(6, 70)
    time.sleep(1.2)
    assert r.status()["M6"] == 70         # PING toutes les 300 ms
    r.fermer()


def test_telemetrie(robot):
    recu = []
    robot._sur_telem = recu.append
    robot.telemetrie(20)
    time.sleep(0.2)
    robot.telemetrie(0)
    assert len(recu) >= 3 and recu[-1].moteurs == (0,) * 8


def test_sauvegarde(robot):
    robot.regler("RAMP", 3, 60)
    assert robot.gcfg()["saved"] == 0
    assert robot.sauvegarder()
    assert robot.gcfg()["saved"] == 1
    assert robot.cfg(3)["ramp"] == 60


# ---- séquences ----------------------------------------------------------- #
def test_sequence_desherbage(robot):
    r = Reglages(dents_duree_ms=100, panneaux_duree_ms=100)
    s = Sequences(robot, r)
    s.desherber(duree_ms=200)
    cmds = [l for _, sens, l in robot.sim.journal if sens == ">"]
    i_desc = cmds.index("PULSE:5:50:100")
    i_rot = cmds.index("M6:70")
    i_drive = cmds.index("DRIVE:20,0:200")
    i_stop6 = cmds.index("STOP:6")
    i_rel = cmds.index("PULSE:5:-50:100")
    assert i_desc < i_rot < i_drive < i_stop6 < i_rel    # rotation arrêtée avant de relever
    assert robot.status()["M6"] == 0


def test_sequence_semis_et_panneaux(robot):
    s = Sequences(robot, Reglages(panneaux_duree_ms=100))
    s.semer(duree_ms=200)
    cmds = [l for _, sens, l in robot.sim.journal if sens == ">"]
    assert "M8:60" in cmds and "DRIVE:30,0:200" in cmds and "STOP:8" in cmds
    s.deployer_panneaux()
    assert "PULSE:7:60:100" in [l for _, sens, l in robot.sim.journal if sens == ">"]
    assert robot.cfg(7)["tmax"] == 35


# ---- GPS ----------------------------------------------------------------- #
def _nmea(corps: str) -> str:
    x = 0
    for c in corps:
        x ^= ord(c)
    return f"${corps}*{x:02X}"


def test_decodage_gga_rtk():
    p = PositionGPS(0, 0)
    gga = _nmea("GNGGA,123519,1330.8220,N,00206.5880,E,4,18,0.8,210.0,M,46.9,M,1.0,0000")
    q = decoder_trame(gga, p)
    assert q.rtk_fixe and q.nb_sat == 18
    assert abs(q.lat - 13.51370) < 1e-4 and abs(q.lon - 2.10980) < 1e-4
    rmc = _nmea("GNRMC,123519,A,1330.8220,N,00206.5880,E,1.2,87.5,171026,,,R")
    r = decoder_trame(rmc, q)
    assert r.cap_deg == 87.5 and abs(r.vitesse_ms - 0.617) < 0.01 and r.qualite == 4
    assert decoder_trame("$GNGGA,bad*00", p) is None


def test_repere_local():
    rep = RepereLocal(13.5137, 2.1098)
    x, y = rep.vers_xy(13.5137 + 0.0009, 2.1098)
    assert abs(y - 100.2) < 1.0 and abs(x) < 1e-6
    lat, lon = rep.vers_latlon(50.0, -20.0)
    x2, y2 = rep.vers_xy(lat, lon)
    assert abs(x2 - 50) < 1e-6 and abs(y2 + 20) < 1e-6


# ---- navigation ---------------------------------------------------------- #
def test_lignes_parcelle():
    l = lignes_parcelle((0, 0), (20, 0), largeur_m=1.0, ecartement_m=0.5)
    assert len(l) == 3
    assert l[0] == ((0, 0), (20, 0))
    assert l[1][0][0] == 20 and abs(l[1][0][1] + 0.5) < 1e-9     # retour, décalé à droite
    assert cap_vers(0, 0, 20, 0) == 90.0 and cap_vers(0, 0, 0, 5) == 0.0


def test_navigation_simulee_suit_la_ligne(robot):
    gps = GPSSimule(13.5137, 2.1098, cap_deg=60.0)      # 30° d'erreur initiale
    vision = DetecteurSimule(lambda x, y: 3 < x < 6)
    nav = Navigateur(robot, gps, gps.repere, vision=vision, gps_simule=gps)
    nav.p.pas_ms = 300
    nav.seq.r.dents_duree_ms = 50
    nav.executer_mission([((0, 0), (12, 0))], outil="desherbage")
    assert abs(gps.y) < 0.5 and gps.x >= 12 - 0.6          # arrivé sur la ligne
    assert abs(nav.etat.ecart_lateral) < 0.5
    cmds = [l for _, s, l in robot.sim.journal if s == ">"]
    assert "M6:70" in cmds and "STOP:6" in cmds          # désherbage conditionnel
    assert cmds[-1] == "STOP"


def test_navigation_refuse_sans_rtk(robot):
    gps = GPSSimule(13.5137, 2.1098)
    gps.qualite = 5                                        # RTK flottant
    nav = Navigateur(robot, gps, gps.repere, gps_simule=gps)
    with pytest.raises(Exception, match="RTK"):
        nav.executer_mission([((0, 0), (5, 0))])
    assert robot.status()["M1"] == 0


def test_decision_vision():
    a = AnalyseImage([Detection("arachide", 0.9, 0.5, 0.5),
                      Detection("striga_gesnerioides", 0.8, 0.6, 0.5),
                      Detection("cyperus_rotundus", 0.7, 0.3, 0.5)])
    presence, dec = a.decision(0.75)
    assert presence and abs(dec - 0.2) < 1e-9
    assert a.decision(0.85) == (False, 0.0)
