"""Interface en ligne de commande du programme Windows.

    python -m robot_pc --port COM5 terminal          # console interactive
    python -m robot_pc --port COM5 check             # PING/VER/GCFG/STATUS/PERF
    python -m robot_pc --port COM5 premier-demarrage # assistant de mise en service
    python -m robot_pc --port COM5 panneaux deployer
    python -m robot_pc --port COM5 semer 20000
    python -m robot_pc --port COM5 desherber 15000
    python -m robot_pc --port COM5 mission --gps COM7 --lat0 13.5 --lon0 2.1 ...
    python -m robot_pc --port SIM ...                # sans carte (simulateur)
"""
from __future__ import annotations

import argparse
import logging
import sys
import time

from . import protocol as P
from .link import RobotLink
from .protocol import RobotError
from .sequences import Sequences, premier_demarrage


def _ouvrir(args) -> RobotLink:
    journal = (lambda s, l: print(f"  {s} {l}")) if args.verbose else None
    if args.port.upper() == "SIM":
        from .simulator import SimulateurESP32
        return RobotLink(SimulateurESP32(), checksum=args.checksum, journal=journal)
    return RobotLink.ouvrir(args.port, args.baud, checksum=args.checksum, journal=journal)


def cmd_check(robot: RobotLink, args) -> int:
    print("PING   :", robot.envoyer("PING"))
    print("VER    :", robot.version())
    g = robot.gcfg()
    print("GCFG   :", g)
    if g.get("saved") != 1:
        print("  ! réglages non enregistrés (saved=0) : envoyer SAVE")
    if g.get("uwdt") == 0:
        print("  ! watchdog UART désactivé (uwdt=0) : UWDT:1000 avant exploitation")
    print("STATUS :", robot.status())
    for n in P.MOTEURS:
        print(f"CFG:{n}  :", robot.envoyer(f"CFG:{n}"))
    robot.envoyer("PERF")            # remise à zéro
    time.sleep(1.0)
    perf = robot.perf()
    print("PERF   :", perf)
    if perf.get("exec_max_us", 0) > 5000:
        print("  ! exec_max_us > 5000 µs : budget de la boucle moteurs dépassé")
    return 0


def cmd_terminal(robot: RobotLink, args) -> int:
    print("Console robot — tapez une commande (HELP), 'q' pour quitter. "
          "La télémétrie #T s'affiche si TELEM:ms est actif.")
    robot._sur_telem = lambda t: print(f"  #T armé={int(t.arme)} {t.moteurs} t={t.ms}")
    while True:
        try:
            ligne = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if ligne.lower() in ("q", "quit", "exit"):
            break
        if not ligne:
            continue
        try:
            print("<", robot.envoyer(ligne))
        except RobotError as e:
            print("<", e)
    return 0


def cmd_premier_demarrage(robot: RobotLink, args) -> int:
    print("ATTENTION : roues soulevées, dents relevées, panneaux et semoir dégagés.")
    if input("Continuer ? [o/N] ").strip().lower() not in ("o", "oui"):
        return 1
    premier_demarrage(robot)
    return 0


def cmd_panneaux(robot: RobotLink, args) -> int:
    s = Sequences(robot)
    (s.deployer_panneaux if args.action == "deployer" else s.ranger_panneaux)()
    print("terminé")
    return 0


def cmd_semer(robot: RobotLink, args) -> int:
    Sequences(robot).semer(args.duree_ms)
    print("terminé")
    return 0


def cmd_desherber(robot: RobotLink, args) -> int:
    Sequences(robot).desherber(args.duree_ms)
    print("terminé")
    return 0


def cmd_mission(robot: RobotLink, args) -> int:
    from .gps_rtk import GPSSimule, LecteurGPS, RepereLocal
    from .navigator import Navigateur, lignes_parcelle
    from .vision import DetecteurSimule, DetecteurYOLO

    repere = RepereLocal(args.lat0, args.lon0)
    gps_sim = None
    if args.gps.upper() == "SIM":
        gps_sim = GPSSimule(args.lat0, args.lon0, cap_deg=args.cap0)
        gps = gps_sim
    else:
        gps = LecteurGPS(args.gps, args.gps_baud)
        print("attente du fix RTK fixe…")
        if not gps.attendre_fix(4, 180):
            print("pas de fix RTK fixe : mission annulée")
            return 2
    vision = None
    if args.outil == "desherbage":
        if args.modele:
            vision = DetecteurYOLO(args.modele, args.camera)
        elif args.gps.upper() == "SIM":
            vision = DetecteurSimule(lambda x, y: int(x * 2) % 3 == 0)
    lignes = lignes_parcelle((0.0, 0.0), (args.longueur, 0.0), args.largeur,
                             args.ecartement)
    print(f"{len(lignes)} lignes de {args.longueur} m, outil={args.outil}")
    nav = Navigateur(robot, gps, repere, vision=vision, gps_simule=gps_sim,
                     sur_etat=lambda e: print(
                         f"\r pas {e.pas:4d}  x={e.x:6.2f} y={e.y:6.2f} cap={e.cap if e.cap is None else round(e.cap):>4}"
                         f"  e_lat={e.ecart_lateral:+.2f} m  reste={e.distance_restante:5.1f} m"
                         f"  désherb={'ON ' if e.desherbage_actif else 'off'}  {e.message:<28}", end=""))
    try:
        nav.executer_mission(lignes, outil=args.outil)
        print("\nmission terminée")
        return 0
    except Exception as e:
        print(f"\nmission interrompue : {e}")
        return 3
    finally:
        gps.fermer()
        if vision:
            vision.fermer()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="robot_pc", description="Pilotage du robot agricole (ESP32-S3 v2)")
    ap.add_argument("--port", required=True, help="COM5, /dev/ttyUSB0 ou SIM")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--checksum", action="store_true", help="ajouter *HH aux commandes")
    ap.add_argument("-v", "--verbose", action="store_true", help="afficher les échanges")
    sp = ap.add_subparsers(dest="cmd", required=True)
    sp.add_parser("check").set_defaults(f=cmd_check)
    sp.add_parser("terminal").set_defaults(f=cmd_terminal)
    sp.add_parser("premier-demarrage").set_defaults(f=cmd_premier_demarrage)
    p = sp.add_parser("panneaux"); p.add_argument("action", choices=["deployer", "ranger"]); p.set_defaults(f=cmd_panneaux)
    p = sp.add_parser("semer"); p.add_argument("duree_ms", type=int, nargs="?", default=20000); p.set_defaults(f=cmd_semer)
    p = sp.add_parser("desherber"); p.add_argument("duree_ms", type=int, nargs="?", default=15000); p.set_defaults(f=cmd_desherber)
    p = sp.add_parser("mission")
    p.add_argument("--gps", default="SIM", help="port du ZED-F9P (COM7) ou SIM")
    p.add_argument("--gps-baud", type=int, default=38400)
    p.add_argument("--lat0", type=float, default=13.5137)   # Niamey par défaut
    p.add_argument("--lon0", type=float, default=2.1098)
    p.add_argument("--cap0", type=float, default=0.0, help="cap initial (simulation)")
    p.add_argument("--longueur", type=float, default=20.0, help="longueur des lignes (m)")
    p.add_argument("--largeur", type=float, default=2.0, help="largeur travaillée (m)")
    p.add_argument("--ecartement", type=float, default=0.5, help="inter-rang (m)")
    p.add_argument("--outil", choices=["aucun", "semis", "desherbage"], default="aucun")
    p.add_argument("--modele", help="modèle YOLO (.pt/.onnx/.engine) NigerWeedNet")
    p.add_argument("--camera", default=0)
    p.set_defaults(f=cmd_mission)
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    try:
        robot = _ouvrir(args)
    except Exception as e:
        print(f"ouverture du port {args.port} impossible : {e}")
        return 1
    try:
        return args.f(robot, args)
    except RobotError as e:
        print(f"erreur robot : {e}")
        return 2
    except KeyboardInterrupt:
        print("\ninterruption : ESTOP")
        return 130
    finally:
        robot.fermer(estop=True)


if __name__ == "__main__":
    sys.exit(main())
