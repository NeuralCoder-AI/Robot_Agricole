# robot_pc — Programme Windows de pilotage du robot agricole (ESP32-S3, firmware v2)

Le PC porte l'intelligence (séquencement, GPS-RTK, vision) ; l'ESP32 exécute des
ordres simples et se protège seul (watchdogs, TMAX, ESTOP). Ce package applique
les principes de la section « Pilotage autonome depuis Windows » de la référence
firmware v2.

## Installation (Windows, Python 3.10+)

```bat
pip install pyserial
pip install -e .                 :: depuis le dossier robot_pc
pip install ultralytics opencv-python   :: optionnel : vision
```

Pilote de l'adaptateur USB-TTL (CP2102/CH340/FT232). Avec un FTDI, réduire le
« Latency Timer » à 1 ms (Gestionnaire de périphériques → port COM → Paramètres
du port → Avancé).

## Structure

| Module | Rôle |
|---|---|
| `protocol.py` | fabrication des commandes, checksum `*HH`, classement des lignes reçues (`OK`/`ERR:`/`#T`/`#D`) |
| `link.py` | `RobotLink` : fil de lecture dédié, file de réponses, évènements `#D n` par moteur, PING automatique toutes les 300 ms, exceptions typées (`RobotEstop`, `RobotBusy`, `RobotTimeout`) |
| `sequences.py` | séquences de la référence (panneaux, semis, désherbage, virages, pivots) + assistant `premier_demarrage()` |
| `simulator.py` | simulateur de l'ESP32 (protocole complet) pour développer sans carte : `--port SIM` |
| `gps_rtk.py` | lecture NMEA du ZED-F9P (GGA/RMC/VTG), qualité RTK, repère local ENU, `GPSSimule` |
| `vision.py` | détection d'adventices (YOLO/NigerWeedNet ou simulée), décision binaire + décalage latéral |
| `navigator.py` | suivi de ligne par petits pas `DRIVE:a,d:ms` asservi GPS-RTK (loi Stanley simplifiée), pivot en bout de ligne, désherbage conditionnel par vision, génération des lignes de parcelle |
| `cli.py` | commandes `check`, `terminal`, `premier-demarrage`, `panneaux`, `semer`, `desherber`, `mission` |

## Utilisation rapide

```bat
python -m robot_pc --port COM5 check                 :: PING, VER, GCFG (saved=1 ?), STATUS, CFG:1..8, PERF
python -m robot_pc --port COM5 terminal              :: console interactive (HELP, ARM, M1:20 ...)
python -m robot_pc --port COM5 premier-demarrage     :: assistant : sens (INV), seuil (MIN), SAVE
python -m robot_pc --port COM5 panneaux deployer
python -m robot_pc --port COM5 semer 20000
python -m robot_pc --port COM5 desherber 15000
python -m robot_pc --port COM5 mission --gps COM7 --lat0 13.5137 --lon0 2.1098 ^
       --longueur 50 --largeur 10 --ecartement 0.5 --outil desherbage --modele nigerweednet.engine
python -m robot_pc --port SIM mission --outil desherbage --longueur 6     :: tout simulé
```

Depuis Python :

```python
from robot_pc import RobotLink, Sequences

with RobotLink.ouvrir("COM5") as robot:
    robot.armer()
    robot.drive(40, 0, ms=5000)        # DRIVE:40,0:5000
    robot.attendre_fin(1, 2, 3, 4)     # #D 1 … #D 4
    Sequences(robot).desherber(15000)  # dents ↓, M6, DRIVE 15 s, STOP:6, dents ↑
```

## Règles appliquées

* Une commande à la fois ; la réponse est attendue avant la suivante ; `ERR:` devient une exception.
* `PING` automatique si rien n'a été émis depuis 300 ms (watchdog UART 1000 ms).
* Les durées sont comptées par l'ESP32 ; le PC attend les `#D n`. Aucun `#D` n'arrive après `STOP`/`ESTOP` : `attendre_fin()` renvoie alors `False`.
* `armer()` vérifie `armed=1` dans `STATUS` ; `fermer()` envoie `ESTOP` avant de libérer le port.
* Navigation : pas de 300–1000 ms recalculés à chaque fois depuis le GPS-RTK (fix fixe exigé, donnée < 1,5 s) ; arrêt immédiat et dents relevées en cas de perte GPS.

## Tests

```bat
pip install pytest
python -m pytest tests          :: 20 tests, sans matériel (simulateur, GPS et vision simulés)
```
