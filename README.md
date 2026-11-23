# 🚜 Robot Agricole Autonome — Semis & Désherbage de l'Arachide (Niger / Milieu Sahélien)

Monorepo officiel de la plateforme robotique agricole autonome conçue pour le **semis**, la **révision de semis (re-semis)** et le **désherbage inter-rangs** de l'arachide (*Arachis hypogaea*) en milieu sahélien (Niger).

---

## 🏗️ Architecture du Monorepo

```text
Robot_Agricole/
├── robot_uc/                 # Code embarqué sur l'Unité Centrale (HP t530, déployable seul via sparse-checkout)
│   ├── main.py               # Point d'entrée serveur FastAPI + WebSockets + streaming MJPEG
│   ├── requirements.txt      # Dépendances Python légères (FastAPI, OpenCV, Shapely, PySerial, etc.)
│   ├── backend/
│   │   ├── api/              # Routes REST/WS, gestionnaire multi-clients, détection IP LAN
│   │   ├── assets_gen/       # Génération PBR / GLB pour le jumeau numérique
│   │   ├── config/           # Paramètres géométriques/agronomiques (constants.py, robot_config.py)
│   │   ├── hardware/         # ESP32HardwareBridge UART/Wi-Fi, simulateur 200 Hz, GPS RTK NMEA, SolarBMS
│   │   ├── models/           # Modèles ONNX légers (yolov8n.onnx, mobilenetv3_peanut.onnx) via cv2.dnn
│   │   ├── navigation/       # RobotController, AgriNavEngine (Shapely/EKF), KinematicEngine, ReseedingPlanner
│   │   ├── system/           # AutoDeployManager (Safe-Reload Guard & GitWatcher)
│   │   └── vision/           # CameraManager, RealVisionPipeline, YoloVisionPipeline, PeanutPlantClassifier
│   └── web/                  # Cockpit Web 3D Three.js (index.html) & PWA mobile (mobile.html, css/, js/)
├── firmware/
│   └── esp32_s3/             # Firmware bas niveau ESP32-S3 v2.1 (robot_agricole.ino, config.h, webpage.h)
├── mobile_app/               # Application mobile React Native / Expo + relais node_ws_bridge.js
├── tests/                    # Suite complète pytest (69 tests) + verify_box3_simulation.js
├── deploy/                   # Scripts Linux/systemd HP t530 (setup_robot_uc.sh, start_dynamic.sh, etc.)
├── docs/
│   ├── redaction/            # Chapitres du mémoire (Chapitre III FINAL, Chapitre IV FINAL, Conclusion FINAL, Rapport)
│   ├── reference/            # Référence UART/firmware v2 (PDF) & ARCHITECTURE.md
│   └── photos/               # 9 photographies terrain du prototype (relevé du 26/10/2026)
├── archive/
│   └── robot_pc_windows/     # Ancien programme PC Windows (référence historique)
├── pytest.ini                # Configure pythonpath = robot_uc & testpaths = tests
└── .gitignore                # Exclut .venv/, .cache/, .npm/, node_modules/, __pycache__/, *.log, .env, *.sqlite
```

---

## ⚙️ Plateforme Matérielle & Spécifications Clés

- **Unité Centrale embarquée (`ROBOT_UC`)** : HP t530 Thin Client (AMD GX-215JJ fanless, 8 Go DDR4, SSD 64 Go) sous **Ubuntu Server 26.04.1 LTS**.
- **Contrôleur temps réel bas niveau** : **ESP32-S3 (Firmware v2.1)** pilotant **8 ponts en H BTS7960** (`M1`–`M4` traction 4×4, `M5` 4 dents de terrage, `M6` fraise rotative 23 cm, `M7` panneaux solaires articulés, `M8` semoir 2 tuyaux) sur bus UART1 (`115200` bauds, trames ASCII + checksum XOR `*HH`, boucle `200 Hz`, watchdog `UWDT` `1000 ms`, temps mort `80 ms`).
- **Caméra embarquée** : `T'nB INCAM1080A` USB (`1280×720 @ 30 FPS`, vision hybride Dual-ONNX via `cv2.dnn` : `yolov8n.onnx` + `mobilenetv3_peanut.onnx`, sans PyTorch ni Ultralytics).
- **Localisation** : Fusion **EKF 6 états** (`GPS RTK` NMEA 0183 `$GNGGA/$GNRMC/$GNVTG` + odométrie + IMU).
- **Alimentation solaire** : `2 × Sunshine Solar 265 Wc` en parallèle (`530 Wc` crête), MPPT, parc **LiFePO4 `24 V / 100 Ah`** (`2 400 Wh`), gestionnaire `SolarBMS`.
- **Géométrie & Agronomie (relevé terrain 26/10/2026)** : châssis `1,69 m` (hors-tout outils `2,15 m` / `2,20 m`) × `1,21 m`, voie `1,03 m`, empattement `1,10 m`, garde au sol `24 cm`, entraxe mécanique des 2 tuyaux de semis **`46 cm` (`±23 cm`)**, profondeur de semis **`5 cm`**, espacement intra-rang sahélien **`15 cm`**.

---

## 💻 Développement Local (PC Linux)

### 1. Environnement Python
```bash
python3 -m venv .venv
.venv/bin/pip install -r robot_uc/requirements.txt
```

### 2. Lancer le serveur FastAPI + Cockpit 3D
```bash
cd robot_uc
../.venv/bin/python -m uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```
Ouvrir ensuite `http://localhost:8000` (Cockpit PC 3D) ou `http://localhost:8000/mobile` (interface mobile Web).

### 3. Lancer la pile complète LAN (FastAPI + Bridge WS Node.js + Expo Mobile)
```bash
./deploy/start_dynamic.sh
```

---

## 🧪 Tests Automatisés

Depuis la racine du dépôt (grâce à `pytest.ini`, aucun `PYTHONPATH` manuel n'est nécessaire) :

```bash
# Suite backend, protocole ESP32, vision Dual-ONNX, EKF GPS RTK, SolarBMS & API (69 tests)
.venv/bin/pytest -q

# Validation cinématique géométrique SAT (500 + 500 frames, 0 collision)
node tests/verify_box3_simulation.js
```

---

## 🚀 Déploiement sur l'Unité Centrale (HP t530 — Ubuntu Server 26.04.1 LTS)

Pour ne cloner que la partie embarquée (`robot_uc/` + `deploy/`, sans les photos ni les mémoires `.docx`) sur le SSD 64 Go du HP t530 :

```bash
git clone --filter=blob:none --no-checkout <URL_DEPOT_GIT> ~/Robot_Agricole
cd ~/Robot_Agricole
git sparse-checkout init --cone
git sparse-checkout set robot_uc deploy
git checkout main

# Installation unique du service systemd (DEPLOYMENT_TARGET=ROBOT_UC)
sudo ./deploy/setup_robot_uc.sh
```

Consulter [`deploy/README_DEPLOY_UC.md`](deploy/README_DEPLOY_UC.md) pour la configuration de l'auto-déploiement Git/SSH avec **Safe-Reload Guard**.

