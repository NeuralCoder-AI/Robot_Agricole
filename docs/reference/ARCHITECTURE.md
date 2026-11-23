# Architecture Modulaire du Robot Agricole ESP32-S3

Ce document décrit l'architecture modulaire complète du système du Robot Agricole autonome pour la culture de l'arachide (écartement inter-rangs 46 cm), conforme au protocole matériel Firmware v2.

---

## 1. Vue d'Ensemble des Dossiers

```text
Robot_Agricole/
├── robot_uc/                    # Code embarqué sur l'Unité Centrale HP t530 (déployable seul via sparse-checkout)
│   ├── main.py                  # Point d'entrée FastAPI + WebSockets + MJPEG
│   ├── requirements.txt         # Dépendances Python légères (sans PyTorch)
│   ├── backend/                 # Packages Python modulaires
│   │   ├── config/              # Constantes physiques & agronomiques (constants.py, robot_config.py, robot_specs.json)
│   │   ├── hardware/            # hardware_bridge, esp32_simulator (nvs_robot.json), gps_rtk_reader, solar_bms
│   │   ├── models/              # Modèles ONNX (yolov8n.onnx, mobilenetv3_peanut.onnx) exécutés via cv2.dnn
│   │   ├── navigation/          # agri_nav_engine (Shapely/EKF), kinematic_engine, obstacle_avoidance, reseeding_planner, robot_controller
│   │   ├── vision/              # camera_manager, real_vision_pipeline, yolo_pipeline, yolo_onnx_detector, peanut_classifier, ai_vision_brain
│   │   ├── system/              # auto_deploy_manager (Safe-Reload Guard & GitWatcher)
│   │   ├── api/                 # network_utils, ws_manager, server_context, routes
│   │   └── assets_gen/          # pbr_builder (génération textures sol et rocher)
│   └── web/                     # Dashboard PC 3D & Télécommande Mobile Web PWA (monté sur /static et /js)
│       ├── index.html           # Dashboard PC 3D Three.js
│       ├── mobile.html          # Télécommande mobile Web
│       ├── css/                 # Feuilles de style (main.css, dashboard.css, mobile.css)
│       └── js/                  # Modules JavaScript ES6 (config/, core/, ui/, mobile/)
├── firmware/
│   └── esp32_s3/                # Firmware ESP32-S3 v2.1 (robot_agricole.ino, config.h, webpage.h)
├── mobile_app/                  # Application Mobile Native (React Native / Expo / TypeScript) + node_ws_bridge.js
├── tests/                       # Suite automatisée pytest (69 tests) + verify_box3_simulation.js + assets/
├── deploy/                      # Scripts HP t530 (setup_robot_uc.sh, setup_dev_autocommit_push.sh, start_dynamic.sh, README_DEPLOY_UC.md)
├── docs/
│   ├── redaction/               # Chapitres du mémoire (Chapitre III FINAL, Chapitre IV FINAL, Conclusion FINAL, Rapport)
│   ├── reference/               # Référence UART/firmware v2 (PDF) & ARCHITECTURE.md
│   └── photos/                  # 9 photos terrain (26/10/2026)
├── archive/
│   └── robot_pc_windows/        # Ancien programme PC Windows (référence historique)
├── pytest.ini                   # Configure pythonpath = robot_uc & testpaths = tests
├── .gitignore
└── README.md
```

---

## 2. Spécifications Physiques & Dimensions Exactes (Relevé 26/10/2026)

- **Longueur châssis haut** : 169.0 cm (1.69 m)
- **Longueur hors-tout avec outils** : 2.20 m (porte-à-faux avant fraise M6 : 1.05 m, porte-à-faux arrière 4 dents M5 : 1.15 m)
- **Largeur totale hors-tout** : 121.0 cm (1.21 m) avec pneus larges (19.0 cm)
- **Voie** : 103.0 cm | **Empattement** : 110.0 cm | **Garde au sol** : 24.0 cm
- **Outils agronomiques** :
  - M5 : 4 dents d'enfouissement / décompactage (2 petites avant 13 cm, 2 grandes arrière 25 cm, largeur barre 84 cm).
  - M6 : Fraise rotative inter-rang de 23.0 cm (M6:70).
  - M7 : Panneaux solaires déployables.
  - M8 : Semoir 2 becs espacés de 46.0 cm (impulsion `PULSE:8:80:400`).
- **Seuil de saut de rang en bout de rang** : 50.0 cm ($2 \times$ largeur grosse dent 25 cm).
  - $\le 50\text{ cm}$ (ex: 46 cm arachide) $\rightarrow$ saut d'une ligne (rang $i+2$).
  - $> 50\text{ cm}$ (ex: 75 cm maïs) $\rightarrow$ rang immédiatement suivant (rang $i+1$).

---

## 3. Moteur Cinématique d'Évitement d'Obstacles (`AgriNavEngine`)

- **Modélisation géométrique Shapely** : Le robot est modélisé par son polygone orienté exact ($2.20\text{ m} \times 1.21\text{ m}$) avec une marge de sécurité minimale de 25 cm.
- **Interdiction des angles à 90°** : Calcul trigonométrique oblique exact ($\alpha \in [15^\circ, 60^\circ]$).
- **Suivi Segment par Segment Strict (Strict Waypoint Rail Follower)** :
  1. **Phase 1** : Arrêt franc et relevage complet des dents M5 (`PULSE:5:-50:4000`), arrêt fraise (`STOP:6`).
  2. **Phase 2 (Recul W0)** : Recul en ligne droite jusqu'au sommet $W_0$ situé à $Z_{W0} = Z_{obs} - 2.00\text{ m} - R_{obs}$ pour garantir une distance d'approche longitudinale suffisante.
  3. **Sous-étape A (Pivot TANK sur place)** : Rotation pure sur place ($v = 0$) jusqu'à alignement avec le cap du segment oblique ($|\Delta\theta| \le 2.5^\circ$). Zéro avance avant alignement.
  4. **Sous-étape B (Roulage sur segment)** : Progression avec asservissement Cross-Track ($e_{ct} < 3\text{ cm}$).
  5. **Sous-étape C (Verrouillage dur W1->W2)** : Le robot doit obligatoirement dépasser $Z_{robot} \ge Z_{obs} + R_{obs} + 2.20\text{ m}$ sur le segment parallèle avant d'entamer le rabattement oblique vers la ligne de culture, garantissant ainsi qu'aucun flanc ou roue ne frotte contre le rocher.
- **Verrouillage Absolu du Plan (`LOCK_BYPASS_PLAN`)** : Le côté choisi (Droite ou Gauche) après scan complet du couloir ne peut plus être inversé en cours de contournement, empêchant toute boucle d'hésitation.

---

## 4. Communication Matérielle ESP32-S3 (Firmware v2)

- **Protocole Série / Réseau** : Trames ASCII sécurisées par Checksum XOR 8 bits (`<COMMANDE>*<HEX>`).
- **Filtrage console strict** : Aucune pollution de logs (`#T`, `PING`, `PONG` invisibles).
- **Mode Couplé (Hardware-in-the-Loop)** : Verrouillage de la simulation tant que la liaison physique n'est pas établie si activé.

