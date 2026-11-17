# Architecture Modulaire du Robot Agricole ESP32-S3

Ce document décrit l'architecture modulaire complète du système du Robot Agricole autonome pour la culture de l'arachide (écartement inter-rangs 46 cm), conforme au protocole matériel Firmware v2.

---

## 1. Vue d'Ensemble des Dossiers

```
Robot_Agricole/
├── APP_Mobile/                  # Code source unifié et environnement de travail
│   ├── main.py                  # Point d'entrée FastAPI (< 60 lignes)
│   ├── robot_specs.json         # Spécifications physiques et agronomiques
│   ├── nvs_robot.json           # Mémoire NVS simulée pour l'ESP32-S3
│   ├── backend/                 # Packages Python modulaires
│   │   ├── config/              # Constantes physiques & agronomiques (constants.py)
│   │   ├── hardware/            # Communication ESP32 (hardware_bridge, esp32_simulator)
│   │   ├── navigation/          # agri_nav_engine (Shapely), obstacle_path_planner, robot_controller
│   │   ├── vision/              # ai_vision_brain (détection inter-rang 46 cm & adventices)
│   │   ├── api/                 # network_utils, ws_manager, server_context, routes
│   │   └── assets_gen/          # pbr_builder (génération textures sol et rocher)
│   ├── static/                  # Dashboard PC & Télécommande Mobile Web PWA
│   │   ├── index.html           # Dashboard PC 3D Three.js
│   │   ├── mobile.html          # Télécommande mobile Web (< 120 lignes, 0 inline JS/CSS)
│   │   ├── css/                 # Feuilles de style (main.css, mobile.css)
│   │   └── js/                  # Modules JavaScript ES6
│   │       ├── config/          # constants.js partagé
│   │       ├── core/            # ws_client.js, state_store.js
│   │       ├── ui/              # console_logger.js
│   │       └── mobile/          # mobile_main.js, touch_controls.js, mobile_missions.js
│   ├── mobile_app/              # Application Mobile Native (React Native / Expo / TypeScript)
│   │   ├── App.tsx              # Conteneur principal (< 90 lignes)
│   │   └── src/
│   │       ├── config/          # constants.ts
│   │       ├── hooks/           # useRobotConnection.ts
│   │       ├── components/      # HeaderStatus, WeedingTab, SeedingTab, ConsoleView, HardwareModal
│   │       └── types/           # robot.ts
│   └── tests/                   # Suite de tests automatisés pytest
│       ├── conftest.py          # Fixtures partagées
│       ├── test_config_and_specs.py
│       ├── test_hardware_protocol.py
│       ├── test_row_and_weeding.py
│       └── test_obstacle_avoidance.py
└── Robot_Simulation/            # Clone synchronisé à l'identique de APP_Mobile
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
