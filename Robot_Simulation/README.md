# 🚜 Robot Agricole - Jumeau Numérique & Serveur de Contrôle (ESP32-S3 Firmware v2)

Ce projet fournit une solution complète de bout en bout pour le contrôle et la simulation d'un robot agricole autonome piloté par un microcontrôleur **ESP32-S3 (Firmware v2)**. Il comprend le **Jumeau Numérique (Simulateur 200 Hz)**, la **Couche d'abstraction série / autonomie**, l'**API temps réel FastAPI + WebSockets** et un **Dashboard Web interactif**.

---

## 🏗️ Architecture du Système

```
+---------------------------------------------------------------------------------+
|                                 APPLICATION MOBILE                              |
|                          (WebSocket /ws  &  REST /api)                         |
+---------------------------------------+-----------------------------------------+
                                        |
+---------------------------------------v-----------------------------------------+
|                    SERVEUR FASTAPI & WEBSOCKETS (main.py)                       |
|   - Diffusion temps réel : Télémétrie #T, événements #D, Logs série, État      |
|   - Réception : Commandes manuelles (Drive, Tank, Pulse), Missions autonomes    |
|   - Serveur Web : Dashboard HTML5/CSS3/JS avec Jumeau Numérique (static/)       |
+---------------------------------------+-----------------------------------------+
                                        |
+---------------------------------------v-----------------------------------------+
|                  CONTRÔLEUR D'AUTONOMIE (robot_controller.py)                   |
|   - Gestionnaire de mode : MANUEL vs AUTOMATIQUE                                |
|   - 3 Séquences autonomes de référence + Navigation par petits pas RTK          |
|   - Thread Reader : Tri des flux (OK/ERR, #T, #D n)                             |
|   - Tâche Heartbeat : PING périodique à 300 ms (maintien du watchdog UWDT)     |
+---------------------------------------+-----------------------------------------+
                                        |  (Port série transparent ou virtuel)
+---------------------------------------v-----------------------------------------+
|                JUMEAU NUMÉRIQUE ESP32-S3 (esp32_simulator.py)                   |
|   - Boucle physique temps réel à 200 Hz (dt = 5 ms)                             |
|   - 8 Ponts BTS7960 (M1..M8) avec lois PWM, Rampes %/s, Temps mort 80 ms       |
|   - Persistance NVS (nvs_robot.json) simulant ESP32 Preferences                 |
|   - Protocoles ASCII UART1, Watchdogs WDT / UWDT, Checksum XOR *HH              |
+---------------------------------------------------------------------------------+
```

---

## ⚙️ Affectation des 8 Moteurs BTS7960

| Moteur | Fonction | Consigne (+ / -) | GPIOs Direction & PWM |
| :--- | :--- | :--- | :--- |
| **M1** | Roue avant gauche | + Avant / - Arrière | GPIO 4 / 5 |
| **M2** | Roue avant droite | + Avant / - Arrière | GPIO 6 / 7 |
| **M3** | Roue arrière gauche | + Avant / - Arrière | GPIO 15 / 16 |
| **M4** | Roue arrière droite | + Avant / - Arrière | GPIO 17 / 18 |
| **M5** | Descente des dents de terrage | + Descendre / - Monter | GPIO 8 / 9 |
| **M6** | Outil rotatif de désherbage | + Normal / - Inverse | GPIO 10 / 11 |
| **M7** | Panneaux solaires articulés | + Déployer / - Ranger | GPIO 12 / 13 |
| **M8** | Distributeur de semoir | + Semer / - Inverse | GPIO 14 / 21 |

* **Enable global des 8 ponts BTS7960** : GPIO 38  
* **Liaison UART1** : 115200 bauds, 8N1, RX = GPIO 1, TX = GPIO 2  

---

## 📡 Protocole ASCII UART1 & Spécifications

* **Sécurité au démarrage** : Le robot démarre obligatoirement en arrêt d'urgence (`armed = 0`). Toute tentative de mouvement sans `ARM` préalable est rejetée par `ERR: ESTOP`.
* **Checksum optionnel `*HH`** : Calculé par le XOR hexadécimal sur 2 caractères de tous les octets précédant le `*`.  
  *Exemples valides :* `PING*10`, `M1:50*43`, `ARM*5E`, `ESTOP*5D`.
* **Règles de filtrage des consignes** : Toute consigne comprise strictement entre `-1` et `+1` est ramenée à `0`. Au-delà de $\pm 100$, elle est écrêtée à $\pm 100$.
* **Formule cinématique différentielle** :
  $$\text{DRIVE:a,d} \implies \begin{cases} \text{Roues gauches } (M1, M3) = \text{clamp}(a + d, -100, 100) \\ \text{Roues droites } (M2, M4) = \text{clamp}(a - d, -100, 100) \end{cases}$$
* **Durées temporisées (`:ms`)** : Entier de 1 à 3 600 000 ms (1 heure). À l'échéance de la minuterie, la consigne revient à 0 en suivant la rampe `RAMP`, et le microcontrôleur émet de manière asynchrone `#D n` (`#D 1` à `#D 4` pour `DRIVE` ou `TANK`).
* **Loi PWM effective** :
  $$\text{PWM}_{\%} = \text{MIN} + (\text{MAX} - \text{MIN}) \times \frac{|v|}{100}$$
* **Watchdog UWDT** : Coupe immédiatement sans rampe les moteurs non temporisés si aucune commande n'est reçue pendant le délai spécifié (défaut : 1000 ms).
* **Temps mort (Dead Time)** : Délais de 80 ms imposé automatiquement lors de toute inversion de sens de rotation d'un moteur pour protéger les ponts en H.

---

## 🤖 Missions Autonomes Programmées

1. **Déploiement / Rangement des Panneaux Solaires (M7)** :
   * `ARM` $\rightarrow$ `PULSE:7:60:30000` (ou `-60` pour ranger) $\rightarrow$ attente de `#D 7`.
2. **Semis Autonome (M8)** :
   * `ARM` $\rightarrow$ `M8:60` $\rightarrow$ `DRIVE:30,0:20000` $\rightarrow$ attente de `#D 1` à `#D 4` $\rightarrow$ `STOP:8`.
3. **Désherbage Intelligent (M5 + M6)** :
   * `ARM` $\rightarrow$ `PULSE:5:50:4000` (descente des dents, attente `#D 5`)  
   $\rightarrow$ `M6:70` (activation rotation)  
   $\rightarrow$ `DRIVE:20,0:15000` (avance du robot, attente `#D 1..4`, heartbeat `PING` toutes les 300 ms)  
   $\rightarrow$ `STOP:6` (**Arrêt impératif de la rotation avant relevage des dents**)  
   $\rightarrow$ `PULSE:5:-50:4000` (remontée des dents, attente `#D 5`).
4. **Navigation Autonome par petits pas (Vision / GPS RTK)** :
   * Enchaîne des corrections courtes `DRIVE:a,d:ms` (300 à 1000 ms) avec synchronisation `#D 1..4`.

---

## 🚀 Démarrage et Utilisation

### 1. Activer l'environnement virtuel et lancer les tests
```powershell
.\.venv\Scripts\pytest -v
```

### 2. Démarrer le serveur API & Simulateur
```powershell
.\.venv\Scripts\uvicorn.exe main:app --host 0.0.0.0 --port 8000
```

### 3. Accéder au Dashboard Web
Ouvrez votre navigateur sur : **[http://localhost:8000](http://localhost:8000)**

* **Jumeau Visuel** : Visualisation 3D/animée des 4 roues motrices, de la profondeur des dents M5, de la rotation du désherbage M6, de l'état des panneaux solaires M7 et du semoir M8.
* **Console Série UART1** : Envoi de commandes brutes en direct, monitoring des trames `OK`, `ERR:`, `#T` et `#D n`, calcul automatique du checksum `*HH`.
* **Pilotage Manuel** : D-PAD directionnel, arrêt d'urgence `ESTOP`, armement `ARM`, sliders de vitesse.
* **Séquences Autonomes** : Déclenchement en un clic des 4 missions préprogrammées avec suivi d'état.
