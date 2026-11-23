# 🚀 Déploiement Embarqué & Auto-Mise à Jour sur l'Unité Centrale (`ROBOT_UC`)

Guide express pour déployer **`robot_uc/`** sur l'Unité Centrale embarquée (**HP t530 Thin Client**, AMD GX-215JJ fanless, 8 Go DDR4, SSD 64 Go, **Ubuntu Server 26.04.1 LTS**, 100 % Temps Réel, zéro simulation 3D).

---

### Option Légère (Recommandée sur SSD 64 Go) — Clone via `sparse-checkout`
Pour ne rapatrier sur le HP t530 que le code embarqué (`robot_uc/`) et les scripts de déploiement (`deploy/`), sans les photos terrain ni les documents `.docx` :
```bash
git clone --filter=blob:none --no-checkout <URL_DEPOT_GIT> ~/Robot_Agricole
cd ~/Robot_Agricole
git sparse-checkout init --cone
git sparse-checkout set robot_uc deploy
git checkout main
```

---

### Étape 1 — Sur l'Unité Centrale (HP t530) (1 seule fois)
Lancez le script d'installation embarquée :
```bash
sudo ./deploy/setup_robot_uc.sh
```
Ce script :
1. Active `DEPLOYMENT_TARGET=ROBOT_UC` dans `.env` et `robot_uc/.env` ;
2. Configure Git (`receive.denyCurrentBranch updateInstead` + hook `post-receive`) ;
3. Crée l'environnement virtuel `robot_uc/.venv` et installe `robot_uc/requirements.txt` ;
4. Crée et démarre le service `systemd` **`robot-agricole.service`** (`WorkingDirectory=.../robot_uc`, démarrage automatique au boot, auto-connexion caméra `T'nB INCAM1080A` + `ESP32-S3` + `GPS RTK`).

---

### Étape 2 — Sur votre PC de développement (1 seule fois)
Liez votre PC à l'IP Wi-Fi / Ethernet de l'UC du robot (par exemple `192.168.4.2`) :
```bash
./deploy/setup_dev_autocommit_push.sh 192.168.4.2 <user_uc> /home/<user_uc>/Robot_Agricole
```

---

### Étape 3 — Travailler normalement : chaque `git commit` met à jour le robot !
```bash
git add -A && git commit -m "Amélioration vision IA"
```
- Le hook `.git/hooks/post-commit` pousse immédiatement vers GitHub **et** en SSH/HTTP vers l'UC du robot (même sans Internet au champ).
- Si le robot est à l'arrêt, l'UC applique la mise à jour et redémarre instantanément.
- Si une mission est en cours (**Safe-Reload Guard**), la mise à jour attend la fin ou l'interruption de la mission avant de redémarrer.
