# 🚀 Déploiement Embarqué & Auto-Mise à Jour sur l'Unité Centrale (`ROBOT_UC`)

Guide express en **3 étapes** pour lier votre PC de développement à l'Unité Centrale Linux du robot (100 % Temps Réel, zéro simulation 3D).

---

### Étape 1 — Sur l'Unité Centrale (UC) du robot (1 seule fois)
Lancez le script d'installation embarquée :
```bash
sudo ./deploy/setup_robot_uc.sh
```
Ce script active `DEPLOYMENT_TARGET=ROBOT_UC` dans `.env`, configure Git et crée le service `systemd` **`robot-agricole.service`** (démarrage automatique au boot du robot, auto-connexion caméra `T'nB INCAM1080A` + ESP32).

---

### Étape 2 — Sur votre PC de développement (1 seule fois)
Liez votre PC à l'IP Wi-Fi / Ethernet de l'UC du robot (par exemple `192.168.4.2`) :
```bash
./deploy/setup_dev_autocommit_push.sh 192.168.4.2 <user_uc> /home/<user_uc>/Projets/Robot_Agricole
```

---

### Étape 3 — Travailler normalement : chaque `git commit` met à jour le robot !
```bash
git add -A && git commit -m "Amélioration vision IA"
```
- Le hook `.git/hooks/post-commit` pousse immédiatement vers GitHub **et** en SSH/HTTP vers l'UC du robot (même sans Internet au champ).
- Si le robot est à l'arrêt, l'UC applique la mise à jour et redémarre instantanément.
- Si une mission est en cours (**Safe-Reload Guard**), la mise à jour attend la fin ou l'interruption de la mission avant de redémarrer.

