#!/usr/bin/env bash
# ==============================================================================
# deploy/setup_dev_autocommit_push.sh
# À exécuter sur le PC de développement pour automatiser :
#   git commit  ==>  push GitHub (origin) + push SSH/HTTP direct vers l'UC du robot
# Utilisation :
#   ./deploy/setup_dev_autocommit_push.sh <IP_ROBOT_UC> [USER_UC] [CHEMIN_DEPOT_UC]
# Exemple :
#   ./deploy/setup_dev_autocommit_push.sh 192.168.4.2 robot /home/robot/Robot_Agricole
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

ROBOT_IP="${1:-${ROBOT_IP:-192.168.4.2}}"
ROBOT_USER="${2:-${ROBOT_USER:-amok}}"
ROBOT_REPO_PATH="${3:-${ROBOT_REPO_PATH:-/home/${ROBOT_USER}/Projets/Robot_Agricole}}"

echo "========================================================================"
echo "  CONFIGURATION AUTO-DEPLOY AU GIT COMMIT (PC DEV -> UC ${ROBOT_IP})"
echo "========================================================================"

if [ ! -d "$REPO_ROOT/.git" ]; then
    echo "ℹ️  Initialisation du dépôt Git local dans $REPO_ROOT..."
    git -C "$REPO_ROOT" init -b main
fi

# Configurer le remote 'robot' vers l'UC via SSH
ROBOT_REMOTE_URL="ssh://${ROBOT_USER}@${ROBOT_IP}${ROBOT_REPO_PATH}"
if git -C "$REPO_ROOT" remote | grep -qx "robot"; then
    git -C "$REPO_ROOT" remote set-url robot "$ROBOT_REMOTE_URL"
else
    git -C "$REPO_ROOT" remote add robot "$ROBOT_REMOTE_URL"
fi
echo "[1/3] Remote Git 'robot' configuré : $ROBOT_REMOTE_URL"

# Enregistrer la configuration cible pour le hook post-commit
cat > "$REPO_ROOT/.git/robot_uc_deploy.env" <<EOF
ROBOT_IP="$ROBOT_IP"
ROBOT_USER="$ROBOT_USER"
ROBOT_REPO_PATH="$ROBOT_REPO_PATH"
EOF

# Création du hook Git local .git/hooks/post-commit
POST_COMMIT_HOOK="$REPO_ROOT/.git/hooks/post-commit"
cat > "$POST_COMMIT_HOOK" <<'EOF'
#!/usr/bin/env bash
# Hook exécuté automatiquement juste après chaque `git commit` sur le PC de dev.

REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
CFG_FILE="$REPO_ROOT/.git/robot_uc_deploy.env"
[ -f "$CFG_FILE" ] && source "$CFG_FILE"

ROBOT_IP="${ROBOT_IP:-192.168.4.2}"
SHORT_HASH="$(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || echo 'latest')"
BRANCH="$(git -C "$REPO_ROOT" rev-parse --abbrev-ref HEAD 2>/dev/null || echo 'main')"

echo "🚀 [post-commit] Commit #$SHORT_HASH détecté — déploiement automatique en arrière-plan..."

(
    # 1. Push vers GitHub si origin configuré et réseau dispo
    if git -C "$REPO_ROOT" remote | grep -qx "origin"; then
        git -C "$REPO_ROOT" push origin HEAD >/dev/null 2>&1 || true
    fi

    # 2. Push direct SSH vers l'UC du robot (fonctionne hors Internet sur le terrain)
    if git -C "$REPO_ROOT" remote | grep -qx "robot"; then
        git -C "$REPO_ROOT" push robot HEAD >/dev/null 2>&1 || true
    fi

    # 3. Déclenchement HTTP Safe-Reload Guard sur le serveur FastAPI de l'UC
    curl -fsS -m 5 -X POST "http://${ROBOT_IP}:8000/api/system/trigger-update" \
        -H "Content-Type: application/json" \
        -d "{\"branch\":\"${BRANCH}\",\"commit_hash\":\"${SHORT_HASH}\",\"force_apply_local\":true}" \
        >/dev/null 2>&1 || true
) >/dev/null 2>&1 &
EOF

chmod +x "$POST_COMMIT_HOOK"
echo "[2/3] Hook .git/hooks/post-commit installé et activé !"
echo "[3/3] Désormais, chaque 'git commit -m \"...\"' mettra à jour l'UC (${ROBOT_IP}) automatiquement."

