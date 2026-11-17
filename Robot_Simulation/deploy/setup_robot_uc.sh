#!/usr/bin/env bash
# ==============================================================================
# deploy/setup_robot_uc.sh
# À exécuter UNE SEULE FOIS sur l'Unité Centrale (UC Linux) embarquée du robot.
# Configure :
#   1. Le fichier .env avec DEPLOYMENT_TARGET=ROBOT_UC (100 % Temps Réel, 0 % Sim).
#   2. Le dépôt Git récepteur (receive.denyCurrentBranch updateInstead) + hook post-receive.
#   3. Le service systemd /etc/systemd/system/robot-agricole.service (auto-boot).
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
APP_DIR="$REPO_ROOT/APP_Mobile"
SERVICE_NAME="robot-agricole"
USER_NAME="${SUDO_USER:-$(whoami)}"
BRANCH="${GIT_DEPLOY_BRANCH:-main}"

echo "========================================================================"
echo "  INSTALLATION UNITÉ CENTRALE ROBOT AGRICOLE (DEPLOYMENT_TARGET=ROBOT_UC)"
echo "========================================================================"

# 1. Création / mise à jour des fichiers .env
cat > "$REPO_ROOT/.env" <<EOF
DEPLOYMENT_TARGET=ROBOT_UC
ROBOT_UC_SYSTEMD=1
GIT_DEPLOY_BRANCH=$BRANCH
EOF

cat > "$APP_DIR/.env" <<EOF
DEPLOYMENT_TARGET=ROBOT_UC
ROBOT_UC_SYSTEMD=1
GIT_DEPLOY_BRANCH=$BRANCH
EOF

echo "[1/4] .env configuré avec DEPLOYMENT_TARGET=ROBOT_UC"

# 2. Initialisation / configuration Git pour réception push direct depuis le PC
if [ ! -d "$REPO_ROOT/.git" ]; then
    git -C "$REPO_ROOT" init -b "$BRANCH"
    git -C "$REPO_ROOT" add -A
    git -C "$REPO_ROOT" commit -m "Initial UC deployment" || true
fi

git -C "$REPO_ROOT" config receive.denyCurrentBranch updateInstead

# Hook post-receive : dès qu'un push SSH arrive du PC, déclenche /api/system/trigger-update
HOOK_PATH="$REPO_ROOT/.git/hooks/post-receive"
cat > "$HOOK_PATH" <<'EOF'
#!/usr/bin/env bash
while read oldrev newrev refname; do
    branch_name="${refname#refs/heads/}"
    short_hash="${newrev:0:7}"
    echo "[post-receive] Commit $short_hash reçu sur $branch_name -> notification FastAPI..."
    curl -fsS -X POST http://127.0.0.1:8000/api/system/trigger-update \
        -H "Content-Type: application/json" \
        -d "{\"branch\":\"$branch_name\",\"commit_hash\":\"$short_hash\",\"force_apply_local\":false}" \
        || true
done
EOF
chmod +x "$HOOK_PATH"
echo "[2/4] Dépôt Git + hook post-receive configurés sur l'UC"

# 3. Python virtualenv & dépendances
PYTHON_BIN="$APP_DIR/.venv/bin/python"
if [ ! -x "$PYTHON_BIN" ]; then
    python3 -m venv "$APP_DIR/.venv"
    PYTHON_BIN="$APP_DIR/.venv/bin/python"
fi
"$PYTHON_BIN" -m pip install --upgrade pip >/dev/null 2>&1 || true
if [ -f "$APP_DIR/requirements.txt" ]; then
    "$PYTHON_BIN" -m pip install -r "$APP_DIR/requirements.txt"
fi
echo "[3/4] Environnement Python vérifié : $PYTHON_BIN"

# 4. Création et activation du service systemd robot-agricole.service
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
UNIT_CONTENT="[Unit]
Description=Robot Agricole ESP32-S3 — Cockpit Embarqué Temps Réel (ROBOT_UC)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${USER_NAME}
WorkingDirectory=${APP_DIR}
EnvironmentFile=${APP_DIR}/.env
ExecStart=${PYTHON_BIN} -m uvicorn main:app --host 0.0.0.0 --port 8000
Restart=always
RestartSec=2
StandardOutput=append:${APP_DIR}/fastapi.log
StandardError=append:${APP_DIR}/fastapi.log

[Install]
WantedBy=multi-user.target
"

if command -v systemctl >/dev/null 2>&1 && [ "$(id -u)" -eq 0 ]; then
    printf "%s" "$UNIT_CONTENT" > "$SERVICE_FILE"
    systemctl daemon-reload
    systemctl enable "$SERVICE_NAME"
    systemctl restart "$SERVICE_NAME"
    echo "[4/4] Service systemd '${SERVICE_NAME}.service' installé et démarré !"
elif command -v sudo >/dev/null 2>&1 && command -v systemctl >/dev/null 2>&1; then
    printf "%s" "$UNIT_CONTENT" | sudo tee "$SERVICE_FILE" >/dev/null
    sudo systemctl daemon-reload
    sudo systemctl enable "$SERVICE_NAME"
    sudo systemctl restart "$SERVICE_NAME"
    echo "[4/4] Service systemd '${SERVICE_NAME}.service' installé et démarré via sudo !"
else
    echo "[4/4] systemd non disponible ici — fichier unité généré dans $SCRIPT_DIR/${SERVICE_NAME}.service"
    printf "%s" "$UNIT_CONTENT" > "$SCRIPT_DIR/${SERVICE_NAME}.service"
fi

echo "✅ Installation UC terminée. Cockpit accessible sur http://$(hostname -I 2>/dev/null | awk '{print $1}'):8000"
