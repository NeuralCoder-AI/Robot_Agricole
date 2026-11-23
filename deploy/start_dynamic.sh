#!/usr/bin/env bash
# ==============================================================================
# deploy/start_dynamic.sh — Lancement Natif Linux 100 % LAN / Hotspot (Sans Tunnel)
# Lance :
#   1. FastAPI depuis robot_uc/ (0.0.0.0:8000)
#   2. Relais Node.js mobile_app/node_ws_bridge.js (0.0.0.0:8082 -> 127.0.0.1:8000)
#   3. Expo Metro en mode LAN (npx expo start --lan --port 8081)
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROBOT_UC_DIR="$REPO_ROOT/robot_uc"
MOBILE_DIR="$REPO_ROOT/mobile_app"
LOG_DIR="$REPO_ROOT/logs"
mkdir -p "$LOG_DIR"

if [ -x "$REPO_ROOT/.venv/bin/python" ]; then
    PYTHON_BIN="$REPO_ROOT/.venv/bin/python"
elif [ -x "$ROBOT_UC_DIR/.venv/bin/python" ]; then
    PYTHON_BIN="$ROBOT_UC_DIR/.venv/bin/python"
else
    PYTHON_BIN="python3"
fi

echo "========================================================================"
echo "  ROBOT AGRICOLE ESP32-S3 - COCKPIT LINUX NATIF & CONNEXION HOTSPOT LAN"
echo "========================================================================"

# 1. Libération propre des ports 8000, 8081 et 8082
echo "[1/4] Libération des ports 8000, 8081 et 8082..."
fuser -k 8000/tcp 8081/tcp 8082/tcp 2>/dev/null || true
sleep 0.5

# 2. Détection automatique de l'IP LAN/Hotspot active de la machine Linux
echo "[2/4] Détection temps réel de l'IP LAN / Hotspot Linux..."
ACTIVE_IP="$(cd "$ROBOT_UC_DIR" && "$PYTHON_BIN" -c "import main; print(main.detect_local_wifi_ip(deep_scan=True))" 2>/dev/null | tr -d '[:space:]')"
if [ -z "$ACTIVE_IP" ] || [ "$ACTIVE_IP" = "127.0.0.1" ]; then
    ACTIVE_IP="$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") {print $(i+1); exit}}')"
fi
if [ -z "$ACTIVE_IP" ]; then
    ACTIVE_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
fi
if [ -z "$ACTIVE_IP" ]; then
    ACTIVE_IP="127.0.0.1"
fi

echo "      -> IP Linux Active détectée : $ACTIVE_IP"
echo "      -> Dashboard 3D (PC)        : http://localhost:8000"
echo "      -> WebSocket FastAPI        : ws://${ACTIVE_IP}:8000/ws"
echo "      -> Pont WebSocket Node.js   : ws://${ACTIVE_IP}:8082/ws"
echo "      -> Expo Go LAN (--lan)      : exp://${ACTIVE_IP}:8081"
echo "========================================================================"

# 3. Lancement en arrière-plan de FastAPI (8000) et du Bridge WebSocket (8082)
echo "[3/4] Démarrage de FastAPI (0.0.0.0:8000) et du Bridge WebSocket (0.0.0.0:8082)..."
(cd "$ROBOT_UC_DIR" && "$PYTHON_BIN" -m uvicorn main:app --host 0.0.0.0 --port 8000) > "$LOG_DIR/fastapi.log" 2>&1 &
FASTAPI_PID=$!

node "$MOBILE_DIR/node_ws_bridge.js" > "$LOG_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!

cleanup() {
    echo ""
    echo "[Arrêt] Fermeture des serveurs FastAPI ($FASTAPI_PID), Bridge ($BRIDGE_PID) et Expo..."
    kill "$FASTAPI_PID" "$BRIDGE_PID" 2>/dev/null || true
    fuser -k 8000/tcp 8081/tcp 8082/tcp 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 4. Lancement d'Expo en mode LAN (sans tunnel) sur le port 8081
echo "[4/4] Démarrage d'Expo en mode LAN (exp://${ACTIVE_IP}:8081)..."
cd "$MOBILE_DIR"
REACT_NATIVE_PACKAGER_HOSTNAME="$ACTIVE_IP" EXPO_PUBLIC_PC_LAN_IP="$ACTIVE_IP" npx expo start --lan --port 8081 &
EXPO_PID=$!

wait "$EXPO_PID"
