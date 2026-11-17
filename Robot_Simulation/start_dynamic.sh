#!/usr/bin/env bash
# ==============================================================================
# start_dynamic.sh — Lancement Natif Linux 100 % LAN / Hotspot (Sans Tunnel)
# Lance :
#   1. FastAPI (0.0.0.0:8000)
#   2. Relais Node.js node_ws_bridge.js (0.0.0.0:8082 -> 127.0.0.1:8000)
#   3. Expo Metro en mode LAN (npx expo start --lan --port 8081)
# ==============================================================================

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_ROOT"

PYTHON_BIN="./.venv/bin/python"
if [ ! -x "$PYTHON_BIN" ]; then
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
ACTIVE_IP="$($PYTHON_BIN -c "import main; print(main.detect_local_wifi_ip(deep_scan=True))" 2>/dev/null | tr -d '[:space:]')"
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
./.venv/bin/python -m uvicorn main:app --host 0.0.0.0 --port 8000 > "$PROJECT_ROOT/fastapi.log" 2>&1 &
FASTAPI_PID=$!

node node_ws_bridge.js > "$PROJECT_ROOT/bridge.log" 2>&1 &
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
mkdir -p "$PROJECT_ROOT/.expo"
export HOME="$PROJECT_ROOT"
cd "$PROJECT_ROOT/mobile_app"
REACT_NATIVE_PACKAGER_HOSTNAME="$ACTIVE_IP" EXPO_PUBLIC_PC_LAN_IP="$ACTIVE_IP" npx expo start --lan --port 8081 &
EXPO_PID=$!

wait "$EXPO_PID"

