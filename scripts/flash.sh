#!/usr/bin/env bash
# Build + flash (+ moniteur) sous Linux / WSL2 (IDF natif).
# Arrete d'abord l'agent micro-ROS (container Docker) qui tient /dev/ttyACM0,
# sinon esptool echoue avec "multiple access on port".
#
# Sous WSL2, le port USB doit etre attache avec usbipd :
#   usbipd list ; usbipd attach --wsl --busid <id>   (cote Windows, admin)
#
# Usage : ./scripts/flash.sh [/dev/ttyUSB0] [monitor]

set -euo pipefail
cd "$(dirname "$0")/.."

PORT="${1:-/dev/ttyACM0}"

# 1) Liberer le port : stopper tout container Docker (agent micro-ROS)
RUNNING="$(docker ps -q 2>/dev/null || true)"
if [ -n "$RUNNING" ]; then
    echo ">> Arret des containers Docker (liberation du port)..."
    docker stop $RUNNING
fi

# 2) Build
echo ">> Build..."
idf.py build

# 3) Flash
echo ">> Flash sur $PORT (BOOT+RST si 'Connecting...' bloque)..."
idf.py -p "$PORT" flash

if [ "${2:-}" = "monitor" ]; then
    idf.py -p "$PORT" monitor
fi
