#!/usr/bin/env bash
# Build + flash (+ moniteur) d'UN controleur, ESP-IDF natif Linux / WSL2.
# Arrete d'abord l'agent micro-ROS (container Docker) qui tient le port,
# sinon esptool echoue avec "multiple access on port".
#
# Sous WSL2, le port USB doit etre attache avec usbipd :
#   usbipd list ; usbipd attach --wsl --busid <id>   (cote Windows, admin)
#
# Usage : ./scripts/flash.sh <controleur> [/dev/ttyACM0] [monitor]
#   Waveshare P4-ETH (CH343) -> /dev/ttyACM0 ; DevKitC WROOM (CP2102) -> /dev/ttyUSB0

set -euo pipefail
cd "$(dirname "$0")/.."

CTRL="${1:-mowbot_p4}"
PORT="${2:-/dev/ttyACM0}"
DIR="controllers/$CTRL"
if [ ! -f "$DIR/main/robot.h" ]; then
    echo "ERREUR : controleur inconnu '$CTRL' -- le 1er argument est le controleur, pas le port" >&2
    exit 1
fi

# 1) Liberer le port : stopper tout container Docker (agent micro-ROS)
RUNNING="$(docker ps -q 2>/dev/null || true)"
if [ -n "$RUNNING" ]; then
    echo ">> Arret des containers Docker (liberation du port)..."
    docker stop $RUNNING
fi

# 2) Build, par le script (jamais idf.py directement)
./scripts/build.sh "$CTRL"

# 3) Flash
cd "$DIR"
echo ">> Flash de $CTRL sur $PORT (BOOT+RST si 'Connecting...' bloque)..."
idf.py -p "$PORT" flash

if [ "${3:-}" = "monitor" ]; then
    idf.py -p "$PORT" monitor
fi
