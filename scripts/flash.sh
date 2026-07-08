#!/usr/bin/env bash
# Flash + moniteur sous Linux / WSL2 (IDF natif).
# Sous WSL2, le port USB doit etre attache avec usbipd :
#   usbipd list ; usbipd attach --wsl --busid <id>   (cote Windows, admin)
#
# Usage : ./scripts/flash.sh [/dev/ttyUSB0] [monitor]

set -euo pipefail
cd "$(dirname "$0")/.."

PORT="${1:-/dev/ttyUSB0}"

idf.py -p "$PORT" flash

if [ "${2:-}" = "monitor" ]; then
    idf.py -p "$PORT" monitor
fi
