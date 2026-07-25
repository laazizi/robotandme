#!/bin/bash
# Serveur web du joystick (port 8080).
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
cd "$MOWBOT_HOME/www" || exit 1
exec python3 -m http.server 8080
