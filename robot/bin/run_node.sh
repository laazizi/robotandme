#!/bin/bash
# Lance un noeud/outil du dossier nodes/ avec l'environnement ROS pret.
# Usage : run_node.sh motion_test.py [args...]
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
N="$1"; shift
[ -f "$MOWBOT_NODES/$N" ] || { echo "introuvable : $MOWBOT_NODES/$N"; ls "$MOWBOT_NODES"; exit 1; }
exec python3 -u "$MOWBOT_NODES/$N" "$@"
