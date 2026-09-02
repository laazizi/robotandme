#!/bin/bash
# Lance un noeud/outil du dossier nodes/ avec l'environnement ROS pret.
# Usage : run_node.sh motion_test.py [args...]
#
# EN MODE CONTENEUR, ON RELAIE VERS LE CONTENEUR. L'hote n'a pas de ROS : cette
# machine est en Ubuntu 20.04 et n'a pas droit a Jazzy, toute la pile vit dans
# le conteneur. Sans ce relais, `mowbot node X.py` echouait sur
#   ModuleNotFoundError: No module named 'rclpy'
# ce qui invite a faire `pip install rclpy` -- PIEGE : rclpy ne s'installe pas
# par pip, il fait partie de l'installation ROS. Le paquet pip homonyme n'est
# pas le bon et laisse un environnement casse, penible a demeler.
# Meme logique et memes precautions que bin/ros2, voir son en-tete.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
N="$1"; shift
[ -f "$MOWBOT_NODES/$N" ] || { echo "introuvable : $MOWBOT_NODES/$N"; ls "$MOWBOT_NODES"; exit 1; }

# ROS present localement (Raspberry Pi, ou dans le conteneur) : on lance direct.
if [ -z "$MOWBOT_NO_ROS" ]; then
  exec python3 -u "$MOWBOT_NODES/$N" "$@"
fi

NOM="${MOWBOT_CONTAINER:-mowbot_jazzy}"
DISTRO="${MOWBOT_ROS_DISTRO:-jazzy}"
if ! docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$NOM"; then
  echo "run_node : pas de ROS sur cet hote, et le conteneur $NOM ne tourne pas." >&2
  echo "           le demarrer :  sudo systemctl start mowbot-container" >&2
  exit 1
fi

# -t seulement sur un terminal : sinon un noeud en pipe echoue sur
# "the input device is not a TTY".
TT=""
[ -t 0 ] && [ -t 1 ] && TT="-t"

# PYTHONUNBUFFERED : sans terminal, python tamponne sa sortie et un noeud tue
# par un `timeout` n'affiche RIEN, ce qui fait croire a une panne.
# Le code est monte au MEME chemin dans le conteneur : $MOWBOT_NODES est donc
# valide des deux cotes, rien a traduire.
exec docker exec -i $TT \
  -e PYTHONUNBUFFERED=1 \
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
  "$NOM" bash -c \
  "source /opt/ros/$DISTRO/setup.bash && exec python3 -u \"\$@\"" \
  python3 "$MOWBOT_NODES/$N" "$@"
