#!/usr/bin/env bash
# Publie une TF map -> odom FIGEE, pour visualiser sans SLAM.
#
# POURQUOI : nav.rviz affiche tout dans le repere `map`. Or `map` n'existe que
# si slam_toolbox tourne (service mowbot-nav). Sans lui, RViz met chaque
# message en attente d'une transformation qui n'arrive jamais, sature sa file
# et jette tout :
#   "Message Filter dropping message: frame 'odom' ... queue is full"
#
# Ce script comble le trou en posant map = odom. On voit alors le robot, son
# odometrie et le scan lidar. En revanche il n'y a AUCUNE correction de
# derive : c'est un outil de visualisation, pas de localisation.
#
# A NE PAS lancer si le SLAM tourne : il publie deja map -> odom, et deux
# emetteurs feraient osciller la transformation (exactement le defaut qu'on
# cherche a eviter). Le script verifie et refuse.
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

if pgrep -f "static_transform_publisher.*map.*odom" > /dev/null 2>&1; then
  echo "Une TF map -> odom est deja publiee ici. Rien a faire."
  exit 0
fi

if timeout 8 ros2 node list 2>/dev/null | grep -q slam_toolbox; then
  echo "ERREUR : slam_toolbox tourne, il publie deja map -> odom." >&2
  echo "         Ajouter une seconde source ferait osciller la TF." >&2
  exit 1
fi

echo ">> map -> odom figee (visualisation sans SLAM, pas de correction de derive)"
echo "   Ctrl+C pour arreter. Demarrer le vrai SLAM : mowbot nav (sur le robot)."
exec ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id map --child-frame-id odom
