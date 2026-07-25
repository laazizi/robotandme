#!/bin/bash
# robot_state_publisher : modele URDF + TF des roues.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
# L'URDF est charge par un launch file (nodes/description.launch.py) : le
# passer en argument (-p robot_description:=...) casse l'analyse des arguments
# ROS des qu'il contient des sauts de ligne -> le noeud redemarrait en boucle
# sans jamais publier le modele.
exec ros2 launch "$MOWBOT_NODES/description.launch.py"
