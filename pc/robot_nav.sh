#!/usr/bin/env bash
# PC -> RViz "navigation" : vue du dessus, arene de test, but goto, trajectoire.
# Prerequis Jetson : services mowbot-* actifs + python3 ~/goto_goal.py
#                    (+ python3 ~/test_arena.py pour l'arene virtuelle)
# Cliquez '2D Goal Pose' pour envoyer le robot quelque part.
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
# Rendu logiciel : contourne le bug GLSL du display Map (indexed_8bit_image
# ne se lie pas sur certains pilotes OpenGL -> carte invisible sinon).
# ACTIVE : sans cela le display Map echoue a lier son shader et la carte reste
# INVISIBLE. Message caracteristique dans RViz :
#   "GLSL link result : active samplers with a different type refer to the same
#    texture image unit"  sur rviz/glsl120/indexed_8bit_image
# Le rendu logiciel coute un peu de fluidite mais rend la carte affichable.
# A recommenter si un jour le pilote graphique corrige ce defaut.
export LIBGL_ALWAYS_SOFTWARE=1
exec ros2 run rviz2 rviz2 -d nav.rviz
