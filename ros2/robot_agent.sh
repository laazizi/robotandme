#!/usr/bin/env bash
# PC -> lance l'agent micro-ROS SUR le Jetson, a distance via SSH.
# L'agent bridge l'ESP32 (serie) vers le reseau ROS 2 (DDS/WiFi).
# Laisser ce terminal ouvert : fermer = arreter l'agent. Ctrl+C pour stopper.
#
# Usage : ./robot_agent.sh [user@ip_jetson]   (defaut nvidia@10.31.117.2)

JETSON="${1:-nvidia@10.31.117.2}"
echo ">> Connexion au Jetson ($JETSON) et lancement de l'agent robot..."
echo ">> (mot de passe Jetson demande une fois)"
exec ssh -t "$JETSON" 'bash ~/robot_start.sh'
