#!/usr/bin/env bash
# Visualisation de l'odometrie dans RViz.
# Lance le broadcaster TF (odom->base_link) puis RViz avec la bonne config.
# Prerequis : agent micro-ROS deja lance et ESP32 connecte (topic /odom actif),
#             environnement ROS 2 source (. /opt/ros/<distro>/setup.bash).
#
# Usage : ./ros2/view_odom.sh

set -eo pipefail
cd "$(dirname "$0")"

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERREUR : ROS 2 non source. Faire : . /opt/ros/jazzy/setup.bash" >&2
    exit 1
fi

echo ">> Demarrage du broadcaster TF odom -> base_link..."
python3 odom_tf_broadcaster.py &
TF_PID=$!
trap "kill $TF_PID 2>/dev/null || true" EXIT

sleep 1
echo ">> Lancement de RViz (fermez RViz pour tout arreter)..."
ros2 run rviz2 rviz2 -d odom.rviz
