#!/bin/bash
# Sauvegarde la carte courante du SLAM.
#   - format slam_toolbox (.posegraph + .data) : permet de REPRENDRE le SLAM
#     ou de se localiser dedans ensuite
#   - format image (.pgm + .yaml) : lisible par nav2/map_server, visualisation
# Usage : bash ~/save_map.sh [nom]        (defaut : mowbot)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
NAME="${1:-mowbot}"
mkdir -p /home/nvidia/maps

echo ">> sauvegarde slam_toolbox (posegraph)..."
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: /home/nvidia/maps/$NAME}" 2>&1 | tail -2

echo ">> sauvegarde image (pgm/yaml)..."
ros2 run nav2_map_server map_saver_cli -f "/home/nvidia/maps/$NAME" --ros-args -p save_map_timeout:=10.0 2>&1 | tail -2

echo ">> fichiers :"
ls -la /home/nvidia/maps/ | grep "$NAME"
