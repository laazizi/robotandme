#!/bin/bash
# SLAM : LOCALISATION si une carte sauvegardee existe, sinon CARTOGRAPHIE.
#   - carte presente -> le robot se retrouve tout seul dedans (recalage par
#     scan matching ; un clic RViz "2D Pose Estimate" aide s'il hesite)
#   - pas de carte    -> il en construit une neuve en roulant
# Forcer la cartographie : MOWBOT_SLAM_MODE=mapping bash ~/run_slam.sh
# Sauvegarder la carte  : bash ~/save_map.sh
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

MAP=/home/nvidia/maps/mowbot.posegraph
CFG=/home/nvidia/slam_params.yaml
if [ -f "$MAP" ] && [ "${MOWBOT_SLAM_MODE}" != "mapping" ]; then
  CFG=/home/nvidia/slam_params_loc.yaml
  echo ">> SLAM en LOCALISATION (carte $MAP)"
else
  echo ">> SLAM en CARTOGRAPHIE (nouvelle carte)"
fi
exec ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file "$CFG"
