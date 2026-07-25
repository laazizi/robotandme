#!/bin/bash
# SLAM en ligne : construit la carte pendant qu'on roule et publie map->odom.
# Remplace map_server + map_odom_bridge (carte figee).
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
exec ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args --params-file /home/nvidia/slam_params.yaml
