#!/bin/bash
# Pile NAVIGATION AVEC SLAM : la carte se construit en roulant (slam_toolbox),
# nav2 planifie dessus, le lidar fournit les obstacles.
#   slam_toolbox remplace map_server + map_odom_bridge (plus de carte figee,
#   plus d'arene virtuelle) et publie lui-meme la TF map->odom.
for p in map_odom_bridge map_server lifecycle_manager nav2_bringup bt_navigator \
         controller_server planner_server behavior_server smoother_server \
         velocity_smoother waypoint_follower test_arena run_arena async_slam_toolbox; do
  pkill -f $p 2>/dev/null
done
sleep 3

echo ">> SLAM (carte construite en roulant)..."
setsid bash /home/nvidia/run_slam.sh > /tmp/slam_run.log 2>&1 < /dev/null &
sleep 8

echo ">> pile nav2..."
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
setsid ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/home/nvidia/nav2_params.yaml use_sim_time:=false \
  > /tmp/nav2.log 2>&1 < /dev/null &
echo ">> lance (nav2 monte en ~30 s)"
