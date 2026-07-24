#!/bin/bash
# Lance la pile nav2 avec la carte de l'arene (path planning REEL).
# Prerequis : services mowbot-* actifs (agent+IMU+EKF+description).
# NE PAS lancer goto_goal.py en meme temps (les deux publieraient cmd_vel) !
# Ctrl+C arrete tout proprement.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

pkill -f goto_goal.py 2>/dev/null && echo ">> goto_goal arrete (conflit cmd_vel)"

echo ">> TF map -> odom recalable via 2D Pose Estimate..."
python3 /home/nvidia/map_odom_bridge.py &
PIDS=$!

echo ">> map_server (arene 1.5 m + cylindre)..."
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/home/nvidia/maps/arena.yaml &
PIDS="$PIDS $!"

sleep 1
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p node_names:="[map_server]" -p autostart:=true -p bond_timeout:=0.0 &
PIDS="$PIDS $!"

trap 'echo; echo ">> arret nav2..."; kill $PIDS 2>/dev/null; exit 0' INT TERM

echo ">> pile nav2 (planner + controller + BT)..."
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/home/nvidia/nav2_params.yaml use_sim_time:=false
