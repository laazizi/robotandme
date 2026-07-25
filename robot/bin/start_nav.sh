#!/bin/bash
# Pile NAVIGATION : SLAM (carte en roulant ou localisation) + nav2.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
for p in nav2_bringup bt_navigator controller_server planner_server behavior_server \
         smoother_server velocity_smoother waypoint_follower async_slam_toolbox; do
  pkill -f "$p" 2>/dev/null
done
sleep 3
mowbot_log "demarrage SLAM"
setsid bash "$MOWBOT_BIN/run_slam.sh" > "$MOWBOT_LOGS/slam.log" 2>&1 < /dev/null &
sleep 8
mowbot_log "demarrage nav2"
setsid ros2 launch nav2_bringup navigation_launch.py \
  params_file:="$MOWBOT_CONFIG/nav2_params.yaml" use_sim_time:=false \
  > "$MOWBOT_LOGS/nav2.log" 2>&1 < /dev/null &
mowbot_log "nav2 monte en ~30 s"
