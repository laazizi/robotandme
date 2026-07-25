#!/bin/bash
# Relance PROPRE du SLAM (kill -9 garanti puis verification).
pkill -9 -f async_slam_toolbox 2>/dev/null
sleep 3
if pgrep -f async_slam_toolbox >/dev/null; then echo "ECHEC: slam survit"; exit 1; fi
rm -f /tmp/slam_run.log
setsid bash /home/nvidia/run_slam.sh > /tmp/slam_run.log 2>&1 < /dev/null &
sleep 20
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
echo -n "slam PID : "; pgrep -f async_slam_toolbox | head -1
echo -n "rejets de scans : "; grep -c "expected" /tmp/slam_run.log
echo -n "/map : "; timeout 10 ros2 topic hz /map 2>/dev/null | grep -m1 average || echo muet
echo -n "TF map->odom : "; timeout 6 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -m1 Translation || echo absente
