#!/bin/bash
# Lidar LSLidar N10 : driver -> /scan_raw, scan_fix -> /scan.
# Les TF statiques sont dans le service dedie mowbot-tf (PAS ici : `ros2 run`
# echoue sous systemd et les publishers s'empilaient a chaque redemarrage).
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
WS=/home/nvidia/lidar_ws/install
export LD_LIBRARY_PATH="$WS/lslidar_msgs/lib:$WS/lslidar_driver/lib:$LD_LIBRARY_PATH"
export AMENT_PREFIX_PATH="$WS/lslidar_driver:$WS/lslidar_msgs:$AMENT_PREFIX_PATH"

pkill -f scan_fix.py 2>/dev/null; sleep 1
python3 /home/nvidia/scan_fix.py > /tmp/scan_fix.log 2>&1 &

exec "$WS/lslidar_driver/lib/lslidar_driver/lslidar_driver_node" \
  --ros-args --params-file /home/nvidia/lidar_n10.yaml
