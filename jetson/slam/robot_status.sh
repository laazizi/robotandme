#!/bin/bash
# mowbot — etat complet du robot en un coup d'oeil.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
echo "== services =="
for s in mowbot-agent mowbot-razor mowbot-ekf mowbot-lidar \
         mowbot-description mowbot-rosbridge mowbot-web; do
  printf "   %-22s %s\n" "$s" "$(systemctl is-active $s)"
done
echo "== processus navigation =="
printf "   %-22s %s\n" "slam_toolbox" "$(pgrep -f async_slam_toolbox >/dev/null && echo actif || echo ARRETE)"
printf "   %-22s %s\n" "nav2" "$(timeout 6 ros2 node list 2>/dev/null | grep -cE 'controller_server|planner_server')/2 nodes"
echo "== topics (Hz) =="
for t in /odom /imu/data_raw /odometry/filtered /scan /map; do
  printf "   %-22s %s\n" "$t" "$(timeout 5 ros2 topic hz $t 2>/dev/null | grep -m1 -oP 'average rate: \K[0-9.]+' || echo MUET)"
done
echo "== carte =="
if [ -f /home/nvidia/maps/mowbot.posegraph ]; then
  echo "   carte enregistree : oui (mode LOCALISATION au demarrage)"
  ls -la /home/nvidia/maps/mowbot.posegraph | awk '{print "   "$6" "$7" "$8"  "$5" octets"}'
else
  echo "   carte enregistree : non (mode CARTOGRAPHIE au demarrage)"
fi
