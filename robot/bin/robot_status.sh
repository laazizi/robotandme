#!/bin/bash
# Etat complet du robot en un ecran.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
echo "== machine =="
printf "   %-22s %s\n" "hote" "$(hostname) ($(hostname -I | awk '{print $1}'))"
printf "   %-22s %s\n" "ROS" "$MOWBOT_ROS_DISTRO (domain $ROS_DOMAIN_ID)"
printf "   %-22s %s\n" "installation" "$MOWBOT_HOME"
echo "== peripheriques (liens udev) =="
for l in mowbot_esp32 mowbot_lidar mowbot_imu; do
  printf "   %-22s %s\n" "/dev/$l" "$(readlink /dev/$l 2>/dev/null || echo ABSENT)"
done
echo "== services =="
for s in mowbot-agent mowbot-razor mowbot-ekf mowbot-lidar mowbot-tf \
         mowbot-description mowbot-rosbridge mowbot-web mowbot-nav; do
  printf "   %-22s %s\n" "$s" "$(systemctl is-active $s 2>/dev/null)"
done
echo "== navigation =="
printf "   %-22s %s\n" "slam_toolbox" "$(pgrep -f async_slam_toolbox >/dev/null && echo actif || echo ARRETE)"
printf "   %-22s %s\n" "nav2" "$(timeout 6 ros2 node list 2>/dev/null | grep -cE 'controller_server|planner_server')/2"
echo "== topics (Hz) =="
for t in /odom /imu/data_raw /odometry/filtered /scan /map; do
  printf "   %-22s %s\n" "$t" "$(mowbot_hz $t || echo MUET)"
done
echo "== carte =="
if [ -f "$MOWBOT_MAPS/mowbot.posegraph" ]; then
  echo "   enregistree : OUI -> demarrage en LOCALISATION"
else
  echo "   enregistree : non -> demarrage en CARTOGRAPHIE"
fi
