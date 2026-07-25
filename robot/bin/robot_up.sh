#!/bin/bash
# TOUT demarrer (services + navigation) puis verifier.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
echo "== services =="
for s in mowbot-tf mowbot-agent mowbot-razor mowbot-ekf mowbot-lidar \
         mowbot-description mowbot-rosbridge mowbot-web; do
  st=$(systemctl is-active $s 2>/dev/null)
  if [ "$st" != "active" ]; then
    printf "   %-22s %s -> demarrage\n" "$s" "$st"
    sudo systemctl start $s 2>/dev/null
  else
    printf "   %-22s ok\n" "$s"
  fi
done
echo "== navigation =="
bash "$MOWBOT_BIN/start_nav.sh"
echo "== verification (30 s) =="
sleep 30
for t in /odom /imu/data_raw /odometry/filtered /scan /map; do
  printf "   %-22s %s\n" "$t" "$(mowbot_hz $t || echo MUET)"
done
printf "   %-22s %s/2\n" "nav2" "$(timeout 6 ros2 node list 2>/dev/null | grep -cE 'controller_server|planner_server')"
echo
echo ">> PC : ./robot_nav.sh   |   joystick : http://$(hostname -I | awk '{print $1}'):8080/joystick.html"
