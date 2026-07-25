#!/bin/bash
# ============================================================
#  mowbot — TOUT DEMARRER (une seule commande)
#  Usage : bash ~/robot_up.sh
# ============================================================
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

echo "== 1. services de base (agent, gyro, EKF, lidar, modele, web) =="
for s in mowbot-agent mowbot-razor mowbot-ekf mowbot-lidar \
         mowbot-description mowbot-rosbridge mowbot-web; do
  st=$(systemctl is-active $s)
  if [ "$st" != "active" ]; then
    echo "   $s : $st -> demarrage"
    echo nvidia | sudo -S systemctl start $s 2>/dev/null
  else
    echo "   $s : ok"
  fi
done

echo "== 2. navigation (SLAM + nav2) =="
bash /home/nvidia/start_nav.sh

echo "== 3. verification (30 s) =="
sleep 30
for t in /odom /imu/data_raw /odometry/filtered /scan /map; do
  printf "   %-20s " "$t"
  timeout 5 ros2 topic hz $t 2>/dev/null | grep -m1 -oP "average rate: \K[0-9.]+" || echo "MUET"
done
printf "   %-20s " "nav2"
n=$(timeout 6 ros2 node list 2>/dev/null | grep -cE "controller_server|planner_server")
[ "$n" = "2" ] && echo "ok" || echo "INCOMPLET ($n/2)"
echo
echo ">> Pret. Sur le PC : ./robot_nav.sh   |   joystick : http://$(hostname -I | awk '{print $1}'):8080/joystick.html"
