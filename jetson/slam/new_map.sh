#!/bin/bash
# ============================================================
#  mowbot — NOUVELLE CARTE (nouvel endroit / repartir a zero)
#  Archive l'ancienne carte puis relance le SLAM en cartographie.
#  Usage : bash ~/new_map.sh
# ============================================================
D=/home/nvidia/maps
if ls $D/mowbot.* >/dev/null 2>&1; then
  BK="$D/archive_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$BK" && mv $D/mowbot.* "$BK"/ 2>/dev/null
  echo ">> ancienne carte archivee dans $BK"
else
  echo ">> aucune carte a archiver"
fi
echo ">> relance du SLAM en CARTOGRAPHIE (carte vierge)"
export MOWBOT_SLAM_MODE=mapping
pkill -9 -f async_slam_toolbox 2>/dev/null; sleep 3
rm -f /tmp/slam_run.log
setsid env MOWBOT_SLAM_MODE=mapping bash /home/nvidia/run_slam.sh > /tmp/slam_run.log 2>&1 < /dev/null &
sleep 18
source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0
grep -m1 "SLAM en" /tmp/slam_run.log
printf "carte : "; timeout 8 ros2 topic hz /map 2>/dev/null | grep -m1 -oP "average rate: \K[0-9.]+" || echo MUET
echo ">> roule dans la piece, puis :  bash ~/save_map.sh"
