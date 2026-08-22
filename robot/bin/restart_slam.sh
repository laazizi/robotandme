#!/bin/bash
# Relance du SLAM. Le mode est choisi par run_slam.sh : LOCALISATION si
# maps/mowbot.posegraph existe, CARTOGRAPHIE sinon. Pour repartir d'une carte
# vierge alors qu'une carte est enregistree :  mowbot new-map
#
# Le SLAM est un enfant du service mowbot-nav : on relance donc le SERVICE, ce
# qui redemarre aussi nav2 (compter ~2 min). C'est volontaire -- l'ancienne
# version tuait le SLAM avec `pkill -9` et le relancait avec `setsid`, hors du
# groupe de controle du service. Deux consequences vecues : systemd ne savait
# plus l'arreter (deux slam_toolbox publiant la meme TF map->odom), et le kill -9
# laissait dans /dev/shm des segments Fast DDS de 0 octet capables de rendre un
# noeud vivant mais muet (voir systemd/mowbot-shmclean.service).
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"

rm -f "$MOWBOT_LOGS/slam.log"
sudo systemctl restart mowbot-nav || {
  mowbot_log "ERREUR : impossible de relancer mowbot-nav"; exit 1; }
sleep 40
grep -m1 "SLAM en" "$MOWBOT_LOGS/slam.log" 2>/dev/null
printf "  scans rejetes : %s\n" \
  "$(grep -ci 'queue is full\|expected' "$MOWBOT_LOGS/slam.log" 2>/dev/null)"
if python3 "$MOWBOT_NODES/wait_tf.py" map odom 40; then
  printf "  /map : %s Hz\n" "$(mowbot_hz /map)"
else
  echo ">> pas de TF map->odom. Diagnostic :  mowbot logs nav"
fi
