#!/bin/bash
# TF statiques : base_link -> laser_link et -> imu_link.
# Regroupees ici (service dedie) car : `ros2 run` echoue sous systemd, et
# lancees depuis les scripts capteurs elles s'empilaient a chaque redemarrage
# -> TF concurrentes -> le SLAM rejetait tous les scans.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
# Position du lidar. ATTENTION : elle DIFFERE selon le robot.
#   robot 24 V (DevKitC) : 10 cm DEVANT l'axe des roues
#   robot 12 V (P4)      : 10 cm DERRIERE  -> x NEGATIF
# Se tromper de signe decale chaque mesure de 20 cm dans la carte : les murs
# sont places trop loin (ou trop pres) et le recalage du SLAM travaille contre
# des donnees fausses, sans qu'aucune erreur ne soit signalee.
# Surchargeable sans toucher au script : MOWBOT_LIDAR_X=0.10
# -0.16 : MESURE AU METRE entre l'axe des roues et le centre du lidar, sur le
# robot A. La valeur precedente (-0.10) etait une estimation, et l'erreur de
# 6 cm se voyait : un mur touche par l'avant du chassis etait rapporte a 28.7 cm
# du centre du robot alors que le plateau n'a que 22 cm de rayon.
# Recoupement independant : en resolvant l'ecart a partir du scan seul, on
# obtenait -0.167. Les deux methodes concordent a 7 mm.
# L'AUTRE ROBOT du projet a son lidar DEVANT (+0.10) : ne pas transposer.
# Toute modification ici doit etre reportee dans config/mowbot.urdf
# (joint deck_to_lidar), sinon le modele dessine le lidar a un endroit et les
# scans sortent d'un autre.
LIDAR_X="${MOWBOT_LIDAR_X:--0.16}"
LIDAR_Z="${MOWBOT_LIDAR_Z:-0.28}"
# yaw : un lidar 360 deg n'a pas de champ oriente, mais son ANGLE ZERO doit
# regarder vers l'avant du robot. Boitier tourne = carte PIVOTEE d'autant, donc
# obstacles places a l'oppose de la realite -- sans aucune erreur signalee.
# MESURE sur le robot 12 V (nodes/lidar_front.py, objet au contact de l'avant) :
#   secteur AVANT   : 2.62 m  (rien vu)
#   secteur ARRIERE : 0.41 m  (l'objet est la)
# Le boitier est donc monte a 180 deg -> yaw = pi.
LIDAR_YAW="${MOWBOT_LIDAR_YAW:-3.14159}"
mowbot_log "lidar a x=$LIDAR_X z=$LIDAR_Z yaw=$LIDAR_YAW"
"$STATIC_TF_BIN" --x "$LIDAR_X" --y 0 --z "$LIDAR_Z" \
  --roll 0 --pitch 0 --yaw "$LIDAR_YAW" \
  --frame-id base_link --child-frame-id laser_link &
"$STATIC_TF_BIN" --x 0 --y 0 --z 0.05 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id imu_link &
wait
