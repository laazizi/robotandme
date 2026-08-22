#!/bin/bash
# Lidar -> /scan_raw, puis scan_fix.py -> /scan.
# TF statiques : voir run_tf.sh (service dedie).
#
# DEUX MODELES SUPPORTES, choisis par $MOWBOT_LIDAR (sinon auto-detection) :
#   n10   LSLidar N10   : 10 Hz, 450 pts/tour, 12 m — driver C++ dans ~/lidar_ws
#   ld14  LDRobot LD14  :  6 Hz, 391 pts/tour,  8 m — noeud Python, rien a compiler
# L'auto-detection retient le N10 si son driver est compile, sinon le LD14
# (cas de la Raspberry Pi, ou compiler du C++ est long et risque en memoire).
# Forcer un modele :  export MOWBOT_LIDAR=ld14   (ou n10)
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"

WS="${MOWBOT_LIDAR_WS:-$HOME/lidar_ws/install}"
N10_BIN="$WS/lslidar_driver/lib/lslidar_driver/lslidar_driver_node"

MODEL="${MOWBOT_LIDAR:-}"
# modele reconnu lors du dernier `mowbot detect` (sonde du flux serie)
if [ -z "$MODEL" ] && [ -f "$MOWBOT_HOME/lidar_model.env" ]; then
  . "$MOWBOT_HOME/lidar_model.env"
  MODEL="${MOWBOT_LIDAR:-}"
  [ -n "$MODEL" ] && mowbot_log "modele issu de la detection : $MODEL"
fi
if [ -z "$MODEL" ]; then
  if [ -x "$N10_BIN" ]; then MODEL=n10; else MODEL=ld14; fi
  mowbot_log "modele de lidar suppose : $MODEL"
fi
# LD06/LD19 : meme protocole que le LD14, seule la vitesse serie change
[ "$MODEL" = "ld06" ] && LD_BAUD=230400

mowbot_wait_dev "$DEV_LIDAR" 20 || mowbot_log "ATTENTION : $DEV_LIDAR absent"

# scan_fix : nombre de points fixe, masquage du robot, filtre des faux echos
pkill -f scan_fix.py 2>/dev/null; sleep 1
python3 "$MOWBOT_NODES/scan_fix.py" > "$MOWBOT_LOGS/scan_fix.log" 2>&1 &

case "$MODEL" in
  n10)
    if [ ! -x "$N10_BIN" ]; then
      mowbot_log "ERREUR : driver N10 absent ($N10_BIN). Le compiler dans ~/lidar_ws,"
      mowbot_log "         ou basculer sur le LD14 : export MOWBOT_LIDAR=ld14"
      exit 1
    fi
    export LD_LIBRARY_PATH="$WS/lslidar_msgs/lib:$WS/lslidar_driver/lib:$LD_LIBRARY_PATH"
    export AMENT_PREFIX_PATH="$WS/lslidar_driver:$WS/lslidar_msgs:$AMENT_PREFIX_PATH"
    # binaire appele en direct : `ros2 run` echoue en contexte systemd
    exec "$N10_BIN" --ros-args --params-file "$MOWBOT_CONFIG/lidar_n10.yaml"
    ;;
  ld14|ld06|ld19)
    # Driver NATIF prioritaire : le noeud Python equivalent coute ~25 % de CPU
    # sur une Raspberry Pi 4 (decodage trame par trame en Python), contre
    # quelques pourcents en C++. Une tentative de vectorisation numpy a ete
    # ESSAYEE et ABANDONNEE : sur des lots de 1 a 2 trames, l'overhead depasse
    # le gain -- 34 % de CPU et le scan tombe a 3.7 Hz.
    # CHOIX DU DRIVER : natif par defaut, Python en repli.
    #   MOWBOT_LIDAR_DRIVER=native  -> C++ officiel, ~2.5 % de CPU  (DEFAUT)
    #   MOWBOT_LIDAR_DRIVER=python  -> noeud maison, ~32 % de CPU
    #
    # Le natif a un temps ete ACCUSE A TORT d'avoir casse le SLAM : celui-ci
    # rejetait tous les scans ("Message Filter dropping ... queue is full") et
    # ne publiait plus de carte. Le vrai coupable etait un segment Fast DDS
    # corrompu dans /dev/shm (fichier de 0 octet) qui empechait l'EKF de
    # recevoir /odom -- sans TF odom->base_link, aucun scan n'est placable, quel
    # que soit le driver. Voir systemd/mowbot-shmclean.service. Le retour au
    # noeud Python n'avait donc rien change : il rejetait autant.
    # Mesure a l'appui : ld14_node.py coutait 32 % d'un coeur, soit le premier
    # poste de CPU du robot, pour un resultat identique.
    LD_WS="${MOWBOT_LDLIDAR_WS:-$HOME/ldlidar_ws/install}"
    LD_BIN="$LD_WS/ldlidar_sl_ros2/lib/ldlidar_sl_ros2/ldlidar_sl_ros2_node"
    if [ "${MOWBOT_LIDAR_DRIVER:-native}" = "native" ] && [ -x "$LD_BIN" ]; then
      export LD_LIBRARY_PATH="$LD_WS/ldlidar_sl_ros2/lib:$LD_LIBRARY_PATH"
      export AMENT_PREFIX_PATH="$LD_WS/ldlidar_sl_ros2:$AMENT_PREFIX_PATH"
      # Le driver publie sur le topic demande ; on le dirige vers /scan_raw pour
      # que scan_fix.py garde son role (points fixes, masquage, filtrage).
      # laser_scan_dir=true : sens trigonometrique, comme le noeud Python.
      mowbot_log "driver NATIF ldlidar_sl_ros2"
      exec "$LD_BIN" --ros-args \
        -p product_name:=LDLiDAR_LD14 \
        -p port_name:="$DEV_LIDAR" \
        -p serial_baudrate:="${LD_BAUD:-115200}" \
        -p laser_scan_topic_name:=scan_raw \
        -p point_cloud_2d_topic_name:=pointcloud2d_raw \
        -p frame_id:=laser_link \
        -p laser_scan_dir:=true \
        -p enable_angle_crop_func:=false
    fi
    mowbot_log "noeud Python (repli ; le natif coute 30 % de CPU en moins)"
    exec python3 "$MOWBOT_NODES/ld14_node.py" \
      --ros-args --params-file "$MOWBOT_CONFIG/lidar_ld14.yaml" \
      ${LD_BAUD:+-p baudrate:=$LD_BAUD}
    ;;
  *)
    mowbot_log "ERREUR : MOWBOT_LIDAR inconnu ('$MODEL'). Attendu : n10 ou ld14."
    exit 1
    ;;
esac
