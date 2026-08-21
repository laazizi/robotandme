#!/bin/bash
# Pile NAVIGATION : SLAM (carte en roulant ou localisation) + nav2.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
for p in nav2_bringup bt_navigator controller_server planner_server behavior_server \
         smoother_server velocity_smoother waypoint_follower async_slam_toolbox; do
  pkill -f "$p" 2>/dev/null
done
sleep 3
mowbot_log "demarrage SLAM"
# PAS de setsid : le processus doit rester dans le groupe de controle du
# service, sinon systemd ne peut plus l'arreter et il survit aux redemarrages
# (on se retrouvait avec deux slam_toolbox et deux EKF publiant la meme TF).
bash "$MOWBOT_BIN/run_slam.sh" > "$MOWBOT_LOGS/slam.log" 2>&1 < /dev/null &
SLAM_PID=$!
sleep 8

# slam_toolbox est un noeud a CYCLE DE VIE : lance seul il reste
# "unconfigured", ne s'abonne donc pas a /scan et ne publie ni carte ni TF
# map->odom (constate sous Jazzy : /scan avait 0 abonne). On le fait passer
# explicitement en actif. Sans effet s'il s'est deja auto-configure.
slam_activate() {
  local st
  st="$(ros2 lifecycle get /slam_toolbox 2>/dev/null | head -1)"
  case "$st" in
    *unconfigured*)
      mowbot_log "slam_toolbox : configure puis activate"
      ros2 lifecycle set /slam_toolbox configure >/dev/null 2>&1
      sleep 3
      ros2 lifecycle set /slam_toolbox activate  >/dev/null 2>&1 ;;
    *inactive*)
      mowbot_log "slam_toolbox : activate"
      ros2 lifecycle set /slam_toolbox activate  >/dev/null 2>&1 ;;
    *active*)  mowbot_log "slam_toolbox deja actif" ;;
    *)         mowbot_log "slam_toolbox : etat inconnu ('$st')" ;;
  esac
}
for i in 1 2 3 4 5 6 7 8 9 10; do
  ros2 node list 2>/dev/null | grep -q '^/slam_toolbox$' && break
  sleep 2
done
slam_activate
sleep 2
mowbot_log "slam_toolbox : $(ros2 lifecycle get /slam_toolbox 2>/dev/null | head -1)"

# ATTENDRE que le repere `map` existe avant de lancer nav2. Sans cela le
# planner_server demarre trop tot, ne trouve pas la transformation
# ("Invalid frame ID \"map\" ... frame does not exist"), echoue a se
# configurer, et le lifecycle_manager AVORTE toute la pile. L'ordre de
# demarrage etait la vraie cause : une temporisation fixe ne suffit pas, le
# SLAM met un temps variable a produire sa premiere carte.
mowbot_log "attente du repere map (le SLAM doit publier map->odom)"
MAP_OK=0
for i in $(seq 1 40); do
  # tf2_echo tourne en boucle et ne rend pas la main : on le borne et on
  # cherche sa sortie. `--once` ne fait PAS ce qu'on croit ici (il rendait
  # toujours un code non nul), d'ou un faux negatif qui declenchait l'alerte
  # alors que map->odom etait bien la.
  if timeout 4 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -q "Translation"; then
    MAP_OK=1; mowbot_log "map->odom disponible apres ${i}x2 s"; break
  fi
  # relance l'activation si le SLAM est retombe (redemarrage du service)
  [ $((i % 10)) -eq 0 ] && slam_activate
  sleep 2
done
if [ "$MAP_OK" = "0" ]; then
  mowbot_log "ATTENTION : map->odom absente apres 80 s. nav2 va demarrer mais"
  mowbot_log "            le planner echouera. Verifier : mowbot logs mowbot-nav"
fi

mowbot_log "demarrage nav2"
ros2 launch nav2_bringup navigation_launch.py \
  params_file:="$MOWBOT_CONFIG/nav2_params.yaml" use_sim_time:=false \
  > "$MOWBOT_LOGS/nav2.log" 2>&1 < /dev/null &
NAV2_PID=$!
mowbot_log "nav2 monte en ~30 s"

# On reste au premier plan : le service est Type=simple, et c'est ce `wait` qui
# maintient SLAM et nav2 dans son groupe de controle. A l'arret, systemd tue le
# groupe entier -- plus de processus orphelins.
trap 'mowbot_log "arret : SLAM et nav2"; kill $SLAM_PID $NAV2_PID 2>/dev/null' INT TERM
wait
