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
# Test par un noeud PYTHON et non `ros2 run tf2_ros tf2_echo` : sous systemd,
# `ros2 run` ne trouve pas ses paquets (deja constate pour les TF statiques et
# le driver lidar). Le test echouait donc TOUJOURS, et l'on attendait 80 s pour
# rien avant de lancer nav2 en annoncant a tort une carte absente.
if python3 "$MOWBOT_NODES/wait_tf.py" map odom 90; then
  MAP_OK=1
else
  MAP_OK=0
  # dernier essai : le SLAM est peut-etre retombe en inactif
  slam_activate
  python3 "$MOWBOT_NODES/wait_tf.py" map odom 30 && MAP_OK=1
fi
if [ "$MAP_OK" = "0" ]; then
  mowbot_log "ATTENTION : map->odom absente. nav2 va demarrer mais le planner"
  mowbot_log "            echouera. Verifier : mowbot logs mowbot-nav"
fi

# --- PROFIL DE VITESSE, par robot -------------------------------------------
# config/nav2_params.yaml porte les valeurs du robot B (24 V, motoreducteurs a
# 0.055 m/s). Appliquees au robot A (12 V, capable de 1 m/s) elles le faisaient
# ramper 18 fois trop lentement -- c'etait la cause de la "navigation lente",
# plus lente que le joystick. On genere donc une COPIE du fichier avec les
# limites du robot courant ; le fichier de reference n'est jamais modifie.
#
# Ne PAS remplacer ce mecanisme par une edition directe du YAML : les deux
# robots partagent ce depot, et envoyer les consignes du 12 V au 24 V ferait
# saturer son PID (asservissement qui decroche).
ROBOT="${MOWBOT_ROBOT:-}"
if [ -z "$ROBOT" ] && [ -f "$MOWBOT_HOME/robot_profile.env" ]; then
  . "$MOWBOT_HOME/robot_profile.env"
  ROBOT="${MOWBOT_ROBOT:-}"
fi
# Defaut B par SECURITE : des vitesses trop basses ne cassent rien, l'inverse si.
ROBOT="$(printf '%s' "${ROBOT:-b}" | tr 'A-Z' 'a-z')"

PARAMS="$MOWBOT_CONFIG/nav2_params.yaml"
SPEEDS="$MOWBOT_CONFIG/speeds.env"
if [ -f "$SPEEDS" ]; then
  . "$SPEEDS"
  P="$(printf '%s' "$ROBOT" | tr 'a-z' 'A-Z')"
  GEN="$MOWBOT_LOGS/nav2_params_$ROBOT.yaml"
  if cp "$PARAMS" "$GEN" 2>/dev/null; then
    # Remplace le NOMBRE apres la cle, en gardant l'indentation et le
    # commentaire de fin de ligne (precieux : ces valeurs sont justifiees).
    set_num() {
      eval "v=\${${P}_$2:-}"
      [ -n "$v" ] && sed -i -E "s|^([[:space:]]*)$1:[[:space:]]*-?[0-9.]+|\1$1: $v|" "$GEN"
    }
    set_num min_vel_x        MIN_VEL_X
    set_num max_vel_x        MAX_VEL_X
    set_num max_speed_xy     MAX_SPEED_XY
    set_num max_vel_theta    MAX_VEL_THETA
    set_num acc_lim_x        ACC_LIM_X
    set_num acc_lim_theta    ACC_LIM_THETA
    set_num decel_lim_x      DECEL_LIM_X
    set_num decel_lim_theta  DECEL_LIM_THETA
    set_num sim_time         SIM_TIME
    # velocity_smoother : listes [x, y, theta]. Il BRIDE la sortie du
    # controleur ; l'oublier annulerait tout le reste.
    eval "vx=\${${P}_MAX_VEL_X:-}"; eval "vn=\${${P}_MIN_VEL_X:-}"
    eval "vt=\${${P}_MAX_VEL_THETA:-}"
    eval "ax=\${${P}_ACC_LIM_X:-}";  eval "at=\${${P}_ACC_LIM_THETA:-}"
    eval "dx=\${${P}_DECEL_LIM_X:-}"; eval "dt=\${${P}_DECEL_LIM_THETA:-}"
    [ -n "$vx" ] && sed -i -E "s|^([[:space:]]*)max_velocity:.*|\1max_velocity: [$vx, 0.0, $vt]|" "$GEN"
    [ -n "$vn" ] && sed -i -E "s|^([[:space:]]*)min_velocity:.*|\1min_velocity: [$vn, 0.0, -$vt]|" "$GEN"
    [ -n "$ax" ] && sed -i -E "s|^([[:space:]]*)max_accel:.*|\1max_accel: [$ax, 0.0, $at]|" "$GEN"
    [ -n "$dx" ] && sed -i -E "s|^([[:space:]]*)max_decel:.*|\1max_decel: [$dx, 0.0, $dt]|" "$GEN"
    PARAMS="$GEN"
    mowbot_log "profil de vitesse : robot ${P} (max_vel_x=${vx:-?} m/s, max_vel_theta=${vt:-?} rad/s)"
  else
    mowbot_log "ATTENTION : copie de nav2_params.yaml impossible, profil non applique"
  fi
fi

mowbot_log "demarrage nav2"
ros2 launch nav2_bringup navigation_launch.py \
  params_file:="$PARAMS" use_sim_time:=false \
  > "$MOWBOT_LOGS/nav2.log" 2>&1 < /dev/null &
NAV2_PID=$!
mowbot_log "nav2 monte en ~30 s"

# On reste au premier plan : le service est Type=simple, et c'est ce `wait` qui
# maintient SLAM et nav2 dans son groupe de controle. A l'arret, systemd tue le
# groupe entier -- plus de processus orphelins.
trap 'mowbot_log "arret : SLAM et nav2"; kill $SLAM_PID $NAV2_PID 2>/dev/null' INT TERM
wait
