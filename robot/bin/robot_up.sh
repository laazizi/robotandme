#!/bin/bash
# TOUT demarrer (services + navigation) puis verifier.
#   robot_up.sh              demarre ce qui manque, verifie, repare si besoin
#   robot_up.sh --force      relance TOUTE la pile dans l'ordre, sans diagnostic
#   robot_up.sh --dry-run    montre ce qui serait fait, n'execute rien
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"

DRY=0; FORCE=0
for a in "$@"; do
  case "$a" in
    --force)   FORCE=1 ;;
    --dry-run) DRY=1 ;;
  esac
done

# Ordre de relance. Il n'est pas decoratif : les TF statiques doivent exister
# AVANT le SLAM, sinon slam_toolbox ne trouve pas `laser_link`, jette tous les
# scans ("Message Filter dropping ... the timestamp on the message is earlier
# than all the data in the transform cache") et ne publie jamais map->odom --
# donc nav2 ne demarre pas.
ORDER="mowbot-tf mowbot-description mowbot-agent mowbot-razor mowbot-lidar mowbot-ekf"

# Relance ordonnee de toute la pile.
#
# POURQUOI CETTE COMMANDE EXISTE : apres un DEMARRAGE DE LA MACHINE, les
# services sont tous `active` mais certains noeuds sont muets -- ils
# apparaissent en `_NODE_NAME_UNKNOWN_` dans `ros2 topic info -v`, signe que
# leur participant DDS n'est pas correctement decouvert. Le cas le plus
# penalisant est mowbot-tf : sa TF statique base_link->laser_link n'atteint pas
# les noeuds demarres ensuite, et toute la navigation tombe.
# La cause de cet etat degrade au boot n'est PAS encore identifiee ; relancer
# les services une fois la machine posee suffit a la contourner.
relance() {
  for s in $ORDER; do
    if [ "$DRY" = "1" ]; then echo "   [dry-run] systemctl restart $s"; continue; fi
    printf "   relance %-22s" "$s"
    sudo systemctl restart "$s" 2>/dev/null && echo "ok" || echo "ECHEC"
    sleep 8
  done
  if [ "$DRY" = "1" ]; then echo "   [dry-run] systemctl restart mowbot-nav"; return; fi
  printf "   relance %-22s" "mowbot-nav"
  sudo systemctl restart mowbot-nav 2>/dev/null && echo "ok" || echo "ECHEC"
  echo "   nav2 monte en ~2 min"
  sleep 120
}

if [ "$FORCE" = "1" ]; then
  echo "== relance complete (--force) =="
  relance
else
  echo "== services =="
  for s in $ORDER mowbot-rosbridge mowbot-web; do
    st=$(systemctl is-active "$s" 2>/dev/null)
    if [ "$st" != "active" ]; then
      printf "   %-22s %s -> demarrage\n" "$s" "$st"
      [ "$DRY" = "1" ] || sudo systemctl start "$s" 2>/dev/null
    else
      printf "   %-22s ok\n" "$s"
    fi
  done
  echo "== navigation =="
  [ "$DRY" = "1" ] || bash "$MOWBOT_BIN/start_nav.sh"
fi

echo "== verification (30 s) =="
[ "$DRY" = "1" ] || sleep 30
MUETS=""
for t in /odom /imu/data_raw /odometry/filtered /scan /map; do
  hz=$([ "$DRY" = "1" ] && echo "?" || mowbot_hz "$t")
  case "$hz" in
    ""|0|0.0|MUET) hz="MUET"; MUETS="$MUETS $t" ;;
  esac
  printf "   %-22s %s\n" "$t" "$hz"
done
printf "   %-22s %s/2\n" "nav2" \
  "$([ "$DRY" = "1" ] && echo '?' || timeout 6 ros2 node list 2>/dev/null | grep -cE 'controller_server|planner_server')"

# Reparation automatique : des services actifs mais des topics muets, c'est la
# signature de l'etat degrade decrit au-dessus. Une seule tentative, pour ne
# pas boucler indefiniment.
if [ -n "$MUETS" ] && [ "$FORCE" = "0" ] && [ "$DRY" = "0" ]; then
  echo
  echo "== topics muets :$MUETS -> relance ordonnee =="
  relance
  echo "== nouvelle verification =="
  for t in /odom /odometry/filtered /scan /map; do
    printf "   %-22s %s\n" "$t" "$(mowbot_hz "$t" || echo MUET)"
  done
fi

echo
echo ">> PC : ./robot_nav.sh   |   joystick : http://$(hostname -I | awk '{print $1}'):8080/joystick.html"
