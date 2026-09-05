# Profil de navigation GROS ROBOT : source par start_nav.sh quand MOWBOT_ROBOT=gros.
#
# Robot a deux roues motrices pilotees par un UBOX dual VESC, plus une roue
# FOLLE a 0,95 m devant. Diffdrive donc, et la rotation sur place est permise --
# c'est toute la difference avec le tricycle ackerbot.
#
# ETAPE 1 D'UN PLAN EN DEUX TEMPS (utilisateur, 5 septembre 2026) : diffdrive
# d'abord, puis la roue folle devient une roue directrice a servo et ce robot
# passe en ackermann. La bascule ne demandera que trois nombres --
# empattement 0,95, voie 0,82, rayon 0,20 -- la cinematique tricycle, le
# differentiel electronique et la config nav2 sans rotation existant deja.
#
# Ce script vit a part, comme nav_profile_ackerbot.sh, pour ne PAS toucher au
# bloc A/B de start_nav.sh, qui est valide au sol sur les petits robots.

CTRL="$(printf '%s' "${GROS_CONTROLLER:-mppi}" | tr 'A-Z' 'a-z')"
case "$CTRL" in
  mppi) PARAMS="$MOWBOT_CONFIG/nav2_params_diffdrive_mppi.yaml" ;;
  rpp)  PARAMS="$MOWBOT_CONFIG/nav2_params.yaml" ;;
  *)    mowbot_log "GROS_CONTROLLER='$CTRL' inconnu, repli sur mppi"
        CTRL=mppi; PARAMS="$MOWBOT_CONFIG/nav2_params_diffdrive_mppi.yaml" ;;
esac

if [ ! -f "$PARAMS" ]; then
  mowbot_log "ERREUR : $PARAMS absent, profil gros impossible"
  return 1 2>/dev/null || exit 1
fi

# La GEOMETRIE n'est pas injectee dans le YAML ici, contrairement a l'ackerbot :
# le diffdrive n'a pas de rayon de braquage a deriver du firmware. L'empreinte
# est ecrite en clair dans le fichier, et voie/rayon sont des parametres du
# noeud vesc_diffdrive.py -- un seul endroit chacun, pas de valeur en double.
mowbot_log "profil GROS : diffdrive, controleur $CTRL"
mowbot_log "  empreinte 1,15 x 0,90 m, roue folle a 0,95 m devant l'essieu"
mowbot_log "  $(basename "$PARAMS")"

# RAPPEL AFFICHE A CHAQUE LANCEMENT, parce que l'oublier coute une soiree :
# le pilote des roues n'est PAS un service. Il se lance a la main tant que la
# geometrie n'est pas figee :
#   mowbot node vesc_diffdrive.py --ros-args -p voie:=0.82 -p rayon_roue:=0.200
if ! pgrep -f "[v]esc_diffdrive.py" >/dev/null 2>&1; then
  mowbot_log "  ATTENTION : vesc_diffdrive.py ne tourne pas -> ni /odom ni roues."
  mowbot_log "  Le lancer :  mowbot node vesc_diffdrive.py --ros-args -p voie:=0.82 -p rayon_roue:=0.200"
fi
