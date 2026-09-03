#!/bin/bash
# Profil de navigation ACKERBOT : source par start_nav.sh quand MOWBOT_ROBOT=ackerbot.
# Doit definir PARAMS (le YAML a passer a nav2). Attend mowbot_env.sh deja source.
#
# Meme mecanisme que les profils A/B : on genere une COPIE du YAML de reference
# avec les nombres du profil, le fichier source n'est jamais modifie. Mais les
# CLES ne sont pas celles de DWB : RPP et SmacPlannerHybrid ont les leurs.
#
# La geometrie (rayon de braquage minimal, rotation max) vient de
# config/ackerbot_geometry.env, GENERE depuis le firmware par
# bin/gen_ackerbot_geometry.py. Si ce fichier manque, ON N'INVENTE PAS de rayon :
# le lancement est refuse, parce qu'un rayon faux fait planifier des virages
# que le robot ne peut pas prendre.

# Choix du controleur local. Les deux fichiers partagent TOUT sauf le bloc
# FollowPath : costmaps, arbre, comportements, lisseur, collision_monitor sont
# identiques. Le fichier RPP est la reference validee au sol, on ne le modifie
# jamais ; basculer et revenir se fait avec la seule variable
# ACKERBOT_CONTROLLER de config/speeds.env.
# Ordre de priorite : variable d'environnement, puis ~/mowbot/robot_profile.env
# (PROPRE A LA MACHINE, non deploye par install.sh -- c'est la que se met un
# essai sans faire deriver le depot), puis config/speeds.env (le defaut
# versionne, rpp).
CTRL="${ACKERBOT_CONTROLLER:-}"
if [ -z "$CTRL" ] && [ -f "$MOWBOT_CONFIG/speeds.env" ]; then
  CTRL="$(grep -m1 '^ACKERBOT_CONTROLLER=' "$MOWBOT_CONFIG/speeds.env" 2>/dev/null | cut -d= -f2)"
fi
CTRL="$(printf '%s' "${CTRL:-rpp}" | tr 'A-Z' 'a-z')"
case "$CTRL" in
  mppi) PARAMS="$MOWBOT_CONFIG/nav2_params_ackerbot_mppi.yaml" ;;
  rpp)  PARAMS="$MOWBOT_CONFIG/nav2_params_ackerbot.yaml" ;;
  *)    mowbot_log "ACKERBOT_CONTROLLER='$CTRL' inconnu, repli sur rpp"
        CTRL=rpp; PARAMS="$MOWBOT_CONFIG/nav2_params_ackerbot.yaml" ;;
esac
GEO="$MOWBOT_CONFIG/ackerbot_geometry.env"
SPEEDS="$MOWBOT_CONFIG/speeds.env"

if [ ! -f "$PARAMS" ]; then
  mowbot_log "ERREUR : $PARAMS absent, profil ackerbot impossible"
  exit 1
fi
if [ ! -f "$GEO" ]; then
  mowbot_log "ERREUR : $GEO absent. Il se GENERE depuis le firmware :"
  mowbot_log "         robot/bin/gen_ackerbot_geometry.py (sur le PC, puis redeployer)."
  mowbot_log "         Refus de lancer nav2 avec un rayon de braquage invente."
  exit 1
fi
. "$GEO"
[ -f "$SPEEDS" ] && . "$SPEEDS"
if [ -z "${ACKERBOT_MIN_TURNING_RADIUS:-}" ]; then
  mowbot_log "ERREUR : ACKERBOT_MIN_TURNING_RADIUS absent de $GEO"
  exit 1
fi

GEN="$MOWBOT_LOGS/nav2_params_ackerbot.yaml"
if ! cp "$PARAMS" "$GEN" 2>/dev/null; then
  mowbot_log "ATTENTION : copie de nav2_params_ackerbot.yaml impossible, valeurs de repli du fichier"
  return 0 2>/dev/null || exit 0
fi

# Remplace le NOMBRE apres la cle, en gardant indentation et commentaire de fin
# de ligne. $1 = cle YAML, $2 = valeur (vide = on laisse la valeur du fichier).
set_key() {
  [ -n "${2:-}" ] && sed -i -E "s|^([[:space:]]*)$1:[[:space:]]*-?[0-9.]+|\1$1: $2|" "$GEN"
}
# --- controleur : les CLES DIFFERENT entre RPP et MPPI ---
if [ "$CTRL" = "mppi" ]; then
  set_key vx_max        "${ACKERBOT_MAX_VEL_X:-}"
  set_key vx_min        "${ACKERBOT_MPPI_VX_MIN:-}"
  set_key wz_max        "${ACKERBOT_MAX_VEL_THETA:-}"
  set_key ax_max        "${ACKERBOT_ACC_LIM_X:-}"
  set_key ax_min        "${ACKERBOT_DECEL_LIM_X:-}"
  # min_turning_r : DERIVE du firmware, comme le rayon du planificateur
  set_key min_turning_r "$ACKERBOT_MIN_TURNING_RADIUS"
else
  set_key desired_linear_vel                    "${ACKERBOT_MAX_VEL_X:-}"
  set_key min_approach_linear_velocity          "${ACKERBOT_MIN_APPROACH_VEL:-}"
  set_key regulated_linear_scaling_min_speed    "${ACKERBOT_REG_MIN_SPEED:-}"
  set_key regulated_linear_scaling_min_radius   "$ACKERBOT_MIN_TURNING_RADIUS"
fi
# --- SmacPlannerHybrid (planificateur), identique dans les deux cas ---
set_key minimum_turning_radius                "$ACKERBOT_MIN_TURNING_RADIUS"
# --- velocity_smoother : listes [x, y, theta], il BRIDE la sortie du controleur ---
vx="${ACKERBOT_MAX_VEL_X:-}"; vn="${ACKERBOT_MIN_VEL_X:-}"; vt="${ACKERBOT_MAX_VEL_THETA:-}"
ax="${ACKERBOT_ACC_LIM_X:-}"; dx="${ACKERBOT_DECEL_LIM_X:-}"
[ -n "$vx" ] && [ -n "$vt" ] && sed -i -E "s|^([[:space:]]*)max_velocity:.*|\1max_velocity: [$vx, 0.0, $vt]|" "$GEN"
[ -n "$vn" ] && [ -n "$vt" ] && sed -i -E "s|^([[:space:]]*)min_velocity:.*|\1min_velocity: [$vn, 0.0, -$vt]|" "$GEN"
[ -n "$ax" ] && sed -i -E "s|^([[:space:]]*)max_accel:.*|\1max_accel: [$ax, 0.0, 2.0]|" "$GEN"
[ -n "$dx" ] && sed -i -E "s|^([[:space:]]*)max_decel:.*|\1max_decel: [$dx, 0.0, -2.5]|" "$GEN"

# GARDE-FOU : le YAML genere doit rester valide (un sed rate = nav2 qui ne
# demarre plus, avec des compteurs d'erreur a zero -- deja vecu).
if ! python3 -c "import sys,yaml; yaml.safe_load(open(sys.argv[1]))" "$GEN" 2>/dev/null; then
  mowbot_log "ERREUR : le YAML genere est invalide, repli sur le fichier de reference"
  PARAMS="$MOWBOT_CONFIG/nav2_params_ackerbot.yaml"
else
  PARAMS="$GEN"
fi
mowbot_log "profil ACKERBOT : controleur ${CTRL}, v=${vx:-?} m/s, R_min=${ACKERBOT_MIN_TURNING_RADIUS} m (derive du firmware), w_max=${vt:-?} rad/s"
