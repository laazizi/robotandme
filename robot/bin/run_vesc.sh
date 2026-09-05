#!/bin/bash
# Pilote des roues : /cmd_vel -> UBOX dual VESC, tachymetres -> /odom.
# Lance par mowbot-vesc.service. Sans lui, le robot n'a NI odometrie NI roues.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"

# --- geometrie : mesuree, jamais devinee ---------------------------------
VOIE="${MOWBOT_VOIE:-0.82}"          # entraxe des roues motrices [m]
RAYON="${MOWBOT_RAYON_ROUE:-0.200}"  # rayon de roue [m], A RECALIBRER au sol

# --- VITESSE : VOLONTAIREMENT DOUCE --------------------------------------
# Ce robot pese ~70 kg et evolue en APPARTEMENT (utilisateur, 5 septembre 2026).
# Les defauts ci-dessous sont choisis pour que rien ne parte vite, JAMAIS.
#
# mode duty plutot que regime : en sans-capteur le mode regime ne demarre pas
# sous ~1200 ERPM, soit 0,39 m/s -- une vitesse deja brutale dans un couloir.
# Le mode duty descend bien plus bas ; mesure : 0,077 m/s demandes, 0,076
# obtenus. Sa contrepartie est d'etre en boucle OUVERTE : la vitesse chutera
# sous charge. Acceptable en interieur, a revoir pour tondre.
MODE="${MOWBOT_VESC_MODE:-duty}"
DUTY_MAX="${MOWBOT_VESC_DUTY_MAX:-0.07}"   # ~0,16 m/s = 0,57 km/h
VMAX="${MOWBOT_VESC_VMAX:-0.20}"           # plafond en m/s, avant conversion
FREIN="${MOWBOT_VESC_FREIN_A:-3.0}"        # courant de freinage a l'arret [A]
# 0,25 s au lieu de 0,5 : a 0,16 m/s cela fait 4 cm parcourus avant que le frein
# ne morde, au lieu de 8. Plus bas serait risque -- nav2 ne publie qu'a 8-10 Hz
# et des trous de transmission normaux provoqueraient des a-coups.
DEADMAN="${MOWBOT_VESC_DEADMAN:-0.25}"

mowbot_log "pilote VESC : voie $VOIE m, rayon $RAYON m, mode $MODE"
mowbot_log "  plafonds : duty $DUTY_MAX, vitesse $VMAX m/s, frein $FREIN A, deadman $DEADMAN s"

exec python3 -u "$MOWBOT_NODES/vesc_diffdrive.py" --ros-args \
  -p voie:="$VOIE" \
  -p rayon_roue:="$RAYON" \
  -p mode_commande:="$MODE" \
  -p duty_max:="$DUTY_MAX" \
  -p vitesse_max:="$VMAX" \
  -p frein_a:="$FREIN" \
  -p deadman:="$DEADMAN"
