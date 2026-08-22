#!/bin/bash
# Agent micro-ROS : pont serie ESP32 <-> DDS. Relance auto si le port tombe.
# Hard reset esptool avant lancement : l'ESP32 peut rester bloque en bootloader
# (port muet, roues mortes) quand DTR/RTS sont manipules a l'ouverture.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
BAUD="${MOWBOT_ESP32_BAUD:-460800}"
AGENT="${MOWBOT_AGENT_BIN:-/usr/local/bin/MicroXRCEAgent}"

# Trouve le port de l'ESP32. Le lien udev est prioritaire ; le repli ne se
# contente PAS du vendor id : le lidar LD14 est lui aussi un CP2102 (10c4),
# et l'agent le prenait pour l'ESP32 -- il ouvrait alors le port du lidar,
# privant le lidar de ses donnees et laissant le robot sans /odom.
# On ecarte donc d'abord tout port qui parle un protocole de lidar, puis on
# CONFIRME l'ESP32 en interrogeant son bootloader.
find_port() {
  [ -e "$DEV_ESP32" ] && { echo "$DEV_ESP32"; return; }
  local probe="$MOWBOT_BIN/lidar_probe.py"
  # Port du lidar, resolu via son lien udev : il est exclu d'emblee. Le sonder
  # reviendrait a OUVRIR son port toutes les 3 s, ce qui coupe le flux du
  # driver lidar deja en place (constate : /scan devenu muet).
  local lidar_real=""
  [ -e "$DEV_LIDAR" ] && lidar_real="$(readlink -f "$DEV_LIDAR")"
  for d in /dev/ttyUSB* /dev/ttyACM*; do
    [ -e "$d" ] || continue
    if [ -n "$lidar_real" ] && [ "$(readlink -f "$d")" = "$lidar_real" ]; then
      continue
    fi
    V=$(udevadm info -q property -n "$d" 2>/dev/null | grep -oP 'ID_VENDOR_ID=\K.*')
    case "$V" in 10c4|1a86|303a) ;; *) continue;; esac
    # Les traces partent sur stderr : cette fonction est appelee dans
    # $(find_port), donc TOUT ce qui sort sur stdout est pris pour le nom du
    # port -- l'agent recevait sinon le message de log en guise de peripherique.
    if [ -f "$probe" ] && python3 "$probe" "$d" 2>/dev/null | grep -qE '^(ld14|ld06|n10)$'; then
      mowbot_log "$d est un LIDAR, ignore pour l'agent" >&2
      continue
    fi
    # firmware micro-ROS deja actif ? sa signature suffit, sans reset
    stty -F "$d" "$BAUD" raw -echo 2>/dev/null
    if timeout 3 cat "$d" 2>/dev/null | head -c 600 | grep -qa "XRCE"; then
      echo "$d"; return
    fi
    if mowbot_has_esptool; then
      if ! timeout 20 $ESPTOOL_CMD --port "$d" chip_id 2>&1 | grep -q "Chip is"; then
        mowbot_log "$d ne repond pas comme un ESP32, ignore" >&2
        continue
      fi
    fi
    echo "$d"; return
  done
}

mowbot_log "agent micro-ROS (ROS $MOWBOT_ROS_DISTRO, domain $ROS_DOMAIN_ID)"
trap 'mowbot_log "arret agent"; exit 0' INT TERM
pkill -x MicroXRCEAgent 2>/dev/null && sleep 1

while true; do
  PORT="$(find_port)"
  if [ -z "$PORT" ]; then mowbot_log "ESP32 introuvable, nouvel essai dans 3 s"; sleep 3; continue; fi
  stty -F "$PORT" -hupcl 2>/dev/null
  if mowbot_has_esptool; then
    timeout 30 $ESPTOOL_CMD --port "$PORT" --before default_reset --after hard_reset chip_id >/dev/null 2>&1
    sleep 2
  fi
  mowbot_log "ESP32 sur $PORT"
  if [ -x "$AGENT" ]; then "$AGENT" serial --dev "$PORT" -b "$BAUD"
  # --ipc=host est INDISPENSABLE, autant que --net=host.
  #
  # Par defaut docker donne au conteneur son PROPRE /dev/shm. Or Fast DDS
  # (le RMW par defaut) fait passer les messages locaux par de la memoire
  # PARTAGEE : l'agent annoncait donc des adresses shm que les noeuds de
  # l'hote ne pouvaient pas ouvrir, d'ou le flot d'erreurs
  #   RTPS_TRANSPORT_SHM: Failed init_port fastrtps_portNNNN: open_and_lock_file
  # Consequence observee : apres un redemarrage de l'agent, l'EKF ne se
  # reappariait plus a /odom -- il restait vivant mais ne publiait PLUS RIEN,
  # donc pas de TF odom->base_link, donc le SLAM jetait tous les scans
  # ("queue is full"), donc aucune carte, donc nav2 refusait de demarrer.
  # Toute la chaine tombait a cause de cette seule option manquante.
  else docker run --rm --name mowbot_agent -v /dev:/dev --privileged \
         --net=host --ipc=host \
         microros/micro-ros-agent:"$MOWBOT_ROS_DISTRO" serial --dev "$PORT" -b "$BAUD"; fi
  mowbot_log "agent arrete, relance dans 3 s"; sleep 3
done
