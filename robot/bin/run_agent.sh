#!/bin/bash
# Agent micro-ROS : pont serie ESP32 <-> DDS. Relance auto si le port tombe.
# Hard reset esptool avant lancement : l'ESP32 peut rester bloque en bootloader
# (port muet, roues mortes) quand DTR/RTS sont manipules a l'ouverture.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
BAUD=115200
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
    if python3 -c "import esptool" 2>/dev/null; then
      if ! timeout 20 python3 -m esptool --port "$d" chip_id 2>&1 | grep -q "Chip is"; then
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
  if python3 -c "import esptool" 2>/dev/null; then
    timeout 30 python3 -m esptool --port "$PORT" --before default_reset --after hard_reset chip_id >/dev/null 2>&1
    sleep 2
  fi
  mowbot_log "ESP32 sur $PORT"
  if [ -x "$AGENT" ]; then "$AGENT" serial --dev "$PORT" -b "$BAUD"
  else docker run --rm --name mowbot_agent -v /dev:/dev --privileged --net=host \
         microros/micro-ros-agent:"$MOWBOT_ROS_DISTRO" serial --dev "$PORT" -b "$BAUD"; fi
  mowbot_log "agent arrete, relance dans 3 s"; sleep 3
done
