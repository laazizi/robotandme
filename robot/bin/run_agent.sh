#!/bin/bash
# Agent micro-ROS : pont serie ESP32 <-> DDS. Relance auto si le port tombe.
# Hard reset esptool avant lancement : l'ESP32 peut rester bloque en bootloader
# (port muet, roues mortes) quand DTR/RTS sont manipules a l'ouverture.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
BAUD=115200
AGENT="${MOWBOT_AGENT_BIN:-/usr/local/bin/MicroXRCEAgent}"

find_port() {
  [ -e "$DEV_ESP32" ] && { echo "$DEV_ESP32"; return; }
  for d in /dev/ttyUSB* /dev/ttyACM*; do
    [ -e "$d" ] || continue
    V=$(udevadm info -q property -n "$d" 2>/dev/null | grep -oP 'ID_VENDOR_ID=\K.*')
    case "$V" in 10c4|1a86) echo "$d"; return;; esac
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
