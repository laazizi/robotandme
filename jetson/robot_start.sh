#!/usr/bin/env bash
# mowbot — lanceur de l'agent micro-ROS sur le Jetson (controle du robot).
# Utilise le lien udev stable /dev/mowbot_esp32 (regle 99-mowbot.rules).
# Relance l'agent automatiquement s'il s'arrete (reset/replug de l'ESP32).
#
# Usage : ~/robot_start.sh          (Ctrl+C pour arreter)

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
BAUD=115200

find_port() {
  # 1) lien udev stable (methode propre)
  [ -e /dev/mowbot_esp32 ] && { echo /dev/mowbot_esp32; return; }
  # 2) repli : scan par vendor CH343 (1a86) si la regle udev manque
  for dev in /dev/ttyACM* /dev/ttyUSB*; do
    [ -e "$dev" ] || continue
    VID=$(udevadm info -q property -n "$dev" 2>/dev/null | grep -oP 'ID_VENDOR_ID=\K.*')
    [ "$VID" = "1a86" ] && { echo "$dev"; return; }
  done
}

echo ">> mowbot agent | ROS_DOMAIN_ID=$ROS_DOMAIN_ID | Ctrl+C pour arreter"
trap 'echo; echo ">> arret."; exit 0' INT TERM

pkill -x MicroXRCEAgent 2>/dev/null && { echo ">> agent precedent arrete"; sleep 1; }

while true; do
  PORT="$(find_port)"
  if [ -z "$PORT" ]; then
    echo ">> ESP32 introuvable. Nouvel essai dans 3 s..."
    sleep 3; continue
  fi
  echo ">> ESP32 sur $PORT — lancement de l'agent..."
  MicroXRCEAgent serial --dev "$PORT" -b "$BAUD"
  echo ">> agent arrete (port perdu ?). Relance dans 3 s..."
  sleep 3
done
