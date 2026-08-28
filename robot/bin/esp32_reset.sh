#!/bin/bash
# REBOOT de l'ESP32 + recalibration du biais gyro.
#
# A LANCER ROBOT IMMOBILE. Le firmware calibre le biais du gyroscope pendant la
# premiere seconde apres son demarrage : s'il bouge a cet instant, il prend le
# mouvement pour le zero. Mesure du 28/08/2026 : un ESP32 redemarre pendant une
# manipulation portait un biais de -4.25 deg/s, soit -258 deg/min. Le robot
# tournait dans RViz alors qu'il etait pose, roues a l'arret -- et ni le lidar
# ni le SLAM n'y etaient pour quelque chose.
#
# POURQUOI CE SCRIPT ET PAS `mowbot restart agent` : le redemarrage du service
# fait bien un reset esptool, mais l'agent tient encore le port pendant que
# celui-ci s'execute. Resultat constate : la puce reste MUETTE (0 octet sur le
# port serie), bloquee en bootloader par les lignes DTR/RTS -- plus de /odom, ni
# d'IMU, et l'EKF continue de tourner sur son dernier etat estime.
# Il faut donc liberer le port AVANT de reinitialiser.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"

mowbot_log "arret de l'agent pour liberer $DEV_ESP32"
sudo systemctl stop mowbot-agent
sudo docker rm -f mowbot_agent >/dev/null 2>&1
sleep 4

# -hupcl : le port ne rabaisse plus DTR/RTS a la fermeture, ce qui est
# exactement ce qui relaissait la puce en bootloader.
sudo stty -F "$DEV_ESP32" -hupcl 2>/dev/null || true

mowbot_log "hard reset de l'ESP32"
if ! timeout 60 python3 -m esptool --port "$DEV_ESP32" \
       --before default_reset --after hard_reset chip_id >/dev/null 2>&1; then
  mowbot_log "ATTENTION : esptool a echoue. Debrancher/rebrancher l'USB."
fi
sleep 3

# La puce parle-t-elle ? Un port muet = firmware non demarre : inutile de
# relancer l'agent, il tournerait dans le vide.
OCTETS=0
for _ in 1 2 3; do
  OCTETS=$(timeout -k 2 5 dd if="$DEV_ESP32" bs=1 count=200 iflag=nonblock 2>&1 \
           | grep -oE "^[0-9]+ bytes" | grep -oE "^[0-9]+")
  [ "${OCTETS:-0}" -gt 0 ] && break
  sleep 2
done
if [ "${OCTETS:-0}" -eq 0 ]; then
  mowbot_log "ECHEC : $DEV_ESP32 muet (0 octet). La puce n'a pas demarre son"
  mowbot_log "        firmware. Seul un debranchement/rebranchement USB corrige."
  sudo systemctl start mowbot-agent
  exit 1
fi
mowbot_log "la puce repond ($OCTETS octets)"

sudo systemctl start mowbot-agent
sleep 25
# L'EKF a peut-etre integre du faux lacet : on le repart de zero.
sudo systemctl restart mowbot-ekf
sleep 15

mowbot_log "verification du biais gyro"
python3 - <<'PY'
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

rclpy.init()
n = Node('gyro_check')
g = []
n.create_subscription(Imu, '/imu/data_raw',
                      lambda m: g.append(m.angular_velocity.z),
                      qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 12:
    rclpy.spin_once(n, timeout_sec=0.2)
if not g:
    print("  AUCUNE donnee IMU : le firmware n'a pas demarre, ou pas d'IMU")
    sys.exit(1)
biais = sum(g) / len(g) * 180.0 / math.pi
print(f"  biais gyro z : {biais:+.3f} deg/s  ({biais*60:+.0f} deg/min)")
if abs(biais) < 0.5:
    print("  => correct")
else:
    print("  => TOUJOURS BIAISE. Le robot a-t-il bouge pendant la calibration ?")
    print("     Relancer sans y toucher.")
    sys.exit(1)
PY
