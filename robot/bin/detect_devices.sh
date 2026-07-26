#!/bin/bash
# ============================================================================
#  mowbot — IDENTIFICATION des peripheriques USB et generation des regles udev
#
#  POURQUOI : l'ESP32 (DevKitC) et le lidar LSLidar N10 utilisent la MEME puce
#  CP2102 avec le MEME numero de serie ("0001"). Impossible de les distinguer
#  par vendor/serial. On les identifie donc par un TEST REEL :
#     - ESP32 : repond au protocole bootloader (esptool chip_id)
#     - IMU Razor : puce FTDI (0403:6015), et envoie des trames "#YPR="
#     - lidar : le port restant qui debite en continu
#  Puis on ecrit une regle udev basee sur le PORT USB PHYSIQUE (ID_PATH) de
#  CETTE machine. Les chemins different entre Jetson et Raspberry Pi : ce
#  script est donc A RELANCER apres un changement de SBC ou de prise USB.
#
#  Usage :  sudo bash detect_devices.sh          (interactif, ecrit la regle)
#           bash detect_devices.sh --dry-run     (montre sans ecrire)
# ============================================================================
set -o pipefail
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"

DRY=0
[ "$1" = "--dry-run" ] && DRY=1
RULES=/etc/udev/rules.d/99-mowbot.rules

echo "=============================================================="
echo " Detection des peripheriques mowbot"
echo "=============================================================="

# --- Arreter ce qui occupe les ports (sinon les tests echouent) -------------
if [ "$DRY" = "0" ]; then
  echo ">> arret temporaire des services qui tiennent les ports..."
  for s in mowbot-agent mowbot-lidar mowbot-razor; do
    systemctl is-active --quiet "$s" 2>/dev/null && { systemctl stop "$s"; STOPPED="$STOPPED $s"; }
  done
  pkill -f scan_fix.py 2>/dev/null
  sleep 2
fi

ESP32_PATH=""; LIDAR_PATH=""; IMU_PATH=""
ESP32_SER=""; LIDAR_SER=""; IMU_SER=""

for dev in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$dev" ] || continue
  VID=$(udevadm info -q property -n "$dev" 2>/dev/null | grep -oP 'ID_VENDOR_ID=\K.*')
  PID=$(udevadm info -q property -n "$dev" 2>/dev/null | grep -oP 'ID_MODEL_ID=\K.*')
  IDPATH=$(udevadm info -q property -n "$dev" 2>/dev/null | grep -oP 'ID_PATH=\K.*')
  SER=$(udevadm info -q property -n "$dev" 2>/dev/null | grep -oP 'ID_SERIAL_SHORT=\K.*')
  printf "\n-- %s  (vendor %s:%s, serial %s)\n" "$dev" "$VID" "$PID" "${SER:-aucun}"

  # 1) FTDI -> IMU Razor (verifiee par la presence de trames #YPR/#A-C)
  if [ "$VID" = "0403" ]; then
    stty -F "$dev" 57600 raw -echo 2>/dev/null
    if timeout 3 cat "$dev" 2>/dev/null | head -c 400 | grep -qE '#YPR|#A-C|#G-C'; then
      echo "   -> IMU Razor (trames AHRS detectees)"
      IMU_PATH="$IDPATH"; IMU_SER="$SER"; continue
    fi
    echo "   -> FTDI mais pas de trame AHRS : ignore"
    continue
  fi

  # 2a) ESP32 dont le FIRMWARE micro-ROS tourne deja : il emet en continu des
  #     paquets XRCE-DDS, reconnaissables a la chaine "XRCE". Teste en premier
  #     car c'est non intrusif (aucun reset) et sans dependance a esptool.
  stty -F "$dev" 115200 raw -echo 2>/dev/null
  if timeout 3 cat "$dev" 2>/dev/null | head -c 600 | grep -qa "XRCE"; then
    echo "   -> ESP32 (trames micro-ROS XRCE-DDS detectees)"
    ESP32_PATH="$IDPATH"; ESP32_SER="$SER"; continue
  fi

  # 2b) sinon on interroge le bootloader (ESP32 sans firmware, ou muet).
  #     ESPTOOL_CMD et non une fonction : `timeout <fonction>` ne marche pas.
  if mowbot_has_esptool; then
    if timeout 30 $ESPTOOL_CMD --port "$dev" --before default_reset \
         --after hard_reset chip_id 2>&1 | grep -q "Chip is"; then
      echo "   -> ESP32 (repond au bootloader)"
      ESP32_PATH="$IDPATH"; ESP32_SER="$SER"; continue
    fi
  fi

  # 3) sinon : sonde le flux pour identifier le MODELE de lidar.
  #    Indispensable ici : le LD14 et l'ESP32 partagent la puce CP2102 avec le
  #    meme serial, seul le contenu du flux les distingue.
  MODEL=$(python3 "$(dirname "$(readlink -f "$0")")/lidar_probe.py" "$dev" 2>/dev/null)
  case "$MODEL" in
    ld14|ld06|n10)
      echo "   -> LIDAR modele $MODEL"
      LIDAR_PATH="$IDPATH"; LIDAR_MODEL="$MODEL"; LIDAR_SER="$SER"
      ;;
    *)
      echo "   -> inconnu (aucune signature de lidar)"
      ;;
  esac
done

echo
echo "=============================================================="
printf " ESP32 : %s\n" "${ESP32_PATH:-NON TROUVE}"
printf " LIDAR : %s%s\n" "${LIDAR_PATH:-NON TROUVE}" \
       "$([ -n "$LIDAR_MODEL" ] && echo "   (modele $LIDAR_MODEL)")"
printf " IMU   : %s\n" "${IMU_PATH:-NON TROUVE}"
echo "=============================================================="

# Memorise le modele detecte : run_lidar.sh y lira quel driver lancer, sans
# avoir a re-sonder le port a chaque demarrage.
if [ -n "$LIDAR_MODEL" ] && [ "$DRY" = "0" ]; then
  echo "MOWBOT_LIDAR=$LIDAR_MODEL" > "$MOWBOT_HOME/lidar_model.env" 2>/dev/null && \
    echo " modele memorise dans $MOWBOT_HOME/lidar_model.env"
fi

# --- Generation de la regle -------------------------------------------------
TMP=$(mktemp)
{
  echo "# mowbot — genere par detect_devices.sh le $(date '+%Y-%m-%d %H:%M')"
  echo "# machine : $(hostname)"
  echo "#"
  echo "# Une regle par SERIAL survit a un changement de prise USB ; une regle"
  echo "# par PORT PHYSIQUE (ID_PATH) doit etre regeneree des qu'on deplace la"
  echo "# prise, qu'on ajoute ou retire un hub. On prefere donc le serial des"
  echo "# qu'il est distinctif. Les CP2102 des lidars annoncent tous \"0001\" :"
  echo "# pour eux le port physique reste le seul discriminant."

  # $1=nom du lien  $2=ID_PATH  $3=serial
  emit_rule() {
    local name="$1" path="$2" ser="$3"
    case "$ser" in
      ''|0001|0|000000000000)      # serial absent ou generique -> port physique
        echo "SUBSYSTEM==\"tty\", ENV{ID_PATH}==\"$path\", SYMLINK+=\"$name\", GROUP=\"dialout\", MODE=\"0660\"" ;;
      *)
        echo "# $name : serial unique -> regle stable, insensible a la prise utilisee"
        echo "SUBSYSTEM==\"tty\", ATTRS{serial}==\"$ser\", SYMLINK+=\"$name\", GROUP=\"dialout\", MODE=\"0660\"" ;;
    esac
  }

  [ -n "$ESP32_PATH" ] && emit_rule mowbot_esp32 "$ESP32_PATH" "$ESP32_SER"
  [ -n "$LIDAR_PATH" ] && emit_rule mowbot_lidar "$LIDAR_PATH" "$LIDAR_SER"
  [ -n "$IMU_PATH" ]   && emit_rule mowbot_imu   "$IMU_PATH"   "$IMU_SER"
} > "$TMP"

echo
echo "-- regle udev proposee --"
cat "$TMP"

if [ "$DRY" = "1" ]; then
  echo; echo "(--dry-run : rien n'a ete ecrit)"
  rm -f "$TMP"; exit 0
fi

if [ "$(id -u)" != "0" ]; then
  echo; echo "ERREUR : relancer avec sudo pour ecrire $RULES" >&2
  rm -f "$TMP"; exit 1
fi

cp "$TMP" "$RULES"; rm -f "$TMP"
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
sleep 2
echo
echo "-- liens crees --"
for l in mowbot_esp32 mowbot_lidar mowbot_imu; do
  printf "   /dev/%-14s -> %s\n" "$l" "$(readlink /dev/$l 2>/dev/null || echo ABSENT)"
done

for s in $STOPPED; do systemctl start "$s"; done
[ -n "$STOPPED" ] && echo && echo ">> services redemarres :$STOPPED"
