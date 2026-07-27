#!/bin/bash
# ============================================================================
#  mowbot — IDENTIFICATION des peripheriques USB et generation des regles udev
#
#  POURQUOI : plusieurs peripheriques peuvent partager la meme puce et le meme
#  numero de serie -- l'ESP32 DevKitC et le lidar N10 sont tous deux des CP2102
#  annoncant "0001". Ni le vendor ni le serial ne suffisent alors : on identifie
#  chaque appareil par un TEST REEL de ce qu'il RACONTE.
#     - ESP32 : emet des trames micro-ROS "XRCE" (firmware actif), sinon
#               repond au protocole bootloader (esptool)
#     - IMU Razor : puce FTDI qui envoie des trames "#YPR="
#     - lidar : signature de trames propre a son modele (lidar_probe.py)
#
#  La regle ecrite privilegie ensuite le critere le plus STABLE disponible :
#  serial unique, sinon vendor:product unique, sinon port physique. Les deux
#  premiers survivent a un changement de prise ; le dernier non, et impose de
#  relancer ce script.
#
#  A RELANCER apres : changement de SBC, deplacement d'une prise USB, ajout ou
#  retrait d'un hub, ajout d'un peripherique serie.
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
ESP32_VID=""; LIDAR_VID=""; IMU_VID=""
ESP32_PID=""; LIDAR_PID=""; IMU_PID=""

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
    # timeout -k + dd : cf. commentaire du test XRCE plus bas (cat peut bloquer)
    if timeout -k 2 4 dd if="$dev" bs=400 count=1 iflag=nonblock 2>/dev/null | grep -qE '#YPR|#A-C|#G-C'; then
      echo "   -> IMU Razor (trames AHRS detectees)"
      IMU_PATH="$IDPATH"; IMU_SER="$SER"; IMU_VID="$VID"; IMU_PID="$PID"; continue
    fi
    echo "   -> FTDI mais pas de trame AHRS : ignore"
    continue
  fi

  # 2a) ESP32 dont le FIRMWARE micro-ROS tourne deja : il emet en continu des
  #     paquets XRCE-DDS, reconnaissables a la chaine "XRCE". Teste en premier
  #     car c'est non intrusif (aucun reset) et sans dependance a esptool.
  stty -F "$dev" "${MOWBOT_ESP32_BAUD:-460800}" raw -echo 2>/dev/null
  # `timeout 3 cat` peut rester bloque : cat s'immobilise dans l'ouverture du
  # port quand l'adaptateur n'affirme pas de porteuse, et timeout ne peut alors
  # pas l'interrompre. Constate : detect_devices bloque 10 min sur un port.
  # `timeout -k` force la mise a mort, et dd borne la lecture sans attendre
  # d'avoir rempli son tampon.
  if timeout -k 2 4 dd if="$dev" bs=600 count=1 iflag=nonblock 2>/dev/null | grep -qa "XRCE"; then
    echo "   -> ESP32 (trames micro-ROS XRCE-DDS detectees)"
    ESP32_PATH="$IDPATH"; ESP32_SER="$SER"; ESP32_VID="$VID"; ESP32_PID="$PID"; continue
  fi

  # 2b) sinon on interroge le bootloader (ESP32 sans firmware, ou muet).
  #     ESPTOOL_CMD et non une fonction : `timeout <fonction>` ne marche pas.
  if mowbot_has_esptool; then
    if timeout -k 5 40 $ESPTOOL_CMD --port "$dev" --before default_reset \
         --after hard_reset chip_id 2>&1 | grep -q "Chip is"; then
      echo "   -> ESP32 (repond au bootloader)"
      ESP32_PATH="$IDPATH"; ESP32_SER="$SER"; ESP32_VID="$VID"; ESP32_PID="$PID"; continue
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
      LIDAR_VID="$VID"; LIDAR_PID="$PID"
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

  # Combien de ports partagent ce couple vendor:product ? S'il est unique sur
  # la machine, il suffit a identifier l'appareil, et la regle devient
  # insensible a la prise utilisee.
  count_same_chip() {
    local vid="$1" pid="$2" n=0 d v p
    for d in /dev/ttyUSB* /dev/ttyACM*; do
      [ -e "$d" ] || continue
      v=$(udevadm info -q property -n "$d" 2>/dev/null | grep -oP 'ID_VENDOR_ID=\K.*')
      p=$(udevadm info -q property -n "$d" 2>/dev/null | grep -oP 'ID_MODEL_ID=\K.*')
      [ "$v" = "$vid" ] && [ "$p" = "$pid" ] && n=$((n+1))
    done
    echo "$n"
  }

  # $1=nom du lien  $2=ID_PATH  $3=serial  $4=vendor  $5=product
  #
  # On choisit le critere le PLUS STABLE possible, dans cet ordre :
  #   1. serial unique          -> insensible a la prise
  #   2. vendor:product unique  -> insensible a la prise
  #   3. port physique + vendor:product -> il faut relancer apres un
  #      rebranchement, mais au moins un autre appareil ne peut pas heriter
  #      du lien.
  # Le point 3 sans le vendor:product etait un piege reel : apres avoir
  # ECHANGE deux prises, mowbot_lidar et mowbot_esp32 pointaient tous deux
  # sur ttyACM0 -- le lien du lidar avait ete attribue a l'ESP32.
  emit_rule() {
    local name="$1" path="$2" ser="$3" vid="$4" pid="$5"
    case "$ser" in
      ''|0001|0|000000000000) ;;   # serial non distinctif -> on continue
      *)
        echo "# $name : serial unique -> regle stable, insensible a la prise"
        echo "SUBSYSTEM==\"tty\", ATTRS{serial}==\"$ser\", SYMLINK+=\"$name\", GROUP=\"dialout\", MODE=\"0660\""
        return ;;
    esac
    if [ "$(count_same_chip "$vid" "$pid")" = "1" ]; then
      echo "# $name : seule puce $vid:$pid presente -> regle stable, insensible a la prise"
      echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$vid\", ATTRS{idProduct}==\"$pid\", SYMLINK+=\"$name\", GROUP=\"dialout\", MODE=\"0660\""
    else
      echo "# $name : plusieurs puces $vid:$pid -> port physique NECESSAIRE"
      echo "#          (a REGENERER apres tout changement de prise)"
      echo "SUBSYSTEM==\"tty\", ENV{ID_PATH}==\"$path\", ATTRS{idVendor}==\"$vid\", ATTRS{idProduct}==\"$pid\", SYMLINK+=\"$name\", GROUP=\"dialout\", MODE=\"0660\""
    fi
  }

  [ -n "$ESP32_PATH" ] && emit_rule mowbot_esp32 "$ESP32_PATH" "$ESP32_SER" "$ESP32_VID" "$ESP32_PID"
  [ -n "$LIDAR_PATH" ] && emit_rule mowbot_lidar "$LIDAR_PATH" "$LIDAR_SER" "$LIDAR_VID" "$LIDAR_PID"
  [ -n "$IMU_PATH" ]   && emit_rule mowbot_imu   "$IMU_PATH"   "$IMU_SER"   "$IMU_VID"   "$IMU_PID"
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
