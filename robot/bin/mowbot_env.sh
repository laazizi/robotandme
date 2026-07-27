#!/bin/bash
# ============================================================================
#  mowbot — environnement COMMUN a tous les scripts embarques.
#  A sourcer en tete de chaque script :  source "$(dirname "$0")/mowbot_env.sh"
#
#  Rend les scripts portables entre SBC (Jetson Humble / Raspberry Pi Jazzy) :
#  detecte la distro ROS, les chemins, et fournit les helpers communs.
# ============================================================================

# --- Racine de l'installation (ce fichier est dans <racine>/bin/) -----------
MOWBOT_HOME="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export MOWBOT_HOME
export MOWBOT_BIN="$MOWBOT_HOME/bin"
export MOWBOT_NODES="$MOWBOT_HOME/nodes"
export MOWBOT_CONFIG="$MOWBOT_HOME/config"
export MOWBOT_MAPS="$MOWBOT_HOME/maps"
export MOWBOT_LOGS="/tmp/mowbot"
mkdir -p "$MOWBOT_MAPS" 2>/dev/null
# Dossier de logs partage : certains scripts tournent en root (detect_devices),
# les services en utilisateur. Sans droits ouverts, celui qui cree le dossier
# le premier verrouille les autres ("Permission denied" au demarrage du lidar).
if mkdir -p "$MOWBOT_LOGS" 2>/dev/null; then
  chmod 1777 "$MOWBOT_LOGS" 2>/dev/null || true
fi
# repli si le dossier reste inaccessible (appartient a un autre utilisateur)
if [ ! -w "$MOWBOT_LOGS" ]; then
  export MOWBOT_LOGS="/tmp/mowbot-$(id -un)"
  mkdir -p "$MOWBOT_LOGS" 2>/dev/null
fi

# --- Distro ROS : detectee, jamais codee en dur -----------------------------
if [ -z "$MOWBOT_ROS_DISTRO" ]; then
  for d in jazzy humble iron rolling; do
    [ -f "/opt/ros/$d/setup.bash" ] && MOWBOT_ROS_DISTRO="$d" && break
  done
fi
if [ -z "$MOWBOT_ROS_DISTRO" ]; then
  echo "ERREUR : aucune distro ROS 2 trouvee dans /opt/ros" >&2
  return 1 2>/dev/null || exit 1
fi
export MOWBOT_ROS_DISTRO
source "/opt/ros/$MOWBOT_ROS_DISTRO/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Overlay eventuel (driver lidar compile depuis les sources)
[ -f "$HOME/lidar_ws/install/setup.bash" ] && export MOWBOT_LIDAR_WS="$HOME/lidar_ws/install"

# --- Peripheriques : liens udev, avec repli par detection -------------------
# Les liens sont crees par detect_devices.sh, qui identifie chaque appareil par
# ce qu'il RACONTE (trames XRCE, bootloader, signature de lidar) puis ecrit une
# regle udev sur le critere le plus stable disponible : serial unique, sinon
# vendor:product unique, sinon port physique.
# Debit serie de l'ESP32 : DOIT correspondre a SERIAL_BAUDRATE dans
# main/config.h du firmware. Un ecart et l'agent ne dialogue pas du tout.
export MOWBOT_ESP32_BAUD="${MOWBOT_ESP32_BAUD:-460800}"

export DEV_ESP32="/dev/mowbot_esp32"
export DEV_LIDAR="/dev/mowbot_lidar"
export DEV_IMU="/dev/mowbot_imu"

# Chemins absolus de binaires ROS : `ros2 run` echoue en contexte systemd
export STATIC_TF_BIN="/opt/ros/$MOWBOT_ROS_DISTRO/lib/tf2_ros/static_transform_publisher"

# --- Helpers ----------------------------------------------------------------
mowbot_log() { echo "[$(date +%H:%M:%S)] $*"; }

# esptool : selon l'installation c'est un module python, un script esptool.py
# ou une commande esptool (paquet apt). On expose la COMMANDE et non une
# fonction : `timeout 30 <fonction>` echoue ("No such file or directory"),
# timeout n'accepte qu'un executable.
mowbot_esptool_cmd() {
  if python3 -c "import esptool" 2>/dev/null; then echo "python3 -m esptool"
  elif command -v esptool.py >/dev/null 2>&1; then echo "esptool.py"
  elif command -v esptool >/dev/null 2>&1; then echo "esptool"
  fi
}
ESPTOOL_CMD="$(mowbot_esptool_cmd)"
export ESPTOOL_CMD
mowbot_has_esptool() { [ -n "$ESPTOOL_CMD" ]; }

# Attend qu'un peripherique apparaisse (utile au boot). $1=chemin $2=secondes
mowbot_wait_dev() {
  local dev="$1" max="${2:-20}" i=0
  while [ ! -e "$dev" ] && [ "$i" -lt "$max" ]; do sleep 1; i=$((i+1)); done
  [ -e "$dev" ]
}

# Frequence d'un topic (vide si muet). $1=topic
mowbot_hz() {
  timeout 6 ros2 topic hz "$1" 2>/dev/null | grep -m1 -oP 'average rate: \K[0-9.]+'
}
