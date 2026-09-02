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

# --- Fichier de PID, pour que systemd puisse nous arreter -------------------
# En mode conteneur, systemd ne voit que le client `docker exec` : le processus
# reel, DANS le conteneur, lui echappe. Chaque `systemctl restart` empilait donc
# une instance de plus (deux slam_toolbox publiant la meme TF, trois
# planner_server, charge 54 sur 4 coeurs).
#
# On ecrit donc notre PID ici, et l'unite s'en sert pour nous envoyer un TERM.
# POURQUOI C'EST LE SCRIPT QUI L'ECRIT, et non l'unite : une premiere version
# faisait `echo $$` depuis la ligne ExecStart, mais systemd interprete `$$`
# comme un `$` litteral -- il aurait fallu ecrire `$$$$`. Le fichier contenait
# donc un `$` au lieu d'un numero, et rien n'etait jamais tue. Quatre niveaux
# d'echappement imbriques : autant les eviter.
# `exec` plus loin dans les scripts CONSERVE le PID, la valeur reste donc juste.
# ON RETIRE ENSUITE LA VARIABLE DE L'ENVIRONNEMENT. Sans cela, tout script
# enfant qui re-source ce fichier ECRASE le pidfile avec son propre PID, et
# l'arret du service ne tue plus le bon processus. Cas reellement vecu :
# start_nav.sh lance `bash run_slam.sh &`, run_slam.sh re-source ce fichier et
# posait son PID dans mowbot-nav.pid. ct_stop.sh tuait donc le SLAM en laissant
# start_nav.sh et nav2 intacts -- a chaque relance une pile nav2 complete
# s'ajoutait a la precedente. On a mesure TROIS generations vivantes en
# parallele, dont deux route_server et deux opennav_docking qui consommaient du
# CPU pour rien, et `ros2 node list` montrait les noeuds en double.
# `unset` (et non une variable locale) : c'est bien de l'environnement EXPORTE
# qu'il faut la retirer, puisque le probleme est l'heritage par les enfants.
if [ -n "$MOWBOT_PIDFILE" ]; then
  echo $$ > "$MOWBOT_PIDFILE" 2>/dev/null || true
  unset MOWBOT_PIDFILE
fi

# --- PATH : ~/.local/bin ---------------------------------------------------
# Deux choses y vivent et sont indispensables aux scripts :
#   - esptool (installe par pip --user) : reset et identification de l'ESP32 ;
#   - le relais `ros2` en mode conteneur (bin/ros2), sans lequel mowbot_hz
#     appelle un `ros2 topic hz` introuvable et rapporte TOUT comme muet --
#     c'est arrive au premier demarrage sur le Jetson, et le diagnostic etait
#     entierement faux alors que la pile tournait parfaitement.
case ":$PATH:" in
  *":$HOME/.local/bin:"*) ;;
  *) export PATH="$HOME/.local/bin:$PATH" ;;
esac

# --- Distro ROS : detectee, jamais codee en dur -----------------------------
if [ -z "$MOWBOT_ROS_DISTRO" ]; then
  for d in jazzy humble iron rolling; do
    [ -f "/opt/ros/$d/setup.bash" ] && MOWBOT_ROS_DISTRO="$d" && break
  done
fi
# PAS D'ERREUR si /opt/ros est vide : c'est le cas NORMAL sur un hote en mode
# conteneur. Le Jetson Xavier NX est bloque en Ubuntu 20.04 (JetPack 5.1.7 est
# la derniere version qui le supporte) alors que Jazzy exige 24.04 : la pile ROS
# vit dans un conteneur, et l'hote n'a aucun /opt/ros.
# Certains scripts tournent quand meme SUR L'HOTE et ont besoin de cet
# environnement sans avoir besoin de ROS : run_container.sh (il lance le
# conteneur), run_agent.sh (il lance le conteneur de l'agent micro-ROS),
# detect_devices.sh (udev ne tourne pas dans un conteneur). Les faire echouer
# ici les rendrait tous inutilisables.
if [ -z "$MOWBOT_ROS_DISTRO" ]; then
  # Valeur retenue pour NOMMER les choses (le tag de l'image de l'agent, par
  # exemple), sans rien sourcer.
  MOWBOT_ROS_DISTRO="${MOWBOT_ROS_DISTRO_DEFAULT:-jazzy}"
  export MOWBOT_ROS_DISTRO
  export MOWBOT_NO_ROS=1
else
  export MOWBOT_ROS_DISTRO
  # LE FICHIER PEUT NE PAS EXISTER, et il faut le verifier. En mode conteneur,
  # ROS n'est installe QUE dans le conteneur, alors que MOWBOT_ROS_DISTRO peut
  # tres bien etre deja pose dans l'environnement (herite d'un service, du
  # .bashrc, ou d'une commande precedente). On tombait donc dans cette branche
  # sur l'HOTE, et chaque commande mowbot lancee depuis l'hote commencait par :
  #   /opt/ros/jazzy/setup.bash: No such file or directory
  # Bruit pur : la commande fonctionnait ensuite, via le relais bin/ros2.
  if [ -f "/opt/ros/$MOWBOT_ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$MOWBOT_ROS_DISTRO/setup.bash"
  else
    export MOWBOT_NO_ROS=1
  fi
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Overlay eventuel (driver lidar compile depuis les sources)
[ -f "$HOME/lidar_ws/install/setup.bash" ] && export MOWBOT_LIDAR_WS="$HOME/lidar_ws/install"

# Driver NATIF du LD14, s'il a ete compile. On cherche d'abord SOUS
# $MOWBOT_HOME : en mode conteneur, seul ce dossier est monte depuis l'hote,
# donc c'est le seul endroit ou une compilation SURVIT au redemarrage du
# conteneur. $HOME/ldlidar_ws reste accepte pour les installations natives.
if [ -z "$MOWBOT_LDLIDAR_WS" ]; then
  for d in "$MOWBOT_HOME/ldlidar_ws/install" "$HOME/ldlidar_ws/install"; do
    if [ -x "$d/ldlidar_sl_ros2/lib/ldlidar_sl_ros2/ldlidar_sl_ros2_node" ]; then
      export MOWBOT_LDLIDAR_WS="$d"
      break
    fi
  done
fi

# --- Peripheriques : liens udev, avec repli par detection -------------------
# Les liens sont crees par detect_devices.sh, qui identifie chaque appareil par
# ce qu'il RACONTE (trames XRCE, bootloader, signature de lidar) puis ecrit une
# regle udev sur le critere le plus stable disponible : serial unique, sinon
# vendor:product unique, sinon port physique.
# Debit serie de l'ESP32 : DOIT correspondre a SERIAL_BAUDRATE dans
# controllers/common/base/config.h du firmware. Un ecart et l'agent ne dialogue pas du tout.
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
  # `ros2 topic hz` ne produit AUCUNE sortie dans plusieurs contextes (constate
  # sous Jazzy en conteneur, meme sans relais et avec PYTHONUNBUFFERED) :
  # mowbot status rapportait alors TOUS les topics comme muets alors que la pile
  # tournait. On mesure donc avec nodes/hz.py, qui s'abonne pour de vrai.
  local d="${2:-4}"
  if [ -n "$MOWBOT_NO_ROS" ]; then
    # hote sans ROS : la mesure doit se faire DANS le conteneur
    docker exec "${MOWBOT_CONTAINER:-mowbot_jazzy}" bash -c \
      "source /opt/ros/${MOWBOT_ROS_DISTRO}/setup.bash && python3 '$MOWBOT_NODES/hz.py' '$1' $d" 2>/dev/null
  else
    python3 "$MOWBOT_NODES/hz.py" "$1" "$d" 2>/dev/null
  fi
}

# Lance un noeud Python du projet, DANS le conteneur si l'hote n'a pas de ROS.
#   mowbot_py wait_tf.py map odom 40
# Sans ce relais, tout script de l'hote appelant un noeud rclpy echoue en
# "ModuleNotFoundError: rclpy" : new_map.sh, restart_slam.sh, esp32_reset.sh.
mowbot_py() {
  local script="$1"
  shift
  if [ -n "$MOWBOT_NO_ROS" ]; then
    local ct="${MOWBOT_CONTAINER:-mowbot_jazzy}"
    docker exec "$ct" bash -c \
      'source /opt/ros/'"$MOWBOT_ROS_DISTRO"'/setup.bash; exec python3 "$@"' \
      _ "$MOWBOT_NODES/$script" "$@"
  else
    python3 "$MOWBOT_NODES/$script" "$@"
  fi
}
