#!/usr/bin/env bash
# Compilation native sous Linux / WSL2 avec ESP-IDF installe (v5.2+).
# Prerequis : . $IDF_PATH/export.sh deja source, et :
#   pip3 install catkin_pkg lark-parser colcon-common-extensions 'empy==3.3.4'
#
# Usage : ./scripts/build.sh [clean|menuconfig] [serial|eth]

set -euo pipefail
cd "$(dirname "$0")/.."

if [ -z "${IDF_PATH:-}" ]; then
    echo "ERREUR : ESP-IDF non source. Faire : . \$HOME/esp/esp-idf/export.sh" >&2
    exit 1
fi

COMPONENT="components/micro_ros_espidf_component"
if [ ! -d "$COMPONENT" ]; then
    echo ">> Clonage du composant micro-ROS (branche humble)..."
    git clone -b humble https://github.com/micro-ROS/micro_ros_espidf_component.git "$COMPONENT"
fi

ACTION="${1:-build}"
TRANSPORT="${2:-}"

# La lib micro-ROS est compilee PAR CIBLE : si le dernier build etait pour
# l'ESP32 classique (build_esp32.sh), la reconstruire pour le P4.
MARKER="$COMPONENT/.microros_target"
if [ -f "$MARKER" ] && [ "$(cat "$MARKER")" != "esp32p4" ]; then
    echo ">> Cible precedente: $(cat "$MARKER") -> nettoyage libmicroros pour esp32p4..."
    rm -rf "$COMPONENT/libmicroros.a" "$COMPONENT/libmicroros" \
           "$COMPONENT/micro_ros_src" "$COMPONENT/include" "$COMPONENT/micro_ros_dev" "$COMPONENT/esp32_toolchain.cmake" build
fi

# Transport courant lu dans sdkconfig ; defaut = serie
CURRENT="serial"
if [ -f sdkconfig ] && grep -q '^CONFIG_MICRO_ROS_ESP_NETIF_ENET=y' sdkconfig; then
    CURRENT="eth"
fi
[ -z "$TRANSPORT" ] && TRANSPORT="$CURRENT"

if [ "$TRANSPORT" != "$CURRENT" ] && [ -f sdkconfig ]; then
    echo ">> Changement de transport : $CURRENT -> $TRANSPORT"
    rm -f sdkconfig
    idf.py fullclean
fi

if [ ! -f sdkconfig ]; then
    SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.$TRANSPORT" idf.py set-target esp32p4
fi

case "$ACTION" in
    clean)      idf.py fullclean && idf.py build ;;
    menuconfig) idf.py menuconfig ;;
    *)          idf.py build ;;
esac

echo "esp32p4" > "$MARKER"
