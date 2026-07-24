#!/usr/bin/env bash
# Compilation pour ESP32-WROOM-32U DevKitC V4 (cible "esp32", transport serie).
# Prerequis : . $HOME/esp/esp-idf/export.sh deja source.
#
# La lib micro-ROS (components/micro_ros_espidf_component) est compilee PAR
# CIBLE dans le dossier du composant : changer de carte (P4 <-> WROOM) impose
# de la reconstruire. Ce script detecte le changement (marqueur .microros_target)
# et nettoie ce qu'il faut automatiquement.
#
# Usage : ./scripts/build_esp32.sh [clean|menuconfig]

set -eo pipefail
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

TARGET=esp32
BUILD_DIR=build_esp32
SDKCFG=sdkconfig.esp32board   # separe du sdkconfig P4

# Changement de cible (ou cible inconnue) ? -> reconstruire libmicroros.
# Sans marqueur fiable, une libmicroros.a existante peut etre RISC-V (P4) :
# le linker xtensa echouerait avec "relocations in generic ELF (EM: 243)".
MARKER="$COMPONENT/.microros_target"
if [ "$(cat "$MARKER" 2>/dev/null)" != "$TARGET" ] && [ -e "$COMPONENT/libmicroros.a" ]; then
    echo ">> libmicroros d'une autre cible detectee -> nettoyage pour $TARGET..."
    rm -rf "$COMPONENT/libmicroros.a" "$COMPONENT/libmicroros" \
           "$COMPONENT/micro_ros_src" "$COMPONENT/include" "$COMPONENT/micro_ros_dev" "$COMPONENT/esp32_toolchain.cmake" "$BUILD_DIR"
fi

if [ ! -f "$SDKCFG" ]; then
    SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.serial" \
        idf.py -B "$BUILD_DIR" -D SDKCONFIG="$SDKCFG" set-target "$TARGET"
fi

case "${1:-build}" in
    clean)      idf.py -B "$BUILD_DIR" -D SDKCONFIG="$SDKCFG" fullclean
                idf.py -B "$BUILD_DIR" -D SDKCONFIG="$SDKCFG" build ;;
    menuconfig) idf.py -B "$BUILD_DIR" -D SDKCONFIG="$SDKCFG" menuconfig ;;
    *)          idf.py -B "$BUILD_DIR" -D SDKCONFIG="$SDKCFG" build ;;
esac

echo "$TARGET" > "$MARKER"
echo ""
echo ">> OK. Flash : idf.py -B $BUILD_DIR -D SDKCONFIG=$SDKCFG -p /dev/ttyUSB0 flash"
echo "   (DevKitC V4 = puce CP2102 -> /dev/ttyUSB0, auto-reset fiable)"
