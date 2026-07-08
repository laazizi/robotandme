#!/usr/bin/env bash
# Compilation native sous Linux / WSL2 avec ESP-IDF installe (v5.2+).
# Prerequis : . $IDF_PATH/export.sh deja source, et :
#   pip3 install catkin_pkg lark-parser colcon-common-extensions 'empy==3.3.4'
#
# Usage : ./scripts/build.sh [clean|menuconfig]

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

if [ ! -f sdkconfig ]; then
    idf.py set-target esp32p4
fi

case "${1:-}" in
    clean)      idf.py fullclean && idf.py build ;;
    menuconfig) idf.py menuconfig ;;
    *)          idf.py build ;;
esac
