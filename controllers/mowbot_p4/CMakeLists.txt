cmake_minimum_required(VERSION 3.16)

# Controleur mowbot_p4 : robot A, DIFFDRIVE, Waveshare ESP32-P4-ETH, moteurs 12 V.
# Ce dossier ne contient que ce qui est propre au robot (main/robot.h) ; le code
# commun vient de ../common (voir main/CMakeLists.txt) et le composant micro-ROS
# de ../../components, partage entre tous les controleurs.
# Build : ./scripts/build.sh mowbot_p4   (jamais idf.py directement, cf. CLAUDE.md)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mowbot_p4)
