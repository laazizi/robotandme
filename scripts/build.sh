#!/usr/bin/env bash
# Compilation d'UN controleur (controllers/<nom>/), ESP-IDF natif Linux / WSL2.
# Prerequis : . $IDF_PATH/export.sh deja source, et :
#   pip3 install catkin_pkg lark-parser colcon-common-extensions 'empy==3.3.4'
#
# Usage : ./scripts/build.sh <controleur> [build|clean|menuconfig] [serial|eth]
#   controleurs : les dossiers de controllers/ sauf common
#   (sans argument : mowbot_p4, l'ancien comportement)
#
# La cible (esp32p4, esp32...) est lue dans controllers/<nom>/sdkconfig.defaults :
# c'est la seule source de verite, ne pas la dupliquer ici.

set -euo pipefail
cd "$(dirname "$0")/.."
ROOT="$PWD"

usage() {
    echo "Usage : $0 <controleur> [build|clean|menuconfig] [serial|eth]" >&2
    echo "Controleurs disponibles :" >&2
    for d in controllers/*/; do
        d="${d%/}"; n="${d##*/}"
        [ "$n" = "common" ] && continue
        [ -f "$d/main/robot.h" ] && echo "  $n" >&2
    done
    exit 1
}

CTRL="${1:-}"
if [ -z "$CTRL" ]; then
    CTRL="mowbot_p4"
    echo ">> Controleur non precise : $CTRL (defaut)"
fi
DIR="controllers/$CTRL"
if [ ! -f "$DIR/CMakeLists.txt" ] || [ ! -f "$DIR/main/robot.h" ] || [ ! -f "$DIR/sdkconfig.defaults" ]; then
    echo "ERREUR : controleur inconnu ou incomplet : '$CTRL'" >&2
    usage
fi
ACTION="${2:-build}"
case "$ACTION" in build|clean|menuconfig) ;; *)
    echo "ERREUR : action inconnue '$ACTION' (build|clean|menuconfig)" >&2; exit 1 ;;
esac
TRANSPORT="${3:-}"

if [ -z "${IDF_PATH:-}" ]; then
    echo "ERREUR : ESP-IDF non source. Faire : . \$HOME/esp/esp-idf/export.sh" >&2
    exit 1
fi

# GitHub repond 401 au clonage ANONYME d'un depot public par git sur cette
# machine (trace GIT_TRACE_CURL : la requete info/refs part, le serveur la
# refuse ; un curl anonyme passe). Envoyer n'importe quel token GitHub valide
# suffit, mais credential.useHttpPath=true dans la config globale (necessaire
# pour separer plusieurs comptes) empeche l'envoi du token hote sur une URL non
# enregistree. La reconstruction de libmicroros clone une trentaine de depots
# via vcs import, sans qu'on puisse leur passer -c : GIT_CONFIG_PARAMETERS est
# lu par TOUS les git enfants et fait le meme effet.
# Sans ca : "could not read Username", puis "expected flush after ref listing".
export GIT_CONFIG_PARAMETERS="'credential.useHttpPath=false'"

COMPONENT="components/micro_ros_espidf_component"
if [ ! -d "$COMPONENT" ]; then
    echo ">> Clonage du composant micro-ROS (branche humble)..."
    git clone -b humble https://github.com/micro-ROS/micro_ros_espidf_component.git "$COMPONENT"
fi

TARGET="$(grep -oP '^CONFIG_IDF_TARGET="\K[^"]+' "$DIR/sdkconfig.defaults" || true)"
if [ -z "$TARGET" ]; then
    echo "ERREUR : CONFIG_IDF_TARGET absent de $DIR/sdkconfig.defaults" >&2
    exit 1
fi

# Transport courant lu dans le sdkconfig DU controleur ; defaut = serie.
CURRENT="serial"
if [ -f "$DIR/sdkconfig" ] && grep -q '^CONFIG_MICRO_ROS_ESP_NETIF_ENET=y' "$DIR/sdkconfig"; then
    CURRENT="eth"
fi
[ -z "$TRANSPORT" ] && TRANSPORT="$CURRENT"
if [ ! -f "controllers/common/sdkconfig.$TRANSPORT" ]; then
    echo "ERREUR : transport inconnu '$TRANSPORT' (serial|eth)" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Transport RMW de libmicroros. Le colcon.meta du composant est FIGE SUR UDP ;
# on le surcharge par un app-colcon.meta place a la racine du PROJET, c'est a
# dire dans le dossier du controleur (le composant lit ${PROJECT_DIR}/app-colcon.meta).
# Sans ce fichier, une libmicroros reconstruite depuis zero sort en UDP et la
# compilation echoue sur "CONFIG_MICRO_ROS_AGENT_IP undeclared" -- meme en
# transport serie. Ce mecanisme n'existait que dans scripts/build.ps1 ; le
# script Linux marchait par chance, sur une libmicroros deja construite.
# ---------------------------------------------------------------------------
RMW="custom"                                  # serie = transport custom (UART)
[ "$TRANSPORT" = "eth" ] && RMW="udp"
META="$DIR/app-colcon.meta"
read -r -d '' META_CONTENT <<EOF || true
{
    "names": {
        "rmw_microxrcedds": {
            "cmake-args": [
                "-DRMW_UXRCE_TRANSPORT=$RMW"
            ]
        }
    }
}
EOF
if [ "$(cat "$META" 2>/dev/null || true)" != "$META_CONTENT" ]; then
    printf '%s\n' "$META_CONTENT" > "$META"
fi

# libmicroros est construite DANS le dossier du composant, partagee par tous
# les controleurs, et depend de la CIBLE (architecture) ET du TRANSPORT RMW.
# Le marqueur porte les deux ; il vaut mieux reconstruire (~15 min) que lier
# une lib incoherente, dont les symptomes sont obscurs :
#   - mauvaise architecture -> "relocations in generic ELF (EM: 243)"
#   - mauvais transport     -> "CONFIG_MICRO_ROS_AGENT_IP undeclared"
# include/rcl sert de temoin de completude : absent = build interrompu.
MARKER="$COMPONENT/.microros_target"
WANT="$TARGET $RMW"
HAVE="$(cat "$MARKER" 2>/dev/null || echo inconnu)"
if [ "$HAVE" != "$WANT" ] || [ ! -e "$COMPONENT/libmicroros.a" ] || [ ! -d "$COMPONENT/include/rcl" ]; then
    echo ">> libmicroros : '$HAVE' -> '$WANT' (ou incomplete). Reconstruction (~15 min)..."
    rm -rf "$COMPONENT/libmicroros.a" "$COMPONENT/libmicroros" \
           "$COMPONENT/micro_ros_src" "$COMPONENT/include" "$COMPONENT/micro_ros_dev" \
           "$COMPONENT/esp32_toolchain.cmake" "$DIR/build"
    rm -f "$MARKER"
fi

# Le cache CMake FIGE le chemin absolu du projet : deplacer le depot rend tout
# build/ inutilisable, avec un message peu parlant
#   "Build directory ... configured for project ... in a different directory"
# et idf.py refuse de continuer. Plutot que d'exiger un fullclean manuel, on
# detecte le decalage et on repart proprement pour CE controleur.
CACHE="$DIR/build/CMakeCache.txt"
if [ -f "$CACHE" ]; then
    CACHED_DIR="$(grep -m1 '^CMAKE_HOME_DIRECTORY:INTERNAL=' "$CACHE" | cut -d= -f2- || true)"
    if [ -n "$CACHED_DIR" ] && [ "$CACHED_DIR" != "$ROOT/$DIR" ]; then
        echo ">> build/ configure pour un autre chemin ($CACHED_DIR) : nettoyage."
        rm -rf "$DIR/build"
    fi
fi

cd "$DIR"

if [ "$TRANSPORT" != "$CURRENT" ] && [ -f sdkconfig ]; then
    echo ">> Changement de transport : $CURRENT -> $TRANSPORT"
    rm -f sdkconfig
    idf.py fullclean
fi

if [ ! -f sdkconfig ]; then
    # Ordre : commun, puis propre au controleur (cible), puis transport.
    SDKCONFIG_DEFAULTS="../common/sdkconfig.defaults;sdkconfig.defaults;../common/sdkconfig.$TRANSPORT" \
        idf.py set-target "$TARGET"
fi

case "$ACTION" in
    clean)      idf.py fullclean && idf.py build ;;
    menuconfig) idf.py menuconfig ;;
    build)      idf.py build ;;
esac

echo "$WANT" > "$ROOT/$MARKER"
echo ""
echo ">> OK : $DIR/build/$CTRL.bin ($TARGET, transport $TRANSPORT, RMW $RMW)"
echo "   Flash : ./scripts/flash.sh $CTRL [/dev/ttyACM0] [monitor]"
