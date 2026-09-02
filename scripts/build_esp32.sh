#!/usr/bin/env bash
# Conserve pour l'habitude. Equivalent strict de :
#   ./scripts/build.sh mowbot_wroom [build|clean|menuconfig] [serial|eth]
# Le firmware du WROOM vit dans controllers/mowbot_wroom/ depuis le decoupage
# en controleurs ; tout est dans build.sh.
exec "$(dirname "$0")/build.sh" mowbot_wroom "$@"
