#!/bin/bash
# NOUVELLE CARTE : archive la carte enregistree et repart en CARTOGRAPHIE.
#
# La carte "en cours" n'existe QUE dans la memoire de slam_toolbox : l'effacer
# revient donc a relancer le SLAM. Inutile de forcer le mode, run_slam.sh
# choisit la cartographie tout seul des qu'il ne trouve plus
# maps/mowbot.posegraph -- ce que fait l'archivage ci-dessous.
#
# ON PASSE PAR SYSTEMCTL, et non par `pkill -9` + `setsid` comme le faisait ce
# script. Ces deux raccourcis ont chacun coute une panne :
#   - `setsid` sortait le SLAM du groupe de controle du service : systemd ne
#     pouvait plus l'arreter, et l'on se retrouvait avec DEUX slam_toolbox
#     publiant la meme TF map->odom ;
#   - `pkill -9` ne laisse pas Fast DDS liberer ses segments de memoire
#     partagee ; il reste dans /dev/shm des fichiers de 0 octet qui rendent un
#     noeud vivant mais MUET et invisible du graphe (voir l'en-tete de
#     systemd/mowbot-shmclean.service).
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"

if ls "$MOWBOT_MAPS"/mowbot.* >/dev/null 2>&1; then
  BK="$MOWBOT_MAPS/archive_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$BK" && mv "$MOWBOT_MAPS"/mowbot.* "$BK"/ 2>/dev/null
  mowbot_log "carte enregistree archivee : $BK"
else
  mowbot_log "aucune carte enregistree a archiver"
fi

mowbot_log "relance de la navigation : la carte en memoire est perdue"
rm -f "$MOWBOT_LOGS/slam.log"
sudo systemctl restart mowbot-nav || {
  mowbot_log "ERREUR : impossible de relancer mowbot-nav"; exit 1; }

# Le service temporise avant de lancer le SLAM, puis nav2 monte derriere.
sleep 40
grep -m1 "SLAM en" "$MOWBOT_LOGS/slam.log" 2>/dev/null
if mowbot_py wait_tf.py map odom 40; then
  printf "  /map : %s Hz\n" "$(mowbot_hz /map)"
  echo ">> carte vierge. Roule dans le lieu, puis :  mowbot save-map"
else
  echo ">> la carte n'est pas encore publiee. Diagnostic :  mowbot logs nav"
fi
