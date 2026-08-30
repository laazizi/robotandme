#!/bin/bash
# Conteneur de longue duree qui porte toute la pile ROS 2 Jazzy.
#
# POURQUOI. Le Jetson Xavier NX est bloque en Ubuntu 20.04 : JetPack 5.1.7 est
# la derniere version qui le supporte, et ROS 2 Jazzy exige 24.04. Le conteneur
# apporte l'espace utilisateur Jazzy sans toucher a l'hote. Le noyau, lui, reste
# celui de l'hote -- un conteneur ne le remplace pas.
#
# Le conteneur ne fait RIEN par lui-meme : il dort. Les services mowbot y
# entrent par `docker exec`, ce qui leur permet de rester des unites systemd de
# l'hote, avec leurs dependances et leurs redemarrages automatiques.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"

IMAGE="${MOWBOT_IMAGE:-mowbot:jazzy}"
NOM="${MOWBOT_CONTAINER:-mowbot_jazzy}"

docker image inspect "$IMAGE" >/dev/null 2>&1 || {
  mowbot_log "ERREUR : image $IMAGE absente. La construire :"
  mowbot_log "  cd ~/mowbot/docker && docker build -f Dockerfile.jazzy -t $IMAGE ."
  exit 1
}

# `--rm` ne suffit pas : un conteneur tue brutalement (coupure de courant, arret
# du demon docker) reste enregistre, et `docker run --name` echoue alors en
# boucle sur "name already in use". Panne deja vecue avec l'agent micro-ROS :
# l'agent ne demarrait JAMAIS, donc ni /odom ni IMU, donc plus de navigation.
docker rm -f "$NOM" >/dev/null 2>&1

mowbot_log "demarrage du conteneur $NOM ($IMAGE)"
exec docker run --rm --name "$NOM" \
  --net=host \
  `# --ipc=host est aussi indispensable que --net=host : sans lui le conteneur` \
  `# a son PROPRE /dev/shm, et Fast DDS ne partage plus la memoire avec les` \
  `# noeuds de l'hote. Symptome : flot d'erreurs open_and_lock_file.` \
  --ipc=host \
  `# -v /dev:/dev et NON --device= : avec --device le peripherique est copie au` \
  `# demarrage du conteneur, donc un ESP32 ou un lidar rebranche ensuite reste` \
  `# invisible. Le montage partage le meme devtmpfs, le branchement a chaud` \
  `# fonctionne. Les liens udev /dev/mowbot_* apparaissent aussi -- mais ils` \
  `# doivent etre crees sur l'HOTE : udev ne tourne pas dans un conteneur.` \
  -v /dev:/dev --privileged \
  `# Le code est monte au MEME chemin qu'a l'exterieur : les scripts et les` \
  `# unites systemd utilisent des chemins absolus identiques dedans et dehors,` \
  `# et rien n'a besoin d'etre adapte.` \
  -v "$MOWBOT_HOME:$MOWBOT_HOME" \
  -v /tmp/mowbot:/tmp/mowbot \
  -e "HOME=$HOME" \
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
  "$IMAGE"
