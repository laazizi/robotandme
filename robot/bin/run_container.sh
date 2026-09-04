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

# Le workspace du driver lidar N10 (~/lidar_ws) est monte S'IL EXISTE. Le
# binaire compile y vit, et run_lidar.sh le cherche a ce meme chemin DEPUIS LE
# CONTENEUR : sans ce montage il est introuvable, le service boucle en echec
# ("ERREUR : driver N10 absent") et le robot n'a pas de /scan.
# Pourquoi ce trou est reste invisible si longtemps : la Xavier NX tournait avec
# un LD14, dont le driver est un noeud Python livre dans mowbot/nodes -- il n'a
# besoin d'aucun workspace. Le chemin N10 n'avait donc jamais servi en conteneur.
MONTAGE_LIDAR=()
LIDAR_WS="${MOWBOT_LIDAR_WS_HOTE:-$HOME/lidar_ws}"
if [ -d "$LIDAR_WS" ]; then
  MONTAGE_LIDAR=(-v "$LIDAR_WS:$LIDAR_WS")
  mowbot_log "workspace lidar monte : $LIDAR_WS"
fi

mowbot_log "demarrage du conteneur $NOM ($IMAGE)"
exec docker run --rm --name "$NOM" \
  `# --init place tini en PID 1 a la place de la commande du conteneur.` \
  `# INDISPENSABLE : la commande est \`sleep infinity\`, qui n'est pas un init et` \
  `# ne reclame JAMAIS ses enfants. Chaque service arrete laissait donc derriere` \
  `# lui ses processus en <defunct> ; on en a compte 14 accumules, et ils` \
  `# faussaient tout comptage de processus (un groupe paraissait vivant alors` \
  `# qu'il ne restait que des zombies). tini les moissonne au fur et a mesure.` \
  --init \
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
  "${MONTAGE_LIDAR[@]}" \
  -e "HOME=$HOME" \
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
  "$IMAGE"
