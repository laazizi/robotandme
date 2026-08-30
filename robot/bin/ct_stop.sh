#!/bin/bash
# Arrete un service mowbot qui tourne DANS le conteneur.
#   ct_stop.sh <conteneur> <fichier_de_pid> [--force]
#
# POURQUOI UN SCRIPT SEPARE. systemd ne fait AUCUNE substitution de commande :
# ecrire ExecStop=... "kill $(cat fichier)" produit "bad unit file setting" et
# l'unite entiere devient invalide -- tous les services refusent alors de
# demarrer. Et un `$` litteral demande `$$`, ce qui empile les niveaux
# d'echappement jusqu'a l'erreur. Le plus simple est donc de sortir toute la
# logique de l'unite : celle-ci n'appelle qu'un chemin et deux arguments, sans
# le moindre caractere special.
#
# CE QUE CA RESOUT. Quand systemd arrete une unite lancee par `docker exec`, il
# tue le CLIENT sur l'hote ; le processus DANS le conteneur survit. Chaque
# redemarrage empilait donc une instance : deux slam_toolbox publiant la meme
# TF map->odom, trois planner_server, charge 54 sur 4 coeurs -- et un laser qui
# paraissait incoherent alors que le lidar allait parfaitement bien.
CT="${1:-mowbot_jazzy}"
PIDF="$2"
[ -n "$PIDF" ] || { echo "usage: ct_stop.sh <conteneur> <fichier_de_pid>" >&2; exit 1; }

# Pas de conteneur = rien a arreter, ce n'est pas une erreur.
docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$CT" || exit 0
[ -f "$PIDF" ] || exit 0

PID="$(cat "$PIDF" 2>/dev/null)"
case "$PID" in
  ''|*[!0-9]*) exit 0 ;;   # fichier vide ou corrompu : on ne tue rien au hasard
esac

# TERM d'abord : les scripts qui ont un trap (start_nav.sh) tuent alors leurs
# propres enfants, SLAM et nav2 compris.
docker exec "$CT" kill -TERM "$PID" 2>/dev/null
for _ in 1 2 3 4 5 6 7 8 9 10; do
  docker exec "$CT" sh -c "[ -d /proc/$PID ]" 2>/dev/null || { rm -f "$PIDF"; exit 0; }
  sleep 1
done
# Toujours vivant apres 10 s : on insiste.
docker exec "$CT" kill -9 "$PID" 2>/dev/null
rm -f "$PIDF"
exit 0
