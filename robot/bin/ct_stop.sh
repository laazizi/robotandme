#!/bin/bash
# Arret d'un service tournant DANS le conteneur, appele par ExecStop/ExecStartPre.
#
#   ct_stop.sh <nom_conteneur> <pidfile> [nom_unite]
#
# POURQUOI CE SCRIPT EXISTE. Un `docker exec` n'est pas dans le groupe de
# controle de l'unite systemd : quand systemd arrete le service, il tue le
# CLIENT docker sur l'hote, et le processus continue de tourner dans le
# conteneur. Il faut donc aller le tuer explicitement a l'interieur.
#
# ON REPERE LES PROCESSUS PAR UNE MARQUE DANS LEUR ENVIRONNEMENT, pas par un
# pidfile. install.sh passe `-e MOWBOT_UNIT=<unite>` au docker exec, et
# l'environnement SURVIT A `exec` : chaque descendant du service porte donc la
# marque, y compris les noeuds ROS qui ont remplace le script par exec. Rien a
# tenir a jour, rien qui puisse se desynchroniser.
#
# HISTORIQUE DES DEUX MECANISMES QUI ONT ECHOUE, pour ne pas y revenir :
#   1. `pkill -f <script>` : inoperant des que le script fait `exec`, son nom
#      disparait de la ligne de commande.
#   2. Le pidfile seul : deux pannes distinctes.
#      a) MOWBOT_PIDFILE etait HERITE par les enfants, et run_slam.sh ecrasait
#         le pidfile de mowbot-nav avec son propre PID : on tuait le SLAM en
#         laissant nav2 entier (corrige dans mowbot_env.sh par un `unset`).
#      b) Le pidfile etait supprime meme quand la mise a mort echouait. Au tour
#         suivant, `[ -f "$PIDF" ]` etait faux, ct_stop sortait sans rien faire,
#         et l'orphelin devenait definitif. MESURE : trois start_nav.sh
#         simultanes, deux slam_toolbox et deux ekf_node publiant les MEMES TF
#         -- le robot sautait d'une pose a l'autre chaque seconde dans RViz, et
#         c'est la panne qui a motive cette reecriture.
# Le pidfile reste utilise en SECOURS, pour un service lance avant que la marque
# n'existe (deploiement en cours de route).
CT="${1:-mowbot_jazzy}"
PIDF="$2"
UNIT="$3"

docker ps --format '{{.Names}}' | grep -qx "$CT" || exit 0

# --- Recensement des PID a tuer ---------------------------------------------
# `grep -z` : /proc/PID/environ est une suite de chaines separees par des zeros.
# On ecarte le processus de recensement lui-meme, qui herite de la marque.
pids_marques() {
  [ -n "$UNIT" ] || return 0
  docker exec "$CT" sh -c '
    for p in /proc/[0-9]*; do
      [ -r "$p/environ" ] || continue
      if grep -qz "MOWBOT_UNIT='"$UNIT"'" "$p/environ" 2>/dev/null; then
        echo "${p#/proc/}"
      fi
    done' 2>/dev/null
}

pid_du_fichier() {
  [ -n "$PIDF" ] && [ -f "$PIDF" ] || return 0
  P="$(cat "$PIDF" 2>/dev/null)"
  case "$P" in ''|*[!0-9]*) return 0 ;; esac
  docker exec "$CT" sh -c "[ -d /proc/$P ]" 2>/dev/null && echo "$P"
}

# Les GROUPES des PID trouves : tuer le groupe atteint la descendance meme si
# un enfant a perdu la marque. Chaque docker exec a sa propre session, donc son
# propre PGID : on n'atteint jamais un autre service.
groupes() {
  L="$(pids_marques; pid_du_fichier)"
  [ -n "$L" ] || return 0
  docker exec "$CT" ps -o pgid= -p "$(echo $L | tr ' ' ',')" 2>/dev/null \
    | tr -d ' ' | grep -E '^[0-9]+$' | grep -vE '^(0|1)$' | sort -u
}

# Un ZOMBIE reste visible dans ps : sans ce filtre on attendrait le delai
# complet a chaque arret. On ne compte que les processus non-Z.
vivants() {
  G="$1"
  [ -n "$G" ] || return 1
  docker exec "$CT" sh -c \
    "ps -eo pgid=,stat= | awk '\$2 !~ /^Z/ { print \$1 }' | grep -qxE '$(echo $G | tr ' ' '|')'" \
    2>/dev/null
}

G="$(groupes | tr '\n' ' ')"
if [ -z "$G" ]; then rm -f "$PIDF"; exit 0; fi

for g in $G; do docker exec "$CT" kill -TERM "-$g" 2>/dev/null; done

# nav2 et slam_toolbox demandent quelques secondes pour se fermer proprement ;
# les tuer trop tot laisse des segments Fast DDS de 0 octet dans /dev/shm.
for _ in $(seq 1 15); do
  vivants "$G" || break
  sleep 1
done

if vivants "$G"; then
  for g in $G; do docker exec "$CT" kill -9 "-$g" 2>/dev/null; done
  sleep 2
fi

# ON NE SUPPRIME LE PIDFILE QUE SI PLUS RIEN NE TOURNE. Le supprimer alors qu'un
# processus survit, c'est perdre la seule trace permettant de le tuer plus tard
# -- exactement la panne (2b) decrite en tete de fichier.
if vivants "$G"; then
  echo "ct_stop: groupes toujours vivants dans $CT :$G" >&2
  exit 0
fi
rm -f "$PIDF"
exit 0
