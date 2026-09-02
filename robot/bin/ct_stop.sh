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
# ON TRAVAILLE PID PAR PID, jamais par groupe avec des negatifs, et sans awk
# imbrique dans un sh -c dans un bash. La version precedente faisait
#   ps -eo pgid=,stat= | awk '$2 !~ /^Z/ {print $1}' | grep -qxE '2401|404'
# a travers trois niveaux de quoting : elle repondait "plus personne" alors que
# CINQ processus tournaient encore, le kill -9 n'etait donc jamais envoye, et le
# script concluait au succes. Resultat vecu : deux collision_monitor publiant
# tous les deux sur /cmd_vel -- deux filtres concurrents sur la commande des
# moteurs -- et deux lifecycle_manager. Verifie a la trace (bash -x).
#
# `ps -o pid= -p <liste>` ne renvoie que les PID VIVANTS : c'est le test de
# survie, sans awk ni etat a interpreter. Un zombie disparait de lui-meme des
# que tini l'a moissonne (--init dans run_container.sh).

# PID portant la marque de l'unite. L'environnement survit a `exec`, donc chaque
# descendant la porte, y compris les noeuds ROS qui ont remplace le script.
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

# Secours : le PID du fichier, pour un service lance avant que la marque
# n'existe (deploiement en cours de route).
pid_du_fichier() {
  [ -n "$PIDF" ] && [ -f "$PIDF" ] || return 0
  P="$(cat "$PIDF" 2>/dev/null)"
  case "$P" in ''|*[!0-9]*) return 0 ;; esac
  echo "$P"
}

# Les DESCENDANTS des PID marques : un enfant qui aurait perdu la marque reste
# ainsi attrape. On remonte l'arbre par ppid, deux niveaux suffisent en pratique.
avec_descendants() {
  LISTE="$1"
  [ -n "$LISTE" ] || return 0
  docker exec "$CT" sh -c '
    vise="$1"
    tous="$vise"
    for tour in 1 2 3; do
      nouveaux=""
      for p in /proc/[0-9]*; do
        pid="${p#/proc/}"
        [ -r "$p/stat" ] || continue
        ppid=$(awk "{print \$4}" "$p/stat" 2>/dev/null)
        for v in $tous; do
          if [ "$ppid" = "$v" ]; then
            case " $tous $nouveaux " in *" $pid "*) ;; *) nouveaux="$nouveaux $pid" ;; esac
          fi
        done
      done
      [ -n "$nouveaux" ] || break
      tous="$tous $nouveaux"
    done
    echo $tous' _ "$LISTE" 2>/dev/null
}

vivants_parmi() {
  [ -n "$1" ] || return 0
  docker exec "$CT" ps -o pid= -p "$(echo $1 | tr ' ' ',')" 2>/dev/null | tr -d ' '
}

CIBLES="$(printf '%s\n%s\n' "$(pids_marques)" "$(pid_du_fichier)" \
          | grep -E '^[0-9]+$' | grep -vxE '0|1' | sort -u | tr '\n' ' ')"
CIBLES="$(avec_descendants "$CIBLES" | tr ' ' '\n' \
          | grep -E '^[0-9]+$' | grep -vxE '0|1' | sort -u | tr '\n' ' ')"

if [ -z "$CIBLES" ]; then rm -f "$PIDF"; exit 0; fi

for p in $CIBLES; do docker exec "$CT" kill -TERM "$p" 2>/dev/null; done

# nav2 et slam_toolbox demandent quelques secondes pour se fermer proprement ;
# les tuer trop tot laisse des segments Fast DDS de 0 octet dans /dev/shm.
RESTE="$CIBLES"
for _ in $(seq 1 15); do
  RESTE="$(vivants_parmi "$CIBLES" | tr '\n' ' ')"
  [ -n "$(echo $RESTE)" ] || break
  sleep 1
done

# ESCALADE. Certains noeuds rclcpp IGNORENT SIGTERM quand leur executeur est
# bloque : le gestionnaire de signal leve un drapeau que personne ne lit.
# Constate precisement sur collision_monitor et lifecycle_manager, restes
# vivants 45 minutes apres un TERM.
if [ -n "$(echo $RESTE)" ]; then
  for p in $RESTE; do docker exec "$CT" kill -9 "$p" 2>/dev/null; done
  sleep 2
  RESTE="$(vivants_parmi "$CIBLES" | tr '\n' ' ')"
fi

# ON SE PLAINT A VOIX HAUTE, sur stderr, donc dans le journal du service. La
# version precedente envoyait tout dans /dev/null : un echec d'arret etait
# totalement silencieux, et les doublons s'accumulaient sans que rien ne le dise.
if [ -n "$(echo $RESTE)" ]; then
  echo "ct_stop: ECHEC, processus toujours vivants dans $CT :$RESTE" >&2
  exit 0
fi
rm -f "$PIDF"
exit 0
