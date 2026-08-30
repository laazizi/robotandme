#!/bin/bash
# Agent micro-ROS : pont serie ESP32 <-> DDS. Relance auto si le port tombe.
# Hard reset esptool avant lancement : l'ESP32 peut rester bloque en bootloader
# (port muet, roues mortes) quand DTR/RTS sont manipules a l'ouverture.
source "$(dirname "$(readlink -f "$0")")/mowbot_env.sh"
BAUD="${MOWBOT_ESP32_BAUD:-460800}"
AGENT="${MOWBOT_AGENT_BIN:-/usr/local/bin/MicroXRCEAgent}"

# Trouve le port de l'ESP32. Le lien udev est prioritaire ; le repli ne se
# contente PAS du vendor id : le lidar LD14 est lui aussi un CP2102 (10c4),
# et l'agent le prenait pour l'ESP32 -- il ouvrait alors le port du lidar,
# privant le lidar de ses donnees et laissant le robot sans /odom.
# On ecarte donc d'abord tout port qui parle un protocole de lidar, puis on
# CONFIRME l'ESP32 en interrogeant son bootloader.
find_port() {
  [ -e "$DEV_ESP32" ] && { echo "$DEV_ESP32"; return; }
  local probe="$MOWBOT_BIN/lidar_probe.py"
  # Port du lidar, resolu via son lien udev : il est exclu d'emblee. Le sonder
  # reviendrait a OUVRIR son port toutes les 3 s, ce qui coupe le flux du
  # driver lidar deja en place (constate : /scan devenu muet).
  local lidar_real=""
  [ -e "$DEV_LIDAR" ] && lidar_real="$(readlink -f "$DEV_LIDAR")"
  for d in /dev/ttyUSB* /dev/ttyACM*; do
    [ -e "$d" ] || continue
    if [ -n "$lidar_real" ] && [ "$(readlink -f "$d")" = "$lidar_real" ]; then
      continue
    fi
    V=$(udevadm info -q property -n "$d" 2>/dev/null | grep -oP 'ID_VENDOR_ID=\K.*')
    case "$V" in 10c4|1a86|303a) ;; *) continue;; esac
    # L'IDENTITE USB TRANCHE D'ABORD, et sans ouvrir le port. Aucun lidar
    # n'embarque ces puces : 1a86 = CH343/CH340 des cartes ESP32-P4, 303a = USB
    # natif Espressif. Seul 10c4 (CP2102) est ambigu, car c'est la puce des
    # LD14/LD06/N10.
    # C'EST LE CORRECTIF DE LA PANNE : sans ce raccourci, un ESP32 en 1a86
    # partait a la sonde de lidar, en revenait etiquete "n10" (de facon
    # reproductible : le verdict n10 tenait au seul VOLUME lu, et le micro-ROS a
    # 460800 bauds debite beaucoup), et l'agent l'ecartait en boucle avec
    #   "/dev/ttyACM0 est un LIDAR, ignore pour l'agent"
    # Resultat : ni /odom ni /imu, donc pas d'EKF, pas de TF odom->base_link,
    # RViz sans robot et navigation morte apres chaque demarrage.
    case "$V" in 1a86|303a) echo "$d"; return ;; esac
    # Les traces partent sur stderr : cette fonction est appelee dans
    # $(find_port), donc TOUT ce qui sort sur stdout est pris pour le nom du
    # port -- l'agent recevait sinon le message de log en guise de peripherique.
    #
    # Sur puce ambigue, on demande son verdict a la sonde. "microros" est un
    # verdict POSITIF d'ESP32 : on le prend tel quel. L'ancien test de secours
    # (`grep "XRCE"` sur le flux) est supprime : le protocole ne transmet pas
    # cette chaine en clair et il ne s'est jamais declenche, ce qui explique que
    # rien n'ait jamais rattrape l'erreur de la sonde.
    if [ -f "$probe" ]; then
      case "$(python3 "$probe" "$d" 2>/dev/null | tail -1)" in
        microros)      echo "$d"; return ;;
        ld14|ld06|n10) mowbot_log "$d est un LIDAR, ignore pour l'agent" >&2
                       continue ;;
      esac
    fi
    if mowbot_has_esptool; then
      if ! timeout 20 $ESPTOOL_CMD --port "$d" chip_id 2>&1 | grep -q "Chip is"; then
        mowbot_log "$d ne repond pas comme un ESP32, ignore" >&2
        continue
      fi
    fi
    echo "$d"; return
  done
}

mowbot_log "agent micro-ROS (ROS $MOWBOT_ROS_DISTRO, domain $ROS_DOMAIN_ID)"
trap 'mowbot_log "arret agent"; exit 0' INT TERM
pkill -x MicroXRCEAgent 2>/dev/null && sleep 1

while true; do
  PORT="$(find_port)"
  if [ -z "$PORT" ]; then mowbot_log "ESP32 introuvable, nouvel essai dans 3 s"; sleep 3; continue; fi
  stty -F "$PORT" -hupcl 2>/dev/null
  # ON LIBERE LE PORT AVANT LE RESET. C'EST LA CAUSE RACINE DU MUTISME.
  # `docker run --rm` ne suffit pas : quand systemd arrete ce service, il tue le
  # CLIENT docker sur l'hote, et le conteneur mowbot_agent CONTINUE de tourner
  # en tenant /dev/ttyACM0 (verifie : `fuser` montrait micro_ros_agent sur le
  # port juste apres la relance). Le reset esptool s'executait donc sur un port
  # deja occupe : il ne pouvait pas piloter DTR/RTS, la puce restait muette, et
  # l'ancienne version supprimait le conteneur seulement JUSTE AVANT le
  # `docker run` -- donc trop tard, apres le reset.
  # Meme piege que pour les services en `docker exec` (cf. bin/ct_stop.sh) : un
  # processus dans un conteneur echappe au groupe de controle de l'unite.
  # L'unite porte aussi un ExecStop qui retire le conteneur ; ceci est la
  # ceinture, au cas ou l'arret precedent se soit mal passe (coupure
  # d'alimentation, kill -9, arret du demon docker).
  pkill -x MicroXRCEAgent 2>/dev/null
  docker rm -f mowbot_agent >/dev/null 2>&1
  sleep 3

  # ON VERIFIE QUE LA PUCE PARLE AVANT DE LANCER L'AGENT, et on reessaie.
  # Constate trois fois de suite : apres un arret/relance de l'agent, l'ESP32
  # cesse d'emettre -- 0 octet alors que le port est LIBRE, donc ce n'est pas
  # l'agent qui masque le flux. Un `--after hard_reset` la reveille a chaque
  # fois. La cause du mutisme n'est PAS identifiee (verrou du bootloader,
  # firmware qui ne repart pas, alimentation) : on traite le symptome, et on le
  # dit dans le journal plutot que de lancer un agent qui n'aura jamais de
  # client.
  # L'attente de 2 s d'origine etait trop courte : la sequence manuelle qui a
  # fonctionne laissait 5 a 6 s au firmware pour demarrer et ouvrir sa session.
  # Sans cette verification, l'agent tournait "actif" et muet, /odom restait
  # vide, l'EKF ne publiait plus odom->base_link, les costmaps n'activaient pas
  # et nav2 restait `inactive` : une panne totale dont la cause etait invisible.
  if mowbot_has_esptool; then
    N=0
    for essai in 1 2 3; do
      timeout 30 $ESPTOOL_CMD --port "$PORT" --before default_reset \
        --after hard_reset chip_id >/dev/null 2>&1
      sleep 5
      stty -F "$PORT" "$BAUD" raw -echo 2>/dev/null
      # dd et non cat : `cat` peut se bloquer a l'ouverture d'un port dont
      # l'adaptateur n'affirme pas de porteuse, et `timeout` ne l'interrompt
      # plus. Constate ailleurs dans ce projet (detect_devices bloque 10 min).
      # ON ACCUMULE SUR PLUSIEURS LECTURES, on ne se contente pas d'une seule.
      # `iflag=nonblock` rend la main immediatement : une lecture unique renvoie
      # 0 octet des que le tampon est vide a cet instant, ce qui arrive sur un
      # flux parfaitement SAIN. Constate au demarrage : "ESP32 toujours muet
      # apres 3 resets" alors que /odom sortait a 10,17 Hz -- fausse alarme qui
      # envoie chercher une panne inexistante.
      # On garde nonblock malgre tout : `cat` bloquant peut s'immobiliser dans
      # l'ouverture d'un port dont l'adaptateur n'affirme pas de porteuse, et le
      # delai d'attente ne l'interrompt alors plus (constate ailleurs dans ce
      # projet : detect_devices bloque 10 min sur un port).
      N=0
      for _ in 1 2 3 4 5 6; do
        M=$(timeout -k 2 3 dd if="$PORT" bs=200 count=1 iflag=nonblock 2>/dev/null | wc -c)
        N=$((N + ${M:-0}))
        [ "$N" -ge 20 ] && break
        sleep 0.5
      done
      [ "$N" -ge 20 ] && break
      mowbot_log "ESP32 muet apres reset ($N octets), tentative $essai/3"
    done
    if [ "${N:-0}" -ge 20 ]; then
      mowbot_log "ESP32 repond ($N octets lus)"
    else
      mowbot_log "ATTENTION : ESP32 toujours muet apres 3 resets."
      mowbot_log "            Seul un debranchement/rebranchement USB corrige."
    fi
  fi
  mowbot_log "ESP32 sur $PORT"
  if [ -x "$AGENT" ]; then "$AGENT" serial --dev "$PORT" -b "$BAUD"
  # --ipc=host est INDISPENSABLE, autant que --net=host.
  #
  # Par defaut docker donne au conteneur son PROPRE /dev/shm. Or Fast DDS
  # (le RMW par defaut) fait passer les messages locaux par de la memoire
  # PARTAGEE : l'agent annoncait donc des adresses shm que les noeuds de
  # l'hote ne pouvaient pas ouvrir, d'ou le flot d'erreurs
  #   RTPS_TRANSPORT_SHM: Failed init_port fastrtps_portNNNN: open_and_lock_file
  # NE PAS lui attribuer plus que son role : on a d'abord cru que l'absence de
  # cette option expliquait un EKF vivant mais muet. C'etait FAUX -- l'agent
  # fonctionnait sans elle. Le message open_and_lock_file est du bruit normal de
  # Fast DDS quand un port est deja verrouille. On garde --ipc=host parce que
  # c'est la configuration correcte pour partager la memoire avec l'hote, pas
  # parce qu'elle corrige une panne.
  # `--rm` ne suffit PAS : si le conteneur est tue brutalement (kill -9, coupure
  # d'alimentation, arret du demon docker au reboot), il reste enregistre et
  # `docker run --name mowbot_agent` echoue en boucle sur
  #   "Conflict. The container name /mowbot_agent is already in use"
  # L'agent ne demarre alors JAMAIS : ni /odom ni /imu, donc pas d'EKF, donc pas
  # de TF odom->base_link, donc plus de navigation. Constate apres un reboot.
  # Le conteneur a deja ete retire plus haut, AVANT le reset : ce second retrait
  # ne sert que si un exemplaire est apparu entre-temps.
  else docker rm -f mowbot_agent >/dev/null 2>&1
       docker run --rm --name mowbot_agent -v /dev:/dev --privileged \
         --net=host --ipc=host \
         microros/micro-ros-agent:"$MOWBOT_ROS_DISTRO" serial --dev "$PORT" -b "$BAUD"; fi
  mowbot_log "agent arrete, relance dans 3 s"; sleep 3
done
