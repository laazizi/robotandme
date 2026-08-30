#!/bin/bash
# ============================================================================
#  mowbot — INSTALLATION sur un SBC (Jetson, Raspberry Pi, ...)
#
#  Depuis le depot :   bash robot/install.sh
#  Ou a distance   :   scp -r robot nvidia@<ip>:~/mowbot_src
#                      ssh nvidia@<ip> 'bash ~/mowbot_src/install.sh'
#
#  Ce que fait le script :
#   1. copie bin/ nodes/ config/ www/ dans ~/mowbot/
#   2. detecte la distro ROS installee (Humble, Jazzy...) — rien n'est code en dur
#   3. installe les paquets ROS manquants (EKF, SLAM, nav2, rosbridge)
#   4. genere les services systemd au nom de l'utilisateur courant
#   5. IDENTIFIE LES PERIPHERIQUES USB et ecrit les regles udev de CETTE machine
#   6. ajoute la commande `mowbot` au PATH
#
#  Options :  --no-apt      ne pas installer de paquets
#             --no-udev     ne pas toucher aux regles udev
#             --container   la pile ROS tourne dans un conteneur Jazzy, pas en
#                           natif. Pour un hote qui ne peut pas avoir Jazzy --
#                           un Jetson Xavier NX, bloque en Ubuntu 20.04. Les
#                           unites systemd restent sur l'hote et entrent dans le
#                           conteneur par `docker exec`.
#             --no-enable   installer les services sans les activer au boot
# ============================================================================
# Pas de `set -e` : certaines etapes (enable de services, detection USB)
# renvoient un code non nul sans etre bloquantes — l'install doit aller au bout.
set -o pipefail
SRC="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
DEST="$HOME/mowbot"
USER_NAME="$(id -un)"
NO_APT=0; NO_UDEV=0; NO_ENABLE=0; CONTAINER=0
for a in "$@"; do
  case "$a" in
    --no-apt) NO_APT=1;; --no-udev) NO_UDEV=1;; --no-enable) NO_ENABLE=1;;
    --container) CONTAINER=1;;
  esac
done

echo "=============================================================="
echo " Installation mowbot"
echo "   source      : $SRC"
echo "   destination : $DEST"
echo "   utilisateur : $USER_NAME"
echo "   machine     : $(hostname)"
echo "=============================================================="

# --- 1. distro ROS ----------------------------------------------------------
ROS_D=""
for d in jazzy humble iron rolling; do
  [ -f "/opt/ros/$d/setup.bash" ] && ROS_D="$d" && break
done
if [ -z "$ROS_D" ] && [ "$CONTAINER" = "1" ]; then
  # Normal en mode conteneur : l'hote n'a PAS de ROS, c'est tout l'objet de la
  # manoeuvre. La distro est celle de l'image, et les YAML doivent etre
  # substitues pour elle.
  ROS_D="${MOWBOT_ROS_DISTRO:-jazzy}"
  echo ">> aucun ROS sur l'hote : normal en mode conteneur, on cible $ROS_D"
elif [ -z "$ROS_D" ]; then
  echo "ERREUR : aucune distro ROS 2 dans /opt/ros — installer ros-<distro>-ros-base," >&2
  echo "         ou passer --container si la pile doit tourner dans un conteneur." >&2
  exit 1
else
  echo ">> ROS 2 detecte : $ROS_D"
fi

# --- 2. copie des fichiers --------------------------------------------------
echo ">> copie vers $DEST"
mkdir -p "$DEST"/{bin,nodes,config,www,maps}
cp -r "$SRC/bin/." "$DEST/bin/"
cp -r "$SRC/nodes/." "$DEST/nodes/"
cp -r "$SRC/www/." "$DEST/www/" 2>/dev/null || true
# configs : ne pas ecraser un reglage local existant (sauvegarde si different).
# __HOME__ / __USER__ sont substitues : les YAML n'acceptent pas de variable
# d'environnement, et un chemin en dur casse des qu'on change de SBC.
# La comparaison porte sur la version DEJA substituee, sinon chaque
# installation croit voir une difference et empile des .local.* inutiles.
# Nom des plugins nav2 : Humble accepte "paquet/Classe", Jazzy exige
# "paquet::Classe". Avec la mauvaise syntaxe le planner_server refuse de
# demarrer ("does not exist") et le lifecycle_manager abandonne TOUT nav2 :
# les buts sont alors acceptes mais rien ne bouge. On adapte a la distro.
PLUGIN_SED=""
if [ "$ROS_D" != "humble" ]; then
  PLUGIN_SED="-e s|nav2_navfn_planner/|nav2_navfn_planner::|g
              -e s|nav2_behaviors/|nav2_behaviors::|g"
fi

for f in "$SRC"/config/*; do
  b="$(basename "$f")"
  case "$b" in *.local.*) continue;; esac
  sed -e "s|__HOME__|$HOME|g" -e "s|__USER__|$USER_NAME|g" $PLUGIN_SED "$f" > "/tmp/mowbot_cfg_$b"
  if [ -f "$DEST/config/$b" ] && ! cmp -s "/tmp/mowbot_cfg_$b" "$DEST/config/$b"; then
    cp "$DEST/config/$b" "$DEST/config/$b.local.$(date +%Y%m%d_%H%M%S)"
    echo "   (ancien $b sauvegarde en .local.*)"
  fi
  mv "/tmp/mowbot_cfg_$b" "$DEST/config/$b"
done
chmod +x "$DEST"/bin/*.sh "$DEST"/bin/mowbot 2>/dev/null || true

# --- 3. paquets ROS ---------------------------------------------------------
if [ "$CONTAINER" = "1" ]; then
  # Les paquets ROS vivent dans l'image, pas sur l'hote. En revanche esptool est
  # necessaire SUR L'HOTE : c'est lui qui reinitialise l'ESP32 et l'identifie
  # pour les regles udev, et udev ne tourne pas dans un conteneur.
  echo ">> mode conteneur : paquets ROS non installes sur l'hote"
  if ! python3 -c "import esptool" 2>/dev/null; then
    echo "   esptool absent de l'hote : necessaire pour l'ESP32 et les regles udev"
    python3 -m pip install --user -q -U "esptool>=4.12" 2>/dev/null \
      || python3 -m pip install --user --break-system-packages -q -U "esptool>=4.12" 2>/dev/null \
      || echo "   ATTENTION : esptool indisponible"
  fi
elif [ "$NO_APT" = "0" ]; then
  echo ">> paquets ROS ($ROS_D)"
  PKGS="ros-$ROS_D-robot-localization ros-$ROS_D-slam-toolbox ros-$ROS_D-navigation2
        ros-$ROS_D-nav2-bringup ros-$ROS_D-rosbridge-suite ros-$ROS_D-robot-state-publisher
        ros-$ROS_D-tf2-ros python3-serial python3-numpy"
  MISSING=""
  for p in $PKGS; do
    dpkg -s "$p" >/dev/null 2>&1 || MISSING="$MISSING $p"
  done
  if [ -n "$MISSING" ]; then
    echo "   a installer :$MISSING"
    sudo DEBIAN_FRONTEND=noninteractive apt-get update -o DPkg::Lock::Timeout=600 -qq
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -o DPkg::Lock::Timeout=600 $MISSING
  else
    echo "   tout est deja installe"
  fi
  # esptool : necessaire au hard reset de l'ESP32 et a son identification par
  # detect_devices.sh. Sans lui, l'ESP32 n'est jamais confirme et son lien udev
  # n'est pas cree. Depuis Ubuntu 24.04 (PEP 668) `pip install --user` ECHOUE
  # sur un environnement gere par le systeme : on passe donc par apt d'abord,
  # et on force pip en dernier recours.
  # VERSION MINIMALE 4.12 : la 4.7 livree par apt lit MAL la revision de
  # l'ESP32-P4. Elle annonce "v0.0" pour une puce v1.3, et le flash est alors
  # refuse ("requires chip revision in range [v1.0 - v1.99]"). Le piege est
  # qu'on est tente de forcer avec --force, ce qui flasherait un binaire
  # inadapte a la puce.
  ESPTOOL_OK=0
  python3 - <<'PYV' 2>/dev/null && ESPTOOL_OK=1
import sys
try:
    import esptool
    v = tuple(int(x) for x in esptool.__version__.split('.')[:2])
    sys.exit(0 if v >= (4, 12) else 1)
except Exception:
    sys.exit(1)
PYV
  if [ "$ESPTOOL_OK" = "0" ]; then
    echo "   esptool absent ou trop ancien (< 4.12) : installation"
    python3 -m pip --version >/dev/null 2>&1 || \
      sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -o DPkg::Lock::Timeout=600 python3-pip >/dev/null 2>&1
    python3 -m pip install --user -q -U "esptool>=4.12" 2>/dev/null \
      || python3 -m pip install --user --break-system-packages -q -U "esptool>=4.12" 2>/dev/null \
      || echo "   ATTENTION : esptool indisponible -> l'ESP32 ne sera ni identifie ni flashable"
    # pip --user installe dans ~/.local/bin, absent du PATH par defaut
    grep -q '.local/bin' "$HOME/.bashrc" 2>/dev/null || \
      echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
  fi
fi

# --- 4. services systemd ----------------------------------------------------
echo ">> services systemd"
CTNAME="${MOWBOT_CONTAINER:-mowbot_jazzy}"
for f in "$SRC"/systemd/*.service; do
  b="$(basename "$f")"
  sed -e "s|__USER__|$USER_NAME|g" -e "s|__HOME__|$HOME|g" "$f" > "/tmp/$b"

  # MODE CONTENEUR : les noeuds ROS doivent tourner DANS le conteneur, mais
  # leurs unites restent sur l'hote pour garder les dependances systemd, les
  # redemarrages automatiques et les journaux. On remplace donc le lanceur.
  # Le code etant monte au MEME chemin dans le conteneur, les chemins absolus
  # des unites restent valables tels quels -- c'est ce qui permet de ne rien
  # modifier d'autre.
  # shmclean et le conteneur lui-meme sont EXCLUS : le premier purge /dev/shm de
  # l'hote, le second EST le conteneur.
  if [ "$CONTAINER" = "1" ] && \
     [ "$b" != "mowbot-container.service" ] && [ "$b" != "mowbot-shmclean.service" ]; then
    sed -i "s|^ExecStart=/bin/bash |ExecStart=/usr/bin/docker exec $CTNAME /bin/bash |" "/tmp/$b"
    # Sans cette dependance, un service peut demarrer avant le conteneur et
    # echouer sur "No such container" -- puis boucler.
    sed -i "s|^\[Service\]|Requires=mowbot-container.service\nAfter=mowbot-container.service\n\n[Service]|" "/tmp/$b"
  fi
  sudo mv "/tmp/$b" "/etc/systemd/system/$b"
done
sudo systemctl daemon-reload
# shmclean en TETE : il purge les segments Fast DDS residuels avant que le
# moindre noeud ROS demarre (cf. l'en-tete de son unite).
SERVICES="mowbot-shmclean mowbot-tf mowbot-agent mowbot-razor mowbot-lidar
          mowbot-ekf mowbot-description mowbot-rosbridge mowbot-web mowbot-nav"
# Le conteneur en PREMIER : tous les autres en dependent.
[ "$CONTAINER" = "1" ] && SERVICES="mowbot-container $SERVICES"
if [ "$NO_ENABLE" = "0" ]; then
  sudo systemctl enable $SERVICES >/dev/null 2>&1
  echo "   actives au boot : $(echo $SERVICES | wc -w) services"
else
  echo "   installes (non actives : --no-enable)"
fi

# --- 5. peripheriques USB + udev -------------------------------------------
if [ "$NO_UDEV" = "0" ]; then
  echo ">> identification des peripheriques USB (regles udev de cette machine)"
  sudo bash "$DEST/bin/detect_devices.sh" || echo "   (detection incomplete : relancer 'mowbot detect')"
fi
# acces aux ports serie sans sudo
id -nG "$USER_NAME" | grep -qw dialout || sudo usermod -aG dialout "$USER_NAME"

# --- 6. commande `mowbot` dans le PATH -------------------------------------
mkdir -p "$HOME/.local/bin"
ln -sf "$DEST/bin/mowbot" "$HOME/.local/bin/mowbot"
grep -q 'mowbot/bin' "$HOME/.bashrc" 2>/dev/null || \
  echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
grep -q "ros/$ROS_D/setup.bash" "$HOME/.bashrc" 2>/dev/null || {
  echo "source /opt/ros/$ROS_D/setup.bash" >> "$HOME/.bashrc"
  echo "export ROS_DOMAIN_ID=0" >> "$HOME/.bashrc"
}

echo
echo "=============================================================="
echo " Installation terminee."
echo "   mowbot status      etat du robot"
echo "   mowbot up          tout demarrer"
echo "   mowbot             liste des commandes"
echo
echo " A FAIRE ENSUITE si ce SBC est nouveau :"
echo "   - agent micro-ROS : compiler MicroXRCEAgent, ou l'image docker"
echo "   - lidar : compiler le driver (branche N10_V1.0) dans ~/lidar_ws"
echo "   - verifier 'mowbot devices' apres tout changement de prise USB"
echo "=============================================================="
