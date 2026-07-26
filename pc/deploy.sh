#!/usr/bin/env bash
# ============================================================================
#  mowbot — DEPLOIEMENT depuis le PC vers un SBC (Jetson ou Raspberry Pi)
#
#  Usage :
#     ./deploy.sh pi                 installe sur la Raspberry Pi
#     ./deploy.sh jetson             installe sur la Jetson Orin Nano
#     ./deploy.sh pi --ip 1.2.3.4    force l'adresse (saute la recherche)
#     ./deploy.sh pi --code-only     n'envoie que le code (ni apt ni udev)
#     ./deploy.sh pi --setup-key     installe une cle SSH (fin des mots de passe)
#     ./deploy.sh pi --status        etat de la machine, sans rien installer
#     ./deploy.sh pi --dry-run       montre ce qui serait fait
#
#  Le script se charge de tout :
#    1. TROUVE la machine (mDNS, cache ARP par adresse MAC, puis balayage) --
#       les IP changent a chaque reseau, c'est le probleme le plus recurrent
#    2. envoie robot/ par tar+ssh (un seul flux, bien plus rapide que scp -r)
#    3. lance install.sh a distance, qui detecte lui-meme la distro ROS et
#       adapte la configuration (syntaxe des plugins nav2 Humble vs Jazzy)
#    4. identifie les peripheriques USB et ecrit les regles udev de CETTE
#       machine (les chemins USB different d'un SBC a l'autre)
#    5. verifie le resultat : services, topics, liens /dev/mowbot_*
#
#  Rien n'est specifique a une machine dans le code deploye : les deux cibles
#  recoivent le meme robot/, seule l'installation s'adapte.
# ============================================================================
set -o pipefail
cd "$(dirname "$(readlink -f "$0")")"
REPO="$(cd .. && pwd)"

# --- Machines connues -------------------------------------------------------
# Les adresses MAC servent a retrouver la machine apres un changement de
# reseau : c'est le seul identifiant qui ne bouge pas.
declare -A HOSTNAME_OF=( [jetson]="ubuntu"  [pi]="peoples" )
declare -A USER_OF=(     [jetson]="nvidia"  [pi]="nvidia"  )
declare -A MACS_OF=(
  [jetson]="48:8f:4c:ff:dd:03"
  # Pi 5 puis Pi 3B+ : la meme carte SD passe de l'une a l'autre
  [pi]="2c:cf:67:30:a2:bc b8:27:eb:9a:8f:3a"
)
# Adresse de secours : la Jetson expose une IP fixe sur son port USB-C
declare -A FALLBACK_IP_OF=( [jetson]="192.168.55.1" [pi]="" )

TARGET=""; FORCE_IP=""; DRY=0; STATUS_ONLY=0; SETUP_KEY=0
INSTALL_ARGS=""
while [ $# -gt 0 ]; do
  case "$1" in
    jetson|pi)   TARGET="$1" ;;
    --ip)        FORCE_IP="$2"; shift ;;
    --dry-run)   DRY=1 ;;
    --status)    STATUS_ONLY=1 ;;
    --setup-key) SETUP_KEY=1 ;;
    --code-only) INSTALL_ARGS="$INSTALL_ARGS --no-apt --no-udev" ;;
    --no-apt|--no-udev|--no-enable) INSTALL_ARGS="$INSTALL_ARGS $1" ;;
    -h|--help)   sed -n '2,30p' "$0"; exit 0 ;;
    *)           echo "option inconnue : $1" >&2; exit 1 ;;
  esac
  shift
done

if [ -z "$TARGET" ]; then
  echo "Usage : ./deploy.sh {pi|jetson} [options]   (--help pour le detail)" >&2
  exit 1
fi

HOST="${HOSTNAME_OF[$TARGET]}"
USER_R="${USER_OF[$TARGET]}"

say() { printf '\033[1;34m>> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m!! %s\033[0m\n' "$*"; }
die() { printf '\033[1;31mERREUR : %s\033[0m\n' "$*" >&2; exit 1; }

# --- 1. Trouver la machine --------------------------------------------------
find_ip() {
  # a) adresse imposee
  if [ -n "$FORCE_IP" ]; then echo "$FORCE_IP"; return; fi
  # b) mDNS (marche si le reseau ne le bloque pas)
  local ip
  ip=$(getent hosts "$HOST.local" 2>/dev/null | awk '{print $1}' | head -1)
  if [ -n "$ip" ] && ping -c1 -W2 "$ip" >/dev/null 2>&1; then echo "$ip"; return; fi
  # c) cache ARP : la MAC ne change jamais, l'IP oui
  for mac in ${MACS_OF[$TARGET]}; do
    ip=$(ip neigh 2>/dev/null | grep -i "$mac" | awk '{print $1}' | head -1)
    [ -n "$ip" ] && { echo "$ip"; return; }
  done
  # d) balayage du reseau courant, puis relecture du cache ARP
  local pfx
  pfx=$(ip -4 route get 1.1.1.1 2>/dev/null | grep -oP 'src \K\d+\.\d+\.\d+')
  if [ -n "$pfx" ]; then
    warn "recherche sur $pfx.0/24 (quelques secondes)..." >&2
    for i in $(seq 1 254); do (ping -c1 -W1 "$pfx.$i" >/dev/null 2>&1 &) ; done
    sleep 5
    for mac in ${MACS_OF[$TARGET]}; do
      ip=$(ip neigh 2>/dev/null | grep -i "$mac" | awk '{print $1}' | head -1)
      [ -n "$ip" ] && { echo "$ip"; return; }
    done
  fi
  # e) repli : IP fixe (Jetson par USB-C)
  ip="${FALLBACK_IP_OF[$TARGET]}"
  if [ -n "$ip" ] && ping -c1 -W2 "$ip" >/dev/null 2>&1; then echo "$ip"; return; fi
}

say "recherche de $TARGET ($HOST)"
IP="$(find_ip)"
[ -z "$IP" ] && die "$TARGET introuvable. Verifier qu'elle est allumee et sur
        le meme reseau, ou forcer : ./deploy.sh $TARGET --ip <adresse>"
say "$TARGET a l'adresse $IP"

# --- 2. Acces SSH -----------------------------------------------------------
SSH_OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=15 -o LogLevel=ERROR"
ASKPASS=""

if ssh -o BatchMode=yes $SSH_OPTS "$USER_R@$IP" true 2>/dev/null; then
  say "connexion par cle SSH (aucun mot de passe necessaire)"
  SSH="ssh $SSH_OPTS"
else
  # Pas de cle : on passe le mot de passe par un assistant SSH_ASKPASS, ce qui
  # evite sshpass (souvent absent) et ne laisse pas le mot de passe dans la
  # liste des processus.
  ASKPASS="$(mktemp)"
  printf '#!/bin/sh\necho "%s"\n' "${MOWBOT_PASS:-nvidia}" > "$ASKPASS"
  chmod 700 "$ASKPASS"
  trap 'rm -f "$ASKPASS"' EXIT
  SSH="env SSH_ASKPASS=$ASKPASS SSH_ASKPASS_REQUIRE=force setsid -w ssh $SSH_OPTS"
  warn "authentification par mot de passe (MOWBOT_PASS pour le changer)"
  warn "conseil : ./deploy.sh $TARGET --setup-key  pour ne plus le saisir"
fi
run() { eval $SSH "$USER_R@$IP" "'$1'"; }

$SSH "$USER_R@$IP" true 2>/dev/null || die "connexion SSH impossible vers $USER_R@$IP"

# --- Installation d'une cle SSH (optionnel) --------------------------------
if [ "$SETUP_KEY" = "1" ]; then
  [ -f "$HOME/.ssh/id_ed25519.pub" ] || ssh-keygen -t ed25519 -N "" -f "$HOME/.ssh/id_ed25519"
  say "installation de la cle publique sur $TARGET"
  run "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys" \
      < "$HOME/.ssh/id_ed25519.pub"
  say "termine : les prochains deploiements se feront sans mot de passe"
  exit 0
fi

# --- Etat de la machine ----------------------------------------------------
show_status() {
  run 'bash -s' <<'REMOTE'
echo "--- machine ---"
( tr -d "\0" < /proc/device-tree/model 2>/dev/null; echo ) 2>/dev/null | head -1
. /etc/os-release 2>/dev/null; echo "$PRETTY_NAME ($(uname -m)), $(nproc) coeurs, $(free -m | awk '/^Mem:/{print $2}') Mo"
for d in jazzy humble iron rolling; do [ -d "/opt/ros/$d" ] && echo "ROS 2 : $d"; done
command -v vcgencmd >/dev/null 2>&1 && echo "temperature : $(vcgencmd measure_temp) | bridage : $(vcgencmd get_throttled)"
echo "--- peripheriques ---"
for l in mowbot_esp32 mowbot_lidar mowbot_imu; do
  printf "  /dev/%-14s -> %s\n" "$l" "$(readlink /dev/$l 2>/dev/null || echo ABSENT)"
done
[ -f "$HOME/mowbot/lidar_model.env" ] && cat "$HOME/mowbot/lidar_model.env"
echo "--- services ---"
systemctl list-units 'mowbot-*' --no-legend --no-pager 2>/dev/null | awk '{printf "  %-26s %s\n",$1,$4}' || echo "  (aucun)"
REMOTE
}

if [ "$STATUS_ONLY" = "1" ]; then
  show_status
  exit 0
fi

# --- 3. Envoi du code ------------------------------------------------------
if [ "$DRY" = "1" ]; then
  say "--dry-run : on s'arrete ici"
  echo "   enverrait  : $REPO/robot -> $USER_R@$IP:/tmp/mowbot_src"
  echo "   executerait: install.sh$INSTALL_ARGS"
  exit 0
fi

say "envoi du code vers $TARGET"
# tar par le tuyau ssh : un seul flux, sans les milliers d'allers-retours de
# scp -r, et les fichiers compiles ou temporaires sont exclus.
tar czf - -C "$REPO" \
    --exclude='__pycache__' --exclude='*.pyc' --exclude='*.local.*' \
    robot | run "rm -rf /tmp/mowbot_src && mkdir -p /tmp/mowbot_src && tar xzf - -C /tmp/mowbot_src" \
  || die "envoi du code echoue"

# --- 4. Installation a distance -------------------------------------------
say "installation (detection de la distro ROS et des peripheriques)"
# install.sh appelle sudo : on lui fournit le meme assistant de mot de passe,
# via un `sudo` de substitution place en tete de PATH (sudo -A n'est pas
# utilise par le script lui-meme).
run "bash -s" <<REMOTE
set -o pipefail
mkdir -p /tmp/mowbot_sudo
printf '#!/bin/sh\necho "%s"\n' "${MOWBOT_PASS:-nvidia}" > /tmp/mowbot_askpass
chmod 700 /tmp/mowbot_askpass
printf '#!/bin/sh\nexec /usr/bin/sudo -A "\$@"\n' > /tmp/mowbot_sudo/sudo
chmod 700 /tmp/mowbot_sudo/sudo
export SUDO_ASKPASS=/tmp/mowbot_askpass
export PATH=/tmp/mowbot_sudo:\$PATH
export DEBIAN_FRONTEND=noninteractive
bash /tmp/mowbot_src/robot/install.sh$INSTALL_ARGS 2>&1 | tail -30
RC=\$?
rm -f /tmp/mowbot_askpass; rm -rf /tmp/mowbot_sudo
exit \$RC
REMOTE
[ $? -ne 0 ] && warn "l'installation a signale un probleme (voir ci-dessus)"

# --- 5. Verification -------------------------------------------------------
echo
say "verification"
show_status

echo
say "termine. Sur $TARGET :"
echo "   mowbot status      etat detaille"
echo "   mowbot up          tout demarrer"
echo "   mowbot detect      re-identifier les USB (apres tout changement de prise)"
echo
echo "   Depuis ce PC :  ssh $USER_R@$IP 'mowbot status'"
echo "                   ./deploy.sh $TARGET --status"
