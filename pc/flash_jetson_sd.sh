#!/bin/bash
# ============================================================================
#  Grave l'image SD d'un Jetson Xavier NX, puis la PRECONFIGURE pour un
#  demarrage SANS CLAVIER NI ECRAN : utilisateur cree, Wi-Fi, SSH, mDNS.
#
#  Usage :
#    ./flash_jetson_sd.sh --image <fichier.zip|.img> --device /dev/mmcblkX \
#                         [--user nvidia] [--pass nvidia] [--host oldjetson] \
#                         [--ssid aaa] [--wifi-pass 12345678]
#                         [--verify-quick | --no-verify] [--configure-only]
#
#  --configure-only : ne grave RIEN et refait seulement la preconfiguration sur
#  une carte deja gravee. Pour corriger sans repasser vingt minutes a graver.
#    ./flash_jetson_sd.sh ... --dry-run      montre tout, n'ecrit rien
#
#  POURQUOI CE SCRIPT EXISTE. L'image SD de NVIDIA lance `oem-config` au premier
#  demarrage : un assistant qui reclame licence, langue, clavier et creation de
#  compte SUR UN ECRAN. Sans ecran, la carte demarre et attend indefiniment --
#  ni SSH, ni reseau, rien. Il faut donc desamorcer cet assistant et creer le
#  compte AVANT le premier demarrage, hors ligne, sur la carte montee.
# ============================================================================
set -o pipefail

IMG=""; DEV=""; USR="nvidia"; PWD_="nvidia"; HOST="oldjetson"
SSID=""; WPASS=""; DRY=0; YES=0; VERIFY=1; QUICK=0; CONFONLY=0
while [ $# -gt 0 ]; do
  case "$1" in
    --image) IMG="$2"; shift ;;
    --device) DEV="$2"; shift ;;
    --user) USR="$2"; shift ;;
    --pass) PWD_="$2"; shift ;;
    --host) HOST="$2"; shift ;;
    --ssid) SSID="$2"; shift ;;
    --wifi-pass) WPASS="$2"; shift ;;
    --dry-run) DRY=1 ;;
    --no-verify) VERIFY=0 ;;
    --verify-quick) QUICK=1 ;;
    --configure-only) CONFONLY=1 ;;
    --yes) YES=1 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "option inconnue : $1" >&2; exit 1 ;;
  esac
  shift
done
[ -n "$IMG" ] && [ -n "$DEV" ] || { echo "--image et --device sont requis" >&2; exit 1; }

msg() { printf '\n== %s\n' "$*"; }
run() { if [ "$DRY" = "1" ]; then echo "   [dry-run] $*"; else eval "$@"; fi; }

# ---------------------------------------------------------------------------
#  GARDE-FOUS. Se tromper de peripherique efface le disque systeme : ces
#  verifications ne sont pas negociables.
# ---------------------------------------------------------------------------
msg "verification de la cible $DEV"
[ -b "$DEV" ] || { echo "   $DEV n'est pas un peripherique bloc" >&2; exit 1; }

BASE="$(basename "$DEV")"
# 1) refus categorique de tout ce qui porte le systeme
ROOT_SRC="$(findmnt -no SOURCE / 2>/dev/null)"
ROOT_DISK="$(lsblk -no PKNAME "$ROOT_SRC" 2>/dev/null | head -1)"
if [ "$BASE" = "$ROOT_DISK" ] || [ "$DEV" = "$ROOT_SRC" ]; then
  echo "   REFUS : $DEV porte le systeme de fichiers racine." >&2
  exit 1
fi
case "$BASE" in
  nvme*|sda) echo "   REFUS : $DEV ressemble a un disque interne, pas a une carte SD." >&2
             echo "   Si c'est bien une carte, la graver avec un autre outil." >&2
             exit 1 ;;
esac
# 2) taille plausible pour une carte SD
SZ=$(lsblk -bdno SIZE "$DEV" 2>/dev/null)
python3 - "$SZ" <<'PY' || exit 1
import sys
n = int(sys.argv[1])
print(f"   taille : {n/1024**3:.1f} Go")
if n > 1024**4:
    print("   REFUS : plus de 1 To, ce n'est pas une carte SD.")
    sys.exit(1)
if n < 14*1024**3:
    print("   REFUS : moins de 14 Go, l'image ne tiendra pas.")
    sys.exit(1)
PY

if [ "$CONFONLY" = "1" ]; then
  msg "mode --configure-only : aucune gravure, la carte n'est pas effacee"
fi

if [ "$CONFONLY" = "0" ]; then
msg "contenu actuel de $DEV (sera DETRUIT)"
lsblk -o NAME,SIZE,FSTYPE,LABEL,MOUNTPOINT "$DEV" | sed 's/^/   /'
if [ "$YES" != "1" ] && [ "$DRY" != "1" ]; then
  echo
  read -r -p "   Tout effacer sur $DEV ? Taper exactement EFFACER : " rep
  [ "$rep" = "EFFACER" ] || { echo "   annule"; exit 1; }
fi

# demonter d'eventuelles partitions montees, sinon dd ecrit sous les pieds du noyau
for m in $(lsblk -no MOUNTPOINT "$DEV" 2>/dev/null | grep -v '^$'); do
  msg "demontage de $m"; run "sudo umount '$m'"
done

# ---------------------------------------------------------------------------
#  DECOMPRESSION ET GRAVURE
# ---------------------------------------------------------------------------
WORK="$(dirname "$IMG")"
case "$IMG" in
  *.zip)
    msg "verification de l'archive"
    run "unzip -t '$IMG' | tail -2"
    IMGNAME="$(unzip -Z1 "$IMG" | grep -iE '\.img$' | head -1)"
    [ -n "$IMGNAME" ] || { echo "   aucun .img dans l'archive" >&2; exit 1; }
    if [ -f "$WORK/$IMGNAME" ]; then
      echo "   $IMGNAME deja extrait"
    else
      msg "extraction de $IMGNAME (compter plusieurs minutes)"
      run "unzip -o '$IMG' '$IMGNAME' -d '$WORK'"
    fi
    RAW="$WORK/$IMGNAME" ;;
  *.img) RAW="$IMG" ;;
  *) echo "   format non gere : $IMG" >&2; exit 1 ;;
esac
[ "$DRY" = "1" ] || [ -f "$RAW" ] || { echo "   image absente : $RAW" >&2; exit 1; }

msg "gravure sur $DEV"
IMGSZ=$(stat -c%s "$RAW" 2>/dev/null || echo 0)
python3 -c "print(f'   image : {$IMGSZ/1024**3:.2f} Go')"
echo "   ne pas debrancher la carte. Compter 10 a 25 min selon le lecteur."
# iflag=fullblock : sans lui, une lecture courte (un signal qui interrompt read(),
# et status=progress en emet) compte comme un bloc entier et dd peut s'arreter
# AVANT la fin du fichier. Constate : 8.2 Go ecrits sur 16.8, "1961+6 records",
# aucune erreur signalee -- puis un montage qui echoue sur "bad superblock",
# symptome trompeur qui fait chercher du cote du systeme de fichiers.
run "sudo dd if='$RAW' of='$DEV' bs=4M iflag=fullblock conv=fsync status=progress"
run "sync"

# ---- VERIFICATION DE L'ECRITURE ----------------------------------------
# Ne PAS faire confiance a dd. La premiere version de ce script enchainait sur
# le montage sans controler, et a preconfigure dans le vide une carte a moitie
# gravee. Comparer cote a cote est lent (compter le double du temps de gravure)
# mais c'est la seule facon de savoir.
if [ "$VERIFY" = "1" ] && [ "$DRY" != "1" ]; then
  # Trois niveaux, selon la confiance qu'on a dans la carte :
  #   complet (defaut) : relit tout. Le seul qui detecte une carte qui ACCEPTE
  #                      les ecritures sans les stocker -- panne reellement
  #                      rencontree, et qu'aucun compteur de dd ne revele.
  #   --verify-quick   : relit les 2 premiers Go. Attrape une gravure tronquee
  #                      ou une carte franchement morte, en une quarantaine de
  #                      secondes. Ne prouve RIEN sur le reste de la carte.
  #   --no-verify      : rien. A reserver a une carte deja validee une fois.
  CHECKSZ="$IMGSZ"
  if [ "$QUICK" = "1" ]; then
    CHECKSZ=$((2 * 1024 * 1024 * 1024))
    [ "$CHECKSZ" -gt "$IMGSZ" ] && CHECKSZ="$IMGSZ"
  fi
  msg "verification octet par octet ($(python3 -c "print(f'{$CHECKSZ/1024**3:.1f}')") Go a relire)"
  if [ "$QUICK" = "1" ]; then
    echo "   mode rapide : seuls les 2 premiers Go sont controles."
    echo "   un defaut au-dela ne sera PAS vu."
  else
    echo "   compter environ le temps de gravure. --verify-quick pour n'en"
    echo "   controler que les 2 premiers Go."
  fi
  if sudo cmp -n "$CHECKSZ" "$RAW" "$DEV"; then
    if [ "$QUICK" = "1" ]; then
      echo "   les 2 premiers Go sont identiques (le reste n'est pas controle)."
    else
      echo "   IDENTIQUE : la carte porte bien l'image complete."
    fi
  else
    echo
    echo "   ECHEC DE LA VERIFICATION : la carte ne correspond pas a l'image." >&2
    echo "   Ne pas utiliser cette carte. Pistes :" >&2
    echo "     - relancer la gravure (une lecture courte peut avoir tronque dd)" >&2
    echo "     - essayer un autre lecteur de cartes, ou un port USB different" >&2
    echo "     - la carte est peut-etre defectueuse ou de capacite mensongere" >&2
    exit 1
  fi
else
  echo "   verification DESACTIVEE (--no-verify) : on ne sait pas si l'ecriture"
  echo "   est complete. Un montage qui echoue ensuite viendra probablement de la."
fi

run "sudo partprobe '$DEV' 2>/dev/null || true"
sleep 3
fi   # fin du bloc gravure, saute par --configure-only

# ---------------------------------------------------------------------------
#  AGRANDIR LA PARTITION RACINE
# ---------------------------------------------------------------------------
# L'image L4T fait 16.1 Go dont 15 DEJA OCCUPES : au premier demarrage la racine
# est pleine a 100 %, avec zero octet libre. Constate, et les consequences ne
# sautent pas aux yeux :
#   - `ssh-keygen` ne peut pas ecrire ses cles, donc sshd ne demarre jamais ;
#   - bash lui-meme echoue : "cannot create temp file for here-document" ;
#   - et rien de tout cela n'apparait dans un ping ni dans le mDNS, qui
#     repondent parfaitement -- la machine a l'air saine.
# Cette version de L4T n'a PAS de nvresizefs.service : l'agrandissement
# automatique au premier demarrage n'existe pas. On le fait donc ici, hors
# ligne, ou c'est plus simple et plus sur qu'a chaud.
if [ "$CONFONLY" = "0" ] && [ "$DRY" != "1" ]; then
  msg "agrandissement de la partition racine a toute la carte"
  # p1 doit etre la DERNIERE partition du disque, sinon on ecraserait la
  # suivante. Sur cette image c'est le cas (elle commence au secteur 1484800,
  # apres les 21 partitions de firmware), mais on le VERIFIE.
  P1_START=$(cat "/sys/block/$(basename "$DEV")/$(basename "$DEV")p1/start" 2>/dev/null)
  DERNIERE=1
  for d in /sys/block/$(basename "$DEV")/$(basename "$DEV")p*; do
    st=$(cat "$d/start" 2>/dev/null) || continue
    [ -n "$st" ] && [ "$st" -gt "${P1_START:-0}" ] && DERNIERE=0
  done
  if [ "$DERNIERE" = "0" ]; then
    echo "   p1 n'est pas la derniere partition : agrandissement ANNULE"
    echo "   (la racine restera a 16 Go, presque pleine)"
  else
    echo ',+' | sudo sfdisk --force -N 1 "$DEV" 2>&1 | grep -viE "^$|^Disk |^Device |^Old situation|^New situation" | tail -4 | sed 's/^/   /'
    sudo partx -u "$DEV" 2>/dev/null || sudo partprobe "$DEV" 2>/dev/null || true
    sleep 2
    APP_TMP="${DEV}p1"
    [ -b "$APP_TMP" ] || APP_TMP="${DEV}1"
    # e2fsck avant resize2fs : ce dernier refuse d'agrandir un systeme de
    # fichiers qu'il n'a pas verifie.
    sudo e2fsck -fp "$APP_TMP" >/dev/null 2>&1 || true
    sudo resize2fs "$APP_TMP" 2>&1 | tail -2 | sed 's/^/   /'
    python3 -c "
import subprocess
o = subprocess.run(['lsblk','-bdno','SIZE','$APP_TMP'],capture_output=True,text=True).stdout.strip()
print(f'   racine portee a {int(o)/1024**3:.1f} Go') if o.isdigit() else None"
  fi
fi

# ---------------------------------------------------------------------------
#  PRECONFIGURATION HORS LIGNE
# ---------------------------------------------------------------------------
# La partition racine de l'image L4T est APP : la plus grande, en ext4.
msg "recherche de la partition racine (APP)"
if [ "$DRY" = "1" ]; then
  echo "   [dry-run] detection impossible sans gravure"
  exit 0
fi
APP=""
BIGGEST=0
for p in $(lsblk -lno NAME,FSTYPE,SIZE "$DEV" | awk '$2=="ext4"{print $1}'); do
  s=$(lsblk -bdno SIZE "/dev/$p")
  if [ "$s" -gt "$BIGGEST" ]; then BIGGEST=$s; APP="/dev/$p"; fi
done
[ -n "$APP" ] || { echo "   aucune partition ext4 trouvee" >&2; exit 1; }
echo "   APP = $APP ($(python3 -c "print(f'{$BIGGEST/1024**3:.1f} Go')"))"

MNT="$(mktemp -d)"
if ! sudo mount "$APP" "$MNT"; then
  echo "   MONTAGE IMPOSSIBLE de $APP." >&2
  echo "   Neuf fois sur dix la gravure est INCOMPLETE : la table de partitions" >&2
  echo "   est ecrite au tout debut de la carte, donc les partitions semblent" >&2
  echo "   correctes, alors que le systeme de fichiers qu'elles contiennent est" >&2
  echo "   tronque. Verifier le nombre d'octets annonce par dd et relancer." >&2
  exit 1
fi
cleanup() { sudo umount "$MNT" 2>/dev/null; rmdir "$MNT" 2>/dev/null; }
trap cleanup EXIT

[ -f "$MNT/etc/passwd" ] || { echo "   ce n'est pas une racine Linux" >&2; exit 1; }
echo "   systeme : $(grep -m1 PRETTY_NAME "$MNT/etc/os-release" | cut -d'"' -f2)"

# --- 1) utilisateur -------------------------------------------------------
msg "creation de l'utilisateur $USR"
if grep -q "^$USR:" "$MNT/etc/passwd"; then
  echo "   deja present, inchange"
else
  # premier uid/gid libre >= 1000 : ne pas supposer que 1000 est disponible
  UID_N=$(python3 - "$MNT/etc/passwd" <<'PY'
import sys
used = set()
for l in open(sys.argv[1]):
    f = l.split(':')
    if len(f) > 2 and f[2].isdigit():
        used.add(int(f[2]))
u = 1000
while u in used:
    u += 1
print(u)
PY
)
  echo "   uid/gid : $UID_N"
  HASH="$(openssl passwd -6 "$PWD_")"
  echo "$USR:x:$UID_N:$UID_N:$USR,,,:/home/$USR:/bin/bash" | sudo tee -a "$MNT/etc/passwd" >/dev/null
  echo "$USR:$HASH:19000:0:99999:7:::" | sudo tee -a "$MNT/etc/shadow" >/dev/null
  echo "$USR:x:$UID_N:" | sudo tee -a "$MNT/etc/group" >/dev/null
  echo "$USR:!::" | sudo tee -a "$MNT/etc/gshadow" >/dev/null
  sudo mkdir -p "$MNT/home/$USR"
  sudo cp -a "$MNT/etc/skel/." "$MNT/home/$USR/" 2>/dev/null || true
  sudo chown -R "$UID_N:$UID_N" "$MNT/home/$USR"
  sudo chmod 750 "$MNT/home/$USR"

  # Groupes : on n'ajoute QUE ceux qui existent dans l'image. Inventer un groupe
  # absent casserait la connexion. dialout est indispensable (port serie de
  # l'ESP32), video/i2c/gpio le sont pour le materiel du Jetson.
  for g in sudo adm dialout cdrom audio video plugdev games users input i2c gpio \
           weston-launch render docker; do
    if grep -q "^$g:" "$MNT/etc/group"; then
      sudo sed -i "s/^\($g:[^:]*:[^:]*:\)\(.*\)$/\1\2,$USR/; s/,,$USR/,$USR/; s/:,$USR/:$USR/" \
        "$MNT/etc/group"
      echo "   + groupe $g"
    fi
  done
fi

# --- 2) nom de machine ----------------------------------------------------
msg "nom de machine : $HOST"
echo "$HOST" | sudo tee "$MNT/etc/hostname" >/dev/null
# /etc/hosts doit suivre, sinon sudo se plaint a chaque appel ("unable to
# resolve host") et ralentit chaque commande de plusieurs secondes.
if grep -qE "^127\.0\.1\.1" "$MNT/etc/hosts"; then
  sudo sed -i "s/^127\.0\.1\.1.*/127.0.1.1\t$HOST/" "$MNT/etc/hosts"
else
  echo -e "127.0.1.1\t$HOST" | sudo tee -a "$MNT/etc/hosts" >/dev/null
fi

# --- 3) desamorcer oem-config -------------------------------------------
msg "desactivation de l'assistant de premier demarrage"
# Sans cela la carte demarre et ATTEND un ecran : aucun reseau, aucun SSH.
# On cherche les unites reellement presentes plutot que de deviner leur nom,
# qui change entre versions de L4T.
FOUND=0
for u in $(sudo find "$MNT/etc/systemd/system" "$MNT/lib/systemd/system" \
           -name '*oem-config*' 2>/dev/null); do
  echo "   trouve : ${u#$MNT}"
  FOUND=1
done
# retirer les liens qui l'activent
for w in $(sudo find "$MNT/etc/systemd/system" -path '*.wants/*oem-config*' 2>/dev/null); do
  sudo rm -f "$w"; echo "   desactive : ${w#$MNT}"
done
# masquer les services, au cas ou ils seraient reactives par un autre chemin
for s in nv-oem-config.service nv-oem-config-gui.service oem-config.service \
         nvfb-oem-config.service; do
  if [ -e "$MNT/lib/systemd/system/$s" ] || [ -e "$MNT/etc/systemd/system/$s" ]; then
    sudo ln -sf /dev/null "$MNT/etc/systemd/system/$s"
    echo "   masque : $s"
  fi
done
# ---- LA CIBLE PAR DEFAUT : c'est ICI que tout se joue -------------------
# Verifie sur l'image JP514 (JetPack 5.1.5) : /etc/systemd/system/default.target
# est un lien vers nv-oem-config.target, dont l'unite declare
#     Conflicts=rescue.service rescue.target multi-user.target
# Ce Conflicts est le coeur du probleme : demarrer sur cette cible EMPECHE
# activement multi-user.target, donc tout ce que contient
# multi-user.target.wants -- ou ssh.service et avahi-daemon.service sont
# DEJA actives dans l'image. Masquer nv-oem-config.service ne suffit donc pas :
# sans repointer default.target, la carte demarre sans SSH et reste injoignable.
#
# Ce repointage ne perd aucun service : comparaison faite entre
# nv-oem-config.target.wants et multi-user.target.wants sur l'image, tous les
# services L4T (nvfb-early, nvpower, nvpmodel, nvfancontrol, nvweston) sont
# presents dans les deux. Le seul absent de multi-user est nv-oem-config.service
# lui-meme -- exactement celui dont on ne veut pas.
DEF="$(sudo readlink "$MNT/etc/systemd/system/default.target" 2>/dev/null)"
echo "   cible par defaut actuelle : ${DEF:-<aucun lien>}"
case "$(basename "${DEF:-}")" in
  multi-user.target|graphical.target)
    echo "   -> deja correcte, inchangee" ;;
  *)
    sudo ln -sf /lib/systemd/system/multi-user.target \
      "$MNT/etc/systemd/system/default.target"
    echo "   -> REPOINTEE sur multi-user.target (etape indispensable)" ;;
esac
[ "$FOUND" = "0" ] && echo "   (aucune unite oem-config : image deja preconfiguree ?)"

# LICENCE. L'outil officiel de NVIDIA (tools/l4t_create_default_user.sh, fourni
# avec le BSP et non avec l'image SD) prend un --accept-license : c'est
# l'acceptation de la licence qui, avec l'existence d'un utilisateur, decide si
# l'assistant se lance. La documentation ne dit pas quel fichier porte ce
# marqueur, et il change selon les versions de L4T -- on INSPECTE donc plutot
# que de deviner, et on cree les marqueurs connus s'ils sont attendus.
msg "marqueurs de premier demarrage dans /etc/nv"
if [ -d "$MNT/etc/nv" ]; then
  sudo ls -la "$MNT/etc/nv" | sed 's/^/   /'
else
  echo "   /etc/nv absent"
fi
sudo mkdir -p "$MNT/etc/nv"
# nvfirstboot : sa PRESENCE declenche la configuration initiale ; on le retire.
if [ -e "$MNT/etc/nv/nvfirstboot" ]; then
  sudo rm -f "$MNT/etc/nv/nvfirstboot"
  echo "   /etc/nv/nvfirstboot retire (il declenchait la configuration initiale)"
fi
# Marqueur d'acceptation de licence, tel que le pose l'outil officiel.
echo "1" | sudo tee "$MNT/etc/nv/.nv-l4t-license-accepted" >/dev/null 2>&1 || true
# Trace de ce qui a ete fait, lisible depuis la carte une fois demarree.
sudo tee "$MNT/etc/nv/preconfigure-mowbot.txt" >/dev/null <<EOF
Carte preparee hors ligne le $(date -Iseconds) depuis $(hostname).
  utilisateur : $USR        machine : $HOST
  wifi        : ${SSID:-aucun}
  oem-config  : unites masquees, liens .wants retires
Equivaut a  tools/l4t_create_default_user.sh -u $USR -p *** -n $HOST --accept-license
de la BSP L4T, que l'image SD seule ne contient pas.
EOF
echo "   trace ecrite dans /etc/nv/preconfigure-mowbot.txt"

# --- 4) Wi-Fi ------------------------------------------------------------
if [ -n "$SSID" ]; then
  msg "profil Wi-Fi : $SSID"
  ND="$MNT/etc/NetworkManager/system-connections"
  if [ -d "$MNT/etc/NetworkManager" ]; then
    sudo mkdir -p "$ND"
    UUID="$(cat /proc/sys/kernel/random/uuid)"
    sudo tee "$ND/$SSID.nmconnection" >/dev/null <<EOF
[connection]
id=$SSID
uuid=$UUID
type=wifi
autoconnect=true
autoconnect-priority=10
autoconnect-retries=0

[wifi]
mode=infrastructure
ssid=$SSID

[wifi-security]
key-mgmt=wpa-psk
psk=$WPASS

[ipv4]
method=auto

[ipv6]
method=auto
addr-gen-mode=stable-privacy
EOF
    # NetworkManager REFUSE un profil lisible par tous : il l'ignore
    # silencieusement et la carte reste hors reseau.
    sudo chmod 600 "$ND/$SSID.nmconnection"
    sudo chown 0:0 "$ND/$SSID.nmconnection"
    echo "   ecrit, droits 600 (NetworkManager ignore un profil trop permissif)"
    # la radio ne doit pas etre bloquee au demarrage
    sudo rm -f "$MNT/var/lib/NetworkManager/NetworkManager.state" 2>/dev/null || true
  else
    echo "   NetworkManager absent de l'image : profil non ecrit"
  fi
fi

# --- 5) SSH et mDNS ------------------------------------------------------
# Sur l'image JP514 ces deux services sont DEJA dans multi-user.target.wants :
# ce qui les empechait de demarrer n'etait pas leur activation mais la cible par
# defaut (voir ci-dessus). Les liens ci-dessous sont donc une simple ceinture de
# securite, sans effet sur une image standard.
msg "SSH et decouverte par nom"

# CLES D'HOTE. L'image L4T est livree SANS cles ssh_host_* : c'est oem-config
# qui les genere au premier demarrage. En le desactivant on supprime le seul
# mecanisme qui les creait, et sshd refuse de demarrer :
#     sshd: no hostkeys available -- exiting.
#     ssh.service: Start request repeated too quickly.
# La machine demarre alors tout a fait normalement -- ping, mDNS, Wi-Fi, nom
# d'hote -- mais le port 22 reste ferme. Symptome trompeur : tout fonctionne
# SAUF l'unique moyen d'entrer sur une carte sans clavier ni ecran.
if ls "$MNT"/etc/ssh/ssh_host_*_key >/dev/null 2>&1; then
  echo "   cles d'hote deja presentes"
else
  echo "   generation des cles d'hote (absentes de l'image)"
  # `ssh-keygen -A -f <prefixe>` cree d'un coup les types manquants (RSA, ECDSA,
  # ed25519) AVEC les bons droits, en travaillant sous <prefixe>/etc/ssh au lieu
  # du /etc/ssh local. Verifie sur OpenSSH 9.6.
  if sudo ssh-keygen -A -f "$MNT"; then
    ls "$MNT"/etc/ssh/ssh_host_*_key 2>/dev/null | while read -r k; do
      echo "     + $(basename "$k")"
    done
  else
    echo "   ECHEC de la generation : sshd ne demarrera pas." >&2
    echo "   Le corriger a la main :  sudo ssh-keygen -A -f <racine de la carte>" >&2
  fi
fi

WANTS="$MNT/etc/systemd/system/multi-user.target.wants"
sudo mkdir -p "$WANTS"
for s in ssh.service sshd.service; do
  if [ -e "$MNT/lib/systemd/system/$s" ]; then
    sudo ln -sf "/lib/systemd/system/$s" "$WANTS/$s"
    echo "   SSH active ($s)"
    break
  fi
done
[ -e "$MNT/lib/systemd/system/ssh.service" ] || \
  echo "   ATTENTION : openssh-server absent de l'image, SSH impossible"
if [ -e "$MNT/lib/systemd/system/avahi-daemon.service" ]; then
  sudo ln -sf /lib/systemd/system/avahi-daemon.service "$WANTS/avahi-daemon.service"
  echo "   mDNS active -> joignable en $HOST.local"
else
  echo "   avahi absent : $HOST.local ne marchera pas, il faudra chercher l'IP"
fi
# Empeche l'assistant de se relancer via cloud-init s'il est present
[ -d "$MNT/etc/cloud" ] && sudo touch "$MNT/etc/cloud/cloud-init.disabled" && \
  echo "   cloud-init desactive"

msg "termine"
cat <<EOF
   Carte prete. Au premier demarrage :
     - compter 2 a 3 min (le systeme redimensionne et se configure)
     - la trouver :  python3 pc/find_board.py $HOST.local
     - se connecter : ssh $USR@<ip>
   Si $HOST.local ne repond pas, brancher un cable Ethernet : plus fiable
   qu'un Wi-Fi mal configure pour un premier demarrage.
EOF
