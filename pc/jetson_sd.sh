#!/bin/bash
# ============================================================================
#  Carte SD Jetson, de bout en bout : telechargement, verification, gravure,
#  et preconfiguration pour un premier demarrage SANS CLAVIER NI ECRAN.
#
#    ./jetson_sd.sh --device /dev/mmcblk0
#    ./jetson_sd.sh --device /dev/mmcblk0 --host oldjetson --ssid aaa --wifi-pass 12345678
#    ./jetson_sd.sh --device /dev/mmcblk0 --dry-run        montre tout, n'ecrit rien
#    ./jetson_sd.sh --identify                             lit juste l'identite de la carte
#
#  Enchaine :
#    1. telechargement de l'image officielle NVIDIA (reprise si interrompu)
#    2. controle d'integrite de l'archive
#    3. extraction
#    4. identification de la carte SD, et comparaison avec la fois precedente
#    5. gravure verifiee octet par octet
#    6. utilisateur, nom de machine, Wi-Fi, SSH, mDNS
#
#  Appelle flash_jetson_sd.sh pour les etapes 5 et 6.
# ============================================================================
set -o pipefail
HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"

# Image SD du Jetson Xavier NX. C'est la DERNIERE publiee pour cette carte :
# JetPack 5.1.7 / L4T 35.6.5 est la derniere version compatible Xavier NX, mais
# NVIDIA n'a pas reconstruit l'image SD au-dela de 5.1.5 -- d'ou le nom JP514.
# Ubuntu 20.04. Au-dela, JetPack ne cible plus que Orin et Thor.
URL="https://developer.download.nvidia.com/embedded/L4T/r35_Release_v6.0/JP514-xnx-sd-card-image_b11.zip"
ZIP_SIZE=8058626384
IMG_SIZE=18056478720
IMG_NAME="sd-blob.img"

DL_DIR="$HOME/Téléchargements/jetson"
DEV=""; HOST="oldjetson"; USR="nvidia"; PASS="nvidia"
SSID=""; WPASS=""; DRY=0; YES=0; IDENT=0; EXTRA=""
while [ $# -gt 0 ]; do
  case "$1" in
    --device) DEV="$2"; shift ;;
    --host) HOST="$2"; shift ;;
    --user) USR="$2"; shift ;;
    --pass) PASS="$2"; shift ;;
    --ssid) SSID="$2"; shift ;;
    --wifi-pass) WPASS="$2"; shift ;;
    --dir) DL_DIR="$2"; shift ;;
    --dry-run) DRY=1; EXTRA="$EXTRA --dry-run" ;;
    --yes) YES=1; EXTRA="$EXTRA --yes" ;;
    --no-verify) EXTRA="$EXTRA --no-verify" ;;
    --verify-quick) EXTRA="$EXTRA --verify-quick" ;;
    --identify) IDENT=1 ;;
    -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
    *) echo "option inconnue : $1" >&2; exit 1 ;;
  esac
  shift
done

etape() { printf '\n\033[1m### %s\033[0m\n' "$*"; }
STATE="$DL_DIR/.cartes-vues"

# ---------------------------------------------------------------------------
#  IDENTITE DE LA CARTE
# ---------------------------------------------------------------------------
# Cette etape n'existait pas au depart, et son absence a coute deux gravures :
# une carte defaillante avait ete remplacee, et rien ne permettait de verifier
# que le lecteur voyait bien la NOUVELLE. On releve donc l'identite gravee en
# usine (registre CID) avant toute ecriture, et on la compare aux precedentes.
identite() {
  local d="$1" sysdev
  sysdev="/sys/block/$(basename "$d")/device"
  [ -d "$sysdev" ] || { echo "   (pas d'infos constructeur : lecteur USB plutot que SD ?)"; return; }
  local name manfid serial date size
  name=$(cat "$sysdev/name" 2>/dev/null)
  manfid=$(cat "$sysdev/manfid" 2>/dev/null)
  serial=$(cat "$sysdev/serial" 2>/dev/null)
  date=$(cat "$sysdev/date" 2>/dev/null)
  size=$(lsblk -bdno SIZE "$d" 2>/dev/null)
  printf "   modele %s   fabricant %s   serie %s   date %s\n" \
    "${name:-?}" "${manfid:-?}" "${serial:-?}" "${date:-?}"
  python3 -c "print(f'   capacite annoncee : {$size/1024**3:.2f} Gio')" 2>/dev/null

  [ -n "$serial" ] || return
  mkdir -p "$DL_DIR"
  local vu
  vu=$(grep "^$serial " "$STATE" 2>/dev/null | tail -1)
  if [ -n "$vu" ]; then
    local ancien_size
    ancien_size=$(echo "$vu" | awk '{print $3}')
    if [ "$ancien_size" != "$size" ]; then
      echo "   ATTENTION : cette meme serie annoncait $(python3 -c "print(f'{$ancien_size/1024**3:.2f}')") Gio"
      echo "   auparavant. Les registres d'usine ne changent JAMAIS sur une carte"
      echo "   authentique : capacite mensongere probable. Ne pas l'utiliser."
    else
      echo "   deja vue le $(echo "$vu" | awk '{print $4}'), meme capacite"
    fi
  else
    echo "   carte inconnue jusqu'ici (premiere utilisation)"
  fi
  echo "$serial ${name:-?} $size $(date -Iseconds)" >> "$STATE"
}

if [ "$IDENT" = "1" ]; then
  [ -n "$DEV" ] || DEV=/dev/mmcblk0
  etape "identite de $DEV"
  identite "$DEV"
  exit 0
fi
[ -n "$DEV" ] || { echo "--device est requis (ex: --device /dev/mmcblk0)" >&2; exit 1; }

mkdir -p "$DL_DIR"
ZIP="$DL_DIR/$(basename "$URL")"
IMG="$DL_DIR/$IMG_NAME"

# ---------------------------------------------------------------------------
#  1. TELECHARGEMENT
# ---------------------------------------------------------------------------
etape "1/6  telechargement de l'image NVIDIA"
if [ -f "$IMG" ] && [ "$(stat -c%s "$IMG")" = "$IMG_SIZE" ]; then
  echo "   image deja extraite et complete, telechargement inutile"
elif [ -f "$ZIP" ] && [ "$(stat -c%s "$ZIP")" = "$ZIP_SIZE" ]; then
  echo "   archive deja complete ($(python3 -c "print(f'{$ZIP_SIZE/1024**3:.2f}')") Go)"
elif [ "$DRY" = "1" ]; then
  echo "   [dry-run] telechargerait $URL"
else
  echo "   7.51 Go a recuperer. Reprise automatique si la liaison tombe."
  # --speed-limit/--speed-time sont indispensables : sans eux le transfert
  # est reste bloque a debit NUL pendant dix minutes sans que curl abandonne.
  # Avec, il coupe des que le debit s'effondre et --retry relance aussitot.
  curl -L -C - --retry 50 --retry-delay 5 --retry-all-errors \
       --speed-limit 100000 --speed-time 30 \
       -o "$ZIP" "$URL" || { echo "   telechargement echoue" >&2; exit 1; }
  n=$(stat -c%s "$ZIP")
  [ "$n" = "$ZIP_SIZE" ] || {
    echo "   taille inattendue : $n au lieu de $ZIP_SIZE" >&2
    echo "   relancer la commande, curl reprendra ou il s'est arrete." >&2
    exit 1; }
fi

# ---------------------------------------------------------------------------
#  2. INTEGRITE
# ---------------------------------------------------------------------------
etape "2/6  controle d'integrite de l'archive"
if [ -f "$IMG" ] && [ "$(stat -c%s "$IMG")" = "$IMG_SIZE" ]; then
  echo "   image deja extraite, controle de l'archive inutile"
elif [ "$DRY" = "1" ]; then
  echo "   [dry-run] unzip -t"
else
  unzip -t "$ZIP" >/dev/null 2>&1 || {
    echo "   ARCHIVE CORROMPUE. La supprimer et relancer :" >&2
    echo "     rm '$ZIP'" >&2
    exit 1; }
  echo "   aucune erreur dans les donnees compressees"
fi

# ---------------------------------------------------------------------------
#  3. EXTRACTION
# ---------------------------------------------------------------------------
etape "3/6  extraction de $IMG_NAME"
if [ -f "$IMG" ] && [ "$(stat -c%s "$IMG")" = "$IMG_SIZE" ]; then
  echo "   deja extraite, taille correcte"
elif [ "$DRY" = "1" ]; then
  echo "   [dry-run] unzip"
else
  LIBRE=$(df -B1 --output=avail "$DL_DIR" | tail -1)
  python3 -c "
import sys
if $LIBRE < $IMG_SIZE + 1024**3:
    print(f'   PLACE INSUFFISANTE : {$LIBRE/1024**3:.1f} Go libres, il en faut 17.8')
    sys.exit(1)
print(f'   {$LIBRE/1024**3:.1f} Go libres, suffisant')" || exit 1
  unzip -o "$ZIP" "$IMG_NAME" -d "$DL_DIR" || { echo "   extraction echouee" >&2; exit 1; }
  n=$(stat -c%s "$IMG")
  [ "$n" = "$IMG_SIZE" ] || { echo "   image extraite incomplete : $n" >&2; exit 1; }
  echo "   $IMG_NAME extraite, taille conforme"
fi

# ---------------------------------------------------------------------------
#  4. IDENTITE DE LA CARTE
# ---------------------------------------------------------------------------
etape "4/6  identification de la carte $DEV"
identite "$DEV"

# ---------------------------------------------------------------------------
#  5 et 6. GRAVURE ET PRECONFIGURATION
# ---------------------------------------------------------------------------
etape "5/6 et 6/6  gravure verifiee, puis preconfiguration"
CMD=("$HERE/flash_jetson_sd.sh" --image "$IMG" --device "$DEV"
     --user "$USR" --pass "$PASS" --host "$HOST")
[ -n "$SSID" ] && CMD+=(--ssid "$SSID")
[ -n "$WPASS" ] && CMD+=(--wifi-pass "$WPASS")
# shellcheck disable=SC2086
exec bash "${CMD[@]}" $EXTRA
