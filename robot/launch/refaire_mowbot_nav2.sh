#!/bin/bash
# Regenere mowbot_nav2.launch.py depuis le navigation_launch.py de nav2 installe.
#
# A LANCER APRES CHAQUE MISE A JOUR DE NAV2, jamais editer le fork a la main :
# les retraits sont rejoues mecaniquement, donc les corrections apportees en
# amont a nav2 sont reprises, et le script s'arrete net si la structure du
# fichier amont a change (nom de paquet, forme des blocs).
#
#   bash refaire_mowbot_nav2.sh                    # cherche le fichier tout seul
#   bash refaire_mowbot_nav2.sh /chemin/vers/navigation_launch.py
set -euo pipefail
ICI="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"

AMONT="${1:-}"
if [ -z "$AMONT" ]; then
  # Dans le conteneur ou sur une machine avec ROS : le fichier est sous /opt/ros.
  AMONT="$(ls -1 /opt/ros/*/share/nav2_bringup/launch/navigation_launch.py 2>/dev/null | head -1 || true)"
fi
if [ -z "$AMONT" ] && command -v docker >/dev/null 2>&1; then
  # Depuis l'hote de la jetson : nav2 n'est que dans le conteneur.
  CT="${MOWBOT_CONTAINER:-mowbot_jazzy}"
  if docker ps --format '{{.Names}}' | grep -qx "$CT"; then
    AMONT="/tmp/navigation_launch_amont.py"
    docker exec "$CT" sh -c 'cat /opt/ros/*/share/nav2_bringup/launch/navigation_launch.py' > "$AMONT"
  fi
fi
[ -n "$AMONT" ] && [ -s "$AMONT" ] || {
  echo "navigation_launch.py introuvable. Donner son chemin en argument." >&2; exit 1; }
echo "amont : $AMONT"

AMONT="$AMONT" SORTIE="$ICI/mowbot_nav2.launch.py" python3 - <<'PY'
import os, sys
src = open(os.environ['AMONT']).read()
lines = src.split('\n')

# 1. la liste des noeuds geres par lifecycle_manager
avant = len(lines)
lines = [l for l in lines if l.strip() not in ("'route_server',", "'docking_server',")]
if avant - len(lines) != 2:
    sys.exit(f"ECHEC : {avant-len(lines)} entree(s) retiree(s) de lifecycle_nodes au lieu de 2.\n"
             "La liste amont a change : verifier navigation_launch.py a la main.")

# 2. les blocs Node(...) / ComposableNode(...) des deux paquets. On equilibre les
# parentheses plutot que de compter des lignes, pour survivre a une remise en
# forme du fichier amont.
CIBLES = ("package='nav2_route'", "package='opennav_docking'")
retires = 0
while True:
    idx = next((i for i, l in enumerate(lines) if any(c in l for c in CIBLES)), None)
    if idx is None:
        break
    debut = idx
    while debut >= 0 and not lines[debut].strip().endswith('Node('):
        debut -= 1
    if debut < 0:
        sys.exit("ECHEC : ouverture de bloc Node( introuvable au-dessus du paquet cible.")
    bal, fin = 0, debut
    while fin < len(lines):
        bal += lines[fin].count('(') - lines[fin].count(')')
        if bal == 0:
            break
        fin += 1
    else:
        sys.exit("ECHEC : parentheses non equilibrees, bloc non delimitable.")
    lines = lines[:debut] + lines[fin+1:]
    retires += 1
if retires != 4:
    sys.exit(f"ECHEC : {retires} bloc(s) retire(s) au lieu de 4 "
             "(2 Node + 2 ComposableNode). Structure amont modifiee.")

res = '\n'.join(lines)
for interdit in ('nav2_route', 'route_server', 'opennav_docking', 'docking_server'):
    if interdit in res:
        sys.exit(f"ECHEC : il reste des traces de {interdit}.")

# On garde l'en-tete explicatif du fork deja en place, et on ne remplace que le
# code : sinon la justification des retraits disparaitrait a chaque resync.
sortie = os.environ['SORTIE']
entete = ''
if os.path.exists(sortie):
    ancien = open(sortie).read()
    if '\nimport os' in ancien:
        entete = ancien[:ancien.index('\nimport os')]
if not entete:
    sys.exit(f"ECHEC : en-tete du fork introuvable dans {sortie}. "
             "Le recreer a la main avant de resynchroniser.")

open(sortie, 'w').write(entete + res[res.index('\nimport os'):])
print(f"  ecrit : {sortie}")
PY

python3 -m py_compile "$ICI/mowbot_nav2.launch.py"
echo "  python valide"
echo "  noeuds geres :"
sed -n '/lifecycle_nodes = \[/,/\]/p' "$ICI/mowbot_nav2.launch.py" | grep -oP "'\K[a-z_]+(?=')" | sed 's/^/    /'
