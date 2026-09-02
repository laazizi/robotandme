#!/usr/bin/env bash
# Test hote de la cinematique Ackermann. Ne touche ni a l'ESP-IDF ni au robot.
set -u
cd "$(dirname "$(readlink -f "$0")")"
echec=0
for geo in "-0.36f:ackerbot_p4 (roue DERRIERE)" "+0.36f:variante roue DEVANT"; do
  xs="${geo%%:*}"; nom="${geo#*:}"
  echo "=============================================================="
  echo "  $nom   STEER_X_M = $xs"
  echo "=============================================================="
  gcc -std=gnu11 -O2 -Wall -Wextra -I. -o /tmp/test_ackermann \
      -DSTEER_X_M="($xs)" test_ackermann.c -lm || { echo "COMPILATION KO"; exit 2; }
  /tmp/test_ackermann || echec=1
  echo
done
echo "=============================================================="
echo "  geometrie nav2 (robot/config/ackerbot_geometry.env) a jour avec robot.h ?"
echo "=============================================================="
python3 ../../../../robot/bin/gen_ackerbot_geometry.py --check || echec=1
echo
[ "$echec" -eq 0 ] && echo "### LES DEUX GEOMETRIES PASSENT" || echo "### AU MOINS UNE GEOMETRIE ECHOUE"
exit "$echec"
