#!/usr/bin/env python3
"""Valide les fichiers de configuration AVANT tout deploiement.

POURQUOI CE SCRIPT EXISTE : le double tiret dans un commentaire XML rend le
fichier invalide, et cette erreur a ete commise CINQ fois sur ce projet -- trois
fois avant le 04/09/2026, deux fois ce jour-la. La derniere a envoye un
bt_ackerbot.xml casse sur le robot et redemarre nav2 dessus.

    robot/bin/check_config.py            valide tout robot/config/
    robot/bin/check_config.py f1 f2 ...  valide ces fichiers

Sort 1 des qu'un fichier est invalide. install.sh valide deja a l'installation ;
ceci sert AVANT, pour les deploiements cibles a la main.
"""
import glob
import os
import re
import sys
import xml.etree.ElementTree as ET

import yaml


def double_tiret(chemin):
    """Le piege maison : '--' hors de <!-- et -->, qui invalide le commentaire."""
    t = open(chemin, encoding="utf-8").read()
    i = 0
    while i < len(t):
        if t.startswith("<!--", i):
            i += 4
        elif t.startswith("-->", i):
            i += 3
        elif t.startswith("--", i):
            return t[:i].count("\n") + 1
        else:
            i += 1
    return None


def valide(chemin):
    nom = os.path.basename(chemin)
    try:
        if nom.endswith((".yaml", ".yml")):
            yaml.safe_load(open(chemin, encoding="utf-8"))
        elif nom.endswith((".xml", ".urdf", ".xacro")):
            ligne = double_tiret(chemin)
            if ligne:
                print("  INVALIDE %s : double tiret dans un commentaire, ligne %d" % (nom, ligne))
                return False
            ET.parse(chemin)
        else:
            return True
    except Exception as e:
        print("  INVALIDE %s : %s" % (nom, str(e).split("\n")[0][:90]))
        return False
    print("  ok       %s" % nom)
    return True


def main():
    cibles = sys.argv[1:]
    if not cibles:
        ici = os.path.dirname(os.path.abspath(__file__))
        cibles = sorted(glob.glob(os.path.join(ici, "..", "config", "*")))
    cibles = [c for c in cibles if os.path.isfile(c) and ".local." not in os.path.basename(c)]
    mauvais = [c for c in cibles if not valide(c)]
    if mauvais:
        print("ECHEC : %d fichier(s) invalide(s), NE PAS DEPLOYER" % len(mauvais))
        return 1
    print("%d fichier(s) valides" % len(cibles))
    return 0


if __name__ == "__main__":
    sys.exit(main())
