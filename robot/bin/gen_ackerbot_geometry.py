#!/usr/bin/env python3
"""Derive la geometrie nav2 de l'Ackermann DEPUIS LE FIRMWARE, jamais a la main.

Lit controllers/ackerbot_p4/main/robot.h (STEER_X_M, STEER_MAX_RAD,
TRACK_WIDTH_M) et ecrit robot/config/ackerbot_geometry.env, que start_nav.sh
source pour injecter minimum_turning_radius & co dans nav2_params_ackerbot.yaml.

Pourquoi un fichier GENERE et non la lecture directe de robot.h par le SBC :
install.sh ne deploie pas controllers/ sur le robot. Le .env voyage avec
config/, et ce script garantit qu'il ne peut pas se desynchroniser :

    gen_ackerbot_geometry.py            regenere le .env
    gen_ackerbot_geometry.py --check    echoue (code 1) si le .env est PERIME
                                        (appele par kin_ackermann/test/run.sh)

Regle du CLAUDE.md : le rayon de braquage minimal se DERIVE (R = |x_s|/tan(dmax)),
il ne se saisit jamais cote SBC. Une macro qui codait tan(0,52) en dur a deja
produit 75 % d'erreur sur cette valeur ; d'ou ce garde-fou.
"""
import math, re, sys, os

ICI = os.path.dirname(os.path.abspath(__file__))
DEPOT = os.path.abspath(os.path.join(ICI, "..", ".."))
ROBOT_H = os.path.join(DEPOT, "controllers", "ackerbot_p4", "main", "robot.h")
SPEEDS = os.path.join(DEPOT, "robot", "config", "speeds.env")
SORTIE = os.path.join(DEPOT, "robot", "config", "ackerbot_geometry.env")


def lire_define(texte, nom):
    m = re.search(r"^#define\s+%s\s+\(?\s*([-+]?[0-9.]+)f?\s*\)?" % re.escape(nom), texte, re.M)
    if not m:
        sys.exit("ERREUR : %s introuvable dans %s" % (nom, ROBOT_H))
    return float(m.group(1))


def lire_speed(nom):
    for ligne in open(SPEEDS, encoding="utf-8"):
        m = re.match(r"^%s=([-+0-9.]+)" % re.escape(nom), ligne)
        if m:
            return float(m.group(1))
    sys.exit("ERREUR : %s introuvable dans %s" % (nom, SPEEDS))


def generer():
    h = open(ROBOT_H, encoding="utf-8").read()
    xs = lire_define(h, "STEER_X_M")
    dmax = lire_define(h, "STEER_MAX_RAD")
    voie = lire_define(h, "TRACK_WIDTH_M")
    vmax = lire_speed("ACKERBOT_MAX_VEL_X")

    r_min = abs(xs) / math.tan(dmax)
    k_max = voie * math.tan(dmax) / (2.0 * abs(xs))
    d_crit = math.degrees(math.atan(2.0 * abs(xs) / voie))
    # A R_min la roue exterieure va a (1+k_max) v : pour rester sous vmax roue,
    # la vitesse d'essieu en virage serre est bornee a vmax/(1+k_max). RPP
    # regule la vitesse en courbe ; on lui donne cette borne comme plancher
    # de securite, et w_max coherent avec R_min a la vitesse de croisiere.
    v_virage = vmax / (1.0 + k_max)
    w_max = vmax / r_min

    return "\n".join([
        "# GENERE par bin/gen_ackerbot_geometry.py DEPUIS controllers/ackerbot_p4/main/robot.h",
        "# NE PAS EDITER A LA MAIN : relancer le script. `--check` detecte un fichier perime.",
        "# Source : STEER_X_M=%+.3f m  STEER_MAX_RAD=%.4f rad (%.1f deg)  TRACK_WIDTH_M=%.4f m"
        % (xs, dmax, math.degrees(dmax), voie),
        "# Vitesse de croisiere ACKERBOT_MAX_VEL_X=%.2f m/s (config/speeds.env)" % vmax,
        "",
        "# R = |x_s| / tan(delta_max) : rayon de braquage MINIMAL, a fournir a nav2",
        "ACKERBOT_MIN_TURNING_RADIUS=%.3f" % r_min,
        "# k = voie*tan(delta_max)/(2|x_s|) : a la butee, roue int a %.0f %%, roue ext a %.0f %%"
        % (100 * (1 - k_max), 100 * (1 + k_max)),
        "ACKERBOT_K_MAX=%.3f" % k_max,
        "# angle ou la roue interieure s'arreterait (au-dela elle recule) : %.1f deg" % d_crit,
        "# vitesse d'essieu max en virage serre pour que la roue exterieure reste sous vmax",
        "ACKERBOT_V_VIRAGE_SERRE=%.3f" % v_virage,
        "# w_max = vmax / R_min : rotation max coherente avec le rayon a la vitesse de croisiere",
        "ACKERBOT_MAX_VEL_THETA=%.2f" % w_max,
        "# position signee de la roue directrice (negatif = DERRIERE l'essieu moteur)",
        "ACKERBOT_STEER_X=%+.3f" % xs,
        "",
    ])


def main():
    contenu = generer()
    if "--check" in sys.argv:
        actuel = open(SORTIE, encoding="utf-8").read() if os.path.exists(SORTIE) else ""
        if actuel != contenu:
            sys.stderr.write("PERIME : %s ne correspond plus a robot.h -- relancer %s\n"
                             % (os.path.relpath(SORTIE, DEPOT), os.path.basename(__file__)))
            sys.exit(1)
        print("ackerbot_geometry.env a jour")
        return
    open(SORTIE, "w", encoding="utf-8").write(contenu)
    print("ecrit : %s" % os.path.relpath(SORTIE, DEPOT))
    print(contenu)


if __name__ == "__main__":
    main()
