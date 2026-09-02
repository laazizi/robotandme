#!/usr/bin/env python3
"""Le robot SUIT-IL les commandes de nav2 ? Compare /cmd_vel a /odom.

    mowbot node cmd_check.py            30 s d'observation
    mowbot node cmd_check.py 120        duree en secondes

A LANCER PENDANT QUE LE ROBOT NAVIGUE. Au repos il n'y a rien a mesurer.

POURQUOI CET OUTIL. Quand le robot reste plante alors que nav2 a bien calcule un
chemin, il y a deux explications possibles, et elles demandent des correctifs
opposes :
  1. nav2 ne trouve aucune commande valide  -> "No valid trajectories" dans le
     journal, c'est un probleme de PLANIFICATION ou de contour ;
  2. nav2 commande, mais le robot n'execute pas -> les roues bourdonnent sans
     entrainer le chassis (friction statique des moteurs), la commande part et
     rien ne bouge.
Le journal de nav2 ne distingue PAS ces deux cas : dans les deux il constate
l'absence de progres. Ici on regarde les deux bouts de la chaine.

/cmd_vel est bien le dernier maillon : le controleur publie sur cmd_vel_nav, le
velocity_smoother sur cmd_vel_smoothed, le collision_monitor sur /cmd_vel, et
c'est /cmd_vel que le firmware de l'ESP32 ecoute.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from tf2_ros import Buffer, TransformListener

# Sous ce seuil on considere la commande comme nulle : il y a toujours un peu de
# bruit sur les valeurs publiees.
CMD_NULLE = 0.005
# Sous ce seuil on considere le robot comme immobile. L'odometrie a du bruit
# meme a l'arret, mesure sur ce robot : quelques mm/s.
IMMOBILE_LIN = 0.01
# Rotation : 0.05 rad/s = 2,9 deg/s. NE PAS reutiliser le seuil lineaire ici --
# 0.01 rad/s vaut 0,57 deg/s, soit le bruit de l'odometrie, et toute rotation
# lente mais REELLE etait alors comptee comme non executee.
IMMOBILE_ROT = 0.05
# Nombre minimal de commandes de mouvement avant d'oser un verdict. En dessous,
# le pourcentage n'a aucun sens : sur un echantillon unique il vaut 0 ou 100 %.
MINI_VERDICT = 20
# Tranches de distance au but, en metres. La derniere correspond a
# xy_goal_tolerance : c'est la zone ou nav2 freine VOLONTAIREMENT
# (RotateToGoal phase 2, slowing_factor 5.0). Des commandes petites y sont
# normales ; ce qui compte est de savoir si le robot les EXECUTE.
TRANCHES = [(1.00, 99.0, 'loin (> 1 m)'),
            (0.30, 1.00, 'approche (0,30 a 1 m)'),
            (0.12, 0.30, 'finale (0,12 a 0,30 m)'),
            (0.00, 0.12, 'dans la tolerance (< 0,12 m)')]


class Suivi(Node):
    def __init__(self, duree):
        super().__init__('mowbot_cmd_check')
        self.duree = duree
        self.cmd = None          # derniere commande (v, w)
        self.cmd_t = 0.0
        self.paires = []          # (v_cmd, w_cmd, v_reel, w_reel)
        # Distance au but, pour separer l'approche finale du reste. Le but est
        # la DERNIERE pose du chemin global ; la position du robot vient de la
        # TF map->base_link, car le chemin est dans `map` et /odom dans `odom`.
        self.par_distance = []    # (distance, v_cmd, v_reel, w_cmd, w_reel)
        self.but = None
        self.buf = Buffer()
        TransformListener(self.buf, self)
        self.create_subscription(Path, '/plan', self._plan, 1)
        # /cmd_vel est publie en RELIABLE par le collision_monitor ; un abonne
        # BEST_EFFORT recoit bien d'un publieur RELIABLE (l'inverse est faux).
        self.create_subscription(Twist, '/cmd_vel', self.sur_cmd,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.sur_odom,
                                 qos_profile_sensor_data)

    def _plan(self, m):
        if m.poses:
            p = m.poses[-1].pose.position
            self.but = (p.x, p.y)

    def _distance_au_but(self):
        if self.but is None:
            return None
        try:
            t = self.buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        return math.hypot(t.transform.translation.x - self.but[0],
                          t.transform.translation.y - self.but[1])

    def sur_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)
        self.cmd_t = time.time()

    def sur_odom(self, m):
        # On APPARIE une mesure d'odometrie a la derniere commande, et seulement
        # si celle-ci est recente : sinon on comparerait la vitesse actuelle a
        # une consigne perimee. 0.5 s laisse le temps au robot de reagir sans
        # remonter a une commande sans rapport.
        if self.cmd is None or time.time() - self.cmd_t > 0.5:
            return
        self.paires.append((self.cmd[0], self.cmd[1],
                            m.twist.twist.linear.x, m.twist.twist.angular.z))
        d = self._distance_au_but()
        if d is not None:
            self.par_distance.append((d, self.cmd[0], m.twist.twist.linear.x,
                                      self.cmd[1], m.twist.twist.angular.z))


def med(xs):
    xs = sorted(xs)
    return xs[len(xs) // 2] if xs else 0.0


def rapport_approche(pd):
    """L'approche finale : nav2 commande-t-il trop peu, ou le robot ne suit pas ?

    C'est LA question a laquelle le taux de suivi global ne repond pas : nav2
    freine volontairement dans les derniers centimetres, donc des commandes
    petites y sont NORMALES. Ce qu'on veut savoir, c'est si elles sont
    executees. On separe donc les relevés par distance au but.
    """
    if not pd:
        print("\n--- APPROCHE FINALE : aucun but actif pendant l'observation ---")
        print("  (il faut un but en cours : /plan doit publier)")
        return
    print("\n--- APPROCHE FINALE, par distance au but ---")
    print("  %-26s %5s %11s %11s %8s %7s"
          % ("tranche", "n", "cmd med", "mesure med", "suivi", "inertes"))
    for lo, hi, nom in TRANCHES:
        ech = [(vc, vr) for d, vc, vr, wc, wr in pd if lo <= d < hi
               and abs(vc) > CMD_NULLE]
        if not ech:
            print("  %-26s     -   aucune commande d'avance" % nom)
            continue
        cmd = med([abs(vc) for vc, vr in ech])
        mes = med([abs(vr) for vc, vr in ech])
        bouge = [(vc, vr) for vc, vr in ech if abs(vr) >= IMMOBILE_LIN]
        inertes = len(ech) - len(bouge)
        suivi = med([abs(vr) / abs(vc) for vc, vr in bouge]) if bouge else 0.0
        print("  %-26s %5d %8.3f m/s %8.3f m/s %6.0f %% %4d (%.0f %%)"
              % (nom, len(ech), cmd, mes, 100 * suivi, inertes,
                 100 * inertes / len(ech)))
    print()
    print("  LECTURE. Une consigne mediane qui CHUTE quand on approche est")
    print("  normale : nav2 freine expres (RotateToGoal, slowing_factor 5.0),")
    print("  et xy_goal_tolerance vaut 0,12 m -- le robot n'a PAS besoin")
    print("  d'atteindre le point exact.")
    print("  Ce qui serait un vrai defaut : un taux de suivi qui s'effondre ou")
    print("  des inertes qui explosent dans la derniere tranche. Cela voudrait")
    print("  dire que les moteurs ne savent pas executer les petites consignes,")
    print("  et le correctif serait alors dans le FIRMWARE (minimum de PWM ou")
    print("  terme d'anticipation, FF_GAIN), pas dans nav2.")


def rapport(paires):
    if not paires:
        print("\naucune paire commande/mesure : le robot n'a pas navigue "
              "pendant l'observation.")
        print("Relancer PENDANT un deplacement (mowbot node cmd_check.py 120).")
        return
    print("\n%d paires commande/mesure" % len(paires))

    # --- avance ---------------------------------------------------------------
    av = [(vc, vr) for vc, wc, vr, wr in paires if abs(vc) > CMD_NULLE]
    if av:
        rates = [(vc, vr) for vc, vr in av if abs(vr) < IMMOBILE_LIN]
        faits = [(vc, vr) for vc, vr in av if abs(vr) >= IMMOBILE_LIN]
        print("\n--- AVANCE : %d commandes ---" % len(av))
        print("  sans mouvement mesure : %d (%.0f %%)"
              % (len(rates), 100 * len(rates) / len(av)))
        if faits:
            print("  plus petite consigne suivie d'un mouvement : %.3f m/s"
                  % min(abs(vc) for vc, vr in faits))
            suivi = [abs(vr) / abs(vc) for vc, vr in faits]
            print("  taux de suivi median : %.0f %%" % (100 * med(suivi)))
        print("  CONSIGNES de nav2 : mediane %.3f m/s, max %.3f m/s"
              % (med([abs(vc) for vc, vr in av]),
                 max(abs(vc) for vc, vr in av)))
    else:
        print("\n--- AVANCE : aucune commande pendant l'observation ---")

    # --- rotation -------------------------------------------------------------
    # SEUIL PROPRE AUX RAD/S. Une premiere version utilisait le meme 0.01 pour
    # les m/s et les rad/s : 0.01 rad/s vaut 0,57 deg/s, soit le bruit de
    # l'odometrie. Toute rotation lente mais REELLE etait comptee comme non
    # executee, ce qui a produit un faux "37 % de rotations ignorees".
    ro = [(wc, wr) for vc, wc, vr, wr in paires if abs(wc) > CMD_NULLE]
    if ro:
        rates_r = [(wc, wr) for wc, wr in ro if abs(wr) < IMMOBILE_ROT]
        faits_r = [(wc, wr) for wc, wr in ro if abs(wr) >= IMMOBILE_ROT]
        print("\n--- ROTATION : %d commandes ---" % len(ro))
        print("  sans rotation mesuree : %d (%.0f %%)"
              % (len(rates_r), 100 * len(rates_r) / len(ro)))
        if faits_r:
            print("  plus petite consigne suivie d'une rotation : %.3f rad/s"
                  % min(abs(wc) for wc, wr in faits_r))
            suivi = [abs(wr) / abs(wc) for wc, wr in faits_r]
            print("  taux de suivi median : %.0f %%" % (100 * med(suivi)))
    else:
        print("\n--- ROTATION : aucune commande pendant l'observation ---")

    # --- repartition de l'effort ---------------------------------------------
    dt = 1.0 / 10.0
    dist = sum(abs(vr) for vc, wc, vr, wr in paires) * dt
    rot_place = sum(abs(wr) for vc, wc, vr, wr in paires
                    if abs(vr) < IMMOBILE_LIN) * dt
    rot_route = sum(abs(wr) for vc, wc, vr, wr in paires
                    if abs(vr) >= IMMOBILE_LIN) * dt
    print("\n--- REPARTITION DE L'EFFORT ---")
    print("  distance parcourue (approx) : %.2f m" % dist)
    print("  rotation SUR PLACE          : %.0f deg (%.1f tours)"
          % (math.degrees(rot_place), math.degrees(rot_place) / 360.0))
    print("  rotation EN AVANCANT        : %.0f deg" % math.degrees(rot_route))
    if dist > 0.05:
        par_m = math.degrees(rot_route) / dist
        print("  rotation en avancant, par metre : %.0f deg/m" % par_m)
        if par_m > 200:
            print("    >> eleve : le robot corrige son cap sans arret.")
        else:
            print("    (normal pour un trajet avec virages)")
    print("  La rotation SUR PLACE n'est pas un defaut : chaque nouveau but")
    print("  demande 120 a 360 deg de reorientation. Compter les buts avant")
    print("  de conclure :  mowbot logs nav | grep -c 'Begin navigating'")

    # --- conclusion -----------------------------------------------------------
    # DENOMINATEUR = LES PAIRES, pas la somme des deux colonnes. Une premiere
    # version divisait par len(av)+len(ro), ce qui compte DEUX FOIS les paires
    # ou avance et rotation sont commandees ensemble : elle annoncait
    # "295 sur 1200" alors qu'il n'y avait que 861 paires, et sous-estimait donc
    # le pourcentage tout en le presentant comme exact.
    commandees = [(vc, wc, vr, wr) for vc, wc, vr, wr in paires
                  if abs(vc) > CMD_NULLE or abs(wc) > CMD_NULLE]
    inertes = [(vc, wc) for vc, wc, vr, wr in commandees
               if abs(vr) < IMMOBILE_LIN and abs(wr) < IMMOBILE_ROT]
    actives = [(vc, wc) for vc, wc, vr, wr in commandees
               if abs(vr) >= IMMOBILE_LIN or abs(wr) >= IMMOBILE_ROT]
    print("\n=== VERDICT ===")
    if len(commandees) < MINI_VERDICT:
        print("  PAS ASSEZ DE DONNEES : %d commande(s) de mouvement, il en"
              % len(commandees))
        print("  faut au moins %d. Relancer pendant que le robot navigue :"
              % MINI_VERDICT)
        print("    mowbot node cmd_check.py 60")
        return
    part = len(inertes) / len(commandees)
    print("  %d commandes de mouvement, dont %d sans effet mesure (%.0f %%)"
          % (len(commandees), len(inertes), 100 * part))
    if part <= 0.15:
        print("  >> LE ROBOT SUIT LES COMMANDES.")
        print("  S'il reste bloque, chercher du cote de nav2 :")
        print("    mowbot logs nav | grep -E 'No valid trajectories|Oscillation'")
        return

    # LE DISCRIMINANT. La friction statique fait echouer les consignes les plus
    # FAIBLES : sous le seuil, rien ne bouge ; au-dessus, tout bouge. Le delai
    # d'acceleration, lui, frappe indifferemment -- une consigne forte qui vient
    # d'etre emise n'a pas encore produit de mouvement. On compare donc
    # l'amplitude des commandes ratees a celle des commandes suivies.
    amp = lambda c: max(abs(c[0]) / 0.30, abs(c[1]) / 1.0)   # normalise
    m_in, m_ac = med([amp(c) for c in inertes]), med([amp(c) for c in actives])
    print("  amplitude mediane des commandes sans effet : %.2f" % m_in)
    print("  amplitude mediane des commandes suivies    : %.2f" % m_ac)
    if m_ac > 0 and m_in < 0.6 * m_ac:
        print("  >> Les commandes sans effet sont nettement PLUS FAIBLES :")
        print("     c'est bien la signature de la FRICTION STATIQUE.")
        print("     Le correctif est dans le FIRMWARE, pas dans nav2 : minimum")
        print("     de PWM ou terme d'anticipation cote moteur (FF_GAIN, a 0).")
        print("     NE PAS passer par min_speed_xy : DWB n'echantillonne que la")
        print("     fenetre atteignable (0,08 m/s depuis l'arret), un seuil")
        print("     au-dessus empeche le robot de demarrer. Voir speeds.env.")
    else:
        print("  >> Les commandes sans effet ne sont PAS plus faibles que les")
        print("     autres : ce n'est donc pas de la friction, mais le DELAI")
        print("     D'ACCELERATION -- l'odometrie est lue avant que le robot")
        print("     ait eu le temps de repondre a une commande fraiche.")
        print("     Rien a corriger cote moteur.")


def main():
    duree = 30.0
    if len(sys.argv) > 1:
        try:
            duree = float(sys.argv[1])
        except ValueError:
            print("duree invalide : %r" % sys.argv[1], file=sys.stderr)
            return 1
    print("observation de %.0f s. FAIRE NAVIGUER LE ROBOT pendant ce temps." % duree)
    rclpy.init()
    n = Suivi(duree)
    fin = time.time() + duree
    try:
        while rclpy.ok() and time.time() < fin:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    rapport(n.paires)
    rapport_approche(n.par_distance)
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
