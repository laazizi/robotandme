#!/usr/bin/env python3
"""Parcours de points de passage EN BOUCLE, a lancer depuis le PC.

    source /opt/ros/jazzy/setup.bash
    python3 waypoints.py

Les points se modifient DANS CE FICHIER, dans le tableau POINTS ci-dessous.
A chaque arrivee le robot fait un tour sur lui-meme pour marquer le point.
Ctrl+C annule le but en cours : le robot s'arrete avec le script.

Prerequis :
  . nav2 actif sur le robot          ->  mowbot status
  . nav2_msgs sur ce PC             ->  sudo apt install ros-jazzy-nav2-msgs
"""

import math as _m


class Position:
    """Position collee depuis le journal de RViz. z est ignore (robot au sol)."""

    def __init__(self, x, y, z=0.0):
        self.x, self.y = float(x), float(y)


class Orientation:
    """Quaternion colle depuis le journal de RViz, converti en cap.

    RViz journalise un but sous cette forme exacte :
        Setting goal pose: Frame:map, Position(0.55, -0.15, 0),
        Orientation(0, 0, -0.404904, 0.914359) = Angle: -0.833747
    Ces deux classes existent pour que ce texte soit du PYTHON VALIDE : on colle
    la ligne telle quelle dans POINTS, sans rien convertir a la main.
    Le cap est extrait du quaternion par atan2(2(wz+xy), 1-2(y2+z2)), qui est
    l'angle autour de l'axe vertical -- le seul qui compte pour un robot au sol.
    """

    def __init__(self, x, y, z, w):
        self.cap = _m.degrees(_m.atan2(2.0 * (w * z + x * y),
                                       1.0 - 2.0 * (y * y + z * z)))


# ============================================================================
#  LES POINTS  --  c'est ici qu'on edite
# ============================================================================
#
#  DEUX ECRITURES ACCEPTEES, melangeables dans la meme liste :
#
#    (x, y, cap)                              cap en DEGRES, simple a lire
#    [Position(x, y, z), Orientation(x, y, z, w)]   colle du journal RViz
#
#  La seconde permet de recuperer un but directement depuis RViz : cliquer
#  '2D Goal Pose', puis copier la ligne "Setting goal pose:" du terminal.
#
#   (x, y, cap)
#     x, y : metres dans le repere `map`. L'ORIGINE EST L'ENDROIT OU LE SLAM A
#            DEMARRE, pas un coin de la piece. x vers l'avant du robot au
#            demarrage, y vers sa gauche.
#     cap  : orientation VOULUE en arrivant sur le point, en degres.
#            0 = vers +x,  90 = vers +y,  180 = vers -x,  -90 = vers -y.
#
# Le parcours BOUCLE : apres le dernier point on repart au premier. Verifier
# donc que le retour du dernier au premier est franchissable.

POINTS = [
    [Position(-2.09363, -0.253322, 0), Orientation(0, 0, 0.0966599, 0.995317)],
    [Position(-1.263, 0.565716, 0), Orientation(0, 0, 0.797103, 0.603844)],
    [Position(-0.235288, 1.98, 0), Orientation(0, 0, -0.378732, 0.925506)],
    [Position(-0.509342, 0.451657, 0), Orientation(0, 0, -0.976593, 0.215097)],
    [Position(-2.22064, 1.08515, 0), Orientation(0, 0, -0.874643, 0.484768)],
    [Position(-2.63389, 0.332151, 0), Orientation(0, 0, -0.479188, 0.877712)]
]

TOURS = 0                 # nombre de tours ; 0 = boucle infinie
ROTATION_ARRIVEE = 360    # degres de rotation a chaque arrivee ; 0 = aucune
DELAI_PAR_POINT = 120     # secondes avant d'abandonner un point ; 0 = illimite
REPERE = 'map'            # repere des coordonnees ci-dessus

# POURQUOI 120 SECONDES, et pas moins. Un delai trop court serait PIRE que pas
# de delai : il couperait un degagement legitime. nav2 s'autorise 6 cycles de
# degagement, chacun pouvant enchainer un recul (~4 s), une rotation (~3 s) et
# une attente (5 s) : une soixantaine de secondes rien que pour se sortir d'un
# mauvais pas. On ajoute le trajet, et on arrondit au double.
# CE DELAI EXISTE PARCE QUE NAV2 N'EN A AUCUN. Le point INATTEIGNABLE n'est pas
# le probleme : il echoue, les degagements s'epuisent, on passe au suivant. Le
# probleme est le point PRESQUE atteignable -- le robot tourne autour, avance de
# trois centimetres, sans jamais entrer dans la tolerance d'arrivee. Il n'y a
# alors aucun echec, donc aucun degagement, et NavigateToPose ne rend jamais la
# main : le tour reste suspendu indefiniment sur ce point.

# ============================================================================
#  A partir d'ici, plus rien a regler
# ============================================================================
import math
import os
import sys
import time

# AVANT d'importer rclpy : le domaine doit correspondre a celui du robot, sinon
# le PC et le robot ne se voient pas DU TOUT et le script attendrait le serveur
# indefiniment, sans le moindre message d'erreur reseau.
os.environ.setdefault('ROS_DOMAIN_ID', '0')

try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
except ImportError:
    sys.exit("ROS 2 n'est pas dans l'environnement.\n"
             "  source /opt/ros/jazzy/setup.bash")
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

try:
    from nav2_msgs.action import NavigateToPose, Spin
except ImportError:
    # nav2_msgs porte la DEFINITION des actions. Ce n'est pas la pile nav2,
    # juste des messages : deux petits paquets.
    sys.exit("nav2_msgs est absent de ce PC.\n"
             "  sudo apt install ros-jazzy-nav2-msgs")

# Codes d'erreur du comportement Spin, pour dire POURQUOI ca a echoue plutot que
# d'afficher un numero nu. Source : nav2_msgs/action/Spin.
CODES_SPIN = {
    0: 'aucune erreur', 700: 'inconnue', 701: 'delai depasse',
    702: 'erreur de TF', 703: 'collision devant',
}

# Couleurs des marqueurs RViz, par etat du point. Le vert et le rouge portent
# l'information utile : ce qui a ete atteint et ce qui a ete manque, visible d'un
# coup d'oeil sans lire la sortie du terminal.
COULEURS = {
    'attente':  (0.55, 0.60, 0.66, 0.85),
    'en cours': (1.00, 0.78, 0.10, 1.00),
    'atteint':  (0.24, 0.70, 0.36, 1.00),
    'manque':   (0.85, 0.20, 0.42, 1.00),
}
TOPIC_MARQUEURS = '/mowbot/waypoints'


# Un pas de rotation ne depasse jamais 180 deg. Le comportement Spin de nav2
# cumule bien le lacet (son en-tete porte un champ relative_yaw_), mais seuls
# les EN-TETES sont installes : impossible de verifier l'implementation. Or si
# une version normalisait l'angle, demander 360 deg ferait tourner de ZERO.
# Decouper est correct dans les deux cas, pour 0,2 s de plus par pas.
PAS_MAX_DEG = 180.0


def normaliser(points):
    """Ramene chaque point a (x, y, cap_en_degres), quelle que soit l'ecriture.

    On normalise UNE FOIS au demarrage plutot que de disperser des tests dans
    tout le code : le reste du script ne connait que des triplets.
    """
    out = []
    for i, pt in enumerate(points, 1):
        if isinstance(pt, Position):
            out.append((pt.x, pt.y, 0.0)); continue
        if isinstance(pt, (list, tuple)) and pt and isinstance(pt[0], Position):
            pos = pt[0]
            cap = 0.0
            for e in pt[1:]:
                if isinstance(e, Orientation):
                    cap = e.cap
            out.append((pos.x, pos.y, cap)); continue
        if isinstance(pt, (list, tuple)) and len(pt) in (2, 3) \
                and all(isinstance(v, (int, float)) for v in pt):
            x, y = float(pt[0]), float(pt[1])
            out.append((x, y, float(pt[2]) if len(pt) == 3 else 0.0)); continue
        sys.exit("POINTS[%d] = %r\n"
                 "Ecritures acceptees :\n"
                 "  (x, y, cap)                                   cap en degres\n"
                 "  [Position(x, y, z), Orientation(x, y, z, w)]  colle de RViz"
                 % (i, pt))
    return out


def verifier_reglages():
    """Controle les tableaux en tete de fichier, avant de toucher a ROS.

    POURQUOI. Les points s'editent a la main juste au-dessus, et une
    coordonnee collee au mauvais endroit produisait une trace python
    incomprehensible au fond du code d'affichage :
        r, v, b, a = COULEURS[self.etats[i]]
        ValueError: not enough values to unpack (expected 4, got 3)
    alors que la cause etait un point de passage atterri dans COULEURS.
    Un message clair ici coute trois lignes et fait gagner un quart d'heure.
    """
    for nom, val in COULEURS.items():
        if len(val) != 4:
            sys.exit("COULEURS['%s'] a %d valeurs au lieu de 4 (r, v, b, alpha) :\n"
                     "  %r\n"
                     "Ces trois nombres ressemblent a un POINT DE PASSAGE.\n"
                     "Un point s'ajoute dans le tableau POINTS, pas dans COULEURS."
                     % (nom, len(val), val))
        if not all(0.0 <= c <= 1.0 for c in val):
            sys.exit("COULEURS['%s'] = %r : les composantes doivent etre entre "
                     "0.0 et 1.0." % (nom, val))
    if not POINTS:
        sys.exit("POINTS est vide : ajouter au moins un point (x, y, cap).")
    # POINTS est deja normalise en triplets a ce stade.
    for i, pt in enumerate(POINTS, 1):
        if len(pt) != 3:
            sys.exit("POINTS[%d] = %r : la normalisation a echoue." % (i, pt))


def pas_de_rotation(total_deg):
    """Decoupe une rotation en pas de PAS_MAX_DEG au plus, signe conserve."""
    reste = abs(float(total_deg))
    signe = 1.0 if float(total_deg) >= 0 else -1.0
    out = []
    while reste > 1e-6:
        p = min(reste, PAS_MAX_DEG)
        out.append(signe * p)
        reste -= p
    return out


class Parcours(Node):
    """Enchaine NavigateToPose puis Spin sur chaque point, en boucle.

    POURQUOI PLUS FollowWaypoints. Le serveur waypoint_follower enchaine les
    points lui-meme et ne rend pas la main entre deux : impossible d'y glisser
    une rotation d'arrivee, et impossible d'imposer un delai PAR POINT. On
    pilote donc NavigateToPose et Spin directement. Le noeud waypoint_follower
    reste en place dans nav2, il n'est simplement plus utilise par ce script.
    Ce qu'on perd : son champ missed_waypoints. Ce qu'on gagne : l'issue
    detaillee de CHAQUE point et le temps mis, plus informatif pour un outil de
    test -- et la rotation d'arrivee, qui etait la demande.
    """

    def __init__(self):
        super().__init__('mowbot_waypoints')
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin = ActionClient(self, Spin, 'spin')
        self.encours = None      # but actif, pour l'annulation au Ctrl+C
        self.atteints = 0
        self.manques = 0
        # UN ETAT PAR POINT, pour colorer les marqueurs.
        self.etats = ['attente'] * len(POINTS)
        # TRANSIENT_LOCAL : RViz recoit les marqueurs meme s'il se connecte
        # APRES le script. Sans cela il faut relancer le script chaque fois
        # qu'on ouvre RViz, ou attendre le tour suivant.
        self.pub = self.create_publisher(
            MarkerArray, TOPIC_MARQUEURS,
            QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # REPUBLICATION PERIODIQUE, en plus des changements d'etat.
        # TRANSIENT_LOCAL ne suffit pas : l'affichage MarkerArray de RViz
        # s'abonne en VOLATILE par defaut, il ne recoit donc PAS le dernier
        # message retenu. Comme le script ne publie qu'aux changements d'etat --
        # toutes les 15 a 30 s avec une rotation d'arrivee --, RViz pouvait
        # rester vide longtemps et donner l'impression que rien ne marche.
        # Dix marqueurs a 1 Hz ne coutent rien et rendent l'affichage immediat.
        self.create_timer(1.0, self.marqueurs)

    # --- mecanique d'attente ------------------------------------------------
    def _pomper(self, fut, delai):
        """Fait tourner l'executeur jusqu'a ce que `fut` aboutisse.

        Renvoie False si le delai est depasse. On pompe a la main plutot que
        d'appeler spin_until_future_complete : cela garde la main sur le delai
        et permet d'annuler proprement.
        """
        fin = time.time() + delai if delai else None
        while rclpy.ok() and not fut.done():
            if fin and time.time() > fin:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return fut.done()

    def _pomper_court(self, secondes):
        fin = time.time() + secondes
        while rclpy.ok() and time.time() < fin:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _executer(self, client, but, delai, nom):
        """Envoie un but et attend son issue.

        Renvoie (etat, resultat) ou etat vaut :
        'ok' | 'echec' | 'refuse' | 'delai' | 'absent' | 'interrompu'
        """
        if not client.wait_for_server(timeout_sec=10.0):
            return 'absent', None
        f = client.send_goal_async(but)
        if not self._pomper(f, 15):
            return 'absent', None
        h = f.result()
        if not h.accepted:
            return 'refuse', None
        self.encours = h
        fr = h.get_result_async()
        if not self._pomper(fr, delai):
            # DELAI DEPASSE : on ANNULE. Sans cela le robot poursuit son but
            # alors que le script est passe au point suivant -- deux intentions
            # concurrentes sur /cmd_vel.
            print("     delai de %d s depasse sur %s, annulation"
                  % (delai, nom))
            h.cancel_goal_async()
            self._pomper_court(3)
            self.encours = None
            return 'delai', None
        self.encours = None
        if not rclpy.ok():
            return 'interrompu', None
        r = fr.result()
        return ('ok' if r.status == GoalStatus.STATUS_SUCCEEDED
                else 'echec'), r.result

    # --- actions -------------------------------------------------------------
    def marqueurs(self):
        """Publie les points et le circuit pour RViz.

        Trois marqueurs par point -- une sphere coloree selon l'etat, son
        numero en texte, et une fleche donnant le cap voulu -- plus une ligne
        fermee qui montre le circuit, retour du dernier au premier inclus.
        C'est ce retour qu'on oublie de verifier et qui bloque un parcours.
        """
        arr = MarkerArray()
        for i, (x, y, cap) in enumerate(POINTS):
            r, v, b, a = COULEURS[self.etats[i]]

            m = Marker()
            m.header.frame_id = REPERE
            m.ns = 'points'; m.id = i
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = float(x); m.pose.position.y = float(y)
            m.pose.position.z = 0.06
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.22
            m.color.r, m.color.g, m.color.b, m.color.a = r, v, b, a
            arr.markers.append(m)

            t = Marker()
            t.header.frame_id = REPERE
            t.ns = 'numeros'; t.id = i
            t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
            t.pose.position.x = float(x); t.pose.position.y = float(y)
            t.pose.position.z = 0.34
            t.pose.orientation.w = 1.0
            t.scale.z = 0.24
            t.color.r, t.color.g, t.color.b, t.color.a = r, v, b, 1.0
            t.text = str(i + 1)
            arr.markers.append(t)

            f = Marker()
            f.header.frame_id = REPERE
            f.ns = 'caps'; f.id = i
            f.type = Marker.ARROW; f.action = Marker.ADD
            f.pose.position.x = float(x); f.pose.position.y = float(y)
            f.pose.position.z = 0.06
            f.pose.orientation.z = math.sin(math.radians(cap) / 2.0)
            f.pose.orientation.w = math.cos(math.radians(cap) / 2.0)
            f.scale.x = 0.34; f.scale.y = 0.05; f.scale.z = 0.05
            f.color.r, f.color.g, f.color.b, f.color.a = r, v, b, 0.9
            arr.markers.append(f)

        if len(POINTS) > 1:
            l = Marker()
            l.header.frame_id = REPERE
            l.ns = 'circuit'; l.id = 0
            l.type = Marker.LINE_STRIP; l.action = Marker.ADD
            l.pose.orientation.w = 1.0
            l.scale.x = 0.03
            l.color.r, l.color.g, l.color.b, l.color.a = 0.35, 0.55, 0.62, 0.65
            from geometry_msgs.msg import Point
            for x, y, _ in POINTS + [POINTS[0]]:
                p = Point(); p.x = float(x); p.y = float(y); p.z = 0.04
                l.points.append(p)
            arr.markers.append(l)

        self.pub.publish(arr)

    def aller_a(self, x, y, cap):
        p = PoseStamped()
        p.header.frame_id = REPERE
        # Horodatage a ZERO et non `now()` : le serveur comprend alors "la
        # derniere pose connue". Avec l'heure du PC, un decalage d'horloge entre
        # les deux machines fait rejeter le but pour extrapolation.
        p.header.stamp.sec = 0
        p.header.stamp.nanosec = 0
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(math.radians(cap) / 2.0)
        p.pose.orientation.w = math.cos(math.radians(cap) / 2.0)
        but = NavigateToPose.Goal()
        but.pose = p
        return self._executer(self.nav, but, DELAI_PAR_POINT, "le trajet")

    def tourner(self, total_deg):
        """Rotation d'arrivee, en pas de 180 deg au plus.

        Une rotation peut echouer en 'collision devant' : le contour de ce robot
        n'est PAS circulaire (0.54 x 0.50 m), donc tourner sur place a moins de
        quelques centimetres d'un mur balaie ses coins dedans. Ce n'est pas une
        anomalie, et surtout ce n'est pas une raison d'arreter le parcours.
        """
        for i, pas in enumerate(pas_de_rotation(total_deg), 1):
            but = Spin.Goal()
            but.target_yaw = float(math.radians(pas))
            # time_allowance genereux : c'est NOTRE delai qui tranche, sinon on
            # aurait deux limites concurrentes sur la meme manoeuvre.
            but.time_allowance.sec = 30
            etat, res = self._executer(self.spin, but, 45, "la rotation")
            if etat != 'ok':
                code = getattr(res, 'error_code', None) if res else None
                detail = CODES_SPIN.get(code, 'code %s' % code) if code else ''
                print("     rotation interrompue au pas %d/%d (%s%s)"
                      % (i, len(pas_de_rotation(total_deg)), etat,
                         ' : ' + detail if detail else ''))
                return etat
        return 'ok'

    def annuler(self):
        """Arrete le robot avec le script.

        Sans cela le but reste actif cote robot apres un Ctrl+C : la tondeuse
        continue vers le point suivant alors que le script est mort, et il
        faudrait arreter nav2 pour l'immobiliser.
        """
        if self.encours is not None:
            print("annulation du but en cours...")
            try:
                self.encours.cancel_goal_async()
                self._pomper_court(3)
            except Exception:
                pass
            self.encours = None


def main():
    global POINTS
    POINTS = normaliser(POINTS)
    verifier_reglages()
    pas = pas_de_rotation(ROTATION_ARRIVEE)
    print("%d point(s) dans le repere %s :" % (len(POINTS), REPERE))
    for i, (x, y, cap) in enumerate(POINTS, 1):
        print("  %2d   x=%7.3f   y=%7.3f   cap=%4.0f deg" % (i, x, y, cap))
    print("tours            : %s"
          % (TOURS if TOURS else "infini (Ctrl+C pour arreter)"))
    print("rotation arrivee : %s"
          % ("%d deg en %d pas de %.0f deg" % (ROTATION_ARRIVEE, len(pas),
                                               abs(pas[0])) if pas else "aucune"))
    print("delai par point  : %s"
          % ("%d s" % DELAI_PAR_POINT if DELAI_PAR_POINT else "illimite"))
    print("ROS_DOMAIN_ID    = %s" % os.environ['ROS_DOMAIN_ID'])
    print()
    print("Dans RViz : Add > By topic > %s > MarkerArray" % TOPIC_MARQUEURS)
    print("  gris = en attente   jaune = en cours   vert = atteint   rouge = manque")

    rclpy.init()
    n = Parcours()
    tour = 0
    try:
        print("\nattente des serveurs navigate_to_pose et spin (nav2)...")
        if not n.nav.wait_for_server(timeout_sec=30.0):
            print("\nnavigate_to_pose introuvable apres 30 s.\n"
                  "  . nav2 tourne-t-il ?        mowbot status\n"
                  "  . meme reseau et meme ROS_DOMAIN_ID des deux cotes ?\n"
                  "  . essai rapide :            ros2 topic list | grep odom",
                  file=sys.stderr)
            return 1
        if pas and not n.spin.wait_for_server(timeout_sec=10.0):
            print("serveur spin absent : le parcours se fera SANS rotation "
                  "d'arrivee.", file=sys.stderr)
            pas = []
        # PUBLICATION IMMEDIATE, avant le premier deplacement : on veut voir la
        # liste et le circuit dans RViz sans attendre que le robot bouge.
        n.marqueurs()
        while rclpy.ok():
            tour += 1
            if TOURS and tour > TOURS:
                break
            print("\n=== %s ===" % ("tour %d/%d" % (tour, TOURS) if TOURS
                  else "tour %d  (boucle infinie, Ctrl+C pour arreter)" % tour))
            # Nouveau tour : tous les points repassent en attente, sinon on
            # garderait les couleurs du tour precedent et on ne verrait plus
            # ce qui vient d'etre atteint ou manque.
            n.etats = ['attente'] * len(POINTS)
            n.marqueurs()
            for i, (x, y, cap) in enumerate(POINTS, 1):
                if not rclpy.ok():
                    break
                n.etats[i - 1] = 'en cours'
                n.marqueurs()
                print("  point %d/%d  ->  x=%.3f  y=%.3f  cap=%.0f deg"
                      % (i, len(POINTS), x, y, cap))
                t0 = time.time()
                etat, _ = n.aller_a(x, y, cap)
                if etat == 'ok':
                    print("     atteint en %.1f s" % (time.time() - t0))
                    n.atteints += 1
                    n.etats[i - 1] = 'atteint'
                    n.marqueurs()
                    
                else:
                    # ON NE S'ARRETE PAS sur un point manque : c'est justement
                    # ce qu'on veut observer tour apres tour.
                    print("     NON ATTEINT (%s) apres %.1f s"
                          % (etat, time.time() - t0))
                    n.manques += 1
                    n.etats[i - 1] = 'manque'
                    n.marqueurs()
            print("  cumul : %d atteints, %d manques" % (n.atteints, n.manques))
        print("\nTERMINE : %d tour(s), %d atteints, %d manques"
              % (tour - 1, n.atteints, n.manques))
    except KeyboardInterrupt:
        print()
        n.annuler()
        print("interrompu : %d atteints, %d manques" % (n.atteints, n.manques))
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
