#!/usr/bin/env python3
"""Normalise et NETTOIE /scan_raw -> /scan.

1) Nombre de points FIXE : le LSLidar N10 sort 449/450/451 rayons selon les
   tours ; slam_toolbox enregistre la taille du 1er scan et rejette les autres
   ("contains 451 readings, expected 449"). On reechantillonne a N points.

2) Masquage des parties du ROBOT : certains montants/antennes sont dans le
   champ du lidar (detectes par detect_self.py : echos a distance constante).
   Sans filtrage, la costmap voit le robot encercle d'obstacles -> nav2 refuse
   de planifier. Les rayons concernes sont mis a l'infini (= rien vu).

3) Suppression des points ISOLES (faux echos). Mesure sur ce robot :
   14.7 % des points n'avaient aucun voisin a moins de 15 cm, et 31 % des
   directions variaient de plus de 10 cm d'un tour a l'autre. Deux causes :
     - effet de bord : quand le faisceau frole l'arete d'un objet, une partie
       du spot touche l'objet et l'autre le fond ; la distance retournee tombe
       ENTRE les deux -> un point fantome apparait derriere l'obstacle reel ;
     - reflexions speculaires sur le parquet (lidar a triangulation).
   Une vraie surface est continue : ses points ont des voisins a distance
   comparable. Un faux echo est seul. On exige donc au moins un voisin
   coherent dans une fenetre de +/-2 rayons -- pas deux, sinon un pied de
   chaise fin (1 a 2 points a 2 m) serait efface avec le bruit.
"""
import math
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

N = 450

# Secteurs occultes : (angle_debut_deg, angle_fin_deg, distance_max_m).
# Seuls les echos PLUS PROCHES que la distance indiquee sont supprimes :
# au-dela, dans le meme secteur, ce sont de vrais obstacles -> conserves.
# LES SECTEURS DEPENDENT DU ROBOT. Appliquer ceux du tricycle au gros robot
# masquerait des directions parfaitement libres tout en laissant ses roues
# polluer la carte. Le choix se fait sur MOWBOT_ROBOT, comme partout ailleurs.
SECTEURS_PAR_ROBOT = {
    # --- tricycle ackerbot : montants et antenne, releves par detect_self.py ---
    'ackerbot': [
        (233.0, 250.0, 0.45),   # montant arriere-droit (~33 cm)
        (261.0, 280.0, 0.60),   # antenne / support (~48 cm)
        (282.0, 311.0, 0.40),   # montant arriere-gauche (~28 cm)
        (52.0,  64.0,  0.55),   # structure avant-gauche (~49 cm)
        (250.0, 262.0, 0.65),   # structure arriere (~60 cm)
    ],
    # --- GROS ROBOT : ce sont ses PROPRES ROUES MOTRICES ---
    # Le lidar est a 0,28 m et l'axe des roues a 0,20 m : le faisceau coupe donc
    # chaque roue 8 cm au-dessus de son centre. A cette hauteur la roue (rayon
    # 0,20 m) presente une demi-corde de sqrt(0,20^2 - 0,08^2) = 0,183 m, et
    # elle s'etend lateralement de 0,37 a 0,45 m. Vu du lidar, place au centre
    # exact de l'essieu, cela couvre 63,6 a 116,4 degres -- 61 a 119 avec 3
    # degres de marge, et le symetrique a droite.
    # Les echos attendus sont entre 0,37 et 0,49 m : le plafond a 0,55 m les
    # supprime tous SANS aveugler le robot au-dela. Les obstacles lateraux
    # a plus de 55 cm restent vus, ce qui est l'essentiel pour naviguer.
    # LA VRAIE SOLUTION serait de REMONTER LE LIDAR au-dessus de 0,40 m, la
    # hauteur du sommet des roues : plus aucun masquage ne serait necessaire.
    'gros': [
        ( 61.0, 119.0, 0.55),   # roue motrice GAUCHE
        (241.0, 299.0, 0.55),   # roue motrice DROITE
    ],
}

_robot = os.environ.get('MOWBOT_ROBOT', '').strip().lower()
SELF_SECTORS = SECTEURS_PAR_ROBOT.get(_robot, SECTEURS_PAR_ROBOT['ackerbot'])

# Filtre des points isoles (cf. point 3 du docstring).
ISO_ENABLE = True
ISO_WINDOW = 2      # rayons voisins examines de chaque cote
ISO_TOL_M  = 0.15   # ecart de distance en deca duquel un voisin est "coherent"
ISO_MIN_NB = 1      # nombre de voisins coherents exiges pour garder le point

# Au-dela de cet ecart entre l'horodatage du driver et l'heure locale, on
# considere le premier comme faux et on le remplace (cf. methode cb).
# 1 s est large devant la latence normale (~2 ms) et bien en deca des 89 s
# observes au demarrage du driver natif.
STAMP_MAX_AGE_S = 1.0


class ScanFix(Node):
    def __init__(self):
        super().__init__('scan_fix')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub = self.create_publisher(LaserScan, '/scan', qos)
        self.create_subscription(LaserScan, '/scan_raw', self.cb, qos)
        self.mask = None
        self.n_dropped = 0
        self.n_seen = 0
        self.stamp_warned = False
        self.shift_logged = False
        self.get_logger().info(
            f'/scan_raw -> /scan : {N} points, {len(SELF_SECTORS)} secteurs robot masques'
            f'{", filtre points isoles actif" if ISO_ENABLE else ""}')
        if ISO_ENABLE:
            # un point de mesure toutes les 30 s : verifie que le filtre retire
            # bien du bruit, et pas la moitie du scan (reglage trop agressif).
            self.create_timer(30.0, self.report)

    def report(self):
        if self.n_seen:
            self.get_logger().info(
                f'points isolés filtrés : {100.0 * self.n_dropped / self.n_seen:.1f} %')
        self.n_dropped = self.n_seen = 0

    def build_mask(self, amin, amax):
        ang = np.degrees(np.linspace(amin, amax, N)) % 360.0
        lim = np.full(N, -1.0)
        for a0, a1, dmax in SELF_SECTORS:
            sel = (ang >= a0) & (ang <= a1) if a0 <= a1 else (ang >= a0) | (ang <= a1)
            lim[sel] = np.maximum(lim[sel], dmax)
        return lim

    def drop_isolated(self, r, range_max):
        """Efface les echos sans voisin coherent (faux echos de bord/reflet).

        Le scan est circulaire : np.roll relie naturellement le dernier rayon
        au premier, sans cas particulier aux extremites.
        """
        valid = np.isfinite(r) & (r > 0.0) & (r <= range_max)
        neighbours = np.zeros(len(r), dtype=int)
        for k in range(1, ISO_WINDOW + 1):
            for shifted in (np.roll(r, k), np.roll(r, -k)):
                sv = np.isfinite(shifted) & (shifted > 0.0) & (shifted <= range_max)
                neighbours += (sv & (np.abs(shifted - r) < ISO_TOL_M)).astype(int)
        lonely = valid & (neighbours < ISO_MIN_NB)
        r = r.copy()
        r[lonely] = float('inf')
        self.n_dropped += int(lonely.sum())
        self.n_seen += int(valid.sum())
        return r

    def angle_shift(self, m):
        """Ramene le scan a la convention CANONIQUE -pi..+pi.

        Les deux pilotes du LD14 lisent le meme flux materiel mais n'etiquettent
        pas ses angles pareil :
          - nodes/ld14_node.py (Python)   annonce -pi..+pi
          - ldlidar_sl_ros2   (natif C++) annonce   0..2*pi
        Comme ils partent du MEME zero materiel, leurs etiquettes different de
        exactement 180 deg. Basculer d'un pilote a l'autre faisait donc tourner
        tout le scan d'un demi-tour, sans rien changer d'autre.

        Mesure a l'appui (nodes/loc_check.py, robot A) : accord scan/carte de
        17.5 % avec le natif brut, 98.0 % en appliquant -180 deg.

        On normalise ICI, et pas dans la TF : `MOWBOT_LIDAR_YAW=180` decrit un
        fait PHYSIQUE (le boitier du lidar est monte a l'envers) et doit rester
        vrai quel que soit le pilote. Compenser dans la TF marcherait mais
        rendrait la geometrie du robot dependante d'un choix logiciel -- le
        piege reviendrait au prochain changement de driver.
        """
        if m.angle_min > -0.1 and m.angle_max > 6.0:
            return -math.pi
        return 0.0

    def cb(self, m):
        n_in = len(m.ranges)
        if n_in < 2:
            return
        shift = self.angle_shift(m)
        amin, amax = m.angle_min + shift, m.angle_max + shift
        if not self.shift_logged:
            self.shift_logged = True
            if shift:
                self.get_logger().info(
                    f'convention d\'angles du driver : {math.degrees(m.angle_min):.0f}'
                    f'..{math.degrees(m.angle_max):.0f} deg -> ramenee a '
                    f'{math.degrees(amin):.0f}..{math.degrees(amax):.0f} deg')
        if self.mask is None:
            # mask construit sur les angles CORRIGES : les secteurs de
            # SELF_SECTORS sont des directions physiques mesurees sur le robot.
            self.mask = self.build_mask(amin, amax)

        src = np.linspace(m.angle_min, m.angle_max, n_in)
        dst = np.linspace(m.angle_min, m.angle_max, N)
        r = np.array(m.ranges, dtype=float)
        # np.interp n'accepte pas l'infini : on le remplace le temps du calcul
        # par une valeur hors portee, puis on remet l'infini apres coup. Sans
        # cette remise, /scan annoncait des mesures a range_max+1 (9.00 m avec
        # le LD14) la ou le lidar n'avait RIEN vu -- valeurs ignorees par nav2,
        # mais trompeuses a la lecture et dans les outils de diagnostic.
        r[~np.isfinite(r)] = m.range_max + 1.0
        out_r = np.interp(dst, src, r)
        out_r[out_r > m.range_max] = float('inf')

        # masquage : echo proche dans un secteur "robot" -> considere inexistant
        hide = (self.mask > 0) & (out_r < self.mask)
        out_r[hide] = float('inf')

        if ISO_ENABLE:
            out_r = self.drop_isolated(out_r, m.range_max)

        out = LaserScan()
        out.header.frame_id = m.header.frame_id
        # HORODATAGE : on CONSERVE celui du driver, et on ne le remplace que
        # s'il est aberrant.
        #
        # Les deux erreurs a eviter, rencontrees l'une apres l'autre :
        #  - le recopier aveuglement : au demarrage, le driver natif date ses
        #    scans sur sa propre reference (ecart mesure : 89 s). Le
        #    collision_monitor declarait alors la source invalide et ARRETAIT le
        #    robot ("Robot to stop due to invalid source"), chaque but echouait.
        #  - le remplacer systematiquement par l'heure courante : le scan
        #    reclame alors une TF a l'instant present, or l'EKF publie
        #    odom->base_link avec quelques millisecondes de retard. Le filtre de
        #    slam_toolbox attend une transformation qui n'existe pas encore,
        #    sature et jette TOUS les scans ("queue is full") -- plus aucune
        #    carte n'etait publiee.
        # En regime etabli le driver est juste a ~2 ms : on lui fait confiance.
        now = self.get_clock().now()
        st = m.header.stamp
        age = now.nanoseconds * 1e-9 - (st.sec + st.nanosec * 1e-9)
        if abs(age) > STAMP_MAX_AGE_S:
            if not self.stamp_warned:
                self.get_logger().warning(
                    f'horodatage du driver aberrant ({age:+.1f} s) : remplace '
                    f'par l\'heure locale')
                self.stamp_warned = True
            out.header.stamp = now.to_msg()
        else:
            out.header.stamp = st
        out.angle_min = amin
        out.angle_max = amax
        out.angle_increment = (amax - amin) / (N - 1)
        out.time_increment = m.time_increment
        out.scan_time = m.scan_time
        out.range_min = m.range_min
        out.range_max = m.range_max
        out.ranges = out_r.astype(np.float32).tolist()
        self.pub.publish(out)


def main():
    rclpy.init()
    n = ScanFix()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
