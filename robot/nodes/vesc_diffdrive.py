#!/usr/bin/env python3
"""Pilote diffdrive pour deux VESC (UBOX) : /cmd_vel -> roues, tachymetres -> /odom.

    mowbot node vesc_diffdrive.py
    mowbot node vesc_diffdrive.py --ros-args -p voie:=0.62 -p rayon_roue:=0.165

CE QUE FAIT CE NOEUD, ET CE QU'IL NE FAIT PAS. Il traduit, il ne regule pas :
l'asservissement de vitesse vit DANS le VESC, qui fait du FOC a plusieurs
dizaines de kHz sur son propre processeur. D'ou l'emploi de COMM_SET_RPM et non
d'un rapport cyclique -- commander un regime laisse le VESC tenir la vitesse
malgre la pente et l'herbe, ce qu'aucune boucle ecrite ici ne ferait aussi bien.

L'UBOX EST FAIT DE DEUX VESC : un au bout de l'USB, l'autre derriere le bus CAN
interne. Le second est trouve automatiquement au demarrage (parametre id_can
pour forcer), comme dans bin/vesc_test.py.

ODOMETRIE, ET SA LIMITE MESUREE. Les pas viennent du tachymetre du VESC :
391 pas par tour de roue mesures le 5 septembre 2026, soit 1,21 mm de
resolution. C'est 6,5 fois plus grossier que les encodeurs en quadrature du
tricycle, ce qui reste tres acceptable. MAIS le VESC ne compte QUE ce qu'il
commute lui-meme : roue tournee a la main, moteur relache ou robot pousse ne
produisent AUCUN pas -- verifie sur les deux VESC, ni regime ni tachymetre.
Et au demarrage il perd des pas : 372 comptes au premier tour contre 391 aux
suivants, 5 % de moins, le FOC sans capteur ne suivant pas le rotor tant que la
roue n'est pas lancee. L'odometrie de ce noeud est donc juste EN MARCHE, et
aveugle a l'arret. C'est pour cela que l'EKF fusionne des VITESSES et prend son
cap du gyroscope, jamais de la difference des roues.

SECURITE :
  * homme-mort de 500 ms sur /cmd_vel, comme le firmware ESP32. Sans commande
    fraiche, on envoie un regime nul.
  * le VESC a son propre delai de garde (~1 s) : si ce noeud meurt, les roues
    s'arretent seules. Deux barrieres independantes.
  * vitesse plafonnee par le parametre vitesse_max, appliquee AVANT conversion.
  * a l'arret du noeud : regime nul puis frein relache (roue libre).
"""
import math
import struct
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry

try:
    import serial
except ImportError:
    sys.exit("pyserial manquant : pip3 install --user pyserial")

COMM_GET_VALUES = 4
COMM_SET_RPM = 8
COMM_SET_CURRENT_BRAKE = 7
COMM_FORWARD_CAN = 34
COMM_FW_VERSION = 0


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encadrer(charge):
    tete = bytes([2, len(charge)]) if len(charge) < 256 else bytes([3]) + struct.pack('>H', len(charge))
    return tete + charge + struct.pack('>H', crc16(charge)) + b'\x03'


class Lien:
    """Acces serie au couple de VESC. Un seul fil ecrit, protege par un verrou."""

    # UN HOQUET USB NE DOIT PAS TUER LE PILOTE. Vecu le 5 septembre 2026 :
    # "device reports readiness to read but returned no data (device
    # disconnected or multiple access on port?)" a fait tomber le noeud, donc
    # /odom, donc toute la navigation. Le VESC coupe alors ses roues seul grace
    # a son delai de garde -- c'est la bonne reaction -- mais le robot devient
    # impilotable jusqu'a une intervention humaine. On rattrape l'exception, on
    # rouvre le port, et on continue.
    def __init__(self, port, id_can):
        self.chemin = port
        self.casse = False
        self.t_reouverture = 0.0
        # DELAI D'ATTENTE COURT, ET C'EST DELIBERE. Avec timeout=0.2 chaque
        # read() bloquait les 0,2 s entieres faute d'avoir ses 256 octets, alors
        # que le VESC repond en quelques millisecondes. Deux roues par cycle
        # faisaient 0,4 s, et /odom tombait a 2,5 Hz -- MESURE -- quand l'EKF en
        # attend 10. Avec 0,02 s la boucle de _lire() rend la main des que le
        # paquet est decode, l'echeance globale restant tenue par `delai`.
        self.s = serial.Serial(port, 115200, timeout=0.02)
        self.id_can = id_can
        self.verrou = threading.Lock()

    def _reouvrir(self, journal=None):
        """Tente de rouvrir le port, au plus une fois par seconde."""
        if time.time() - self.t_reouverture < 1.0:
            return False
        self.t_reouverture = time.time()
        try:
            try:
                self.s.close()
            except Exception:
                pass
            self.s = serial.Serial(self.chemin, 115200, timeout=0.02)
            self.casse = False
            if journal:
                journal.warning("liaison VESC retablie sur %s" % self.chemin)
            return True
        except Exception as e:
            if journal:
                journal.error("reouverture de %s impossible : %s" % (self.chemin, e))
            return False

    def _envoyer(self, charge, second):
        if second:
            charge = bytes([COMM_FORWARD_CAN, self.id_can]) + charge
        try:
            with self.verrou:
                self.s.write(encadrer(charge))
        except (serial.SerialException, OSError):
            self.casse = True

    def _lire(self, delai=0.25):
        fin = time.time() + delai
        tampon = b''
        while time.time() < fin:
            # in_waiting quand il y a de quoi lire, sinon 1 octet : on ne
            # bloque jamais sur une quantite qui n'arrivera pas.
            try:
                tampon += self.s.read(self.s.in_waiting or 1)
            except (serial.SerialException, OSError):
                # Port disparu ou lecture impossible : on ne releve PAS
                # l'exception, la boucle de controle doit survivre.
                self.casse = True
                return None
            while tampon:
                # NE JAMAIS JETER UN OCTET DE DEBUT INCOMPLET. La version
                # precedente testait "tampon[0] == 2 AND len(tampon) >= 2" : avec
                # une lecture qui ne rend qu'un octet, le 0x02 seul echouait sur
                # la longueur, tombait dans la resynchronisation et etait
                # SUPPRIME -- le reste de la trame devenait orphelin et rien ne
                # se decodait plus. Invisible tant qu'on lisait 256 octets d'un
                # coup, fatal des qu'on lit octet par octet. On distingue donc
                # "trame incomplete, attendre" de "octet parasite, jeter".
                if tampon[0] == 2:
                    if len(tampon) < 2:
                        break                      # attendre la longueur
                    n = tampon[1]
                    if len(tampon) < 2 + n + 3:
                        break                      # attendre la fin de trame
                    charge = tampon[2:2 + n]
                    crc = struct.unpack('>H', tampon[2 + n:4 + n])[0]
                    tampon = tampon[2 + n + 3:]
                    if crc == crc16(charge):
                        return charge
                    continue
                elif tampon[0] == 3:
                    if len(tampon) < 3:
                        break
                    n = struct.unpack('>H', tampon[1:3])[0]
                    if len(tampon) < 3 + n + 3:
                        break
                    charge = tampon[3:3 + n]
                    crc = struct.unpack('>H', tampon[3 + n:5 + n])[0]
                    tampon = tampon[3 + n + 3:]
                    if crc == crc16(charge):
                        return charge
                    continue
                else:
                    tampon = tampon[1:]            # octet parasite
        return None

    def version(self, second=False):
        with self.verrou:
            self.s.reset_input_buffer()
        self._envoyer(bytes([COMM_FW_VERSION]), second)
        r = self._lire()
        return (r[1], r[2]) if r and r[0] == COMM_FW_VERSION and len(r) >= 3 else None

    def regime(self, erpm, second):
        self._envoyer(bytes([COMM_SET_RPM]) + struct.pack('>i', int(erpm)), second)

    def frein(self, amperes, second):
        self._envoyer(bytes([COMM_SET_CURRENT_BRAKE]) + struct.pack('>i', int(amperes * 1000)), second)

    def valeurs(self, second):
        """Rend (tachymetre, regime, tension, temperature) ou None.

        Les offsets valent pour les firmwares 6.x et 7.x. Ils sont controles a
        chaque lecture par la coherence de la tension : si la trame changeait de
        forme, on rend None plutot qu'une odometrie fausse."""
        with self.verrou:
            self.s.reset_input_buffer()
        self._envoyer(bytes([COMM_GET_VALUES]), second)
        # ECHEANCE COURTE, PLUS COURTE ENCORE POUR LE MAITRE. Mesure du
        # 5 septembre 2026, douze aller-retours chacun :
        #   maitre        12/12 | 0,021 s de moyenne | 0,021 s au pire
        #   second (CAN)  11/12 | 0,058 s de moyenne | 0,404 s au pire
        # Le relais CAN est trois fois plus lent et perd une reponse sur douze.
        # Avec l'ancienne echeance de 0,3 s chaque perte coutait 0,3 s de boucle,
        # et comme on ne publie rien si UNE roue manque, /odom tombait a 6,8 Hz.
        # En renoncant plus vite, une reponse perdue ne coute que 0,12 s et fait
        # sauter une publication au lieu d'effondrer la cadence.
        r = self._lire(0.12 if second else 0.06)
        if not r or r[0] != COMM_GET_VALUES or len(r) < 53:
            return None
        v_in = struct.unpack('>h', r[27:29])[0] / 10.0
        if not (5.0 < v_in < 120.0):
            return None
        return (struct.unpack('>i', r[45:49])[0],
                struct.unpack('>i', r[23:27])[0],
                v_in,
                struct.unpack('>h', r[1:3])[0] / 10.0)


class VescDiffdrive(Node):
    def __init__(self):
        super().__init__('vesc_diffdrive')
        p = self.declare_parameter
        p('port', '/dev/ttyACM0')
        p('id_can', -1)                # -1 = chercher tout seul
        p('voie', 0.0)                 # entraxe des roues motrices [m]  -- A RENSEIGNER
        p('rayon_roue', 0.0)           # rayon effectif [m]              -- A RENSEIGNER
        p('pas_par_tour', 391.0)       # mesure au banc le 05/09/2026
        p('vitesse_max', 0.60)         # [m/s] plafond applique avant conversion
        p('inverser_gauche', False)
        p('inverser_droite', False)
        p('gauche_est_second', False)  # la roue gauche est-elle sur le VESC du CAN ?
        p('periode', 0.05)             # 20 Hz
        p('deadman', 0.5)              # [s], comme le firmware ESP32
        p('arret_libre', True)         # a consigne nulle : relacher, ou tenir 0 ?
        p('publier_tf', False)         # l'EKF publie odom->base_link, pas nous

        g = lambda n: self.get_parameter(n).value
        self.voie = g('voie')
        self.rayon = g('rayon_roue')
        self.pas_tour = g('pas_par_tour')
        self.v_max = g('vitesse_max')
        self.inv = (g('inverser_gauche'), g('inverser_droite'))
        self.gauche_second = g('gauche_est_second')
        self.deadman = g('deadman')
        self.arret_libre = g('arret_libre')
        self.publier_tf = g('publier_tf')

        if self.voie <= 0 or self.rayon <= 0:
            self.get_logger().error(
                "voie=%.3f et rayon_roue=%.3f doivent etre renseignes (metres). "
                "ON NE DEVINE PAS une geometrie : une valeur fausse donne une "
                "odometrie fausse, et nav2 planifie alors sur du sable."
                % (self.voie, self.rayon))
            raise SystemExit(2)

        self.circ = 2 * math.pi * self.rayon
        # pas de tachymetre -> metres. Un pas vaut circonference / pas_par_tour.
        self.m_par_pas = self.circ / self.pas_tour
        # m/s -> ERPM : tours de roue par seconde x (pas_par_tour/6) x 60
        self.erpm_par_ms = (self.pas_tour / 6.0) * 60.0 / self.circ

        try:
            self.lien = Lien(g('port'), None)
        except Exception as e:
            self.get_logger().error("ouverture de %s impossible : %s" % (g('port'), e))
            raise SystemExit(2)

        if self.lien.version() is None:
            self.get_logger().error("le VESC maitre ne repond pas sur %s" % g('port'))
            raise SystemExit(2)

        idc = g('id_can')
        if idc < 0:
            # PLUSIEURS TENTATIVES, ESPACEES. Le second VESC est derriere le bus
            # CAN interne et n'est pas pret instantanement apres une mise sous
            # tension : constate le 5 septembre 2026, le noeud a refuse de
            # demarrer juste apres un redemarrage de l'UBOX alors que la moitie
            # en question repondait parfaitement quelques secondes plus tard.
            # Abandonner du premier coup obligeait a relancer a la main.
            trouve = None
            for essai in range(6):
                for i in range(11):
                    self.lien.id_can = i
                    if self.lien.version(second=True):
                        trouve = i
                        break
                if trouve is not None:
                    break
                self.get_logger().warning(
                    "aucun second VESC au balayage %d/6, nouvelle tentative dans 2 s "
                    "(l'UBOX vient peut-etre d'etre mis sous tension)" % (essai + 1))
                time.sleep(2.0)
            if trouve is None:
                self.lien.id_can = None
                self.get_logger().error(
                    "AUCUN second VESC sur le bus CAN apres 6 balayages : une seule "
                    "roue serait pilotee, ce qui ferait tourner le robot en rond. Arret.")
                raise SystemExit(2)
            self.lien.id_can = trouve
            self.get_logger().info("second VESC trouve a l'identifiant CAN %d" % trouve)
        else:
            self.lien.id_can = idc

        self.cmd = (0.0, 0.0)
        self.t_cmd = 0.0
        self.x = self.y = self.th = 0.0
        self.pas0 = {}
        self.t_odom = time.time()

        self.create_subscription(Twist, 'cmd_vel', self.sur_cmd, 10)
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf = None
        if self.publier_tf:
            from tf2_ros import TransformBroadcaster
            self.tf = TransformBroadcaster(self)

        self.create_timer(g('periode'), self.boucle)
        # Surveillance de la cadence REELLE : un /odom trop lent ne se voit pas,
        # il degrade silencieusement l'EKF et la navigation. On le DIT.
        self.n_odom = 0
        self.t_cadence = time.time()
        self.create_timer(10.0, self.controler_cadence)
        self.get_logger().info(
            "pret : voie %.3f m, rayon %.4f m, %.0f pas/tour -> %.3f mm par pas, "
            "%.0f ERPM par m/s" % (self.voie, self.rayon, self.pas_tour,
                                   self.m_par_pas * 1000, self.erpm_par_ms))

    def controler_cadence(self):
        t = time.time()
        hz = self.n_odom / max(1e-6, t - self.t_cadence)
        self.n_odom = 0
        self.t_cadence = t
        if hz < 5.0:
            self.get_logger().warning(
                "/odom a seulement %.1f Hz : l'EKF est cale sur 10 Hz et la "
                "navigation va en souffrir. Regarder la liaison serie." % hz)
        else:
            self.get_logger().info("/odom a %.1f Hz" % hz)

    def sur_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)
        self.t_cmd = time.time()

    def _second(self, gauche):
        """Sur quel VESC se trouve cette roue ?"""
        return self.gauche_second if gauche else not self.gauche_second

    def boucle(self):
        if self.lien.casse:
            if not self.lien._reouvrir(self.get_logger()):
                return
        v, w = self.cmd
        if time.time() - self.t_cmd > self.deadman:
            v = w = 0.0                       # homme-mort
        vg = v - w * self.voie / 2.0
        vd = v + w * self.voie / 2.0
        # Saturation qui PRESERVE LA COURBURE : on divise les deux par le meme
        # facteur plutot que d'ecreter chacune, sinon le robot part de travers
        # des qu'une roue sature.
        pire = max(abs(vg), abs(vd))
        if pire > self.v_max:
            k = self.v_max / pire
            vg *= k
            vd *= k
        # A CONSIGNE NULLE, ON RELACHE PLUTOT QUE DE TENIR ZERO.
        # COMM_SET_RPM 0 engage l'asservissement de vitesse sur zero : le VESC
        # injecte du courant pour EMPECHER la roue de tourner. Sur un banc, roues
        # en l'air, elles resistent a la main ; sur le robot, les moteurs
        # chauffent a l'arret sans rien faire d'utile. Un frein a 0 A relache le
        # moteur, et le robot roule librement.
        # arret_libre=False rend le maintien actif : utile le jour ou il faudra
        # tenir dans une pente, a condition de surveiller la temperature.
        if self.arret_libre and abs(vg) < 1e-6 and abs(vd) < 1e-6:
            for second in (False, True):
                self.lien.frein(0.0, second)
        else:
            for roue, vit, inv in (('g', vg, self.inv[0]), ('d', vd, self.inv[1])):
                erpm = vit * self.erpm_par_ms * (-1 if inv else 1)
                self.lien.regime(erpm, self._second(roue == 'g'))
        self.publier_odom()

    def publier_odom(self):
        lect = {}
        for roue in ('g', 'd'):
            r = self.lien.valeurs(self._second(roue == 'g'))
            if r is None:
                return                        # trame douteuse : on ne publie RIEN
            lect[roue] = r
        t = time.time()
        dt = t - self.t_odom
        if dt <= 0:
            return
        self.t_odom = t
        d = {}
        for roue in ('g', 'd'):
            pas = lect[roue][0]
            inv = self.inv[0] if roue == 'g' else self.inv[1]
            if roue not in self.pas0:
                self.pas0[roue] = pas
            d[roue] = (pas - self.pas0[roue]) * self.m_par_pas * (-1 if inv else 1)
            self.pas0[roue] = pas
        dc = (d['g'] + d['d']) / 2.0
        dth = (d['d'] - d['g']) / self.voie
        self.th += dth
        self.x += dc * math.cos(self.th)
        self.y += dc * math.sin(self.th)

        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = math.sin(self.th / 2.0)
        o.pose.pose.orientation.w = math.cos(self.th / 2.0)
        o.twist.twist.linear.x = dc / dt
        o.twist.twist.angular.z = dth / dt
        # L'EKF ne prend QUE les vitesses (cf. ekf.yaml) : la pose ci-dessus est
        # informative, elle derive avec le patinage et personne ne la fusionne.
        self.pub.publish(o)
        self.n_odom += 1
        if self.tf:
            tr = TransformStamped()
            tr.header = o.header
            tr.child_frame_id = 'base_link'
            tr.transform.translation.x = self.x
            tr.transform.translation.y = self.y
            tr.transform.rotation = o.pose.pose.orientation
            self.tf.sendTransform(tr)

    def arreter(self):
        for second in (False, True):
            try:
                self.lien.regime(0, second)
                self.lien.frein(0.0, second)
            except Exception:
                pass


def main():
    rclpy.init()
    try:
        n = VescDiffdrive()
    except SystemExit as e:
        rclpy.shutdown()
        return e.code
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.arreter()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
