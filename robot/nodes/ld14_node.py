#!/usr/bin/env python3
"""Pilote LDRobot LD14 / LD06 / LD19 -> publie /scan_raw.

POURQUOI UN NOEUD PYTHON plutot que le driver officiel ldlidar_stl_ros2 :
le protocole tient en 40 lignes, et compiler du C++ sur une Raspberry Pi 3B+
(899 Mo de RAM) est long et expose a un echec par saturation memoire. Ce
noeud n'a aucune dependance a compiler, et reste coherent avec les autres
noeuds mowbot. Debit a soutenir : ~2350 points/s, negligeable en Python.

TRAME (47 octets, verifiee sur le materiel) :
  0x54                header
  0x2C                VerLen : 0x2C = 12 mesures par trame
  vitesse   uint16    deg/s  (6 Hz mesures sur le LD14)
  angle_deb uint16    en 0.01 deg
  12 x (distance uint16 en mm + intensite uint8)
  angle_fin uint16    en 0.01 deg
  horodate  uint16
  crc8      uint8     polynome 0x4D

SENS DE ROTATION : le LD14 fait croitre son angle dans le sens HORAIRE vu de
dessus, alors que ROS attend l'inverse (trigonometrique). L'angle est donc
inverse -- sans cela la carte serait le miroir de la realite. Reglable par le
parametre `invert_angle` si le lidar est monte tete en bas.

Publie sur /scan_raw : c'est scan_fix.py qui normalise ensuite le nombre de
points, masque les parties du robot et filtre les faux echos (cf. son
docstring), exactement comme pour le lidar N10.
"""
import math
import struct

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

HEADER = 0x54
VERLEN = 0x2C
MARKER = bytes((HEADER, VERLEN))   # motif cherche par bytes.find()
FRAME_LEN = 47
POINTS_PER_FRAME = 12


def _crc_table(poly=0x4D):
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if (c & 0x80) else ((c << 1) & 0xFF)
        table.append(c)
    return table


CRC_TABLE = _crc_table()


def crc8(data):
    crc = 0
    for b in data:
        crc = CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


class LD14(Node):
    def __init__(self):
        super().__init__('ld14')
        self.declare_parameter('port', '/dev/mowbot_lidar')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'laser_link')
        self.declare_parameter('topic', '/scan_raw')
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 8.0)
        self.declare_parameter('invert_angle', True)
        self.declare_parameter('check_crc', True)

        g = lambda n: self.get_parameter(n).value
        self.frame_id = g('frame_id')
        self.rmin = g('range_min')
        self.rmax = g('range_max')
        self.invert = g('invert_angle')
        self.check_crc = g('check_crc')

        port, baud = g('port'), g('baudrate')
        self.ser = serial.Serial(port, baud, timeout=0.5)
        self.pub = self.create_publisher(LaserScan, g('topic'), qos_profile_sensor_data)

        self.buf = bytearray()
        self.pts = []          # (angle_deg croissant, distance_m, intensite)
        self.last_angle = None
        self.turn = 0.0        # degres accumules depuis le debut du tour
        self.n_frames = 0
        self.n_bad_crc = 0
        self.n_scans = 0
        self.speed_hz = 0.0

        self.get_logger().info(f'LD14 sur {port} @ {baud} -> {g("topic")}')
        # 100 Hz suffit : le lidar debite ~9 ko/s, soit ~90 octets par appel.
        # Un timer a 500 Hz faisait 78 % de CPU sur une Raspberry Pi 4 -- la
        # machine entiere saturait (charge 25 sur 4 coeurs) et l'EKF comme le
        # SLAM n'avaient plus de temps pour tourner.
        self.create_timer(0.01, self.poll)
        self.create_timer(20.0, self.report)

    def report(self):
        msg = (f'{self.n_scans} tours publies, {self.n_frames} trames, '
               f'rotation {self.speed_hz:.1f} Hz')
        if self.n_frames:
            msg += f', CRC invalides {100.0 * self.n_bad_crc / self.n_frames:.1f} %'
        self.get_logger().info(msg)
        self.n_frames = self.n_bad_crc = self.n_scans = 0

    def poll(self):
        try:
            n = self.ser.in_waiting
            if n:
                self.buf += self.ser.read(n)
        except (OSError, serial.SerialException) as e:
            self.get_logger().warning(f'lecture serie : {e}')
            return

        # Resynchronisation sur l'en-tete via bytes.find(), implemente en C :
        # parcourir le tampon octet par octet en Python coutait 78 % de CPU.
        # Un octet perdu ne doit pas decaler durablement le decodage, d'ou la
        # recherche du motif a chaque passage plutot qu'un simple pas de 47.
        buf = self.buf
        i = 0
        n = len(buf)
        while True:
            j = buf.find(MARKER, i)
            if j < 0 or j + FRAME_LEN > n:
                break
            frame = bytes(buf[j:j + FRAME_LEN])
            if self.check_crc and crc8(frame[:-1]) != frame[-1]:
                self.n_bad_crc += 1
                i = j + 1       # +1 et non +47 : l'en-tete etait peut-etre fortuit
                continue
            self.decode_frame(frame)
            i = j + FRAME_LEN
        # on ne conserve que la queue incomplete ; un seul del, jamais dans la boucle
        if i:
            del buf[:i]
        if len(buf) > 8192:      # garde-fou si le flux devient illisible
            del buf[:-1024]

    def decode_frame(self, f):
        # NE PAS nommer cette methode `handle` : rclpy.Node expose un attribut
        # interne `self.handle` (le handle C du noeud). Une methode du meme nom
        # le masque et la construction du noeud echoue avec
        # "'method' object does not support the context manager protocol".
        self.n_frames += 1
        speed, a_start = struct.unpack('<HH', f[2:6])
        a_end = struct.unpack('<H', f[42:44])[0]
        self.speed_hz = speed / 360.0
        a0, a1 = a_start / 100.0, a_end / 100.0
        span = (a1 - a0) % 360.0

        for k in range(POINTS_PER_FRAME):
            dist_mm, inten = struct.unpack('<HB', f[6 + k * 3:9 + k * 3])
            ang = (a0 + span * k / (POINTS_PER_FRAME - 1)) % 360.0
            d = dist_mm / 1000.0
            self.pts.append((ang, d if self.rmin <= d <= self.rmax else float('inf'), inten))

        # Fin de tour : l'angle repasse par 0 (il croit puis reboucle).
        if self.last_angle is not None:
            self.turn += (a1 - self.last_angle) % 360.0
        self.last_angle = a1
        if self.turn >= 358.0:
            self.publish()
            self.turn = 0.0
            self.pts = []

    def publish(self):
        if len(self.pts) < 30:
            return
        pts = sorted(self.pts, key=lambda p: p[0])
        if self.invert:
            # sens horaire -> trigonometrique : on renverse l'ordre angulaire
            pts = [((360.0 - a) % 360.0, d, i) for a, d, i in pts]
            pts.sort(key=lambda p: p[0])

        angles = [math.radians(p[0]) - math.pi for p in pts]   # -pi .. +pi
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        m.angle_min = angles[0]
        m.angle_max = angles[-1]
        m.angle_increment = (m.angle_max - m.angle_min) / max(1, len(angles) - 1)
        m.scan_time = 1.0 / self.speed_hz if self.speed_hz > 0 else 0.166
        m.time_increment = m.scan_time / max(1, len(angles))
        m.range_min = float(self.rmin)
        m.range_max = float(self.rmax)
        m.ranges = [float(p[1]) for p in pts]
        m.intensities = [float(p[2]) for p in pts]
        self.pub.publish(m)
        self.n_scans += 1


def main():
    rclpy.init()
    try:
        n = LD14()
    except serial.SerialException as e:
        print(f'ERREUR : port lidar inaccessible ({e})')
        return
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.ser.close()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
