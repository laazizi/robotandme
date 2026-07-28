#!/usr/bin/env python3
"""Carre de validation : 4 cotes de 1 m separes par 4 rotations de 90 deg.

C'est LE test d'ensemble : il enchaine translations et rotations, donc il
revele a la fois une erreur de rayon de roue (cotes trop longs ou trop courts)
et une erreur d'entraxe (coins mal tournes). En theorie le robot revient
exactement a son point de depart, avec son cap d'origine.

Asservissement en BOUCLE FERMEE sur /odom (l'odometrie vient d'etre calibree).
Le GYRO sert de controle INDEPENDANT sur chaque rotation : si l'odometrie et le
gyro divergent, l'entraxe est en cause.

A la fin, l'ecart entre l'odometrie (qui se croit revenue a 0,0) et la position
REELLE mesuree au sol donne l'erreur cumulee du systeme.

    python3 square.py [cote_m] [nb_cotes]      defaut : 1.0 m, 4 cotes
"""
import math, os, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

COTE   = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
NCOTES = int(sys.argv[2])   if len(sys.argv) > 2 else 4
V      = float(os.environ.get('MOWBOT_V', '0.25'))
W      = float(os.environ.get('MOWBOT_W', '0.5'))

def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

def wrap(a):
    while a >  math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

class S(Node):
    def __init__(s):
        super().__init__('square')
        s.pub = s.create_publisher(Twist, '/cmd_vel', 10)
        s.p = None          # (x, y, yaw) selon /odom
        s.gz = 0.0
        s.gyro_angle = 0.0
        s.t_prev = None
        s.biais = 0.0
        s.create_subscription(Odometry, '/odom', s.cb_odom, 10)
        s.create_subscription(Imu, '/imu/data_raw', s.cb_imu, qos_profile_sensor_data)
    def cb_odom(s, m):
        q = m.pose.pose.orientation
        s.p = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(q))
    def cb_imu(s, m):
        now = time.time()
        s.gz = m.angular_velocity.z
        if s.t_prev is not None:
            s.gyro_angle += (s.gz - s.biais) * (now - s.t_prev)
        s.t_prev = now
    def spin(s, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(s, timeout_sec=0.05)
    def send(s, vx, wz):
        tw = Twist(); tw.linear.x = vx; tw.angular.z = wz
        s.pub.publish(tw)
    def stop(s):
        for _ in range(20):
            s.send(0.0, 0.0); rclpy.spin_once(s, timeout_sec=0.02); time.sleep(0.03)
        s.spin(0.8)
    def avance(s, d):
        # ralentit sur les 15 derniers cm : a 0.25 m/s et 8.5 Hz d'odometrie,
        # le robot avance 3 cm entre deux mesures et depassait de ~4 cm.
        x0, y0, _ = s.p
        FREIN = 0.15
        while True:
            x, y, _ = s.p
            parcouru = math.hypot(x - x0, y - y0)
            if parcouru >= d: break
            reste = d - parcouru
            v = V if reste > FREIN else max(0.08, V * reste / FREIN)
            s.send(v, 0.0); rclpy.spin_once(s, timeout_sec=0.02); time.sleep(0.03)
        s.stop()
        x, y, _ = s.p
        return math.hypot(x - x0, y - y0)
    def tourne(s, angle):
        """Rotation asservie sur l'odometrie, angle CUMULE.

        On additionne les petits increments au lieu de comparer au cap de
        depart : `wrap(yaw - yaw0)` rebascule des qu'on depasse 180 deg, la
        condition d'arret n'est alors plus vue et le robot continue de tourner.
        Constate : un coin de 90 deg parti a 260 deg.
        """
        s.gyro_angle = 0.0
        cible = abs(angle); sens = 1.0 if angle > 0 else -1.0
        cumul = 0.0
        prev = s.p[2]
        # ralentit sur les 20 derniers degres : limite le depassement
        FREIN = math.radians(20)
        while cumul < cible:
            reste = cible - cumul
            w = W if reste > FREIN else max(0.15, W * reste / FREIN)
            s.send(0.0, sens * w); rclpy.spin_once(s, timeout_sec=0.02); time.sleep(0.03)
            cur = s.p[2]
            cumul += abs(wrap(cur - prev))      # increment court : jamais ambigu
            prev = cur
        s.stop()
        return cumul, abs(s.gyro_angle)

rclpy.init(); n = S()
t0 = time.time()
while n.p is None and time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.2)
if n.p is None:
    print("pas d'odometrie"); sys.exit(1)

# biais gyro residuel, robot immobile
ech = []
t0 = time.time()
while time.time() - t0 < 2.5:
    rclpy.spin_once(n, timeout_sec=0.1); ech.append(n.gz)
n.biais = sum(ech)/len(ech)
n.gyro_angle = 0.0
print(f"biais gyro : {n.biais:+.4f} rad/s\n")

x0, y0, yaw0 = n.p
print(f"depart : x={x0:+.3f} y={y0:+.3f} cap={math.degrees(yaw0):+.1f} deg")
print(f"carre : {NCOTES} cotes de {COTE} m, rotations de {360/NCOTES:.0f} deg\n")
print(f"{'etape':<10} {'cote (m)':>9} {'rot odom':>10} {'rot gyro':>10} {'ecart':>7}")
print("-" * 52)
ANGLE = 2*math.pi/NCOTES
for i in range(NCOTES):
    d = n.avance(COTE)
    ro, rg = n.tourne(ANGLE)
    ecart = math.degrees(rg - ro)
    print(f"cote {i+1:<5} {d:9.3f} {math.degrees(ro):9.1f}° {math.degrees(rg):9.1f}° {ecart:+6.1f}°")

x, y, yaw = n.p
print("-" * 52)
print(f"\nSELON L'ODOMETRIE, retour a : x={x:+.3f} y={y:+.3f} cap={math.degrees(wrap(yaw-yaw0)):+.1f} deg")
print(f"  erreur de fermeture interne : {math.hypot(x-x0, y-y0)*100:.1f} cm")
print("\n>>> MESURE MAINTENANT L'ECART REEL au sol entre le depart et l'arrivee.")
print("    C'est lui qui donne l'erreur vraie du systeme ; l'odometrie, elle,")
print("    ne peut mesurer que sa propre coherence.")
