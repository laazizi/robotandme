#!/usr/bin/env python3
"""Etat de sante de l'IMU, robot IMMOBILE : le gyro vit-il, et son biais ?

    python3 gyrocheck.py

CE QUI COMPTE, ce n'est pas la moyenne mais le NOMBRE DE VALEURS DISTINCTES.
Un gyro sain est bruite : quelques dizaines de valeurs differentes sur trois
cents echantillons. Une valeur figee a la sixieme decimale sur des centaines de
lectures signifie que le capteur ne repond plus et que le firmware republie sa
derniere mesure -- l'EKF integre alors une rotation qui n'existe pas, et le
robot pivote sans fin dans RViz.

Piege dans lequel je suis tombe : une valeur constante RESSEMBLE a un biais mal
calibre. C'est l'absence de bruit qui trahit la panne, pas l'amplitude.

Repere : un biais sous 0.5 deg/s est correct. Au-dela, l'ESP32 a probablement
ete redemarre pendant que le robot bougeait -- il calibre son zero dans la
premiere seconde. Le corriger :  mowbot esp32-reset  (robot immobile).
"""
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
D = 180 / math.pi
rclpy.init(); n = Node("gyrocheck")
g = {"x": [], "y": [], "z": []}; a = {"x": [], "z": []}; ek = []
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def cb(m):
    g["x"].append(m.angular_velocity.x); g["y"].append(m.angular_velocity.y)
    g["z"].append(m.angular_velocity.z)
    a["x"].append(m.linear_acceleration.x); a["z"].append(m.linear_acceleration.z)
n.create_subscription(Imu, "/imu/data_raw", cb, qos_profile_sensor_data)
n.create_subscription(Odometry, "/odometry/filtered",
                      lambda m: ek.append(yaw(m.pose.pose.orientation)),
                      qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 18:
    rclpy.spin_once(n, timeout_sec=0.2)
if not g["z"]:
    print("  aucune donnee IMU"); raise SystemExit
for k in ("x", "y", "z"):
    v = g[k]
    print(f"  gyro  {k} : moy {sum(v)/len(v)*D:+8.3f} deg/s   distinctes {len(set(v)):4d} / {len(v)}")
for k in ("x", "z"):
    v = a[k]
    print(f"  accel {k} : moy {sum(v)/len(v):+8.3f} m/s2     distinctes {len(set(v)):4d} / {len(v)}")
print()
print("  => GYRO GELE" if len(set(g["z"])) == 1 else "  => LE GYRO VIT")
b = sum(g["z"]) / len(g["z"]) * D
print(f"  biais de lacet : {b:+.3f} deg/s  ({b*60:+.0f} deg/min)")
print("  -> correct" if abs(b) < 0.5 else "  -> TROP FORT (robot bouge pendant la calibration ?)")
if len(ek) > 4:
    d = ek[-1] - ek[0]
    while d > math.pi: d -= 2*math.pi
    while d < -math.pi: d += 2*math.pi
    print(f"  derive du cap EKF : {d*D:+.2f} deg en 18 s")
