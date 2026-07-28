#!/usr/bin/env python3
"""Ou le lidar croit-il que se trouve l'AVANT du robot ?

Place un objet bien identifiable DEVANT le robot (50-100 cm, un carton par
exemple), puis lance ce script. Il indique la direction de l'echo le plus
proche dans le repere du robot.

  0 deg    = devant        -> yaw correct
  180 deg  = derriere      -> boitier tourne de 180 deg, mettre --yaw 3.14159
  +/-90    = sur le cote   -> tourne de 90 deg

Un lidar 360 deg n'a pas de champ oriente, mais son angle zero doit regarder
vers l'avant : sinon toute la carte est pivotee.
"""
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

rclpy.init(); n = Node('lidar_front'); scans = []
n.create_subscription(LaserScan, '/scan', lambda m: scans.append(m), qos_profile_sensor_data)
t0 = time.time()
while len(scans) < 8 and time.time() - t0 < 15:
    rclpy.spin_once(n, timeout_sec=0.2)
if not scans:
    print("pas de /scan : le lidar tourne-t-il ?"); raise SystemExit(1)

m = scans[-1]
r = np.array(m.ranges, dtype=float)
ang = m.angle_min + np.arange(len(r)) * m.angle_increment
ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
if not ok.any():
    print("aucun echo valide"); raise SystemExit(1)

i = np.argmin(np.where(ok, r, np.inf))
print(f"{len(scans)} scans recus, {ok.sum()} echos valides\n")
print(f"echo le plus PROCHE : {r[i]:.2f} m a {math.degrees(ang[i]):+.1f} deg")
print()
print("secteurs (distance minimale vue dans chacun) :")
for nom, a0, a1 in (("AVANT      ", -20,  20), ("gauche     ",  70, 110),
                    ("ARRIERE (+)", 160, 180), ("ARRIERE (-)", -180, -160),
                    ("droite     ", -110, -70)):
    sel = ok & (np.degrees(ang) >= a0) & (np.degrees(ang) <= a1)
    if sel.any():
        print(f"  {nom} [{a0:+4.0f}..{a1:+4.0f}] : {r[sel].min():.2f} m")
    else:
        print(f"  {nom} [{a0:+4.0f}..{a1:+4.0f}] : rien")
print()
d = abs(math.degrees(ang[i]))
if d < 30:
    print(">>> l'objet est vu DEVANT : yaw correct (0)")
elif d > 150:
    print(">>> l'objet est vu DERRIERE : boitier tourne de 180 deg")
    print("    -> MOWBOT_LIDAR_YAW=3.14159")
else:
    s = '+' if math.degrees(ang[i]) > 0 else '-'
    print(f">>> l'objet est vu a {math.degrees(ang[i]):+.0f} deg : boitier tourne")
    print(f"    -> MOWBOT_LIDAR_YAW={-math.radians(math.degrees(ang[i])):.5f}")
