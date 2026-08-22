#!/usr/bin/env python3
"""Attend qu'une transformation TF soit disponible. Code retour 0 si oui.

    python3 wait_tf.py map odom [timeout_s]

POURQUOI un script plutot que `ros2 run tf2_ros tf2_echo` : sous systemd,
`ros2 run` ne trouve pas ses paquets (probleme deja rencontre sur ce projet
pour les TF statiques et le driver lidar). Le test echouait donc TOUJOURS,
et start_nav.sh attendait 80 s pour rien avant de lancer nav2 en annoncant a
tort que la carte etait absente.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def main():
    if len(sys.argv) < 3:
        print("usage: wait_tf.py <cible> <source> [timeout_s]", file=sys.stderr)
        return 2
    cible, source = sys.argv[1], sys.argv[2]
    timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0

    rclpy.init()
    n = Node('wait_tf')
    buf = Buffer()
    TransformListener(buf, n)
    t0 = time.time()
    ok = False
    while time.time() - t0 < timeout:
        rclpy.spin_once(n, timeout_sec=0.2)
        try:
            buf.lookup_transform(cible, source, rclpy.time.Time())
            ok = True
            break
        except Exception:
            pass
    if ok:
        print(f"{cible}->{source} disponible apres {time.time() - t0:.1f} s")
    else:
        print(f"{cible}->{source} ABSENTE apres {timeout:.0f} s", file=sys.stderr)
    n.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
