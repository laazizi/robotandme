#!/usr/bin/env python3
"""Nœud ROS 2 : lit le 9DoF Razor AHRS (série FTDI) et publie /imu/data_raw.

Le Razor (firmware AHRS) sort des lignes texte a 57600 bauds :
    #A-C=ax,ay,az    (accelerometre, LSB brut ADXL345)
    #G-C=gx,gy,gz    (gyroscope, LSB brut ITG-3200)
    #YPR=yaw,pitch,roll   (orientation fusionnee -- IGNOREE : yaw magneto,
                           fausse pres des moteurs)

On publie sensor_msgs/Imu avec gyro (rad/s) + accel (m/s2). Pas d'orientation
(covariance[0] = -1) : l'EKF fusionne la VITESSE de cap (gyro z), insensible
au patinage des roues ET au magnetisme.

Biais gyro calibre au demarrage (robot immobile ~2 s).

Lancement (sur le Jetson) :
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=0
    python3 razor_imu_node.py [/dev/ttyUSB0] [57600]
"""

import sys
import math
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# --- Echelles capteurs (a VERIFIER par test, voir README ci-dessous) ---
# ITG-3200 : 14.375 LSB par deg/s  ->  LSB -> rad/s (echelle validee ~1% par test 360)
GYRO_LSB_TO_RADS = (1.0 / 14.375) * (math.pi / 180.0)
# Signe du gyro Z : le capteur donne NEGATIF sur une rotation a gauche (CCW),
# or ROS veut POSITIF a gauche -> on inverse. (verifie par test de rotation)
GYRO_Z_SIGN = -1.0
# ADXL345 pleine resolution : 256 LSB par g  ->  LSB -> m/s2
ACC_LSB_TO_MS2 = (1.0 / 256.0) * 9.80665

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 57600
CALIB_SAMPLES = 200          # ~2 s a ~100 Hz : biais gyro au repos


def parse3(payload):
    try:
        a, b, c = payload.split(",")
        return float(a), float(b), float(c)
    except Exception:
        return None


class RazorImu(Node):
    def __init__(self):
        super().__init__("razor_imu")
        self.pub = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.ser = serial.Serial(PORT, BAUD, timeout=1.0)
        self.gyro = None          # dernier gyro brut (LSB)
        self.acc = None           # dernier accel brut (LSB)
        self.bias = [0.0, 0.0, 0.0]
        self.get_logger().info(f"Razor sur {PORT} @ {BAUD} — calibration biais gyro (immobile)...")
        self._calibrate_bias()
        self.get_logger().info(f"biais gyro (LSB) = {self.bias}")

    def _read_line(self):
        raw = self.ser.readline()
        if not raw:
            return None
        return raw.decode(errors="ignore").strip()

    def _calibrate_bias(self):
        acc = [0.0, 0.0, 0.0]
        n = 0
        while rclpy.ok() and n < CALIB_SAMPLES:
            line = self._read_line()
            if line and line.startswith("#G-C="):
                g = parse3(line[5:])
                if g:
                    for i in range(3):
                        acc[i] += g[i]
                    n += 1
        if n:
            self.bias = [acc[i] / n for i in range(3)]

    def spin(self):
        imu = Imu()
        imu.header.frame_id = "imu_link"
        # Pas d'orientation fournie (yaw magneto inexploitable pres des moteurs)
        imu.orientation_covariance[0] = -1.0
        # Covariances diagonales indicatives (a affiner)
        for i in range(3):
            imu.angular_velocity_covariance[i * 4] = 1e-3
            imu.linear_acceleration_covariance[i * 4] = 1e-2

        while rclpy.ok():
            line = self._read_line()
            if not line:
                continue
            if line.startswith("#G-C="):
                g = parse3(line[5:])
                if g:
                    self.gyro = g
            elif line.startswith("#A-C="):
                a = parse3(line[5:])
                if a:
                    self.acc = a
            else:
                continue

            # On publie quand on a un gyro frais + un accel connu
            if self.gyro is None or self.acc is None:
                continue

            imu.header.stamp = self.get_clock().now().to_msg()
            imu.angular_velocity.x = (self.gyro[0] - self.bias[0]) * GYRO_LSB_TO_RADS
            imu.angular_velocity.y = (self.gyro[1] - self.bias[1]) * GYRO_LSB_TO_RADS
            imu.angular_velocity.z = GYRO_Z_SIGN * (self.gyro[2] - self.bias[2]) * GYRO_LSB_TO_RADS
            imu.linear_acceleration.x = self.acc[0] * ACC_LSB_TO_MS2
            imu.linear_acceleration.y = self.acc[1] * ACC_LSB_TO_MS2
            imu.linear_acceleration.z = self.acc[2] * ACC_LSB_TO_MS2
            self.pub.publish(imu)
            self.gyro = None      # forcer un nouveau gyro avant la prochaine pub


def main():
    rclpy.init()
    node = RazorImu()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
