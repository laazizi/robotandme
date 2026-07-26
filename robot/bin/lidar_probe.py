#!/usr/bin/env python3
"""Identifie le MODELE de lidar present sur un port serie.

Appele par detect_devices.sh. Necessaire parce que l'ESP32 (DevKitC) et le
LD14 embarquent la MEME puce CP2102 (10c4:ea60) avec le meme numero de serie :
ni le vendor/product ni le serial ne permettent de les distinguer. On les
separe donc par un test reel du flux.

Signatures :
  ld14   115200 bauds, trames de 47 octets ouvrant sur 0x54 0x2C (LD06/LD19
         identiques mais a 230400)
  n10    230400 bauds, debit continu sans la signature LD
  (rien) port silencieux -> ce n'est pas un lidar (ESP32 au repos, par ex.)

Sortie sur stdout, une ligne :  ld14 | ld06 | n10 | inconnu
Code retour 0 si un lidar est reconnu, 1 sinon.
"""
import sys
import time

try:
    import serial
except ImportError:
    print('inconnu')
    sys.exit(1)


def sample(port, baud, seconds=2.0):
    try:
        s = serial.Serial(port, baud, timeout=0.5)
    except Exception:
        return b''
    time.sleep(0.2)
    try:
        s.reset_input_buffer()
        raw = b''
        t0 = time.time()
        while time.time() - t0 < seconds:
            try:
                raw += s.read(2048)
            except Exception:
                break        # decrochage (alimentation faible) : on garde l'acquis
        return raw
    finally:
        s.close()


def count_ld_frames(raw):
    """Trames LDRobot valides : 0x54 0x2C puis 45 octets."""
    n = 0
    i = 0
    while i < len(raw) - 47:
        if raw[i] == 0x54 and raw[i + 1] == 0x2C:
            n += 1
            i += 47
        else:
            i += 1
    return n


def main():
    if len(sys.argv) < 2:
        print('inconnu')
        return 1
    port = sys.argv[1]

    # LD14 d'abord : sa signature est formelle, donc sans ambiguite.
    raw = sample(port, 115200)
    if count_ld_frames(raw) >= 5:
        print('ld14')
        return 0

    raw = sample(port, 230400)
    if count_ld_frames(raw) >= 5:
        print('ld06')       # meme protocole, vitesse doublee
        return 0
    if len(raw) > 2000:     # debite beaucoup sans signature LD -> N10
        print('n10')
        return 0

    print('inconnu')
    return 1


if __name__ == '__main__':
    sys.exit(main())
