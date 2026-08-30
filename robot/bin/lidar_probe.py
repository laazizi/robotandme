#!/usr/bin/env python3
"""Identifie le MODELE de lidar present sur un port serie.

Appele par detect_devices.sh. Necessaire parce que l'ESP32 (DevKitC) et le
LD14 embarquent la MEME puce CP2102 (10c4:ea60) avec le meme numero de serie :
ni le vendor/product ni le serial ne permettent de les distinguer. On les
separe donc par un test reel du flux.

Signatures :
  ld14     115200 bauds, trames de 47 octets ouvrant sur 0x54 0x2C (LD06/LD19
           identiques mais a 230400)
  n10      230400 bauds, debit continu sans la signature LD
  microros trames delimitees par 0x7E a pas regulier -> ESP32 dont le firmware
           micro-ROS tourne. CE N'EST PAS UN LIDAR.
  (rien)   port silencieux -> ce n'est pas un lidar (ESP32 au repos, par ex.)

Sortie sur stdout, une ligne :  ld14 | ld06 | n10 | microros | inconnu
Code retour 0 si un lidar est reconnu, 1 sinon (microros renvoie 1).

POURQUOI `microros` A ETE AJOUTE. Le verdict n10 reposait sur le SEUL volume
lu ("debite beaucoup sans signature LD"). Or l'ESP32-P4 qui emet du micro-ROS a
460800 bauds depasse largement ce seuil : il etait declare n10 de facon
parfaitement reproductible (5 essais sur 5 sur la jetson). Consequences en
cascade, toutes constatees le meme jour :
  - detect_devices.sh voyait un lidar la ou etait l'ESP32 et n'ecrivait donc
    AUCUNE regle udev mowbot_esp32 ;
  - run_agent.sh, prive du lien udev, retombait sur cette meme sonde, lisait
    "n10" et ecartait l'ESP32 : "/dev/ttyACM0 est un LIDAR, ignore pour l'agent"
    en boucle ;
  - sans agent : pas de /odom, pas de /imu, donc pas d'EKF, donc pas de TF
    odom->base_link -- RViz sans robot et navigation morte.
Un verdict positif vaut mieux qu'un seuil : on reconnait desormais le micro-ROS
pour ce qu'il est, au lieu de conclure "lidar" par defaut d'autre idee.
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


def looks_microros(raw):
    """Trames micro-ROS (XRCE-DDS serie) : delimiteur 0x7E a PAS REGULIER.

    On ne se contente pas de compter les 0x7E : le payload d'un lidar en
    contient forcement par hasard. Ce qui est propre au micro-ROS, c'est la
    REGULARITE -- le meme ecart revient entre delimiteurs successifs, parce que
    le firmware republie les memes messages a cadence fixe.
    NE PAS chercher la chaine "XRCE" en clair : le protocole ne la transmet pas.
    C'etait le test en place, il ne s'est jamais declenche (mesure : 0 occurrence
    sur 600 octets d'un ESP32 pourtant parfaitement fonctionnel).
    """
    pos = [i for i, b in enumerate(raw) if b == 0x7E]
    if len(pos) < 5:
        return False
    ecarts = {}
    for a, b in zip(pos, pos[1:]):
        d = b - a
        if 4 <= d <= 200:
            ecarts[d] = ecarts.get(d, 0) + 1
    return bool(ecarts) and max(ecarts.values()) >= 3


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
    # AVANT le seuil de volume : un ESP32 en micro-ROS debite lui aussi
    # beaucoup, et le declarer lidar coupait toute la chaine (voir l'en-tete).
    # On teste aux deux vitesses : le firmware tourne a 460800, pas a 230400.
    if looks_microros(raw) or looks_microros(sample(port, 460800)):
        print('microros')
        return 1

    if len(raw) > 2000:     # debite beaucoup sans signature LD -> N10
        print('n10')
        return 0

    print('inconnu')
    return 1


if __name__ == '__main__':
    sys.exit(main())
