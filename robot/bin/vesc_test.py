#!/usr/bin/env python3
"""Banc d'essai pour un UBOX dual VESC en USB : deux roues, avant/arriere, vitesse.

    robot/bin/vesc_test.py --list          quels ports ressemblent a un VESC
    robot/bin/vesc_test.py --info          version de firmware, et scan du bus CAN
    robot/bin/vesc_test.py                 pilotage au clavier (mode principal)
    robot/bin/vesc_test.py --duty 0.08 --duree 2     impulsion des deux roues
    robot/bin/vesc_test.py --autotest      verifie l'encodage SANS materiel

POURQUOI PAS pyvesc : la bibliotheque traine des retards de version et casse sur
les firmwares recents. Le protocole tient en trente lignes, il est stable depuis
des annees, et n'avoir que pyserial en dependance vaut mieux ici.

L'UBOX, C'EST DEUX VESC. Un seul est au bout de l'USB (le maitre) ; l'autre est
derriere un bus CAN interne. On lui parle en emballant le paquet dans un
COMM_FORWARD_CAN adresse a son identifiant CAN -- 1 par defaut chez Spintend,
d'ou --can-id. `--info` scanne le bus pour le trouver.

SECURITE, a lire avant de brancher les moteurs :
  * Le VESC s'arrete DE LUI-MEME apres ~1 s sans commande. Ce script reemet donc
    en continu a 20 Hz : lacher la touche, fermer le terminal ou perdre l'USB
    arrete les roues. C'est un homme-mort gratuit, ne pas le contourner.
  * Le plafond par defaut est un rapport cyclique de 0,12 -- volontairement
    ridicule. Le relever avec --max exige d'avoir les roues EN L'AIR.
  * Ctrl+C envoie l'arret puis relache le moteur (pas de frein), toujours.
  * On pilote en RAPPORT CYCLIQUE et non en courant : a l'arret un ordre de
    courant peut faire un a-coup violent, alors qu'un rapport cyclique faible
    donne un couple faible. Le mode courant existe (--courant) pour qui sait
    ce qu'il fait.

Ce banc est HORS de l'architecture du robot : les tondeuses passent par un
Cytron MDD10A pilote par l'ESP32. Le VESC est un essai a part, pour les roues
rapides a venir. Rien ici ne publie sur ROS.
"""
import argparse
import glob
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial manquant :  pip3 install --user pyserial")

# --- protocole -------------------------------------------------------------
# Identifiants de commande. Ceux-ci sont stables depuis des annees, mais la
# liste s'ALLONGE d'une version a l'autre : si un jour un ordre part de travers,
# verifier COMM_PACKET_ID dans datatypes.h du firmware installe.
COMM_FW_VERSION = 0
COMM_GET_VALUES = 4
COMM_SET_DUTY = 5
COMM_SET_CURRENT = 6
COMM_SET_CURRENT_BRAKE = 7
COMM_SET_RPM = 8
COMM_FORWARD_CAN = 34


def crc16(data):
    """CRC16-CCITT, polynome 0x1021, initialisation a zero -- celui du VESC."""
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encadrer(charge):
    """Enveloppe une charge utile : 0x02 pour < 256 octets, 0x03 au-dela."""
    if len(charge) < 256:
        tete = bytes([2, len(charge)])
    else:
        tete = bytes([3]) + struct.pack('>H', len(charge))
    return tete + charge + struct.pack('>H', crc16(charge)) + b'\x03'


def vers_can(can_id, charge):
    """Emballe un paquet a destination du second VESC, via le bus CAN interne."""
    return bytes([COMM_FORWARD_CAN, can_id]) + charge


def charge_duty(duty):
    return bytes([COMM_SET_DUTY]) + struct.pack('>i', int(duty * 100000))


def charge_courant(amperes):
    return bytes([COMM_SET_CURRENT]) + struct.pack('>i', int(amperes * 1000))


def charge_frein(amperes):
    return bytes([COMM_SET_CURRENT_BRAKE]) + struct.pack('>i', int(amperes * 1000))


# --- liaison ---------------------------------------------------------------
def ports_candidats():
    """Un VESC se presente en CDC ACM (STM32 : 0483:5740)."""
    trouves = []
    for chemin in sorted(glob.glob('/dev/ttyACM*')):
        vid = pid = ''
        try:
            import subprocess
            sortie = subprocess.run(['udevadm', 'info', '-q', 'property', '-n', chemin],
                                    capture_output=True, text=True, timeout=5).stdout
            for ligne in sortie.splitlines():
                if ligne.startswith('ID_VENDOR_ID='):
                    vid = ligne.split('=', 1)[1]
                elif ligne.startswith('ID_MODEL_ID='):
                    pid = ligne.split('=', 1)[1]
        except Exception:
            pass
        trouves.append((chemin, vid, pid, vid == '0483' and pid == '5740'))
    return trouves


class Vesc:
    def __init__(self, port, can_id=None, bavard=False):
        self.s = serial.Serial(port, 115200, timeout=0.25)
        self.can_id = can_id
        self.bavard = bavard

    def envoyer(self, charge, vers_second=False):
        if vers_second:
            if self.can_id is None:
                raise ValueError("aucun identifiant CAN pour le second moteur")
            charge = vers_can(self.can_id, charge)
        self.s.write(encadrer(charge))

    def lire_paquet(self, delai=0.6):
        """Lit une reponse. Rend la charge utile, ou None."""
        fin = time.time() + delai
        tampon = b''
        while time.time() < fin:
            tampon += self.s.read(256)
            while tampon:
                if tampon[0] == 2 and len(tampon) >= 2:
                    n = tampon[1]
                    total = 2 + n + 3
                    if len(tampon) >= total:
                        charge = tampon[2:2 + n]
                        recu = struct.unpack('>H', tampon[2 + n:4 + n])[0]
                        tampon = tampon[total:]
                        if recu == crc16(charge):
                            return charge
                        continue
                    break
                elif tampon[0] == 3 and len(tampon) >= 3:
                    n = struct.unpack('>H', tampon[1:3])[0]
                    total = 3 + n + 3
                    if len(tampon) >= total:
                        charge = tampon[3:3 + n]
                        recu = struct.unpack('>H', tampon[3 + n:5 + n])[0]
                        tampon = tampon[total:]
                        if recu == crc16(charge):
                            return charge
                        continue
                    break
                else:
                    tampon = tampon[1:]      # resynchronisation
        return None

    def version(self, vers_second=False):
        self.s.reset_input_buffer()
        self.envoyer(bytes([COMM_FW_VERSION]), vers_second)
        r = self.lire_paquet()
        if not r or r[0] != COMM_FW_VERSION or len(r) < 3:
            return None
        return (r[1], r[2])

    def stop(self):
        """Rapport cyclique nul sur les deux, puis frein a zero = roue libre."""
        for second in (False, True):
            if second and self.can_id is None:
                continue
            try:
                self.envoyer(charge_duty(0.0), second)
                self.envoyer(charge_frein(0.0), second)
            except Exception:
                pass

    def fermer(self):
        try:
            self.stop()
            time.sleep(0.05)
        finally:
            self.s.close()


# --- modes -----------------------------------------------------------------
def mode_liste():
    c = ports_candidats()
    if not c:
        print("  aucun /dev/ttyACM* : le VESC est-il branche et alimente ?")
        print("  (le port n'apparait QUE si la carte est alimentee, pas seulement en USB)")
        return 1
    for chemin, vid, pid, est_vesc in c:
        print("  %-16s %s:%s  %s" % (chemin, vid or '????', pid or '????',
                                     "-> VESC probable" if est_vesc else ""))
    return 0


def mode_info(v):
    fw = v.version()
    print("  maitre (USB)      : %s" % ("firmware %d.%d" % fw if fw else "PAS DE REPONSE"))
    print("  scan du bus CAN (identifiants 0 a 10) :")
    trouve = []
    for i in range(11):
        v.can_id = i
        fw = v.version(vers_second=True)
        if fw:
            print("    id %-2d -> firmware %d.%d" % (i, fw[0], fw[1]))
            trouve.append(i)
    if not trouve:
        print("    aucun second VESC vu. Sur un UBOX c'est anormal : verifier que")
        print("    les deux moitiees sont alimentees et que le CAN interne est actif.")
    else:
        print("  -> utiliser  --can-id %d  pour la seconde roue" % trouve[0])
    return 0


def envoyer_consigne(v, gauche, droite, courant):
    faire = charge_courant if courant else charge_duty
    v.envoyer(faire(gauche), False)
    if v.can_id is not None:
        v.envoyer(faire(droite), True)


def mode_impulsion(v, duty, duree, courant):
    unite = "A" if courant else "de rapport cyclique"
    print("  impulsion : %.3f %s pendant %.1f s" % (duty, unite, duree))
    fin = time.time() + duree
    while time.time() < fin:
        envoyer_consigne(v, duty, duty, courant)
        time.sleep(0.05)
    v.stop()
    print("  arrete")
    return 0


AIDE_CLAVIER = """
  ----------------------------------------------------------------
   z / s      les deux roues, avant / arriere      (maintenir)
   a / e      pivoter a gauche / a droite          (maintenir)
   q / d      roue gauche seule / roue droite seule
   + / -      regler la consigne (par pas de 0,01)
   espace     arret immediat
   0          remettre la consigne a zero
   x          quitter (arret + roue libre)
  ----------------------------------------------------------------
  Rien de presse = rien ne tourne : la consigne retombe a zero des
  qu'aucune touche n'est maintenue depuis 0,25 s.
"""


def mode_clavier(v, consigne, maxi, courant):
    import termios, tty, select
    print(AIDE_CLAVIER)
    print("  consigne %.2f   plafond %.2f   %s" %
          (consigne, maxi, "COURANT (amperes)" if courant else "rapport cyclique"))
    if v.can_id is None:
        print("  ATTENTION : pas d'identifiant CAN, SEULE la roue du maitre bougera.")
    reglages = termios.tcgetattr(sys.stdin)
    g = d = 0.0
    derniere = 0.0
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            pret, _, _ = select.select([sys.stdin], [], [], 0.05)
            if pret:
                t = sys.stdin.read(1)
                derniere = time.time()
                if t == 'x':
                    break
                elif t == 'z':   g = d = consigne
                elif t == 's':   g = d = -consigne
                elif t == 'a':   g, d = -consigne, consigne
                elif t == 'e':   g, d = consigne, -consigne
                elif t == 'q':   g, d = consigne, 0.0
                elif t == 'd':   g, d = 0.0, consigne
                elif t == ' ':   g = d = 0.0; derniere = 0.0
                elif t == '0':   consigne = 0.0
                elif t in '+=':  consigne = min(maxi, round(consigne + 0.01, 3))
                elif t == '-':   consigne = max(0.0, round(consigne - 0.01, 3))
                sys.stdout.write("\r  consigne %.2f   gauche %+.2f  droite %+.2f      "
                                 % (consigne, g, d))
                sys.stdout.flush()
            # homme-mort : sans touche depuis 0,25 s, on retombe a zero
            if time.time() - derniere > 0.25:
                g = d = 0.0
            envoyer_consigne(v, g, d, courant)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, reglages)
        v.stop()
        print("\n  arrete, roues libres")
    return 0


def autotest():
    """Verifie l'encodage sans materiel : c'est la seule chose testable a sec."""
    ok = True

    def verifier(nom, obtenu, attendu):
        nonlocal ok
        bon = obtenu == attendu
        ok &= bon
        print("  %-42s %s" % (nom, "ok" if bon else
              "ECHEC\n      obtenu  %s\n      attendu %s" % (obtenu.hex(), attendu.hex())))

    # CRC : vecteur de reference du CCITT (init 0), "123456789" -> 0x31C3
    c = crc16(b'123456789')
    ok &= (c == 0x31C3)
    print("  %-42s %s" % ("CRC16-CCITT sur '123456789' = 0x31C3", "ok" if c == 0x31C3 else "ECHEC (0x%04X)" % c))

    verifier("trame COMM_FW_VERSION",
             encadrer(bytes([COMM_FW_VERSION])),
             bytes([2, 1, 0]) + struct.pack('>H', crc16(bytes([0]))) + b'\x03')

    # duty 0,5 -> 50000 = 0x0000C350
    verifier("SET_DUTY 0,5",
             charge_duty(0.5), bytes([5]) + bytes.fromhex('0000c350'))
    verifier("SET_DUTY -0,1 (marche arriere)",
             charge_duty(-0.1), bytes([5]) + struct.pack('>i', -10000))
    verifier("SET_CURRENT 3,5 A",
             charge_courant(3.5), bytes([6]) + struct.pack('>i', 3500))
    verifier("emballage CAN vers l'id 1",
             vers_can(1, charge_duty(0.0)),
             bytes([34, 1, 5]) + struct.pack('>i', 0))

    # longueur totale d'une trame courte : 2 en-tete + n + 2 crc + 1 fin
    t = encadrer(charge_duty(0.25))
    verifier("longueur de trame SET_DUTY", bytes([len(t)]), bytes([10]))
    verifier("octet de fin", t[-1:], b'\x03')

    # aller-retour : on encadre puis on redecode comme le ferait lire_paquet
    charge = charge_duty(-0.42)
    t = encadrer(charge)
    redecode = t[2:2 + t[1]]
    verifier("aller-retour encadrer/decoder", redecode, charge)

    print("\n  %s" % ("TOUT PASSE" if ok else "AU MOINS UN ECHEC"))
    return 0 if ok else 1


def main():
    p = argparse.ArgumentParser(description="Banc d'essai UBOX dual VESC.")
    p.add_argument('--port', help="/dev/ttyACMx ; detecte seul si omis")
    p.add_argument('--can-id', type=int, default=None,
                   help="identifiant CAN du second VESC (voir --info)")
    p.add_argument('--consigne', type=float, default=0.05,
                   help="consigne de depart (defaut 0,05)")
    p.add_argument('--max', type=float, default=0.12,
                   help="PLAFOND. Ne le relever qu'avec les roues en l'air.")
    p.add_argument('--courant', action='store_true',
                   help="piloter en amperes plutot qu'en rapport cyclique")
    p.add_argument('--duty', type=float, help="impulsion : consigne")
    p.add_argument('--duree', type=float, default=1.0, help="impulsion : duree en s")
    p.add_argument('--list', action='store_true')
    p.add_argument('--info', action='store_true')
    p.add_argument('--autotest', action='store_true')
    a = p.parse_args()

    if a.autotest:
        return autotest()
    if a.list:
        return mode_liste()

    port = a.port
    if not port:
        c = [x for x in ports_candidats() if x[3]] or ports_candidats()
        if not c:
            return mode_liste()
        port = c[0][0]
        print("  port retenu : %s" % port)

    if a.duty is not None and abs(a.duty) > a.max and not a.courant:
        return p.error("--duty %.3f depasse le plafond %.3f (relever --max en connaissance de cause)"
                       % (a.duty, a.max))

    try:
        v = Vesc(port, a.can_id)
    except Exception as e:
        return print("  ouverture de %s impossible : %s" % (port, e)) or 1

    try:
        if a.info:
            return mode_info(v)
        if v.version() is None:
            print("  ATTENTION : le VESC ne repond pas a une demande de version.")
            print("  Le port existe mais rien ne parle : carte alimentee ? bon port ?")
            return 1
        if a.duty is not None:
            return mode_impulsion(v, a.duty, a.duree, a.courant)
        return mode_clavier(v, min(a.consigne, a.max), a.max, a.courant)
    finally:
        v.fermer()


if __name__ == '__main__':
    sys.exit(main())
