#!/usr/bin/env python3
"""Teste un lidar LDRobot LD14/LD06 sur tous les ports serie disponibles.
Cherche l'en-tete de trame 0x54 et decode distances + vitesse de rotation."""
import glob, struct, sys, time
try:
    import serial
except ImportError:
    print("pyserial absent"); sys.exit(2)

ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
if not ports:
    print("AUCUN PORT SERIE -> le lidar n'est pas connecte electriquement.")
    sys.exit(1)
print("ports trouves :", ", ".join(ports))

for port in ports:
    for baud in (115200, 230400):
        print(f"\n=== {port} @ {baud} bauds ===")
        try:
            s = serial.Serial(port, baud, timeout=1)
        except Exception as e:
            print("  ouverture impossible :", e); continue
        time.sleep(0.3); s.reset_input_buffer()
        raw = b''
        t0 = time.time()
        while time.time() - t0 < 3 and len(raw) < 8000:
            raw += s.read(2048)
        s.close()
        print(f"  {len(raw)} octets recus en 3 s")
        if not raw:
            print("  MUET"); continue
        n = raw.count(b'\x54')
        print(f"  en-tetes 0x54 : {n}")
        # trame LD14/LD06 : 0x54 0x2C, 47 octets, 12 mesures
        ok = 0; dists = []; speeds = []
        i = 0
        while i < len(raw) - 47:
            if raw[i] == 0x54 and raw[i+1] == 0x2C:
                f = raw[i:i+47]
                speed = struct.unpack('<H', f[2:4])[0]
                for k in range(12):
                    d = struct.unpack('<H', f[6+k*3:8+k*3])[0]
                    if 0 < d < 12000: dists.append(d)
                speeds.append(speed); ok += 1
                i += 47
            else:
                i += 1
        print(f"  trames valides : {ok}")
        if ok:
            print(f"  vitesse rotation : {sum(speeds)/len(speeds)/360:.1f} Hz")
            if dists:
                print(f"  mesures : {len(dists)}  min {min(dists)/1000:.2f} m  max {max(dists)/1000:.2f} m")
            print("  >>> LIDAR FONCTIONNEL <<<")
            sys.exit(0)
        else:
            print("  donnees recues mais aucune trame LD14 reconnue")
sys.exit(3)
