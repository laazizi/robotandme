#!/usr/bin/env python3
"""Mesure les performances reelles d'un LD14 : debit, couverture, bruit."""
import struct, time, math
import serial, numpy as np

s = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(0.3); s.reset_input_buffer()
DUR = 8.0
raw = b''
t0 = time.time()
while time.time() - t0 < DUR:
    raw += s.read(4096)
s.close()
el = time.time() - t0
print(f"lu {len(raw)} octets en {el:.1f} s  ({len(raw)/el/1024:.1f} ko/s)")

pts = []          # (angle_deg, dist_m, intensite)
speeds = []
i = 0
frames = 0
while i < len(raw) - 47:
    if raw[i] == 0x54 and raw[i+1] == 0x2C:
        f = raw[i:i+47]
        speed = struct.unpack('<H', f[2:4])[0]      # deg/s
        a0 = struct.unpack('<H', f[4:6])[0] / 100.0
        a1 = struct.unpack('<H', f[42:44])[0] / 100.0
        da = (a1 - a0) % 360.0
        for k in range(12):
            d = struct.unpack('<H', f[6+k*3:8+k*3])[0]
            inten = f[8+k*3]
            ang = (a0 + da * k / 11.0) % 360.0
            pts.append((ang, d/1000.0, inten))
        speeds.append(speed); frames += 1
        i += 47
    else:
        i += 1

print(f"trames : {frames}   mesures : {len(pts)}")
print(f"debit reel : {len(pts)/el:.0f} points/s")
print(f"rotation   : {np.mean(speeds)/360:.2f} Hz")
print(f"-> points par tour : {len(pts)/el/(np.mean(speeds)/360):.0f}")

d = np.array([p[1] for p in pts])
a = np.array([p[0] for p in pts])
inten = np.array([p[2] for p in pts])
valid = d > 0
print(f"\nmesures valides : {valid.sum()} / {len(d)}  ({100*valid.sum()/len(d):.1f} %)")
print(f"distance   : min {d[valid].min():.2f} m   max {d[valid].max():.2f} m   moy {d[valid].mean():.2f} m")
print(f"intensite  : moy {inten[valid].mean():.0f}")
print(f"couverture angulaire : {a.min():.0f} a {a.max():.0f} deg")

# bruit : dispersion des mesures dans un meme secteur de 1 degre
print("\n--- BRUIT (dispersion par secteur de 1 deg, secteurs vus >=4 fois) ---")
sect = (a[valid]).astype(int)
dv = d[valid]
stds = []
for k in range(360):
    m = sect == k
    if m.sum() >= 4:
        stds.append(dv[m].std())
if stds:
    stds = np.array(stds)
    print(f"secteurs analyses : {len(stds)}")
    print(f"ecart-type median : {np.median(stds)*100:.1f} cm")
    print(f"secteurs > 10 cm  : {(stds>0.10).sum()} ({100*(stds>0.10).sum()/len(stds):.1f} %)")
