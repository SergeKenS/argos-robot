"""Suite de consignes enchaînées sans arrêt, roues enregistrées par tranches de 0,1 s.
Usage : python3 seq.py g,d,duree g,d,duree ...   (mm/s sens robot, s)"""
import math, re, sys, time
import serial
from serial.tools import list_ports
ENTRAXE, M = 0.119, math.pi * 0.067 / 1760
MOTIF = re.compile(rb'\$MAll:(-?\d+),(-?\d+),(-?\d+),(-?\d+)#')
etapes = [tuple(float(v) for v in a.split(',')) for a in sys.argv[1:]]
port = next(p.device for p in list_ports.comports() if (p.vid, p.pid) == (0x1A86, 0x7522))
l = serial.Serial(); l.port, l.baudrate, l.timeout = port, 115200, 0.01
l.dtr = l.rts = False; l.exclusive = True; l.open(); time.sleep(0.5); l.reset_input_buffer()
l.write(b'$upload:1,0,0#')
rel, tampon = [], b''
def lire(jusqua):
    global tampon
    while time.monotonic() < jusqua:
        tampon += l.read(256)
        while True:
            m = MOTIF.search(tampon)
            if m is None: break
            tampon = tampon[m.end():]
            c = [int(v) for v in m.groups()]
            rel.append((time.monotonic(), -c[3] * M, c[2] * M))
changements = []
try:
    lire(time.monotonic() + 0.5)
    t0 = t = time.monotonic()
    for g, d, duree in etapes:
        l.write(f'$spd:0,0,{round(d)},{round(-g)}#'.encode())
        changements.append((t - t0, g, d)); t += duree; lire(t)
    l.write(b'$pwm:0,0,0,0#'); changements.append((t - t0, 0, 0)); lire(t + 0.8)
finally:
    l.write(b'$pwm:0,0,0,0#'); time.sleep(0.05); l.write(b'$upload:0,0,0#'); l.close()
pas, debut = 0.1, t0
prec = [r for r in rel if r[0] <= t0][-1]
while debut < rel[-1][0]:
    tr = [r for r in rel if debut < r[0] <= debut + pas]
    if tr:
        f = tr[-1]; dt = f[0] - prec[0]
        note = ''.join(f'  <- consigne {g:+.0f} {d:+.0f}' for tc, g, d in changements if debut - t0 <= tc < debut - t0 + pas)
        print(f'{debut - t0 + pas:5.1f}  {(f[1]-prec[1])/dt*1000:+6.0f} {(f[2]-prec[2])/dt*1000:+6.0f}{note}')
        prec = f
    debut += pas
