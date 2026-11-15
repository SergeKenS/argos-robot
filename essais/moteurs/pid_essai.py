"""Compare des réglages du régulateur de la carte ($MPID:P,I,D#) sur la même épreuve :
rotation à gauche depuis l'arrêt, puis inversion à droite, à ±V mm/s par roue.
Pour chaque réglage : temps avant le départ, temps pour atteindre 90 %, dépassement,
irrégularité à vitesse établie. Remet le réglage d'origine à la fin.

Usage : python3 pid_essai.py <V mm/s> P,I,D [P,I,D ...]
"""
import math, re, statistics, sys, time
import serial
from serial.tools import list_ports

ORIGINE = '0.8,0.06,0.5'
M = math.pi * 0.067 / 1760
MOTIF = re.compile(rb'\$MAll:(-?\d+),(-?\d+),(-?\d+),(-?\d+)#')
V = float(sys.argv[1])
DUREE = 3.5

port = next(p.device for p in list_ports.comports() if (p.vid, p.pid) == (0x1A86, 0x7522))
l = serial.Serial(); l.port, l.baudrate, l.timeout = port, 115200, 0.01
l.dtr = l.rts = False; l.exclusive = True; l.open(); time.sleep(0.5); l.reset_input_buffer()
tampon = b''


def lire(jusqua, rel):
    global tampon
    while time.monotonic() < jusqua:
        tampon += l.read(256)
        while True:
            m = MOTIF.search(tampon)
            if m is None:
                break
            tampon = tampon[m.end():]
            c = [int(v) for v in m.groups()]
            rel.append((time.monotonic(), -c[3] * M * 1000, c[2] * M * 1000))


def vitesses(rel, pas=0.05):
    """Vitesse de rotation des roues (droite - gauche) / 2 en mm/s, par tranches de `pas`."""
    sortie, prec, debut = [], rel[0], rel[0][0]
    for r in rel:
        if r[0] - debut >= pas:
            dt = r[0] - prec[0]
            sortie.append((r[0], ((r[2] - prec[2]) - (r[1] - prec[1])) / 2 / dt))
            prec, debut = r, r[0]
    return sortie


def phase(sens):
    """Applique ±V (sens +1 = gauche) pendant DUREE ; renvoie (départ, 90 %, dépassement %, irrégularité %)."""
    rel = []
    lire(time.monotonic() + 0.1, rel)
    t0 = time.monotonic()
    l.write(f'$spd:0,0,{round(sens * V)},{round(sens * V)}#'.encode())
    lire(t0 + DUREE, rel)
    v = [(t - t0, sens * w) for t, w in vitesses(rel) if t > t0]
    depart = next((t for t, w in v if w > 0.15 * V), None)
    t90 = next((t for t, w in v if w > 0.9 * V), None)
    etabli = [w for t, w in v if t90 is not None and t > t90]
    depassement = (max(etabli) / V - 1) * 100 if etabli else float('nan')
    irregularite = statistics.pstdev(etabli) / V * 100 if len(etabli) > 3 else float('nan')
    return depart, t90, depassement, irregularite


def f(x):
    return '  -- ' if x is None else f'{x:5.2f}'


try:
    l.write(b'$upload:1,0,0#')
    print(f'Rotation ±{V:.0f} mm/s par roue. Colonnes : départ (s), 90 % (s), dépassement %, irrégularité %')
    print('  P,I,D            | depuis l\'arrêt             | après inversion')
    for reglage in sys.argv[2:]:
        l.write(f'$MPID:{reglage}#'.encode())
        time.sleep(0.5)
        # Après un réglage, la carte cesse d'envoyer ses compteurs : on les redemande
        essai = []
        while not essai:
            l.write(b'$upload:1,0,0#')
            lire(time.monotonic() + 0.3, essai)
        a = phase(+1)
        b = phase(-1)
        l.write(b'$pwm:0,0,0,0#')
        time.sleep(1.2)
        print(f'  {reglage:16s} | {f(a[0])} {f(a[1])} {a[2]:+5.0f} {a[3]:5.0f}     | {f(b[0])} {f(b[1])} {b[2]:+5.0f} {b[3]:5.0f}')
finally:
    l.write(b'$pwm:0,0,0,0#'); time.sleep(0.1)
    l.write(f'$MPID:{ORIGINE}#'.encode()); time.sleep(0.3)
    l.write(b'$upload:0,0,0#'); l.close()
