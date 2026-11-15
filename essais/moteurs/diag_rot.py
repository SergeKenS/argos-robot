"""Diagnostic de la rotation : pilote la carte moteurs en direct (sans ROS) et
enregistre les deux roues à pleine cadence, pour voir QUAND et COMMENT elles calent.

Usage : python3 diag_rot.py <gauche mm/s> <droite mm/s> <durée s> [fichier.csv]
  vitesses dans le sens du robot (positif = marche avant)
  ex. rotation à gauche : python3 diag_rot.py -100 100 4
  ex. essai à blanc (rien ne bouge) : python3 diag_rot.py 0 0 2
Arrêter d'abord pont_moteurs (un seul programme à la fois sur le port).
"""
import math
import re
import signal
import sys
import time

import serial
from serial.tools import list_ports

ENTRAXE = 0.119                      # m
M_PAR_IMP = math.pi * 0.067 / 1760   # m par impulsion
MOTIF = re.compile(rb'\$MAll:(-?\d+),(-?\d+),(-?\d+),(-?\d+)#')


def arret(*_):
    raise KeyboardInterrupt


def main():
    gauche, droite, duree = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    fichier = sys.argv[4] if len(sys.argv) > 4 else None
    for s in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(s, arret)
    port = next(p.device for p in list_ports.comports() if (p.vid, p.pid) == (0x1A86, 0x7522))
    liaison = serial.Serial()
    liaison.port, liaison.baudrate, liaison.timeout = port, 115200, 0.01
    liaison.dtr = liaison.rts = False
    liaison.exclusive = True
    liaison.open()
    time.sleep(0.5)
    liaison.reset_input_buffer()
    liaison.write(b'$upload:1,0,0#')

    releves = []   # (t, gauche m, droite m)
    tampon = b''

    def lire(jusqua):
        nonlocal tampon
        while time.monotonic() < jusqua:
            tampon += liaison.read(256)
            while True:
                m = MOTIF.search(tampon)
                if m is None:
                    break
                tampon = tampon[m.end():]
                c = [int(v) for v in m.groups()]
                # Motor 4 = gauche (miroir), Motor 3 = droite
                releves.append((time.monotonic(), -c[3] * M_PAR_IMP, c[2] * M_PAR_IMP))

    try:
        lire(time.monotonic() + 0.5)
        if not releves:
            liaison.write(b'$upload:1,0,0#')
            lire(time.monotonic() + 0.5)
        t0 = time.monotonic()
        if gauche or droite:
            liaison.write(f'$spd:0,0,{round(droite)},{round(-gauche)}#'.encode())
        lire(t0 + duree)
        liaison.write(b'$pwm:0,0,0,0#')
        t1 = time.monotonic()
        lire(t1 + 1.0)
    finally:
        liaison.write(b'$pwm:0,0,0,0#')
        time.sleep(0.05)
        liaison.write(b'$upload:0,0,0#')
        liaison.close()

    if fichier:
        with open(fichier, 'w') as f:
            f.write('t,gauche_m,droite_m\n')
            for t, g, d in releves:
                f.write(f'{t - t0:.4f},{g:.5f},{d:.5f}\n')

    # Bilan par tranches de 0,25 s : vitesse de chaque roue et lacet cumulé
    avant = [r for r in releves if r[0] <= t0]
    ref = avant[-1] if avant else releves[0]
    print(f'{len(releves)} relevés ({len(releves) / (releves[-1][0] - releves[0][0]):.0f}/s), '
          f'consigne gauche {gauche:+.0f} droite {droite:+.0f} mm/s pendant {duree} s')
    print('  t (s)   gauche  droite (mm/s)   lacet cumulé (deg)')
    pas, debut, prec = 0.25, t0, ref
    while debut < releves[-1][0]:
        tranche = [r for r in releves if debut < r[0] <= debut + pas]
        if tranche:
            fin = tranche[-1]
            dt = fin[0] - prec[0]
            vg, vd = (fin[1] - prec[1]) / dt * 1000, (fin[2] - prec[2]) / dt * 1000
            lacet = math.degrees(((fin[2] - ref[2]) - (fin[1] - ref[1])) / ENTRAXE)
            marque = '  <- arrêt' if debut <= t1 < debut + pas else ''
            print(f'  {debut - t0 + pas:5.2f}  {vg:+7.0f} {vd:+7.0f}        {lacet:+8.1f}{marque}')
            prec = fin
        debut += pas
    fin = releves[-1]
    print(f'Total : gauche {(fin[1] - ref[1]) * 1000:+.0f} mm, droite {(fin[2] - ref[2]) * 1000:+.0f} mm, '
          f'lacet {math.degrees(((fin[2] - ref[2]) - (fin[1] - ref[1])) / ENTRAXE):+.1f} deg, '
          f'avance {((fin[2] - ref[2]) + (fin[1] - ref[1])) / 2 * 1000:+.0f} mm')


if __name__ == '__main__':
    main()
