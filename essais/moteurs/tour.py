"""Vérifie la rotation réelle avec la caméra : photo, rotation d'environ <angle>
degrés mesurée par les encodeurs, photo, puis décalage entre les deux images.
Après un tour complet, les deux images doivent se superposer : le décalage restant
donne l'erreur réelle des encodeurs (glissement, entraxe).

Usage : python3 tour.py <angle deg, + = gauche> [vitesse roue mm/s]
"""
import math
import re
import sys
import time

import cv2
import numpy as np
import serial
from serial.tools import list_ports

ENTRAXE = 0.119
M_PAR_IMP = math.pi * 0.067 / 1760
PX_PAR_DEGRE = 7.6
MOTIF = re.compile(rb'\$MAll:(-?\d+),(-?\d+),(-?\d+),(-?\d+)#')


def port(pid):
    return next(p.device for p in list_ports.comports() if (p.vid, p.pid) == (0x1A86, pid))


def photo(camera):
    for _ in range(8):   # vide les images en attente
        ok, image = camera.read()
    gris = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return cv2.createCLAHE(3.0, (8, 8)).apply(gris)


def decalage(a, b):
    """Glissement horizontal médian (px) des détails de l'image a vers l'image b."""
    orb = cv2.ORB_create(1500)
    ka, da = orb.detectAndCompute(a, None)
    kb, db = orb.detectAndCompute(b, None)
    paires = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(da, db)
    paires = sorted(paires, key=lambda p: p.distance)[:150]
    dx = np.array([kb[p.trainIdx].pt[0] - ka[p.queryIdx].pt[0] for p in paires])
    dy = np.array([kb[p.trainIdx].pt[1] - ka[p.queryIdx].pt[1] for p in paires])
    med = np.median(dx)
    proches = int(np.sum(np.abs(dx - med) < 8))
    return med, np.median(dy), proches, len(paires)


def main():
    angle = float(sys.argv[1])
    vitesse = float(sys.argv[2]) if len(sys.argv) > 2 else 100
    sens = 1 if angle > 0 else -1

    tourelle = serial.Serial(port(0x7523), 115200, timeout=0.05)
    time.sleep(2)                      # l'ESP32 redémarre à l'ouverture
    tourelle.write(b'105,30\n')
    camera = cv2.VideoCapture(0)
    camera.set(3, 640)
    camera.set(4, 480)
    time.sleep(1)
    avant = photo(camera)

    carte = serial.Serial()
    carte.port, carte.baudrate, carte.timeout = port(0x7522), 115200, 0.01
    carte.dtr = carte.rts = False
    carte.exclusive = True
    carte.open()
    time.sleep(0.5)
    carte.reset_input_buffer()
    carte.write(b'$upload:1,0,0#')
    tampon, depart, lacet = b'', None, 0.0

    def lire(duree):
        nonlocal tampon, depart, lacet
        fin = time.monotonic() + duree
        while time.monotonic() < fin:
            tampon += carte.read(256)
            trouves = MOTIF.findall(tampon)
            if trouves:
                tampon = tampon[tampon.rfind(b'#') + 1:]
                c = [int(v) for v in trouves[-1]]
                gauche, droite = -c[3] * M_PAR_IMP, c[2] * M_PAR_IMP
                if depart is None:
                    depart = (gauche, droite)
                lacet = math.degrees(((droite - depart[1]) - (gauche - depart[0])) / ENTRAXE)

    try:
        lire(0.5)
        carte.write(f'$spd:0,0,{round(sens * vitesse)},{round(sens * vitesse)}#'.encode())
        t0 = time.monotonic()
        # Arrêt un peu avant : le robot glisse encore de quelques degrés
        while abs(lacet) < abs(angle) - vitesse * 0.09 and time.monotonic() - t0 < 30:
            lire(0.01)
        carte.write(b'$pwm:0,0,0,0#')
        duree = time.monotonic() - t0
        lire(1.5)
    finally:
        carte.write(b'$pwm:0,0,0,0#')
        time.sleep(0.05)
        carte.write(b'$upload:0,0,0#')
        carte.close()

    apres = photo(camera)
    cv2.imwrite('/tmp/tour_avant.jpg', avant)
    cv2.imwrite('/tmp/tour_apres.jpg', apres)
    dx, dy, proches, total = decalage(avant, apres)
    camera.release()
    tourelle.close()
    # La caméra tourne à gauche : les détails glissent vers la droite (dx > 0)
    reste = dx / PX_PAR_DEGRE
    print(f'encodeurs {lacet:+.1f} deg en {duree:.1f} s | image : dx {dx:+.1f} px, dy {dy:+.1f} px '
          f'({proches}/{total} points d\'accord) -> reste réel {reste:+.1f} deg')
    tours = round(lacet / 360)
    if tours:
        reel = 360 * tours + reste
        print(f'rotation réelle {reel:+.1f} deg, rapport réel/encodeurs {reel / lacet:.3f}, '
              f'entraxe effectif {ENTRAXE * lacet / reel * 1000:.1f} mm')


if __name__ == '__main__':
    main()
