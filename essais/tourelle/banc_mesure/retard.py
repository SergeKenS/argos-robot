"""Mesure le retard entre la position annoncée par l'ESP32 et ce que montre l'image.

La tourelle oscille (sinus) devant la balle immobile ; on cherche le retard qui
fait le mieux coller la visée vue par la caméra à la position annoncée.
Usage : python retard.py axe [backend codec fps]
"""
import math
import sys
import time

import cv2
import numpy as np

from outils import CENTRE, Camera, Tourelle, trouver_balle

AXE = int(sys.argv[1]) if len(sys.argv) > 1 else 0
PX_PAR_DEGRE = (7.35, 8.7)[AXE]
tourelle = Tourelle()
tourelle.envoyer('J,2.4,2.1')
if len(sys.argv) > 2:
    camera = Camera(backend=getattr(cv2, sys.argv[2]), codec=sys.argv[3], fps=int(sys.argv[4]))
else:
    camera = Camera()
numero = 0


def essai(frequence, amplitude, duree=6):
    global numero
    vues = []  # (heure image, position balle px)
    ratees = 0
    debut = time.perf_counter()
    while (t := time.perf_counter() - debut) < duree:
        cible = list(CENTRE)
        cible[AXE] += amplitude * math.sin(2 * math.pi * frequence * t)
        tourelle.viser(*cible)
        numero, heure, image = camera.suivante(numero)
        balle = trouver_balle(image)
        if balle:
            vues.append((heure, balle[AXE]))
        else:
            ratees += 1
    vues = np.array(vues[15:])  # laisse passer le démarrage
    histo = np.array([(h, p[AXE]) for h, *p in tourelle.historique])

    meilleur = None
    for retard in np.arange(0.0, 0.300, 0.002):
        annonce = np.interp(vues[:, 0] - retard, histo[:, 0], histo[:, 1])
        # balle_px = pente * angle + origine (la pente doit valoir environ PX_PAR_DEGRE)
        A = np.vstack([annonce, np.ones_like(annonce)]).T
        (pente, origine), *_ = np.linalg.lstsq(A, vues[:, 1], rcond=None)
        erreur = np.sqrt(np.mean((A @ [pente, origine] - vues[:, 1]) ** 2))
        if meilleur is None or erreur < meilleur[1]:
            meilleur = (retard, erreur, pente)
    retard, erreur, pente = meilleur
    cadence = (len(vues) - 1) / (vues[-1, 0] - vues[0, 0])
    print(f'{frequence} Hz, ±{amplitude}° (vitesse max {2 * math.pi * frequence * amplitude:.0f} °/s) : '
          f'retard {1000 * retard:.0f} ms, écart restant {erreur:.1f} px ({erreur / PX_PAR_DEGRE:.2f}°), '
          f'{pente:.2f} px par degré, {cadence:.0f} images/s utiles, {ratees} images sans balle')


print(f'Axe {("bas", "haut")[AXE]}')
essai(0.5, 8)
essai(1.0, 8)
essai(2.0, 5)
essai(3.0, 4)
camera.fermer()
tourelle.fermer()
