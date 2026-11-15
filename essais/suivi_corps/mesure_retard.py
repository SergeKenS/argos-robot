"""Banc de mesure : retard entre la caméra et les encodeurs, et pixels par degré.

Tourelle immobile au centre, le corps pivote à gauche et à droite (environ ±12°).
À chaque image on note où est l'objet vert (s'il est vu) et de combien le décor a
glissé depuis l'image précédente. On cherche ensuite le retard qui fait le mieux
coller ces relevés au lacet donné par les encodeurs :
    position de l'objet dans l'image = constante + pixels_par_degré x lacet(heure - retard)

Résultat attendu : un retard proche de celui mesuré sur la tourelle seule (0,08 s).
Sans cette mesure, la tourelle compenserait la rotation du corps trop tôt ou trop tard.

Usage (sur le Pi, 1 m libre autour du robot, objet vert à 1 ou 2 m devant) :
  python3 mesure_retard.py
"""
import math
import signal
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'tourelle'))
from suivi_fluide import CENTRE_BAS, CENTRE_HAUT, Camera, Tourelle, trouver_objet  # noqa: E402

from base import Base  # noqa: E402

DUREE = 12.0          # s
AMPLITUDE = 0.7       # rad/s : vitesse de rotation maximale du corps
PERIODE = 2.2         # s : durée d'un aller-retour


def demande_d_arret(*_):
    raise KeyboardInterrupt


def ajuster(heures, valeurs, base, retard, par_difference):
    """Droite valeurs = a + k x lacet(heure - retard) ; renvoie (k en px/degré, écart type en px)."""
    lacets = np.array([math.degrees(base.lacet(h - retard)) for h in heures])
    y = np.array(valeurs)
    if par_difference:
        # Glissement entre deux images contre rotation entre ces deux images
        lacets = np.diff(lacets, prepend=math.degrees(base.lacet(heures[0] - retard - 1 / 30)))
        k = float(np.sum(lacets * y) / np.sum(lacets * lacets))
        return k, float(np.std(y - k * lacets))
    k, a = np.polyfit(lacets, y, 1)
    return float(k), float(np.std(y - (a + k * lacets)))


def meilleur_retard(heures, valeurs, base, par_difference):
    essais = [(ajuster(heures, valeurs, base, r, par_difference), r)
              for r in np.arange(0.0, 0.3001, 0.005)]
    (k, ecart), retard = min(essais, key=lambda e: e[0][1])
    return retard, k, ecart


def main():
    for signe in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(signe, demande_d_arret)
    tourelle = Tourelle()
    tourelle.viser(CENTRE_BAS, CENTRE_HAUT)
    camera = Camera(0)
    base = Base()
    fenetre = cv2.createHanningWindow((320, 240), cv2.CV_32F)
    objets, decor = [], []      # (heure, x de l'objet) ; (heure, glissement du décor)
    precedente = None
    numero = 0
    try:
        time.sleep(0.5)
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < DUREE:
            t = time.perf_counter() - t0
            # Va-et-vient, plus un second rythme pour que le mouvement ne se répète pas
            rotation = AMPLITUDE * (0.7 * math.sin(2 * math.pi * t / PERIODE)
                                    + 0.3 * math.sin(2 * math.pi * t / (PERIODE * 0.37)))
            base.commander(0.0, rotation if t < DUREE - 1 else 0.0)
            numero, heure, image = camera.suivante(numero)
            objet = trouver_objet(image)
            if objet is not None:
                objets.append((heure, objet[0]))
            petite = cv2.cvtColor(cv2.resize(image, (320, 240)), cv2.COLOR_BGR2GRAY).astype(np.float32)
            if precedente is not None:
                (dx, _), force = cv2.phaseCorrelate(precedente, petite, fenetre)
                if force > 0.05:
                    decor.append((heure, dx * 2))     # image réduite de moitié
            precedente = petite
        time.sleep(0.3)
        images = numero
        print(f'{images} images en {DUREE:.0f} s, objet vu sur {len(objets)}, décor mesuré sur {len(decor)}')
        debattement = [math.degrees(base.lacet(t0 + i * 0.05)) for i in range(int(DUREE / 0.05))]
        print(f'Lacet du corps : de {min(debattement):+.1f} à {max(debattement):+.1f} deg')
        if len(objets) > 60:
            retard, k, ecart = meilleur_retard(*zip(*objets), base, False)
            print(f'Objet vert : retard {retard:.3f} s, {k:+.2f} px par degré de lacet, écart restant {ecart:.1f} px')
            for r in (0.0, 0.04, 0.08, 0.12, 0.16):
                print(f'    avec un retard de {r:.2f} s : écart {ajuster(*zip(*objets), base, r, False)[1]:.1f} px')
        if len(decor) > 60:
            retard, k, ecart = meilleur_retard(*zip(*decor), base, True)
            print(f'Décor      : retard {retard:.3f} s, {k:+.2f} px par degré de lacet, écart restant {ecart:.1f} px par image')
    except KeyboardInterrupt:
        print('Arrêt demandé')
    finally:
        base.fermer()
        camera.fermer()
        tourelle.fermer()


if __name__ == '__main__':
    main()
