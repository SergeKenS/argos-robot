"""Étalonnage par le robot lui-même, objet vert immobile devant lui (30 à 60 cm).

Deux grandeurs manquent pour que la tête compense bien les mouvements du corps :
  1. la distance de l'objet, estimée par sa taille dans l'image :
         distance = K_DISTANCE / rayon_en_pixels
  2. l'avance de la caméra sur l'axe des roues (AVANCE_CAMERA, en mètres). Quand
     le corps pivote, la caméra se déplace aussi de côté : un objet proche glisse
     donc plus dans l'image qu'un objet lointain :
         glissement = px_par_degré x (1 + AVANCE_CAMERA / distance)

Méthode, sans mètre ruban : tourelle immobile, le robot pivote devant l'objet,
recule de RECUL mètres (mesurés par les encodeurs) en notant le rayon de l'objet,
pivote de nouveau, puis revient.
  - rayon contre recul            -> K_DISTANCE et la distance de départ
  - glissement aux deux distances -> px par degré et AVANCE_CAMERA

Usage (sur le Pi, 60 cm libres derrière le robot) :  python3 etalonnage.py
"""
import math
import signal
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'tourelle'))
from suivi_fluide import CENTRE_BAS, CENTRE_HAUT, Camera, Tourelle, trouver_objet  # noqa: E402

from base import Base  # noqa: E402
from mesure_retard import meilleur_retard  # noqa: E402

RECUL = 0.25          # m
VITESSE = 0.08        # m/s
AMPLITUDE = 0.5       # rad/s : vitesse de rotation maximale pendant le va-et-vient
PERIODE = 2.2         # s
DUREE_PIVOT = 6.6     # s : trois va-et-vient


def demande_d_arret(*_):
    raise KeyboardInterrupt


class Banc:
    def __init__(self):
        self.tourelle = Tourelle()
        self.tourelle.viser(CENTRE_BAS, CENTRE_HAUT)
        self.camera = Camera(0)
        self.base = Base()
        self.numero = 0
        time.sleep(1.0)      # laisse la caméra régler sa luminosité

    def image(self):
        """Attend une image ; renvoie (heure, objet ou None)."""
        self.numero, heure, image = self.camera.suivante(self.numero)
        return heure, trouver_objet(image)

    def pivoter(self):
        """Va-et-vient en rotation ; renvoie (px par degré, retard, écart, rayon médian)."""
        releves, rayons = [], []
        t0 = time.perf_counter()
        while (t := time.perf_counter() - t0) < DUREE_PIVOT:
            # Cosinus : le corps oscille de part et d'autre de son orientation de départ
            self.base.commander(0.0, AMPLITUDE * math.cos(2 * math.pi * t / PERIODE))
            heure, objet = self.image()
            if objet is not None:
                releves.append((heure, objet[0]))
                rayons.append(objet[2])
        self.arreter()
        retard, k, ecart = meilleur_retard(*zip(*releves), self.base, False)
        return k, retard, ecart, float(np.median(rayons))

    def rouler(self, distance):
        """Avance (ou recule) en ligne droite ; renvoie les (déplacement, rayon) relevés."""
        releves = []
        depart = self.base.etat()[3]
        sens = 1 if distance > 0 else -1
        while abs(self.base.etat()[3] - depart) < abs(distance):
            self.base.commander(sens * VITESSE, 0.0)
            heure, objet = self.image()
            if objet is not None:
                releves.append((self.base.etat(heure - 0.03)[3] - depart, objet[2]))
        self.arreter()
        return releves

    def arreter(self):
        fin = time.perf_counter() + 0.8
        while time.perf_counter() < fin:
            self.base.commander(0.0, 0.0)
            time.sleep(0.02)

    def fermer(self):
        self.base.fermer()
        self.camera.fermer()
        self.tourelle.fermer()


def main():
    for signe in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(signe, demande_d_arret)
    banc = Banc()
    try:
        k_pres, retard_pres, ecart_pres, rayon_pres = banc.pivoter()
        print(f'Près : {k_pres:.2f} px par degré (retard {retard_pres:.3f} s, écart {ecart_pres:.1f} px), rayon {rayon_pres:.1f} px')
        recul = banc.rouler(-RECUL)
        k_loin, retard_loin, ecart_loin, rayon_loin = banc.pivoter()
        print(f'Loin : {k_loin:.2f} px par degré (retard {retard_loin:.3f} s, écart {ecart_loin:.1f} px), rayon {rayon_loin:.1f} px')
        retour = banc.rouler(RECUL)

        # 1 / rayon = (distance de départ + recul) / K : une droite
        reculs = np.array([-d for d, _ in recul] + [RECUL - d for d, _ in retour])
        inverses = np.array([1 / r for _, r in recul] + [1 / r for _, r in retour])
        pente, origine = np.polyfit(reculs, inverses, 1)
        k_distance = 1 / pente
        depart = origine * k_distance
        ecart = np.std(k_distance * inverses - (depart + reculs))
        print(f'K_DISTANCE = {k_distance:.1f} (m x px) ; objet à {depart:.3f} m de la caméra au départ ; '
              f'écart de la droite {ecart * 100:.1f} cm sur {len(reculs)} relevés')

        # k = px_par_degré x (1 + avance / distance), aux deux distances
        d_pres, d_loin = k_distance / rayon_pres, k_distance / rayon_loin
        avance = (k_pres - k_loin) / (k_loin / d_pres - k_pres / d_loin)
        px_par_degre = k_pres / (1 + avance / d_pres)
        print(f'Distances {d_pres:.3f} m et {d_loin:.3f} m -> AVANCE_CAMERA = {avance:.3f} m, '
              f'{px_par_degre:.2f} px par degré pour un objet très loin')
    except KeyboardInterrupt:
        print('Arrêt demandé')
    finally:
        banc.fermer()


if __name__ == '__main__':
    main()
