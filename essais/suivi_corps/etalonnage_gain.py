"""Étalonnage du gain de compensation, en condition de suivi (objet au centre de l'image).

La tête suit l'objet vert immobile pendant que le corps pivote, à deux distances
(sur place, puis après un recul de 25 cm). À chaque distance, analyse.py dit de
combien le gain utilisé était faux. On en tire la loi :
    glissement (px par degré de lacet) = PX_CORPS_LOIN + PX_CORPS_PAR_RAYON x rayon

Usage (sur le Pi, objet vert à 30 ou 40 cm, 60 cm libres derrière) :
  python3 etalonnage_gain.py
"""
import math
import subprocess
import sys
import time

import numpy as np

import tete_corps
from tete_corps import PX_PAR_DEGRE_BAS, Regard, Robot, essai_pivot

RECUL = 0.25


def rouler(robot, distance, vitesse=0.08):
    depart = robot.base.etat()[3]
    while abs(robot.base.etat()[3] - depart) < abs(distance):
        robot.base.commander(math.copysign(vitesse, distance), 0.0)
        robot.pas()
    fin = time.perf_counter() + 0.8
    while time.perf_counter() < fin:
        robot.base.commander(0.0, 0.0)
        robot.pas()


def mesurer(robot, fichier):
    """Un essai de pivot avec journal ; renvoie (rayon médian, px par degré de lacet réels)."""
    robot.journal = open(fichier, 'w')
    robot.journal.write('heure,vu,x,y,rayon,bas,haut,lacet,parcouru,ego,cible_bas,cible_haut,'
                        'direction,vitesse,ordre_vitesse,ordre_rotation\n')
    essai_pivot(robot, math.radians(45))
    robot.journal.close()
    robot.journal = None
    d = np.genfromtxt(fichier, delimiter=',', names=True)
    d = d[d['vu'] == 1]
    # Par rapport au sol, l'objet ne bouge pas : bas - écart = constante - glissement x lacet
    visee = d['bas'] - (d['x'] - 320) / PX_PAR_DEGRE_BAS
    pente, _ = np.polyfit(d['lacet'], visee, 1)
    reste = np.std(visee - np.polyval([pente, _], d['lacet']))
    k = -pente * PX_PAR_DEGRE_BAS
    rayon = float(np.median(d['rayon']))
    print(f'    rayon {rayon:.1f} px : {k:.2f} px par degré de lacet (reste {reste * PX_PAR_DEGRE_BAS:.0f} px)')
    return rayon, k


def main():
    robot = Robot()
    try:
        r1, k1 = mesurer(robot, '/tmp/gain_pres.csv')
        rouler(robot, -RECUL)
        r2, k2 = mesurer(robot, '/tmp/gain_loin.csv')
        rouler(robot, RECUL)
        par_rayon = (k1 - k2) / (r1 - r2)
        print(f'PX_CORPS_PAR_RAYON = {par_rayon:.3f} ; PX_CORPS_LOIN = {k1 - par_rayon * r1:.2f} '
              f'(valeurs utilisées : {tete_corps.PX_CORPS_PAR_RAYON}, {tete_corps.PX_CORPS_LOIN})')
    except KeyboardInterrupt:
        print('Arrêt demandé')
    finally:
        robot.fermer()


if __name__ == '__main__':
    main()
