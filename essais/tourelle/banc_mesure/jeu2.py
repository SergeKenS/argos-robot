"""Avec la balle immobile comme repère : bruit au repos, pixels par degré, jeu des servos.

Usage : python jeu2.py axe   (0 = bas, 1 = haut)
"""
import sys
import time

import numpy as np

from outils import CENTRE, Camera, Tourelle, trouver_balle

AXE = int(sys.argv[1]) if len(sys.argv) > 1 else 0
NOM = ('bas', 'haut')[AXE]
tourelle = Tourelle()
if len(sys.argv) > 3:
    tourelle.envoyer(f'J,{sys.argv[2]},{sys.argv[3]}')
    print(f'Jeu réglé : bas {sys.argv[2]}, haut {sys.argv[3]}')
camera = Camera()
numero = 0


def aller(decalage):
    cible = list(CENTRE)
    cible[AXE] += decalage
    tourelle.viser(*cible)


def mesurer(duree, garder=4):
    """Position de la balle sur l'axe (px) : moyenne des `garder` dernières images vues pendant `duree`."""
    global numero
    fin = time.perf_counter() + duree
    valeurs = []
    while time.perf_counter() < fin:
        numero, _, image = camera.suivante(numero)
        balle = trouver_balle(image)
        if balle:
            valeurs.append(balle[AXE])
    return float(np.mean(valeurs[-garder:])) if valeurs else float('nan'), len(valeurs)


# 1. Bruit au repos sur 3 s
aller(0)
time.sleep(1)
serie = []
vues = total = 0
fin = time.perf_counter() + 3
while time.perf_counter() < fin:
    numero, _, image = camera.suivante(numero)
    balle = trouver_balle(image)
    total += 1
    if balle:
        vues += 1
        serie.append(balle)
serie = np.array(serie)
print(f'Repos : balle vue {vues}/{total} images, x {serie[:, 0].mean():.1f} ± {serie[:, 0].std():.2f} px, '
      f'y {serie[:, 1].mean():.1f} ± {serie[:, 1].std():.2f} px, aire {serie[:, 2].mean():.0f} px')

# 2. Pixels par degré : positions atteintes toujours en montant, après un recul
points = []
for decalage in (-10, -5, 0, 5, 10):
    aller(decalage - 6)
    time.sleep(0.7)
    aller(decalage)
    time.sleep(0.5)
    px, n = mesurer(0.4)
    points.append((decalage, px))
pente, origine = np.polyfit([p[0] for p in points], [p[1] for p in points], 1)
residus = [px - (pente * d + origine) for d, px in points]
print(f'Axe {NOM} : {pente:+.2f} px par degré (écarts à la droite : '
      + ' '.join(f'{r:+.1f}' for r in residus) + ' px)')

# 3. Jeu : escalier de 0,25° de -3° à +3° et retour, deux fois
PAS, AMPLITUDE = 0.25, 3.0
aller(-AMPLITUDE - 6)
time.sleep(0.7)
aller(-AMPLITUDE)
time.sleep(0.6)
depart, _ = mesurer(0.4)
n = round(2 * AMPLITUDE / PAS)
montee = [-AMPLITUDE + i * PAS for i in range(1, n + 1)]
descente = montee[-2::-1] + [-AMPLITUDE]
table = {}
for tour in range(2):
    for sens, liste in (('montée', montee), ('descente', descente)):
        for commande in liste:
            aller(commande)
            px, _ = mesurer(0.3, garder=3)
            table.setdefault(round(commande + AMPLITUDE, 2), {})[(tour, sens)] = (px - depart) / pente

print(f'Jeu, axe {NOM} : angle réel (degrés depuis le départ) selon l\'angle commandé')
print('commande | montée 1 | descente 1 | montée 2 | descente 2')
colonnes = ((0, 'montée'), (0, 'descente'), (1, 'montée'), (1, 'descente'))
for commande in sorted(table):
    print(f'{commande:8.2f} | ' + ' | '.join(
        f'{table[commande][c]:8.2f}' if c in table[commande] else '        ' for c in colonnes))
ecarts = [table[c][(1, 'descente')] - table[c][(1, 'montée')] for c in table
          if (1, 'descente') in table[c] and (1, 'montée') in table[c]]
print(f'Écart descente - montée (tour 2) : moyenne {np.mean(ecarts):.2f}°, min {np.min(ecarts):.2f}°, max {np.max(ecarts):.2f}°')

camera.fermer()
tourelle.fermer()
