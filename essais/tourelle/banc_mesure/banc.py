"""Banc d'essai du suivi, balle immobile.

Pour simuler une balle qui bouge, on déplace le point de l'image où la balle
doit être amenée : la tourelle doit faire exactement le même travail.

Usage : python banc.py fluide|essai3 [essais...]   (essais : repos sauts sinus cercle)
"""
import math
import sys
import time

import numpy as np

sys.path.insert(0, r'c:\Users\kenfa\Documents\Projets\Robot+Lidar\essais\tourelle')
import suivi_fluide as sf
import suivi_rouge as e3

MODE = sys.argv[1] if len(sys.argv) > 1 else 'fluide'
ESSAIS = sys.argv[2:] or ['repos', 'sauts', 'sinus', 'cercle']
# Part du retard due à la caméra : l'image montre la scène d'il y a 0,06 s
RETARD_CAMERA = 0.06

tourelle = sf.Tourelle()
camera = sf.Camera()
for _ in range(20):
    numero = camera.suivante(0)[0]


class LoiFluide:
    """La loi de suivi_fluide.py telle quelle."""

    def __init__(self):
        self.suivi = sf.Suivi()
        self.cible = (sf.CENTRE_BAS, sf.CENTRE_HAUT)

    def pas(self, heure, image, centre):
        objet = sf.trouver_objet(image)
        if objet is not None:
            self.cible = self.suivi.voir(heure, objet, tourelle, centre)
        tourelle.viser(*self.cible)


class LoiEssai3:
    """La loi de suivi_rouge.py, recopiée : regarder puis bouger."""

    def __init__(self):
        self.bas, self.haut = e3.CENTRE_BAS, e3.CENTRE_HAUT
        self.envoye = None
        self.prochaine = 0

    def pas(self, heure, image, centre):
        maintenant = time.time()
        # Même détection que le nouveau suivi (celle de l'essai 3 se trompe
        # d'objet dans cette lumière), arrondie au pixel comme à l'époque
        objet = sf.trouver_objet(image)
        if objet is not None:
            ecart_x, ecart_y = int(objet[0]) - int(centre[0]), int(objet[1]) - int(centre[1])
            if maintenant >= self.prochaine and max(abs(ecart_x), abs(ecart_y)) > e3.ZONE_MORTE:
                if abs(ecart_x) > e3.ZONE_MORTE:
                    self.bas = e3.limiter(self.bas - ecart_x / e3.PIXELS_PAR_DEGRE_BAS, e3.CENTRE_BAS)
                if abs(ecart_y) > e3.ZONE_MORTE:
                    self.haut = e3.limiter(self.haut - ecart_y / e3.PIXELS_PAR_DEGRE_HAUT, e3.CENTRE_HAUT)
                self.prochaine = maintenant + e3.ATTENTE
        commande = (round(self.bas), round(self.haut))
        if commande != self.envoye:
            tourelle.serie.write(f'{commande[0]},{commande[1]}\n'.encode())
            self.envoye = commande


class LoiPredictive:
    """Loi fluide + estimation de la vitesse de l'objet (filtre alpha-bêta) et avance."""

    def __init__(self, alpha, beta, avance, saut=2.0, zone=0.0):
        self.alpha, self.beta, self.avance, self.saut, self.zone = alpha, beta, avance, saut, zone
        self.heure = None
        self.direction = [sf.CENTRE_BAS, sf.CENTRE_HAUT]
        self.vitesse = [0.0, 0.0]
        self.cible = [sf.CENTRE_BAS, sf.CENTRE_HAUT]

    def pas(self, heure, image, centre):
        objet = sf.trouver_objet(image)
        if objet is not None:
            visee = tourelle.position(heure - sf.RETARD)
            ecarts = (objet[0] - centre[0], objet[1] - centre[1])
            mesure = (visee[0] - ecarts[0] / sf.PX_PAR_DEGRE_BAS, visee[1] - ecarts[1] / sf.PX_PAR_DEGRE_HAUT)
            dt = heure - self.heure if self.heure is not None else None
            self.heure = heure
            avant = tourelle.position(heure - sf.RETARD - 0.04)
            vitesse_tourelle = ((visee[0] - avant[0]) / 0.04, (visee[1] - avant[1]) / 0.04)
            for axe, centre_axe in ((0, sf.CENTRE_BAS), (1, sf.CENTRE_HAUT)):
                if dt is None or dt > 0.2:
                    self.direction[axe], self.vitesse[axe] = mesure[axe], 0.0
                else:
                    prevue = self.direction[axe] + self.vitesse[axe] * dt
                    surprise = mesure[axe] - prevue
                    if abs(surprise) > self.saut:
                        self.direction[axe], self.vitesse[axe] = mesure[axe], 0.0
                    else:
                        self.direction[axe] = prevue + self.alpha * surprise
                        # Tourelle en plein rattrapage (bien plus rapide que l'objet) :
                        # la mesure est moins sûre, on ne touche pas à la vitesse estimée
                        if abs(vitesse_tourelle[axe] - self.vitesse[axe]) < PORTE:
                            self.vitesse[axe] += self.beta * surprise / dt
                # Vitesse utile : on ignore les petites vitesses (bruit quand l'objet est immobile)
                vitesse = self.vitesse[axe]
                utile = math.copysign(max(0.0, abs(vitesse) - VMIN), vitesse)
                if abs(ecarts[axe]) > self.zone or utile != 0:
                    self.cible[axe] = sf.limiter(self.direction[axe] + utile * self.avance, centre_axe)
        tourelle.viser(*self.cible)


# Options "NOM=valeur" : réglages de l'ESP32 (V, A, G, J) ou de la loi prédictive
options = dict(o.split('=') for o in ESSAIS if '=' in o)
ESSAIS = [e for e in ESSAIS if '=' not in e] or ['repos', 'sauts', 'sinus', 'cercle']
for lettre in 'VAG':
    if lettre in options:
        tourelle.serie.write(f'{lettre},{options[lettre]}\n'.encode())
# Constantes de suivi_fluide.py modifiables pour un essai, ex. ZONE_MORTE=2 AVANCE=0.1
for nom, valeur in options.items():
    if hasattr(sf, nom):
        setattr(sf, nom, float(valeur))
if 'ZONE' in options:
    sf.ZONE_MORTE = float(options['ZONE'])
VMIN = float(options.get('VMIN', 0))
PORTE = float(options.get('PORTE', 1e9))
if MODE == 'fluide':
    loi = LoiFluide()
elif MODE == 'pred':
    loi = LoiPredictive(float(options.get('ALPHA', 0.6)), float(options.get('BETA', 0.15)),
                        float(options.get('AVANCE', 0.12)), float(options.get('SAUT', 2.0)),
                        float(options.get('ZONE', 0.0)))
else:
    loi = LoiEssai3()
print('Options :', options)


def derouler(duree, consigne):
    """Fait tourner le suivi `duree` s. consigne(t) = point (x, y) de l'image où amener la balle.

    Renvoie le journal [(t, écart x, écart y)] ; écart = balle vue - consigne, en pixels.
    """
    global numero
    journal = []
    perdues = 0
    debut = time.perf_counter()
    while True:
        numero, heure, image = camera.suivante(numero)
        t = heure - debut
        if t > duree:
            break
        centre = consigne(t - RETARD_CAMERA)
        loi.pas(heure, image, centre)
        balle = sf.trouver_objet(image)  # mesure indépendante de la loi testée
        if balle:
            journal.append((t, balle[0] - centre[0], balle[1] - centre[1]))
        else:
            perdues += 1
    if perdues:
        print(f'    ({perdues} images sans balle)')
    return np.array(journal)


CX, CY = sf.LARGEUR / 2, sf.HAUTEUR / 2
print(f'=== Mode {MODE} ===')
derouler(3, lambda t: (CX, CY))  # mise en place : balle au centre

if 'repos' in ESSAIS:
    j = derouler(10, lambda t: (CX, CY))
    r = np.hypot(j[:, 1], j[:, 2])
    print(f'Repos 10 s : écart moyen {r.mean():.1f} px, max {r.max():.1f} px, '
          f'tremblement (écart-type) x {j[:, 1].std():.2f} y {j[:, 2].std():.2f} px')

if 'sauts' in ESSAIS:
    for nom, axe, taille in (('horizontal', 0, 75), ('vertical', 1, 60)):
        temps20, temps5, finals, depassements = [], [], [], []
        signe = 1
        for _ in range(6):
            decalage = signe * taille / 2
            signe = -signe
            if axe == 0:
                j = derouler(2.5, lambda t: (CX + decalage, CY))
            else:
                j = derouler(2.5, lambda t: (CX, CY + decalage))
            e = j[:, 1 + axe]

            def stabilise(seuil):
                dehors = np.nonzero(np.abs(e) > seuil)[0]
                if len(dehors) == 0:
                    return 0.0
                return j[dehors[-1] + 1, 0] if dehors[-1] + 1 < len(j) else float('nan')

            temps20.append(stabilise(20))
            temps5.append(stabilise(5))
            finals.append(np.abs(e[j[:, 0] > 1.8]).mean())
            # Dépassement : écart de signe opposé à l'écart de départ
            depassements.append(max(0.0, float(np.max(-e * np.sign(e[0])))))
        temps20, temps5 = temps20[1:], temps5[1:]  # le 1er saut est deux fois plus petit
        print(f'Sauts de {taille} px ({nom}) : sous 20 px en {np.nanmean(temps20):.2f} s (pire {np.nanmax(temps20):.2f}), '
              f'sous 5 px en {np.nanmean(temps5):.2f} s (pire {np.nanmax(temps5):.2f}, '
              f'{int(np.isnan(temps5).sum())} jamais), écart final {np.mean(finals):.1f} px, '
              f'dépassement moyen {np.mean(depassements[1:]):.1f} px')
    derouler(1.5, lambda t: (CX, CY))

if 'sinus' in ESSAIS:
    for frequence, amplitude in ((0.25, 75), (0.5, 75), (1.0, 75), (0.5, 150)):
        j = derouler(8, lambda t: (CX + amplitude * math.sin(2 * math.pi * frequence * t), CY))
        e = j[j[:, 0] > 2, 1]
        vitesse = 2 * math.pi * frequence * amplitude
        print(f'Sinus horizontal {frequence} Hz ±{amplitude} px (jusqu\'à {vitesse:.0f} px/s, '
              f'{vitesse / sf.PX_PAR_DEGRE_BAS:.0f} °/s) : écart efficace {np.sqrt((e ** 2).mean()):.1f} px, '
              f'max {np.abs(e).max():.1f} px')
        derouler(1, lambda t: (CX, CY))

if 'cercle' in ESSAIS:
    j = derouler(10, lambda t: (CX + 75 * math.cos(2 * math.pi * 0.5 * t), CY + 60 * math.sin(2 * math.pi * 0.5 * t)))
    r = np.hypot(j[j[:, 0] > 2, 1], j[j[:, 0] > 2, 2])
    print(f'Cercle 0,5 Hz (75 x 60 px) : écart efficace {np.sqrt((r ** 2).mean()):.1f} px, max {r.max():.1f} px')
    derouler(1, lambda t: (CX, CY))

camera.fermer()
tourelle.fermer()
