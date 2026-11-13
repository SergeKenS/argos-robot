"""Garde un objet rouge au centre de l'image, en continu (essai 4).

Le PC voit l'objet et envoie la cible à l'ESP32 (suivi_fluide/suivi_fluide.ino),
qui y amène la tourelle en douceur.

Différence avec suivi_rouge.py (essai 3, « regarder puis bouger ») : on ne
s'arrête plus 0,3 s après chaque correction. L'image arrive en retard, mais ce
retard est connu (RETARD) et l'ESP32 annonce sa position : pour chaque image,
on retrouve où visait la tourelle quand l'image a été prise.
    direction de l'objet = visée à ce moment-là + écart vu dans l'image
Cette direction ne dépend pas du mouvement de la tourelle : on peut donc
corriger à chaque image, sans attendre et sans osciller.
On estime aussi la vitesse de l'objet, pour viser un peu en avant de lui.

Objet perdu : la tourelle le cherche en décrivant un 8, fait une pause au
centre, et recommence tant qu'il n'est pas retrouvé.

Usage : python suivi_fluide.py   (fermer d'abord le moniteur série d'Arduino)
Quitter : touche q dans la fenêtre de l'image.
"""
import math
import threading
import time
from collections import deque

import cv2
import numpy as np
import serial
from serial.tools import list_ports

# Caméra de la tourelle pour OpenCV (le numéro peut changer quand on rebranche)
CAMERA = 1
LARGEUR, HAUTEUR = 640, 480

# Tourelle : centres et débattement (les mêmes que dans suivi_fluide.ino)
CENTRE_BAS, CENTRE_HAUT = 100, 30
AMPLITUDE = 45

# Mesuré : quand l'angle augmente de 1°, l'image glisse de 7,6 px vers la
# droite (servo du bas) ou de 8,8 px vers le bas (servo du haut)
PX_PAR_DEGRE_BAS = 7.6
PX_PAR_DEGRE_HAUT = 8.8
# Mesuré : l'image montre la position que l'ESP32 annonçait 0,08 s plus tôt
RETARD = 0.08

# Écart toléré autour du centre, en pixels : en dessous, on ne corrige pas.
# Les servos ne savent pas faire de pas plus fins (environ 0,4°) : vouloir
# mieux les fait trembler.
ZONE_MORTE = 3
# Objet perdu depuis ce temps (s) : la tourelle part à sa recherche
PERTE = 1.5

# Prédiction : l'objet bouge pendant que l'image voyage et que la tourelle
# réagit. On estime sa vitesse et on vise là où il sera dans AVANCE secondes.
AVANCE = 0.14
# Lissage : part de la surprise (mesure - prévision) reportée sur la direction
# estimée de l'objet, et sur sa vitesse estimée
LISSAGE_DIRECTION = 0.6
LISSAGE_VITESSE = 0.15
# Surprise (degrés) au-delà de laquelle l'objet a « sauté » : on oublie sa vitesse
SAUT = 2.0
# Vitesse (degrés par seconde) en dessous de laquelle l'objet est dit immobile
VITESSE_MIN = 4
# Tourelle plus rapide que l'objet de cet écart (degrés par seconde) : elle
# rattrape un saut, la mesure est moins sûre, on ne touche pas à la vitesse
ECART_VITESSE_MAX = 40

# Recherche en forme de 8 : amplitude (bas, haut) en degrés, durée d'un 8 (s),
# pause au centre entre deux 8 (s)
AMPLITUDE_RECHERCHE = (40, 20)
DUREE_RECHERCHE = 6
PAUSE_RECHERCHE = 5

# Rouge de la balle en HSV (mesuré) : teinte 168 à 180 ou 0 à 8, saturation
# d'au moins 140, luminosité d'au moins 50
ROUGE_1 = (np.array([0, 140, 50]), np.array([8, 255, 255]))
ROUGE_2 = (np.array([168, 140, 50]), np.array([180, 255, 255]))
# Taille minimale de l'objet, en pixels
AIRE_MIN = 60


def trouver_objet(image):
    """Renvoie (x, y, rayon) de la plus grande tache rouge compacte, ou None.

    x et y sont le centre de la tache, plus précis que le pixel.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    masque = cv2.inRange(hsv, *ROUGE_1) | cv2.inRange(hsv, *ROUGE_2)
    masque = cv2.morphologyEx(masque, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    nombre, _, taches, centres = cv2.connectedComponentsWithStats(masque)

    meilleur = None
    for i in range(1, nombre):  # la tache 0 est le fond
        aire = taches[i, cv2.CC_STAT_AREA]
        largeur, hauteur = taches[i, cv2.CC_STAT_WIDTH], taches[i, cv2.CC_STAT_HEIGHT]
        # Compacte : remplit au moins la moitié de son rectangle, ni trop
        # large ni trop haute. Un fil ou une trace fine est ignoré.
        if aire < AIRE_MIN or aire < 0.5 * largeur * hauteur or not 1 / 3 < largeur / hauteur < 3:
            continue
        if meilleur is None or aire > meilleur[0]:
            meilleur = (aire, centres[i][0], centres[i][1])
    if meilleur is None:
        return None
    aire, x, y = meilleur
    return float(x), float(y), math.sqrt(aire / math.pi)


def limiter(angle, centre):
    """Garde un angle dans le débattement autour du centre et dans 0° à 180°."""
    return min(max(angle, max(centre - AMPLITUDE, 0)), min(centre + AMPLITUDE, 180))


class Camera:
    """Lit la caméra sans arrêt, dans un fil d'exécution à part.

    On a ainsi toujours la dernière image, avec son heure d'arrivée. Sans cela,
    si le programme prend du retard, les images attendent dans une file et on
    travaille sur du vieux (le même piège que la file d'ordres de l'essai 3).
    """

    def __init__(self):
        self.capture = cv2.VideoCapture(CAMERA)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, LARGEUR)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, HAUTEUR)
        ok, image = self.capture.read()
        if not ok:
            raise SystemExit(f'Caméra {CAMERA} : aucune image (occupée par un autre programme ?)')
        if image.shape[:2] != (HAUTEUR, LARGEUR):
            # Les pixels par degré ont été mesurés dans cette taille d'image
            raise SystemExit(f'Image en {image.shape[1]} x {image.shape[0]} au lieu de {LARGEUR} x {HAUTEUR}')
        self.nouvelle = threading.Condition()
        self.numero, self.heure, self.image = 0, 0.0, None
        self.actif = True
        threading.Thread(target=self._lire, daemon=True).start()

    def _lire(self):
        while self.actif:
            ok, image = self.capture.read()
            heure = time.perf_counter()
            if ok:
                with self.nouvelle:
                    self.numero, self.heure, self.image = self.numero + 1, heure, image
                    self.nouvelle.notify_all()

    def suivante(self, numero):
        """Attend une image plus récente que `numero` ; renvoie (numéro, heure, image)."""
        with self.nouvelle:
            self.nouvelle.wait_for(lambda: self.numero > numero, timeout=1)
            return self.numero, self.heure, self.image

    def fermer(self):
        self.actif = False
        time.sleep(0.1)
        self.capture.release()


class Tourelle:
    """Liaison avec l'ESP32 : envoie les cibles et retient les positions qu'il annonce."""

    def __init__(self):
        # L'ESP32 se reconnaît à sa puce USB CH340 (identifiant 1A86)
        port = next((p.device for p in list_ports.comports() if p.vid == 0x1A86), None)
        if port is None:
            raise SystemExit('ESP32 introuvable : est-il branché ?')
        self.serie = serial.Serial(port, 115200, timeout=0.05)
        # Dernières positions annoncées : (heure de réception, bas, haut)
        self.positions = deque([(time.perf_counter(), CENTRE_BAS, CENTRE_HAUT)], maxlen=500)
        self.actif = True
        threading.Thread(target=self._ecouter, daemon=True).start()
        time.sleep(2)  # l'ouverture du port redémarre l'ESP32

    def _ecouter(self):
        """Fil à part : note chaque position "P,temps_ms,bas,haut" dès qu'elle arrive."""
        while self.actif:
            ligne = self.serie.readline()
            if ligne.startswith(b'P,'):
                try:
                    _, _, bas, haut = ligne.decode().split(',')
                    self.positions.append((time.perf_counter(), float(bas), float(haut)))
                except ValueError:
                    pass  # ligne abîmée : on l'ignore

    def viser(self, bas, haut):
        self.serie.write(f'{bas:.2f},{haut:.2f}\n'.encode())

    def position(self, heure):
        """Position (bas, haut) que l'ESP32 annonçait à cette heure."""
        apres = None
        for point in reversed(list(self.positions)):
            if point[0] <= heure:
                if apres is None or apres[0] - point[0] > 0.05:
                    return point[1], point[2]  # tourelle à l'arrêt à ce moment
                # Entre deux annonces rapprochées : on interpole
                part = (heure - point[0]) / (apres[0] - point[0])
                return (point[1] + part * (apres[1] - point[1]),
                        point[2] + part * (apres[2] - point[2]))
            apres = point
        return self.positions[0][1], self.positions[0][2]

    def fermer(self):
        # Remet la tourelle au centre avant de quitter
        self.viser(CENTRE_BAS, CENTRE_HAUT)
        time.sleep(0.5)
        self.actif = False
        time.sleep(0.1)
        self.serie.close()


class Suivi:
    """Estime, image après image, la direction de l'objet et sa vitesse, et en déduit la cible."""

    def __init__(self):
        self.heure = None                           # heure de la dernière image où l'objet a été vu
        self.direction = [CENTRE_BAS, CENTRE_HAUT]  # direction estimée de l'objet (degrés)
        self.vitesse = [0.0, 0.0]                   # sa vitesse estimée (degrés par seconde)
        self.cible = [CENTRE_BAS, CENTRE_HAUT]      # cible à envoyer à l'ESP32

    def voir(self, heure, objet, tourelle, centre=(LARGEUR / 2, HAUTEUR / 2)):
        """Met à jour la cible d'après l'objet vu dans l'image arrivée à `heure`, et la renvoie.

        centre : point de l'image où amener l'objet.
        """
        # Où visait la tourelle quand l'image a été prise, et 0,04 s plus tôt
        visee = tourelle.position(heure - RETARD)
        avant = tourelle.position(heure - RETARD - 0.04)
        ecoule = None if self.heure is None else heure - self.heure
        self.heure = heure

        for axe, px_par_degre, centre_axe in ((0, PX_PAR_DEGRE_BAS, CENTRE_BAS),
                                              (1, PX_PAR_DEGRE_HAUT, CENTRE_HAUT)):
            ecart = objet[axe] - centre[axe]
            # L'image doit glisser de -ecart pixels, soit -ecart / pixels_par_degre degrés
            mesure = visee[axe] - ecart / px_par_degre

            if ecoule is None or ecoule > 0.2:
                # Objet tout juste trouvé : on part de la mesure, sans vitesse
                self.direction[axe], self.vitesse[axe] = mesure, 0.0
                self.cible[axe] = limiter(mesure, centre_axe)
                continue

            # Prévision d'après l'image précédente, puis correction par la mesure
            prevision = self.direction[axe] + self.vitesse[axe] * ecoule
            surprise = mesure - prevision
            if abs(surprise) > SAUT:
                self.direction[axe], self.vitesse[axe] = mesure, 0.0
            else:
                self.direction[axe] = prevision + LISSAGE_DIRECTION * surprise
                vitesse_tourelle = (visee[axe] - avant[axe]) / 0.04
                if abs(vitesse_tourelle - self.vitesse[axe]) < ECART_VITESSE_MAX:
                    self.vitesse[axe] += LISSAGE_VITESSE * surprise / ecoule

            # Objet immobile et presque centré : on ne bouge pas. Sinon, on
            # vise là où il sera dans AVANCE secondes.
            vitesse = self.vitesse[axe]
            vitesse = math.copysign(max(0.0, abs(vitesse) - VITESSE_MIN), vitesse)
            if abs(ecart) > ZONE_MORTE or vitesse != 0:
                self.cible[axe] = limiter(self.direction[axe] + vitesse * AVANCE, centre_axe)
        return tuple(self.cible)


def recherche(t):
    """Cible (bas, haut) du balayage en forme de 8, t secondes après son début."""
    tour = 2 * math.pi * t / DUREE_RECHERCHE
    return (CENTRE_BAS + AMPLITUDE_RECHERCHE[0] * math.sin(tour),
            CENTRE_HAUT + AMPLITUDE_RECHERCHE[1] * math.sin(2 * tour))


def main():
    tourelle = Tourelle()
    camera = Camera()
    suivi = Suivi()

    cible = (CENTRE_BAS, CENTRE_HAUT)
    numero = 0
    derniere_vue = time.perf_counter()
    # Recherche : heure de début du 8 en cours (None = pas de recherche), fin de la pause
    debut_recherche, fin_pause = None, 0
    # Bilan affiché chaque seconde dans le terminal
    bilan, vues, images = time.perf_counter(), 0, 0

    while True:
        numero, heure, image = camera.suivante(numero)
        maintenant = time.perf_counter()
        objet = trouver_objet(image)

        if objet is not None:
            if maintenant - derniere_vue > PERTE:
                print('Objet retrouvé')
            derniere_vue = maintenant
            debut_recherche, fin_pause = None, 0
            cible = suivi.voir(heure, objet, tourelle)
            x, y, rayon = objet
            cv2.circle(image, (round(x), round(y)), round(rayon), (0, 255, 0), 2)
        elif maintenant - derniere_vue > PERTE:
            if debut_recherche is None and maintenant >= fin_pause:
                print('Objet perdu : recherche')
                debut_recherche = maintenant
            if debut_recherche is not None:
                cible = recherche(maintenant - debut_recherche)
                if maintenant - debut_recherche >= DUREE_RECHERCHE:
                    # Le 8 est fini, tourelle au centre : pause avant le suivant
                    cible = (CENTRE_BAS, CENTRE_HAUT)
                    debut_recherche, fin_pause = None, maintenant + PAUSE_RECHERCHE
        tourelle.viser(*cible)

        vues, images = vues + (objet is not None), images + 1
        if maintenant - bilan >= 1:
            if objet:
                ecart = f'écart {objet[0] - LARGEUR / 2:+6.1f} {objet[1] - HAUTEUR / 2:+6.1f} px'
            else:
                ecart = 'objet non vu'
            print(f'vu {vues:2d}/{images} images | {ecart} | bas {cible[0]:6.1f} haut {cible[1]:5.1f}')
            bilan, vues, images = maintenant, 0, 0

        cv2.drawMarker(image, (LARGEUR // 2, HAUTEUR // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
        cv2.imshow('Camera tourelle', image)
        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            break

    camera.fermer()
    tourelle.fermer()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
