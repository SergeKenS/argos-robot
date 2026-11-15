"""Tête et corps ensemble : la tourelle garde l'objet vert au centre pendant que le corps bouge.

Principe (suite de essais/tourelle/suivi_fluide.py) :
  - la tête est rapide (30 corrections par seconde), le corps est lent ;
  - la direction de l'objet est estimée PAR RAPPORT AU SOL, pas par rapport au robot :
        direction au sol = angle de la tourelle + effet des mouvements du corps
    L'effet des mouvements du corps (classe Ego) vient des encodeurs, 175 fois par
    seconde : dès que le corps pivote, la tourelle contre-tourne, sans attendre que
    l'image montre une erreur. La caméra ne corrige plus que le reste.

Unité des angles : le degré de servo de la tourelle, comme dans suivi_fluide.py.
Un degré réel du corps vaut plus d'un degré de servo (voir Ego).

Usage (sur le Pi, objet vert devant le robot, 1 m libre autour) :
  python3 tete_corps.py --essai pivot              # le corps pivote, la tête compense
  python3 tete_corps.py --essai pivot --comparer   # puis le même essai sans compensation
  Ajouter --flux pour voir l'image en direct : http://argos-pi.local:8081
"""
import argparse
import math
import signal
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'tourelle'))
from suivi_fluide import (AMPLITUDE, AVANCE, CENTRE_BAS, CENTRE_HAUT,  # noqa: E402
                          HAUTEUR, LARGEUR, LISSAGE_DIRECTION, LISSAGE_VITESSE,
                          PERTE, PX_PAR_DEGRE_BAS, PX_PAR_DEGRE_HAUT, RETARD, VITESSE_MIN,
                          ZONE_MORTE, Camera, Tourelle, recherche, trouver_objet)
from suivi_fluide import DUREE_RECHERCHE  # noqa: E402  (durée d'un 8 de la tête, s)

from base import Base  # noqa: E402

# Mesuré par mesure_retard.py : l'image montre le lacet que les encodeurs donnaient
# 0,03 s plus tôt (0,08 s pour la tourelle : l'ESP32 annonce sa position en avance)
RETARD_CORPS = 0.03
# Mesuré par etalonnage_gain.py, objet vert suivi par la tête :
#   un degré de rotation du corps fait glisser l'image de 8,18 px si l'objet est
#   loin, et de 0,067 px de plus par pixel de rayon de l'objet (objet proche : la
#   caméra, en avant de l'axe des roues, se déplace aussi de côté)
PX_CORPS_LOIN = 8.18
PX_CORPS_PAR_RAYON = 0.067
#   distance de l'objet (m) = K_DISTANCE / son rayon dans l'image (px), par etalonnage.py
K_DISTANCE = 11.0
# Degrés de servo pour un degré réel (le servo tourne un peu moins qu'annoncé)
SERVO_PAR_DEGRE = PX_CORPS_LOIN / PX_PAR_DEGRE_BAS

# La tourelle met du temps à exécuter un ordre : on compense le mouvement que le
# corps aura fait dans AVANCE_CORPS secondes (réglé par la mesure, voir README)
AVANCE_CORPS = 0.15

# Le corps suit la tête : rotation (deg/s) par degré d'angle de la tourelle, plus
# une part de la vitesse à laquelle cet angle change. Réglé en simulation sur deux
# enregistrements d'objet déplacé à la main (voir README, problème 5).
GAIN_CORPS = 2.5
GAIN_VARIATION = 0.2
LISSAGE_CORPS = 0.1      # s
# Avance : le robot se place à DISTANCE_ARRET de l'objet (m, mesurés depuis la caméra)
DISTANCE_ARRET = 0.40
GAIN_AVANCE = 1.2         # (m/s) par mètre d'erreur de distance
GAIN_RECUL = 2.5          # plus fort en recul : un objet trop près est plus urgent
FREINAGE = 0.4            # m/s² : freinage sur lequel le robot compte à l'approche du but
VITESSE_SUIVI = 0.50      # m/s au plus en avançant (essai des vitesses, README)
VITESSE_RECUL = 0.50      # m/s au plus en reculant (objet trop près)
# Robot arrêté : il repart si l'erreur dépasse DISTANCE_DEPART ; en route, il
# s'arrête sous DISTANCE_CALME (m)
DISTANCE_DEPART = 0.06
DISTANCE_CALME = 0.025
# Tourelle tournée de cet angle (degrés) ou plus : le corps pivote sans avancer
ANGLE_SANS_AVANCE = 35.0
# Recherche quand l'objet est perdu depuis PERTE secondes :
#   1. la tête seule décrit un 8 (DUREE_RECHERCHE secondes), le corps ne bouge pas ;
#   2. le corps fait un tour complet du côté où l'objet a disparu, tête au centre,
#      qui monte et descend pour couvrir le sol et la hauteur de la caméra ;
#   3. arrêt, état « perdu », et on recommence après PAUSE_PERDU secondes.
ROTATION_RECHERCHE = 40.0     # deg/s : plus vite, l'image est floue et l'objet échappe à la détection
HAUT_RECHERCHE = (22, 13)     # tête pendant le tour : angle moyen et amplitude (degrés de servo)
PERIODE_HAUT_RECHERCHE = 2.0  # s : durée d'une montée-descente de la tête
PAUSE_PERDU = 30.0            # s
CONFIRMATION = 3              # images de suite avec l'objet pour dire « retrouvé »
# Image douteuse : objet vu plus petit que cette part de sa taille habituelle
# (à moitié caché par une main) : on ne s'en sert pas pour estimer
TAILLE_DOUTEUSE = 0.6
# Corps en mouvement : il s'arrête quand la tourelle est à moins de ZONE_CALME
# degrés de son centre. Corps au repos : il repart au-delà de ZONE_DEPART degrés.
ZONE_CALME = 1.5
ZONE_DEPART = 5.0
# Rotation maximale du corps en suivi (deg/s) : la tête compense bien jusque-là
ROTATION_SUIVI = 120.0


def limiter(angle, centre):
    """Garde un angle dans le débattement de la tourelle."""
    return min(max(angle, max(centre - AMPLITUDE, 0)), min(centre + AMPLITUDE, 180))


class Ego:
    """Effet des mouvements du corps sur la direction de l'objet, en degrés de servo.

    Si l'objet ne bouge pas :   angle de la tourelle + ego = constante.
    Le corps pivote de 1° à gauche : l'objet part de `gain` degrés vers la droite
    pour la tourelle. Le corps avance : un objet sur le côté s'écarte encore.
    """

    def __init__(self, base, actif=True):
        self.base = base
        self.actif = actif          # False : pas de compensation (pour comparer)
        self.gain = SERVO_PAR_DEGRE     # degrés de servo par degré de lacet
        self.par_metre = 0.0            # degrés de servo par mètre avancé
        self.reference = (0.0, 0.0, 0.0)    # (ego, lacet en degrés, distance) au dernier recalage
        self.recaler(time.perf_counter())

    def valeur(self, heure):
        """Ego à `heure` (passée ou présente)."""
        if not self.actif:
            return 0.0
        ego, lacet, distance = self.reference
        _, _, theta, parcouru = self.base.etat(heure)
        return (ego + self.gain * (math.degrees(theta) - lacet)
                - self.par_metre * (parcouru - distance))

    def futur(self, avance):
        """Ego prévu dans `avance` secondes, si le corps garde sa vitesse."""
        maintenant = time.perf_counter()
        return self.valeur(maintenant) + (self.valeur(maintenant) - self.valeur(maintenant - 0.06)) / 0.06 * avance

    def recaler(self, heure, rayon=None, angle=0.0):
        """Fige l'ego acquis, puis adapte les gains à l'objet (rayon en px, angle de
        la tourelle par rapport à son centre en degrés de servo)."""
        ego = self.valeur(heure)
        _, _, theta, parcouru = self.base.etat(heure)
        self.reference = (ego, math.degrees(theta), parcouru)
        if rayon is not None:
            self.gain = (PX_CORPS_LOIN + PX_CORPS_PAR_RAYON * rayon) / PX_PAR_DEGRE_BAS
            distance = K_DISTANCE / rayon
            self.par_metre = (math.degrees(math.sin(math.radians(angle / SERVO_PAR_DEGRE)) / distance)
                              * SERVO_PAR_DEGRE)


class Regard:
    """Estime la direction de l'objet par rapport au sol, sa vitesse et sa distance."""

    def __init__(self):
        self.heure = None                           # heure de la dernière image où l'objet a été vu
        self.direction = [CENTRE_BAS, CENTRE_HAUT]  # direction de l'objet (axe bas : par rapport au sol)
        self.vitesse = [0.0, 0.0]                   # sa vitesse (degrés par seconde)
        self.visee = [CENTRE_BAS, CENTRE_HAUT]      # où la tourelle doit viser (axe bas : par rapport au sol)
        self.rayon = None                           # rayon lissé de l'objet (px)
        self.ecart = (0.0, 0.0)                     # dernier écart au centre de l'image (px)

    def voir(self, heure, objet, tourelle, ego, centre=(LARGEUR / 2, HAUTEUR / 2)):
        """Met l'estimation à jour d'après l'objet vu dans l'image arrivée à `heure`."""
        position = tourelle.position(heure - RETARD)
        # Axe bas : on ajoute l'effet du corps au moment où l'image a été prise
        visee = (position[0] + ego.valeur(heure - RETARD_CORPS), position[1])
        ecoule = None if self.heure is None else heure - self.heure
        self.heure = heure
        self.ecart = (objet[0] - centre[0], objet[1] - centre[1])
        nouveau = ecoule is None or ecoule > 0.2
        if nouveau or self.rayon is None:
            self.rayon = objet[2]
        elif objet[2] < TAILLE_DOUTEUSE * self.rayon:
            # Objet à moitié caché : son centre est faux. On garde l'estimation en
            # cours. Si l'objet a vraiment rapetissé, le rayon lissé le rejoint en
            # moins d'une demi-seconde et les images sont de nouveau acceptées.
            self.rayon += 0.1 * (objet[2] - self.rayon)
            self.direction = [d + v * ecoule for d, v in zip(self.direction, self.vitesse)]
            return
        else:
            self.rayon += 0.2 * (objet[2] - self.rayon)

        for axe, px_par_degre in ((0, PX_PAR_DEGRE_BAS), (1, PX_PAR_DEGRE_HAUT)):
            mesure = visee[axe] - self.ecart[axe] / px_par_degre
            if nouveau:
                # Objet tout juste trouvé : on part de la mesure, sans vitesse
                self.direction[axe], self.vitesse[axe] = mesure, 0.0
                self.visee[axe] = mesure
                continue
            # Prévision d'après l'image précédente, puis correction par la mesure
            prevision = self.direction[axe] + self.vitesse[axe] * ecoule
            surprise = mesure - prevision
            # Petite surprise : c'est surtout du bruit, on corrige peu. Grande
            # surprise : l'objet accélère, on croit davantage la mesure.
            # (suivi_fluide.py remettait la vitesse à zéro au-delà de 2° : avec un
            # objet rapide, il ne prédisait plus rien. Mesuré en rejouant un
            # enregistrement : erreur de prédiction de 49 px, 26 px avec cette règle.)
            confiance = min(1.0, LISSAGE_DIRECTION + 0.2 * abs(surprise))
            confiance_vitesse = min(0.5, LISSAGE_VITESSE + 0.1 * abs(surprise))
            self.direction[axe] = prevision + confiance * surprise
            self.vitesse[axe] += confiance_vitesse * surprise / ecoule
            # Objet immobile et presque centré : on ne change pas la visée (les
            # servos trembleraient). Sinon, on vise là où il sera dans AVANCE secondes.
            vitesse = self.vitesse[axe]
            vitesse = math.copysign(max(0.0, abs(vitesse) - VITESSE_MIN), vitesse)
            if abs(self.ecart[axe]) > ZONE_MORTE or vitesse != 0:
                self.visee[axe] = self.direction[axe] + vitesse * AVANCE

    def cible(self, ego):
        """Angles (bas, haut) à envoyer à la tourelle maintenant."""
        return (limiter(self.visee[0] - ego.futur(AVANCE_CORPS), CENTRE_BAS),
                limiter(self.visee[1], CENTRE_HAUT))

    def distance(self):
        """Distance estimée de l'objet (m), ou None s'il n'a jamais été vu."""
        return None if self.rayon is None else K_DISTANCE / self.rayon


class Robot:
    """Rassemble caméra, tourelle et roues, et fait tourner la boucle de la tête."""

    def __init__(self, compensation=True, flux=None):
        self.tourelle = Tourelle()
        self.camera = Camera(0)
        self.base = Base()
        self.ego = Ego(self.base, compensation)
        self.regard = Regard()
        self.flux = flux
        self.numero = 0
        self.vu = False             # objet vu dans la dernière image
        self.corps_au_repos = True  # le corps attend que la tourelle s'écarte de son centre
        self.avance_au_repos = True  # le corps attend que l'objet s'éloigne ou s'approche
        self.distance_arret = DISTANCE_ARRET
        self.vitesse_avant, self.vitesse_arriere = VITESSE_SUIVI, VITESSE_RECUL
        self.ecart_corps = None     # angle lissé de la tourelle vu par le corps (rotation_corps)
        self.variation_corps = 0.0  # sa vitesse de variation (deg/s)
        self.heure_corps = 0.0
        self.journal = None         # fichier CSV ouvert (option --journal), une ligne par image
        self.chercher = False       # chercher l'objet quand il est perdu
        self.discret = False        # ne pas afficher le tableau de suivi
        self.etat = 'suivi'         # 'suivi', 'tete' (le 8), 'corps' (le tour), 'perdu'
        self.derniere_vue = time.perf_counter()
        self.debut = self.derniere_vue      # heure de départ du programme, pour les messages
        self.debut_phase = 0.0      # heure d'entrée dans l'état de recherche en cours
        self.cote = 1               # sens du tour de recherche : +1 à gauche, -1 à droite
        self.lacet_recherche = 0.0  # lacet du corps au début du tour
        self.rotation_recherche = 0.0   # rotation demandée au corps par la recherche (rad/s)
        self.recherches = []        # durée de chaque perte suivie de retrouvailles (s)
        self.vues_de_suite = 0      # nombre d'images de suite où l'objet a été vu

    def dire(self, texte):
        print(f'  [{time.perf_counter() - self.debut:6.2f} s] {texte}', flush=True)

    def rechercher(self, maintenant, cible):
        """Objet non vu : renvoie les angles de la tourelle, et règle rotation_recherche."""
        if maintenant - self.derniere_vue < PERTE:
            return cible        # perte courte : la tête garde la direction, le corps finit son geste
        if self.etat == 'suivi':
            # Côté où l'objet a disparu : là où pointait la tourelle, sinon là où il allait
            angle = self.regard.visee[0] - self.ego.valeur(maintenant) - CENTRE_BAS
            if abs(angle) < 3:
                angle = self.regard.vitesse[0]
            self.cote = -1 if angle < 0 else 1
            self.etat, self.debut_phase = 'tete', maintenant
            self.dire('Objet perdu : recherche avec la tête')
        t = maintenant - self.debut_phase
        if self.etat == 'tete':
            if t < DUREE_RECHERCHE:
                return recherche(t)
            self.etat, self.debut_phase, t = 'corps', maintenant, 0.0
            self.lacet_recherche = self.base.lacet()
            self.dire(f'Recherche avec le corps : un tour à {"gauche" if self.cote > 0 else "droite"}')
        if self.etat == 'corps':
            if abs(self.base.lacet() - self.lacet_recherche) < 2 * math.pi + 0.2:
                self.rotation_recherche = self.cote * math.radians(ROTATION_RECHERCHE)
                return (CENTRE_BAS, HAUT_RECHERCHE[0]
                        + HAUT_RECHERCHE[1] * math.sin(2 * math.pi * t / PERIODE_HAUT_RECHERCHE))
            self.etat, self.debut_phase = 'perdu', maintenant
            self.dire(f'Objet introuvable : arrêt, nouvelle recherche dans {PAUSE_PERDU:.0f} s')
        if maintenant - self.debut_phase > PAUSE_PERDU:
            self.etat, self.debut_phase = 'tete', maintenant
            self.dire('Nouvelle recherche avec la tête')
        return (CENTRE_BAS, CENTRE_HAUT)

    def pas(self):
        """Une image : estimation, puis ordre à la tourelle. Renvoie l'heure de l'image."""
        self.numero, heure, image = self.camera.suivante(self.numero)
        objet = trouver_objet(image)
        self.vu = objet is not None
        maintenant = time.perf_counter()
        self.rotation_recherche = 0.0
        # Pendant une recherche, une seule image ne suffit pas : un reflet ou une
        # petite tache verte du décor arrêterait la recherche pour rien. Il faut
        # voir l'objet sur CONFIRMATION images de suite.
        self.vues_de_suite = self.vues_de_suite + 1 if self.vu else 0
        if self.etat != 'suivi' and self.vues_de_suite < CONFIRMATION:
            self.vu = False
        if self.vu:
            if self.etat != 'suivi':
                self.recherches.append(maintenant - self.derniere_vue)
                self.dire(f'Objet retrouvé après {maintenant - self.derniere_vue:.1f} s')
                self.etat = 'suivi'
            self.derniere_vue = maintenant
            self.regard.voir(heure, objet, self.tourelle, self.ego)
            position = self.tourelle.position(maintenant)
            self.ego.recaler(maintenant, self.regard.rayon, position[0] - CENTRE_BAS)
        cible = self.regard.cible(self.ego)
        if not self.vu and self.chercher:
            cible = self.rechercher(maintenant, cible)
        self.tourelle.viser(*cible)
        if self.journal is not None:
            bas, haut = self.tourelle.position(heure - RETARD)
            _, _, theta, parcouru = self.base.etat(heure - RETARD_CORPS)
            x, y, rayon = objet if self.vu else (-1, -1, -1)
            self.journal.write(f'{heure:.4f},{int(self.vu)},{x:.1f},{y:.1f},{rayon:.1f},{bas:.2f},{haut:.2f},'
                               f'{math.degrees(theta):.3f},{parcouru:.4f},{self.ego.valeur(heure - RETARD_CORPS):.3f},'
                               f'{cible[0]:.2f},{cible[1]:.2f},{self.regard.direction[0]:.2f},'
                               f'{self.regard.vitesse[0]:.1f},{self.base.applique[0]:.3f},{self.base.applique[1]:.3f}\n')
        if self.flux is not None:
            if self.vu:
                cv2.circle(image, (round(objet[0]), round(objet[1])), round(objet[2]), (0, 255, 0), 2)
            cv2.drawMarker(image, (LARGEUR // 2, HAUTEUR // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
            self.flux.publier(image)
        return heure

    def fermer(self):
        self.base.fermer()
        self.camera.fermer()
        self.tourelle.fermer()


def bilan(titre, ecarts, images):
    """Affiche l'écart de l'objet au centre de l'image (px, axe gauche-droite)."""
    if not ecarts:
        print(f'{titre} : objet jamais vu sur {images} images')
        return
    absolus = sorted(abs(e) for e in ecarts)
    print(f'{titre} : vu {len(ecarts)}/{images} images | écart médian {absolus[len(absolus) // 2]:.0f} px, '
          f'9 fois sur 10 sous {absolus[int(len(absolus) * 0.9)]:.0f} px, maximum {absolus[-1]:.0f} px')


def essai_pivot(robot, rotation_max, periode=2.0, duree=8.0):
    """Corps immobile 2 s, puis va-et-vient en rotation ; mesure l'écart de l'objet au centre."""
    for titre, duree_phase, bouge in (('Corps immobile', 2.0, False),
                                      (f'Corps en va-et-vient à {math.degrees(rotation_max):.0f} deg/s', duree, True)):
        ecarts, images = [], 0
        lacets, vitesses = [], []
        t0 = time.perf_counter()
        while (t := time.perf_counter() - t0) < duree_phase:
            # Cosinus : le corps oscille de part et d'autre de son orientation de départ
            rotation = rotation_max * math.cos(2 * math.pi * t / periode) if bouge else 0.0
            robot.base.commander(0.0, rotation)
            robot.pas()
            images += 1
            lacets.append(math.degrees(robot.base.lacet()))
            if robot.vu:
                ecarts.append(robot.regard.ecart[0])
                # Vitesse à laquelle le corps fait défiler l'objet (degrés de servo par seconde)
                heure = robot.regard.heure - RETARD_CORPS
                vitesses.append((robot.ego.valeur(heure) - robot.ego.valeur(heure - 0.06)) / 0.06)
        bilan(titre, ecarts, images)
        if bouge and robot.ego.actif:
            # Si la tourelle compense en retard, l'écart est proportionnel à cette vitesse
            pente = sum(e * v for e, v in zip(ecarts, vitesses)) / sum(v * v for v in vitesses)
            print(f'    retard restant de la compensation : {pente / PX_PAR_DEGRE_BAS * 1000:+.0f} ms '
                  f'(avance utilisée : {AVANCE_CORPS * 1000:.0f} ms)')
        if bouge:
            print(f'    lacet du corps : {max(lacets) - min(lacets):.0f} deg de débattement, '
                  f'distance estimée de l\'objet {robot.regard.distance() or 0:.2f} m')
    fin = time.perf_counter() + 0.8
    while time.perf_counter() < fin:      # arrêt en douceur, la tête suit toujours
        robot.base.commander(0.0, 0.0)
        robot.pas()


def rotation_corps(robot):
    """Rotation à demander au corps (rad/s) pour ramener la tourelle vers son centre.

    Le corps ne regarde pas l'image : il suit la tête. Plus la tourelle est
    tournée, plus il pivote vite ; si l'objet se déplace, il prend aussi sa vitesse.
    """
    regard, ego = robot.regard, robot.ego
    maintenant = time.perf_counter()
    if regard.heure is None or maintenant - regard.heure > PERTE:
        robot.ecart_corps = None
        return 0.0      # objet jamais vu, ou perdu : le corps attend
    # De combien le corps doit pivoter (degrés réels) pour que la tourelle,
    # toujours pointée sur l'objet, se retrouve à son centre
    ecart = (regard.visee[0] - ego.valeur(maintenant) - CENTRE_BAS) / ego.gain
    # Vitesse à laquelle cet écart change (lissée sur LISSAGE_CORPS secondes) :
    # si la tourelle s'écarte de plus en plus vite, le corps accélère sans attendre
    if robot.ecart_corps is None:
        robot.ecart_corps, robot.variation_corps = ecart, 0.0
    else:
        duree = max(maintenant - robot.heure_corps, 0.005)
        lisse = robot.ecart_corps + min(1.0, duree / LISSAGE_CORPS) * (ecart - robot.ecart_corps)
        robot.variation_corps = (lisse - robot.ecart_corps) / duree
        robot.ecart_corps = lisse
    robot.heure_corps = maintenant
    # Au repos, le corps ne part que si la tourelle s'écarte nettement. En mouvement,
    # il continue jusqu'à être presque aligné. Sans ces deux seuils, il ramperait
    # sans fin pour gagner les derniers degrés.
    if robot.corps_au_repos:
        if abs(ecart) < ZONE_DEPART:
            return 0.0
        robot.corps_au_repos = False
    elif abs(ecart) < ZONE_CALME and abs(robot.base.applique[1]) < math.radians(8):
        robot.corps_au_repos = True
        return 0.0
    rotation = GAIN_CORPS * ecart + GAIN_VARIATION * robot.variation_corps
    return math.radians(min(max(rotation, -ROTATION_SUIVI), ROTATION_SUIVI))


def vitesse_corps(robot):
    """Vitesse d'avance à demander au corps (m/s) pour se placer à DISTANCE_ARRET de l'objet.

    À appeler après rotation_corps (qui calcule l'angle de la tourelle).
    """
    regard = robot.regard
    if regard.heure is None or time.perf_counter() - regard.heure > PERTE:
        robot.avance_au_repos = True
        return 0.0      # objet jamais vu, ou perdu : le robot n'avance pas à l'aveugle
    erreur = regard.distance() - robot.distance_arret
    if erreur > 0 and time.perf_counter() - regard.heure > 0.3:
        return 0.0      # objet non vu depuis 0,3 s : on n'avance pas sur un souvenir
    # Mêmes deux seuils que pour la rotation : pas de va-et-vient autour du but
    if robot.avance_au_repos:
        if abs(erreur) < DISTANCE_DEPART:
            return 0.0
        robot.avance_au_repos = False
    elif abs(erreur) < DISTANCE_CALME and abs(robot.base.applique[0]) < 0.03:
        robot.avance_au_repos = True
        return 0.0
    # Loin du but : proportionnel à l'erreur. Près du but : pas plus vite que ce
    # qu'un freinage de FREINAGE m/s² permet d'arrêter sur la distance restante
    # (vitesse = racine de 2 x freinage x distance), sinon le robot dépasse son but.
    if erreur < 0:
        # Objet trop près : on recule franchement, même s'il est sur le côté.
        # (Mesuré : en ralentissant le recul pour un objet de côté, le robot
        # restait sur place et l'objet, à 11 cm, sortait de l'image.)
        return -min(GAIN_RECUL * -erreur, math.sqrt(2 * FREINAGE * -erreur), robot.vitesse_arriere)
    vitesse = min(GAIN_AVANCE * erreur, math.sqrt(2 * FREINAGE * erreur), robot.vitesse_avant)
    # Objet sur le côté : on pivote d'abord, on avance à mesure que le corps s'aligne
    angle = abs(robot.ecart_corps or 0.0)
    return vitesse * max(0.0, 1.0 - angle / ANGLE_SANS_AVANCE)


def essai_vitesses(robot, couples, loin=1.0):
    """Épreuve identique pour chaque couple (vitesse avant, vitesse arrière), objet immobile :
    le robot recule jusqu'à `loin` mètres de l'objet, puis revient à DISTANCE_ARRET.
    Pour chaque trajet : vitesse atteinte, dépassement du but, tenue de l'image."""
    origine = robot.base.etat()[3]
    print('  avant/arrière  trajet    durée  vitesse max  dépassement  arrêt à   écart image (médian / 9 sur 10)  non vu  tourelle max')
    for avant, arriere in couples:
        robot.vitesse_avant, robot.vitesse_arriere = avant, arriere
        for trajet, but in (('recul ', loin), ('avance', DISTANCE_ARRET)):
            robot.distance_arret = but
            robot.avance_au_repos = False
            ecarts, distances, vitesses, tourelles, positions = [], [], [], [], []
            images = non_vus = 0
            t0 = time.perf_counter()
            calme = None        # heure depuis laquelle le robot est arrêté au but
            while (t := time.perf_counter() - t0) < 15:
                robot.pas()
                rotation = rotation_corps(robot)
                vitesse = vitesse_corps(robot)
                # Sécurité : jamais plus de 1,5 m en arrière ni 0,3 m en avant du départ
                parcouru = robot.base.etat()[3] - origine
                if (parcouru < -1.5 and vitesse < 0) or (parcouru > 0.3 and vitesse > 0):
                    vitesse = 0.0
                robot.base.commander(vitesse, rotation)
                images += 1
                non_vus += not robot.vu
                maintenant = time.perf_counter()
                vitesses.append((robot.base.etat(maintenant)[3] - robot.base.etat(maintenant - 0.1)[3]) / 0.1)
                positions.append(robot.base.etat(maintenant)[3])
                if robot.vu:
                    distances.append(robot.regard.distance())
                    if abs(vitesses[-1]) > 0.02:
                        ecarts.append(abs(robot.regard.ecart[0]))
                        tourelles.append(abs(robot.tourelle.position(maintenant)[0] - CENTRE_BAS))
                if robot.avance_au_repos and abs(vitesses[-1]) < 0.01:
                    calme = calme or maintenant
                    if maintenant - calme > 1.0:
                        break
                else:
                    calme = None
            ecarts.sort()
            fin = sorted(distances[-20:])[len(distances[-20:]) // 2] if distances else 0
            # Dépassement : de combien le robot est allé plus loin que l'endroit où il
            # s'est finalement arrêté (mesuré par les encodeurs, plus précis que l'image)
            au_dela = positions[-1] - min(positions) if but == loin else max(positions) - positions[-1]
            print(f'  {avant:.2f} / {arriere:.2f}    {trajet}  {t:5.1f} s  {max(vitesses, key=abs):+8.2f} m/s  '
                  f'{max(au_dela, 0) * 100:8.0f} cm  {fin:6.2f} m  '
                  f'{ecarts[len(ecarts) // 2] if ecarts else 0:10.0f} px / {ecarts[int(len(ecarts) * 0.9)] if ecarts else 0:3.0f} px'
                  f'  {non_vus:14d}  {max(tourelles) if tourelles else 0:8.0f} deg', flush=True)
    robot.distance_arret = DISTANCE_ARRET


def essai_suivi(robot, duree, avancer=False):
    """Le corps suit la tête pendant `duree` secondes : rotation seule, ou aussi
    avance vers l'objet si `avancer`."""
    releves = []        # (t, vu, écart px, angle restant du corps en degrés, ordre de rotation deg/s, distance)
    t0 = time.perf_counter()
    prochain = 0.0
    depart = robot.base.lacet()
    parcouru = robot.base.etat()[3]
    print('  t (s)  vu  écart (px)  tourelle (deg)  lacet (deg)  rotation (deg/s)  distance (m)  avance (m/s)  parcouru (m)')
    while (t := time.perf_counter() - t0) < duree:
        robot.pas()
        rotation = rotation_corps(robot) + robot.rotation_recherche
        robot.base.commander(vitesse_corps(robot) if avancer else 0.0, rotation)
        tourelle = (robot.tourelle.position(time.perf_counter())[0] - CENTRE_BAS) / robot.ego.gain
        ordre = math.degrees(robot.base.applique[1])
        releves.append((t, robot.vu, robot.regard.ecart[0], tourelle, ordre, robot.regard.distance() or 0))
        if t >= prochain and not robot.discret:
            prochain += 0.25
            print(f'  {t:5.2f}  {"oui" if robot.vu else "non"}  {robot.regard.ecart[0]:+8.0f}  {tourelle:+12.1f}  '
                  f'{math.degrees(robot.base.lacet() - depart):+11.1f}  {ordre:+12.0f}  '
                  f'{robot.regard.distance() or 0:12.2f}  {robot.base.applique[0]:+10.2f}  '
                  f'{robot.base.etat()[3] - parcouru:+10.2f}')
    vus = [r for r in releves if r[1]]
    bilan('Suivi', [r[2] for r in vus], len(releves))
    if robot.recherches:
        print(f'    {len(robot.recherches)} recherche(s) réussie(s), en '
              + ', '.join(f'{d:.1f} s' for d in robot.recherches))
    if vus and avancer:
        fin = [r[5] for r in vus if r[0] > duree - 2]
        print(f'    distance de l\'objet : {vus[0][5]:.2f} m au début, {sorted(fin)[len(fin) // 2] if fin else 0:.2f} m '
              f'sur les 2 dernières secondes (but : {DISTANCE_ARRET:.2f} m) ; plus près : {min(r[5] for r in vus):.2f} m')
    if vus:
        # Aligné : tourelle à moins de 5° de son centre, et elle y reste
        hors = [r[0] for r in releves if r[0] >= vus[0][0] and abs(r[3]) > 5]
        sens = [math.copysign(1, r[4]) for r in releves if abs(r[4]) > 3]
        inversions = sum(1 for a, b in zip(sens, sens[1:]) if a != b)
        print(f'    objet vu à {vus[0][0]:.2f} s, tourelle à {vus[0][3]:+.0f} deg ; corps aligné à 5 deg près à '
              f'{(hors[-1] if hors else vus[0][0]):.2f} s ; {inversions} inversion(s) du sens de rotation')
    fin = time.perf_counter() + 0.6
    while time.perf_counter() < fin:
        robot.base.commander(0.0, 0.0)
        robot.pas()


def demande_d_arret(*_):
    raise KeyboardInterrupt


def main():
    arguments = argparse.ArgumentParser(description='Tête et corps ensemble.')
    arguments.add_argument('--essai', choices=['pivot', 'suivi', 'rejoindre', 'vitesses'], default='pivot',
                           help='pivot : le corps va et vient ; suivi : il pivote pour suivre l\'objet ; '
                                'rejoindre : il avance aussi jusqu\'à DISTANCE_ARRET ; vitesses : recul puis '
                                'retour vers l\'objet immobile, pour chaque couple de --couples')
    arguments.add_argument('--couples', default='0.50/0.50',
                           help='vitesses avant/arrière à essayer (m/s), ex. 0.25/0.15,0.35/0.25')
    arguments.add_argument('--duree', type=float, default=8, help='durée de l\'essai de suivi (s)')
    arguments.add_argument('--rotation', type=float, default=60, help='vitesse de rotation maximale du corps (deg/s)')
    arguments.add_argument('--avance-corps', type=float, help='essayer une autre valeur de AVANCE_CORPS (s)')
    arguments.add_argument('--journal', help='fichier CSV : une ligne par image, pour analyser après coup')
    arguments.add_argument('--periode', type=float, default=2.0, help='durée d\'un va-et-vient du corps (s)')
    arguments.add_argument('--sans-recherche', action='store_true', help='objet perdu : le robot attend sans chercher')
    arguments.add_argument('--discret', action='store_true', help='ne pas afficher le tableau 4 fois par seconde')
    arguments.add_argument('--comparer', action='store_true', help='refaire l\'essai sans compensation')
    arguments.add_argument('--flux', type=int, metavar='PORT', nargs='?', const=8081,
                           help='image en direct dans un navigateur (défaut : 8081)')
    options = arguments.parse_args()
    if options.avance_corps is not None:
        global AVANCE_CORPS
        AVANCE_CORPS = options.avance_corps
    for signe in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(signe, demande_d_arret)

    flux = None
    if options.flux:
        from flux_video import FluxVideo
        flux = FluxVideo(options.flux)
    robot = Robot(flux=flux)
    if options.journal:
        robot.journal = open(options.journal, 'w')
        robot.journal.write('heure,vu,x,y,rayon,bas,haut,lacet,parcouru,ego,cible_bas,cible_haut,'
                            'direction,vitesse,ordre_vitesse,ordre_rotation\n')
    try:
        if options.essai == 'vitesses':
            essai_vitesses(robot, [tuple(float(v) for v in c.split('/')) for c in options.couples.split(',')])
        if options.essai == 'rejoindre':
            # Vitesses avant/arrière : le premier couple de --couples
            robot.vitesse_avant, robot.vitesse_arriere = (float(v) for v in options.couples.split(',')[0].split('/'))
        if options.essai in ('suivi', 'rejoindre'):
            robot.chercher = not options.sans_recherche
            robot.discret = options.discret
            robot.derniere_vue = robot.debut = time.perf_counter()
            essai_suivi(robot, options.duree, avancer=options.essai == 'rejoindre')
        for compensation in ([True, False] if options.comparer else [True]) if options.essai == 'pivot' else []:
            robot.ego.recaler(time.perf_counter())
            robot.ego.actif = compensation
            robot.regard = Regard()
            print('--- Avec compensation ---' if compensation else '--- Sans compensation ---')
            essai_pivot(robot, math.radians(options.rotation), options.periode)
    except KeyboardInterrupt:
        print('Arrêt demandé')
    finally:
        robot.fermer()
        if robot.journal is not None:
            robot.journal.close()
        if flux is not None:
            flux.fermer()


if __name__ == '__main__':
    main()
