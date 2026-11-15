"""Liaison avec les roues pour l'essai « tête + corps » (sans ROS).

Rôle : le même que la classe Tourelle de suivi_fluide.py, pour le corps.
  - commander(vitesse, rotation) : ce qu'on veut (m/s, rad/s). Un fil d'exécution
    y amène les roues en douceur, 50 fois par seconde, avec une accélération limitée.
  - lacet(heure) : orientation du corps (radians) à une heure passée, calculée
    avec les encodeurs. Sert à savoir où regardait le corps quand une image a été prise.
  - chien de garde : plus d'ordre depuis 0,5 s, les roues s'arrêtent.

Carte moteurs Yahboom YB-ESF01 : protocole et mesures dans essais/moteurs/README.md.
Régulateur de la carte réglé sur P 0,8 / I 0,6 / D 0,5 (réponse en 0,2 s environ).

Essai seul (le robot pivote à gauche puis à droite, environ 1 m libre autour) :
  python3 base.py
"""
import math
import re
import signal
import threading
import time
from collections import deque

import serial
from serial.tools import list_ports

# Mesuré (essais/moteurs/README.md)
ENTRAXE = 0.122                       # m, entraxe effectif (tours complets vérifiés à la caméra)
M_PAR_IMPULSION = math.pi * 0.067 / 1760   # roue de 67 mm, 1760 impulsions par tour

# Limites du corps
VITESSE_MAX = 0.70        # m/s (roues justes à 2 % jusque-là, essais/moteurs)
ROTATION_MAX = 2.5        # rad/s (environ 140 °/s)
ROUE_MAX = 0.80           # m/s par roue
ACCELERATION = 0.6        # m/s²
ACCELERATION_ROTATION = 8.0   # rad/s²
PERIODE = 0.02            # s : un ordre à la carte toutes les 20 ms
CHIEN_DE_GARDE = 0.5      # s sans ordre avant l'arrêt

MOTIF = re.compile(rb'\$MAll:(-?\d+),(-?\d+),(-?\d+),(-?\d+)#')


def approcher(valeur, cible, pas):
    """Rapproche `valeur` de `cible`, de `pas` au plus."""
    return valeur + min(max(cible - valeur, -pas), pas)


class Base:
    """Roues du robot : ordres lissés et position calculée avec les encodeurs."""

    def __init__(self):
        # Carte moteurs : puce USB 1A86:7522 (l'ESP32 de la tourelle est 1A86:7523)
        port = next((p.device for p in list_ports.comports()
                     if (p.vid, p.pid) == (0x1A86, 0x7522)), None)
        if port is None:
            raise SystemExit('Carte moteurs introuvable : est-elle branchée ?')
        self.serie = serial.Serial()
        self.serie.port, self.serie.baudrate, self.serie.timeout = port, 115200, 0.005
        self.serie.dtr = self.serie.rts = False   # lignes de contrôle au repos
        self.serie.exclusive = True               # refuse si pont_moteurs tient déjà le port
        self.serie.open()
        time.sleep(0.5)            # une commande écrite tout de suite se perd
        self.serie.reset_input_buffer()

        self.verrou = threading.Lock()
        # Historique : (heure, x, y, lacet, distance parcourue), environ 300 par seconde
        self.historique = deque(maxlen=3000)
        self.x = self.y = self.theta = self.distance = 0.0
        self.compteurs = None                 # derniers compteurs (gauche, droite) en mètres
        self.voulu = (0.0, 0.0)               # ce qu'on demande (m/s, rad/s)
        self.applique = (0.0, 0.0)            # ce qui est envoyé, en route vers `voulu`
        self.heure_ordre = 0.0
        self.arrete = True                    # arrêt déjà envoyé à la carte
        self.actif = True
        self._ecrire('$upload:1,0,0#')
        threading.Thread(target=self._ecouter, daemon=True).start()
        threading.Thread(target=self._piloter, daemon=True).start()
        # Attend les premiers compteurs (redemandés s'ils ne viennent pas)
        fin = time.perf_counter() + 3
        while not self.historique:
            if time.perf_counter() > fin:
                self.fermer()
                raise SystemExit('La carte moteurs n\'envoie pas ses compteurs')
            time.sleep(0.3)
            if not self.historique:
                self._ecrire('$upload:1,0,0#')

    def _ecrire(self, texte):
        self.serie.write(texte.encode())

    def _ecouter(self):
        """Fil à part : lit les compteurs et met la position à jour."""
        tampon = b''
        while self.actif:
            try:
                tampon += self.serie.read(256)
            except (serial.SerialException, OSError, TypeError):
                time.sleep(0.05)
                continue
            heure = time.perf_counter()
            trouves = MOTIF.findall(tampon)
            if not trouves:
                continue
            tampon = tampon[tampon.rfind(b'#') + 1:]
            c = [int(v) for v in trouves[-1]]
            # Motor 4 = roue gauche (montée en miroir), Motor 3 = roue droite
            gauche, droite = -c[3] * M_PAR_IMPULSION, c[2] * M_PAR_IMPULSION
            with self.verrou:
                if self.compteurs is not None:
                    d_gauche, d_droite = gauche - self.compteurs[0], droite - self.compteurs[1]
                    avance = (d_gauche + d_droite) / 2
                    rotation = (d_droite - d_gauche) / ENTRAXE
                    self.x += avance * math.cos(self.theta + rotation / 2)
                    self.y += avance * math.sin(self.theta + rotation / 2)
                    self.theta += rotation      # non ramené à ±180° : pas de saut à gérer
                    self.distance += avance
                self.compteurs = (gauche, droite)
                self.historique.append((heure, self.x, self.y, self.theta, self.distance))

    def _piloter(self):
        """Fil à part : amène les roues vers l'ordre voulu, sans à-coup."""
        suivant = time.perf_counter()
        while self.actif:
            suivant += PERIODE
            time.sleep(max(0.0, suivant - time.perf_counter()))
            vitesse, rotation = self.voulu
            if time.perf_counter() - self.heure_ordre > CHIEN_DE_GARDE:
                vitesse, rotation = 0.0, 0.0
            v = approcher(self.applique[0], vitesse, ACCELERATION * PERIODE)
            w = approcher(self.applique[1], rotation, ACCELERATION_ROTATION * PERIODE)
            self.applique = (v, w)
            gauche, droite = v - w * ENTRAXE / 2, v + w * ENTRAXE / 2
            # Roue trop rapide : on ralentit les deux, la courbe est gardée
            plus_rapide = max(abs(gauche), abs(droite))
            if plus_rapide > ROUE_MAX:
                gauche, droite = gauche * ROUE_MAX / plus_rapide, droite * ROUE_MAX / plus_rapide
            mm_gauche, mm_droite = round(gauche * 1000), round(droite * 1000)
            try:
                if mm_gauche == 0 and mm_droite == 0:
                    if not self.arrete:
                        self._ecrire('$spd:0,0,0,0#')
                        self.arrete = True
                else:
                    self._ecrire(f'$spd:0,0,{mm_droite},{-mm_gauche}#')
                    self.arrete = False
            except (serial.SerialException, OSError, TypeError):
                pass

    def commander(self, vitesse, rotation):
        """Vitesse (m/s, + = avant) et rotation (rad/s, + = gauche) voulues. À répéter :
        sans ordre pendant CHIEN_DE_GARDE secondes, les roues s'arrêtent."""
        self.voulu = (min(max(vitesse, -VITESSE_MAX), VITESSE_MAX),
                      min(max(rotation, -ROTATION_MAX), ROTATION_MAX))
        self.heure_ordre = time.perf_counter()

    def etat(self, heure=None):
        """(x, y, lacet, distance) à `heure` (time.perf_counter), ou maintenant.

        Entre deux relevés on interpole ; après le dernier, on garde le dernier.
        """
        with self.verrou:
            points = list(self.historique)
        if heure is None or heure >= points[-1][0]:
            return points[-1][1:]
        apres = points[-1]
        for point in reversed(points):
            if point[0] <= heure:
                duree = apres[0] - point[0]
                part = (heure - point[0]) / duree if duree > 0 else 0.0
                return tuple(a + part * (b - a) for a, b in zip(point[1:], apres[1:]))
            apres = point
        return points[0][1:]

    def lacet(self, heure=None):
        """Orientation du corps (radians, + = gauche) à `heure`, ou maintenant."""
        return self.etat(heure)[2]

    def vitesse_lacet(self, duree=0.06):
        """Vitesse de rotation mesurée (rad/s) sur les `duree` dernières secondes."""
        maintenant = time.perf_counter()
        return (self.lacet(maintenant) - self.lacet(maintenant - duree)) / duree

    def fermer(self):
        """Arrête les roues (arrêt rapide de la carte) et ferme le port."""
        self.actif = False
        time.sleep(2 * PERIODE)
        try:
            self._ecrire('$pwm:0,0,0,0#')
            time.sleep(0.05)
            self._ecrire('$upload:0,0,0#')
        finally:
            self.serie.close()


def demande_d_arret(*_):
    raise KeyboardInterrupt


def essai():
    """Aller-retour en rotation : affiche la consigne lissée et la rotation mesurée."""
    for signe in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(signe, demande_d_arret)
    base = Base()
    etapes = [(1.5, 1.2), (-1.5, 1.2), (0.4, 1.0), (0.0, 0.8)]   # (rad/s, durée)
    ecarts = []
    try:
        depart = base.lacet()
        print('  t (s)  consigne  mesure (deg/s)   lacet (deg)')
        t0 = time.perf_counter()
        fin_etape, prochain = t0, t0
        for rotation, duree in etapes:
            fin_etape += duree
            while time.perf_counter() < fin_etape:
                base.commander(0.0, rotation)
                time.sleep(0.02)
                maintenant = time.perf_counter()
                consigne, mesure = base.applique[1], base.vitesse_lacet()
                ecarts.append(abs(consigne - mesure))
                if maintenant >= prochain:
                    prochain += 0.1
                    print(f'  {maintenant - t0:5.2f}  {math.degrees(consigne):+7.0f}  '
                          f'{math.degrees(mesure):+7.0f}        '
                          f'{math.degrees(base.lacet() - depart):+7.1f}')
        ecarts.sort()
        print(f'Écart consigne-mesure : médiane {math.degrees(ecarts[len(ecarts) // 2]):.0f} deg/s, '
              f'9 fois sur 10 sous {math.degrees(ecarts[int(len(ecarts) * 0.9)]):.0f} deg/s ; '
              f'{len(base.historique) / (time.perf_counter() - t0 + 0.3):.0f} relevés/s')
    except KeyboardInterrupt:
        print('Arrêt demandé')
    finally:
        base.fermer()


if __name__ == '__main__':
    essai()
