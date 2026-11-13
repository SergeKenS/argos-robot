"""Outils communs aux tests : caméra, détection de la balle, liaison avec l'ESP32."""
import threading
import time

import cv2
import numpy as np
import serial
from serial.tools import list_ports

CENTRE = (100.0, 30.0)
LARGEUR, HAUTEUR = 640, 480

# Rouge de la balle mesuré le 2026-11-12 (lumière chaude) : teinte 168 à 8
SAT_MIN, LUM_MIN, TEINTE_BAS, TEINTE_HAUT = 140, 50, 168, 8
AIRE_MIN = 60
NOYAU = np.ones((3, 3), np.uint8)


def trouver_balle(image):
    """Renvoie (x, y, aire) de la plus grande tache rouge compacte, ou None. x, y au sous-pixel."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    masque = (cv2.inRange(hsv, np.array([0, SAT_MIN, LUM_MIN]), np.array([TEINTE_HAUT, 255, 255]))
              | cv2.inRange(hsv, np.array([TEINTE_BAS, SAT_MIN, LUM_MIN]), np.array([180, 255, 255])))
    masque = cv2.morphologyEx(masque, cv2.MORPH_OPEN, NOYAU)
    n, _, stats, centres = cv2.connectedComponentsWithStats(masque)
    meilleur = None
    for i in range(1, n):
        aire = stats[i, cv2.CC_STAT_AREA]
        l, h = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
        # Compacte : remplit bien son rectangle, ni trop large ni trop haute
        if aire < AIRE_MIN or aire < 0.5 * l * h or not 0.5 < l / h < 2:
            continue
        if meilleur is None or aire > meilleur[2]:
            meilleur = (float(centres[i][0]), float(centres[i][1]), int(aire))
    return meilleur


class Camera:
    """Lit la caméra dans un fil à part : on a toujours la dernière image, avec son heure d'arrivée."""

    def __init__(self, numero=1, backend=cv2.CAP_MSMF, codec=None, fps=None):
        self.capture = cv2.VideoCapture(numero, backend)
        if codec:
            self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*codec))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, LARGEUR)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, HAUTEUR)
        if fps:
            self.capture.set(cv2.CAP_PROP_FPS, fps)
        for _ in range(30):
            self.capture.read()
        self.verrou = threading.Condition()
        self.numero = 0
        self.image = None
        self.heure = 0.0
        self.actif = True
        threading.Thread(target=self._lire, daemon=True).start()

    def _lire(self):
        while self.actif:
            ok, image = self.capture.read()
            heure = time.perf_counter()
            if not ok:
                continue
            with self.verrou:
                self.image, self.heure = image, heure
                self.numero += 1
                self.verrou.notify_all()

    def suivante(self, apres=0):
        """Attend une image plus récente que la numéro `apres` ; renvoie (numéro, heure, image)."""
        with self.verrou:
            self.verrou.wait_for(lambda: self.numero > apres, timeout=1)
            return self.numero, self.heure, self.image

    def fermer(self):
        self.actif = False
        time.sleep(0.1)
        self.capture.release()


class Tourelle:
    """Liaison série avec suivi_fluide.ino : envoie les cibles, garde l'historique des positions reçues."""

    def __init__(self):
        port = next((p.device for p in list_ports.comports() if p.vid == 0x1A86), None)
        if port is None:
            raise SystemExit('ESP32 introuvable')
        self.serie = serial.Serial(port, 115200, timeout=0.05)
        self.historique = [(time.perf_counter(), *CENTRE)]  # (heure PC, bas, haut)
        self.actif = True
        threading.Thread(target=self._ecouter, daemon=True).start()
        time.sleep(2.5)  # l'ouverture du port redémarre l'ESP32

    def _ecouter(self):
        while self.actif:
            brut = self.serie.readline()
            if brut.startswith(b'P,'):
                try:
                    _, _, bas, haut = brut.decode().strip().split(',')
                    self.historique.append((time.perf_counter(), float(bas), float(haut)))
                except ValueError:
                    pass

    def envoyer(self, texte):
        self.serie.write((texte + '\n').encode())

    def viser(self, bas, haut):
        self.envoyer(f'{bas:.2f},{haut:.2f}')

    def reglages(self, vitesse=300, acceleration=2000, gain=15):
        self.envoyer(f'V,{vitesse}')
        self.envoyer(f'A,{acceleration}')
        self.envoyer(f'G,{gain}')

    def position(self, heure):
        """Consigne (bas, haut) que l'ESP32 appliquait à cette heure PC."""
        for h, bas, haut in reversed(self.historique):
            if h <= heure:
                return bas, haut
        return self.historique[0][1:]

    def fermer(self):
        self.viser(*CENTRE)
        time.sleep(0.8)
        self.actif = False
        time.sleep(0.1)
        self.serie.close()
