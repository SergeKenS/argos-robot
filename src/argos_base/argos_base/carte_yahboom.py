"""Liaison avec la carte moteurs Yahboom YB-ESF01 (programme « Motor_Version:1.7.3 »).

Protocole vérifié sur la carte elle-même (voir essais/moteurs/README.md) :
  liaison série 115 200 bauds, commandes texte « $...# »
  $spd:a,b,c,d#     vitesse des 4 sorties en mm/s, asservie par la carte
  $pwm:0,0,0,0#     arrêt rapide (0,2 à 0,4 s, contre 2,5 s pour $spd:0)
  $upload:1,0,0#    la carte envoie sans arrêt « $MAll:a,b,c,d# », les compteurs
                    totaux des 4 encodeurs ; $upload:0,0,0# arrête ces envois
Attention : la carte garde la dernière vitesse reçue tant qu'on ne lui dit rien
(pas de chien de garde). C'est au programme qui la pilote de l'arrêter.

Ce fichier ne dépend pas de ROS : il se teste seul (test/test_carte_yahboom.py).
"""
import re
import threading
import time

import serial
from serial.tools import list_ports

# Puce USB de la carte : CH340, identifiant 1a86:7522 (l'ESP32 de la tourelle est 1a86:7523)
VID, PID = 0x1A86, 0x7522

MOTIF_COMPTEURS = re.compile(r'\$MAll:(-?\d+),(-?\d+),(-?\d+),(-?\d+)#')

# Plus de compteurs depuis ce temps (s) : on redemande les envois à la carte.
# Mesuré : la première commande écrite juste après l'ouverture du port se perd.
DELAI_RELANCE = 1.0


def trouver_port():
    """Port série de la carte (ex. /dev/ttyUSB1), reconnue à sa puce USB ; None si absente."""
    for port in list_ports.comports():
        if port.vid == VID and port.pid == PID:
            return port.device
    return None


def lire_compteurs(ligne):
    """« $MAll:a,b,c,d# » → (a, b, c, d) en entiers ; toute autre ligne → None."""
    trouve = MOTIF_COMPTEURS.search(ligne)
    if trouve is None:
        return None
    return tuple(int(valeur) for valeur in trouve.groups())


class CarteYahboom:
    """Envoie les ordres à la carte et garde ses derniers compteurs d'encodeurs.

    Un fil d'exécution à part lit la carte sans arrêt : on a toujours les
    compteurs les plus récents, sans jamais attendre.
    """

    def __init__(self, port, liaison=None):
        """Ouvre le port série (ou utilise « liaison », pour les tests) et lance la lecture."""
        if liaison is None:
            liaison = serial.Serial()
            liaison.port = port
            liaison.baudrate = 115200
            liaison.timeout = 0.05
            # Lignes de contrôle au repos : sur ce genre de carte, elles peuvent
            # servir à la redémarrer ou à la reprogrammer.
            liaison.dtr = False
            liaison.rts = False
            liaison.open()
            # Laisse la liaison se stabiliser : une commande écrite tout de suite se perd
            time.sleep(0.5)
            liaison.reset_input_buffer()
        self.liaison = liaison
        self.verrou = threading.Lock()           # protège les compteurs
        self.verrou_ecriture = threading.Lock()  # deux fils peuvent écrire
        self.compteurs = None        # derniers compteurs reçus (a, b, c, d)
        self.heure_compteurs = None  # heure de leur réception (time.monotonic)
        self.derniere_demande = time.monotonic()
        self.actif = True
        self.envoyer('$upload:1,0,0#')
        self.fil = threading.Thread(target=self._ecouter, daemon=True)
        self.fil.start()

    def envoyer(self, texte):
        with self.verrou_ecriture:
            self.liaison.write(texte.encode())

    def vitesses(self, mm_par_seconde):
        """Consigne de vitesse des 4 sorties, en mm/s (entiers)."""
        valeurs = ','.join(str(int(round(v))) for v in mm_par_seconde)
        self.envoyer(f'$spd:{valeurs}#')

    def arret_rapide(self):
        self.envoyer('$pwm:0,0,0,0#')

    def derniers_compteurs(self):
        """(heure, (a, b, c, d)) des derniers compteurs reçus, ou (None, None)."""
        with self.verrou:
            return self.heure_compteurs, self.compteurs

    def _ecouter(self):
        tampon = b''
        while self.actif:
            # Plus rien depuis DELAI_RELANCE : demande perdue ou carte redémarrée
            maintenant = time.monotonic()
            dernier = self.heure_compteurs or 0.0
            if (maintenant - dernier > DELAI_RELANCE
                    and maintenant - self.derniere_demande > DELAI_RELANCE):
                self.envoyer('$upload:1,0,0#')
                self.derniere_demande = maintenant
            try:
                tampon += self.liaison.read(256)
            except (serial.SerialException, OSError):
                time.sleep(0.1)
                continue
            while b'\n' in tampon:
                ligne, tampon = tampon.split(b'\n', 1)
                compteurs = lire_compteurs(ligne.decode(errors='replace'))
                if compteurs is not None:
                    with self.verrou:
                        self.compteurs = compteurs
                        self.heure_compteurs = time.monotonic()

    def fermer(self):
        """Arrête les moteurs et les envois de la carte, puis ferme le port."""
        try:
            self.arret_rapide()
            self.envoyer('$upload:0,0,0#')
        finally:
            self.actif = False
            self.fil.join(timeout=1)
            self.liaison.close()
