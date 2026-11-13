"""Tests de la liaison avec la carte Yahboom, avec une fausse liaison série.

Lancer : python3 -m pytest src/argos_base/test/
"""
import threading
import time

from argos_base.carte_yahboom import CarteYahboom, lire_compteurs


class FausseLiaison:
    """Remplace le port série : note ce qu'on écrit, renvoie ce que le test a préparé."""

    def __init__(self):
        """Rien d'écrit, rien à lire au départ."""
        self.ecrit = []
        self.a_lire = b''
        self.verrou = threading.Lock()
        self.ferme = False

    def write(self, donnees):
        self.ecrit.append(donnees.decode())

    def read(self, taille):
        time.sleep(0.005)
        with self.verrou:
            morceau, self.a_lire = self.a_lire[:taille], self.a_lire[taille:]
        return morceau

    def recevoir(self, texte):
        with self.verrou:
            self.a_lire += texte.encode()

    def close(self):
        self.ferme = True


def attendre(condition, delai=1.0):
    fin = time.time() + delai
    while time.time() < fin:
        if condition():
            return True
        time.sleep(0.01)
    return False


def test_lire_compteurs():
    assert lire_compteurs('$MAll:0,-12,1767,-2434#') == (0, -12, 1767, -2434)
    assert lire_compteurs('$MSPD:0.00,0.00,0.00,0.00#') is None
    assert lire_compteurs('fail,Command not found!') is None
    assert lire_compteurs('$MAll:1,2#') is None


def test_ouverture_ordres_et_fermeture():
    liaison = FausseLiaison()
    carte = CarteYahboom('inutile', liaison=liaison)
    assert liaison.ecrit == ['$upload:1,0,0#']        # demande les compteurs dès l'ouverture
    carte.vitesses([0, 0, 100.4, -99.6])
    carte.arret_rapide()
    assert liaison.ecrit[1:] == ['$spd:0,0,100,-100#', '$pwm:0,0,0,0#']
    carte.fermer()
    assert liaison.ecrit[-2:] == ['$pwm:0,0,0,0#', '$upload:0,0,0#']
    assert liaison.ferme


def test_compteurs_les_plus_recents_meme_coupes_en_morceaux():
    liaison = FausseLiaison()
    carte = CarteYahboom('inutile', liaison=liaison)
    assert carte.derniers_compteurs() == (None, None)
    liaison.recevoir('Motor_Version:1.7.3\r\n$MAll:0,0,10,')   # ligne coupée en deux
    liaison.recevoir('-10#\r\n$MSPD:1.0,0,0,0#\r\n$MAll:0,0,25,-24#\r\n')
    assert attendre(lambda: carte.derniers_compteurs()[1] == (0, 0, 25, -24))
    heure, _ = carte.derniers_compteurs()
    assert heure is not None
    carte.fermer()


def test_redemande_les_compteurs_si_plus_rien_n_arrive(monkeypatch):
    # Vu sur le Pi : la première demande, écrite juste après l'ouverture, se perd.
    monkeypatch.setattr('argos_base.carte_yahboom.DELAI_RELANCE', 0.1)
    liaison = FausseLiaison()
    carte = CarteYahboom('inutile', liaison=liaison)
    assert attendre(lambda: liaison.ecrit.count('$upload:1,0,0#') >= 2)
    liaison.recevoir('$MAll:0,0,5,-5#\r\n')
    assert attendre(lambda: carte.derniers_compteurs()[1] == (0, 0, 5, -5))
    carte.fermer()
