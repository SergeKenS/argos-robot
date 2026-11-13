"""Tests des fonctionnalités et des cartes, sans ROS ni simulation.

Un faux robot remplace le répartiteur : il note ce qu'on lui demande.
Lancer : python3 -m pytest src/argos_mission/test/test_fonctionnalites.py
"""
import os

from argos_mission.cartes import Cartes
from argos_mission.fonctionnalites.cartographie import Cartographie
from argos_mission.fonctionnalites.service import Service
import pytest

POINTS = """\
points:
  base: {x: 0.0, y: 0.0, theta: 0.0}
  alpha: {x: 1.0, y: 2.0, theta: 0.0}
  beta: {x: 3.0, y: 4.0, theta: 0.0}
patrouille: [alpha, beta, fantome]
"""


class FauxRobot:
    """Les outils du répartiteur, sans Nav2 : chaque appel est noté dans self.appels."""

    def __init__(self, dossier):
        """Prépare une carte « test » avec trois points dans le dossier donné."""
        with open(os.path.join(dossier, 'test.yaml'), 'w') as fichier:
            fichier.write('image: test.pgm\n')
        with open(os.path.join(dossier, 'test_points.yaml'), 'w') as fichier:
            fichier.write(POINTS)
        self.cartes = Cartes(os.path.join(dossier, 'test_points.yaml'))
        self.ignores = self.cartes.charger()
        self.appels = []
        self.messages = []
        self.nav2_accepte = True
        self.slam_demarre = True
        self.sauvegarde_reussit = True

    def signaler(self, niveau, texte):
        self.messages.append((niveau, texte))

    def aller_vers(self, nom):
        self.appels.append(('aller_vers', nom))
        return self.nav2_accepte

    def annuler(self):
        self.appels.append(('annuler',))

    def publier_points(self):
        self.appels.append(('publier_points',))

    def afficher_carte(self, nom):
        self.appels.append(('afficher_carte', nom))

    def placer(self, point):
        self.appels.append(('placer', point))

    def demarrer_slam(self):
        self.appels.append(('demarrer_slam',))
        return self.slam_demarre

    def arreter_slam(self):
        self.appels.append(('arreter_slam',))
        return True

    def sauvegarder_carte(self, nom):
        self.appels.append(('sauvegarder_carte', nom))
        return self.sauvegarde_reussit

    def position_slam(self):
        return {'x': 5.0, 'y': 6.0, 'theta': 1.0}

    def depart_slam(self):
        return {'x': 0.5, 'y': 0.5, 'theta': 0.0}

    def trajets(self):
        return [appel[1] for appel in self.appels if appel[0] == 'aller_vers']

    def niveaux(self):
        return [niveau for niveau, _ in self.messages]


@pytest.fixture
def robot(tmp_path):
    return FauxRobot(str(tmp_path))


# --- Cartes -------------------------------------------------------------------

def test_cartes_chargement(robot):
    cartes = robot.cartes
    assert cartes.actuelle == 'test'
    assert list(cartes.points) == ['base', 'alpha', 'beta']
    # Le point de patrouille inconnu est écarté et signalé.
    assert cartes.patrouille == ['alpha', 'beta']
    assert robot.ignores == ['fantome']
    assert cartes.lister() == ['test']
    assert cartes.existe('test') and not cartes.existe('autre') and not cartes.existe('../test')


def test_cartes_points_relus_apres_sauvegarde(robot):
    cartes = robot.cartes
    cartes.ajouter('chambre_101', {'x': 7.0, 'y': 8.0, 'theta': 0.0})
    cartes.definir_patrouille(['beta', 'chambre_101'])
    cartes.supprimer('beta')
    relues = Cartes(cartes.chemin_points())
    assert relues.charger() == []
    assert list(relues.points) == ['base', 'alpha', 'chambre_101']
    assert relues.patrouille == ['chambre_101']


def test_cartes_refus(robot):
    cartes = robot.cartes
    with pytest.raises(ValueError):
        cartes.ajouter('Chambre 101', {'x': 0.0, 'y': 0.0, 'theta': 0.0})
    with pytest.raises(ValueError):
        cartes.supprimer('inconnu')
    with pytest.raises(ValueError):
        cartes.definir_patrouille(['alpha', 'inconnu'])
    assert cartes.patrouille == ['alpha', 'beta']


# --- Service ------------------------------------------------------------------

def test_appel_reussi_puis_veille(robot):
    service = Service(robot)
    service.commande(['appel', 'alpha'])
    assert service.etat() == 'APPEL alpha'
    assert robot.trajets() == ['alpha']
    service.fin_de_tache('reussi')
    assert service.etat() is None


def test_appel_echoue_garde_le_point(robot):
    service = Service(robot)
    service.commande(['appel', 'alpha'])
    service.fin_de_tache('echec')
    assert service.etat() == 'ECHEC alpha'


def test_appel_point_inconnu_ne_touche_a_rien(robot):
    service = Service(robot)
    service.commande(['appel', 'alpha'])
    service.commande(['appel', 'nulle_part'])
    assert service.etat() == 'APPEL alpha'
    assert robot.trajets() == ['alpha']
    assert robot.niveaux() == ['avertissement']


def test_base_est_un_appel(robot):
    service = Service(robot)
    service.commande(['base'])
    assert service.etat() == 'APPEL base'


def test_destination_refusee(robot):
    robot.nav2_accepte = False
    service = Service(robot)
    service.commande(['appel', 'alpha'])
    assert service.etat() == 'ECHEC alpha'
    assert robot.niveaux() == ['erreur']


def test_trajet_annule_de_l_exterieur(robot):
    service = Service(robot)
    service.commande(['patrouille'])
    service.fin_de_tache('annule')
    assert service.etat() is None


def test_patrouille_boucle_et_saute_un_point_inaccessible(robot):
    service = Service(robot)
    service.commande(['patrouille'])
    assert service.etat() == 'PATROUILLE alpha'
    service.fin_de_tache('reussi')
    assert service.etat() == 'PATROUILLE beta'
    service.fin_de_tache('echec')       # beta inaccessible : on continue
    assert service.etat() == 'PATROUILLE alpha'
    service.fin_de_tache('reussi')
    assert robot.trajets() == ['alpha', 'beta', 'alpha', 'beta']


def test_patrouille_tous_inaccessibles(robot):
    service = Service(robot)
    service.commande(['patrouille'])
    service.fin_de_tache('echec')
    service.fin_de_tache('echec')
    assert service.etat() == 'ECHEC beta'
    assert robot.trajets() == ['alpha', 'beta']


def test_patrouille_non_definie(robot):
    robot.cartes.definir_patrouille([])
    service = Service(robot)
    service.commande(['patrouille'])
    assert service.etat() is None
    assert robot.trajets() == []


def test_patrouille_videe_en_cours_de_route(robot):
    service = Service(robot)
    service.commande(['patrouille'])
    robot.cartes.definir_patrouille([])
    service.points_modifies()
    assert service.etat() is None
    assert ('annuler',) in robot.appels


def test_point_de_patrouille_supprime_en_cours_de_route(robot):
    service = Service(robot)
    service.commande(['patrouille'])
    service.fin_de_tache('reussi')      # en route vers beta
    robot.cartes.supprimer('beta')
    service.points_modifies()
    service.fin_de_tache('reussi')
    assert service.etat() == 'PATROUILLE alpha'


def test_sortir_annule_le_trajet(robot):
    service = Service(robot)
    service.commande(['appel', 'alpha'])
    service.sortir()
    assert service.etat() is None
    assert robot.appels[-1] == ('annuler',)


# --- Cartographie -------------------------------------------------------------

def test_cartographie_enregistrer(robot):
    carto = Cartographie(robot)
    assert carto.entrer()
    assert carto.etat() == 'CARTOGRAPHIE'
    carto.commande(['enregistrer', 'etage_2'])
    assert carto.etat() is None
    noms = [appel[0] for appel in robot.appels]
    # L'ordre compte : sauvegarder avant d'arrêter SLAM, charger la carte avant de placer.
    assert noms == ['demarrer_slam', 'sauvegarder_carte', 'arreter_slam',
                    'afficher_carte', 'placer', 'publier_points']
    assert robot.cartes.actuelle == 'etage_2'
    assert robot.cartes.points == {'base': {'x': 0.5, 'y': 0.5, 'theta': 0.0}}
    assert ('placer', {'x': 5.0, 'y': 6.0, 'theta': 1.0}) in robot.appels
    assert os.path.isfile(robot.cartes.chemin_points('etage_2'))


def test_cartographie_refuse_un_nom_pris_ou_invalide(robot):
    carto = Cartographie(robot)
    carto.entrer()
    carto.commande(['enregistrer', 'test'])
    carto.commande(['enregistrer', 'Mon-Étage'])
    carto.commande(['enregistrer'])
    assert carto.etat() == 'CARTOGRAPHIE'
    assert robot.niveaux() == ['avertissement'] * 3
    assert ('sauvegarder_carte', 'test') not in robot.appels


def test_cartographie_sauvegarde_ratee_on_peut_reessayer(robot):
    robot.sauvegarde_reussit = False
    carto = Cartographie(robot)
    carto.entrer()
    carto.commande(['enregistrer', 'etage_2'])
    assert carto.etat() == 'CARTOGRAPHIE'
    assert ('arreter_slam',) not in robot.appels
    assert robot.cartes.actuelle == 'test'


def test_cartographie_abandonner_revient_a_l_ancienne_carte(robot):
    carto = Cartographie(robot)
    carto.entrer()
    carto.commande(['abandonner'])
    assert carto.etat() is None
    assert robot.appels[-2:] == [('arreter_slam',), ('afficher_carte', 'test')]
    carto.sortir()                       # déjà arrêtée : ne refait rien
    assert robot.appels.count(('arreter_slam',)) == 1


def test_cartographie_demarrage_impossible(robot):
    robot.slam_demarre = False
    carto = Cartographie(robot)
    assert not carto.entrer()
    assert carto.etat() is None
    assert robot.niveaux() == ['erreur']
