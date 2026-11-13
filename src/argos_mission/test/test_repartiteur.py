"""Tests du répartiteur : le vrai code de repartiteur.py, avec un faux ROS.

ROS et Nav2 sont remplacés par de faux modules qui notent ce qu'on leur demande.
On vérifie ainsi l'aiguillage des commandes et les états publiés, sans
simulation. Ce que ces tests ne voient pas : le vrai comportement de Nav2 et
de SLAM Toolbox (à vérifier en simulation, liste dans ETAPES.md).
Lancer : python3 -m pytest src/argos_mission/test/test_repartiteur.py
"""
import enum
import importlib
import json
import os
import sys
import types

import pytest

POINTS = """\
points:
  base: {x: 0.0, y: 0.0, theta: 0.0}
  alpha: {x: 1.0, y: 2.0, theta: 0.0}
  beta: {x: 3.0, y: 4.0, theta: 0.0}
patrouille: [alpha, beta]
"""


class Objet(types.SimpleNamespace):
    """Objet vide dont tout attribut inconnu est un autre objet vide (faux message ROS)."""

    def __getattr__(self, nom):
        valeur = Objet()
        setattr(self, nom, valeur)
        return valeur

    def __call__(self, *_, **__):
        return Objet()


class Resultat(enum.Enum):
    """Faux TaskResult de nav2_simple_commander."""

    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class Editeur:
    """Faux éditeur ROS : garde les messages publiés."""

    def __init__(self):
        """Aucun message au départ."""
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


class FauxNavigateur:
    """Faux BasicNavigator : note les appels, répond ce que le test a prévu."""

    fichier_points = ''

    def __init__(self, node_name=''):
        """Prépare le journal des appels."""
        self.appels = []
        self.editeurs = {}
        self.nav2_accepte = True
        self.resultat = Resultat.SUCCEEDED

    def create_publisher(self, _type, nom, _qos):
        self.editeurs[nom] = Editeur()
        return self.editeurs[nom]

    def create_subscription(self, *_):
        pass

    def declare_parameter(self, _nom, _defaut):
        return types.SimpleNamespace(value=FauxNavigateur.fichier_points)

    def get_logger(self):
        muet = types.SimpleNamespace()
        muet.info = muet.warn = muet.error = lambda texte: None
        return muet

    def get_clock(self):
        return Objet()

    def goToPose(self, pose, behavior_tree=''):
        self.appels.append(('goToPose', pose.pose.position.x, pose.pose.position.y))
        return self.nav2_accepte

    def cancelTask(self):
        self.appels.append(('cancelTask',))

    def getResult(self):
        return self.resultat

    def changeMap(self, chemin):
        self.appels.append(('changeMap', os.path.basename(chemin)))

    def clearAllCostmaps(self):
        self.appels.append(('clearAllCostmaps',))

    def setInitialPose(self, pose):
        self.appels.append(('setInitialPose', pose.pose.position.x, pose.pose.position.y))


def faux_module(nom, **contenu):
    module = types.ModuleType(nom)
    module.__dict__.update(contenu)
    return module


@pytest.fixture
def robot(tmp_path, monkeypatch):
    """Un répartiteur réel, branché sur un faux ROS et une carte « test »."""
    (tmp_path / 'test.yaml').write_text('image: test.pgm\n')
    (tmp_path / 'autre.yaml').write_text('image: autre.pgm\n')
    (tmp_path / 'test_points.yaml').write_text(POINTS)
    FauxNavigateur.fichier_points = str(tmp_path / 'test_points.yaml')

    cycle = faux_module('argos_mission.cycle_de_vie')
    cycle.appels = []
    cycle.reussite = True
    for nom in ('demarrer', 'arreter', 'sauvegarder', 'redemarrer_navigation'):
        def fonction(*_, nom=nom):
            cycle.appels.append(nom)
            return cycle.reussite
        setattr(cycle, nom, fonction)

    faux = {
        'rclpy': faux_module('rclpy'),
        'rclpy.qos': faux_module('rclpy.qos', DurabilityPolicy=Objet(), ReliabilityPolicy=Objet(),
                                 QoSProfile=lambda **_: None),
        'rclpy.signals': faux_module('rclpy.signals', SignalHandlerOptions=Objet()),
        'ament_index_python': faux_module('ament_index_python'),
        'ament_index_python.packages': faux_module(
            'ament_index_python.packages', get_package_share_directory=lambda _: '/faux'),
        'geometry_msgs': faux_module('geometry_msgs'),
        'geometry_msgs.msg': faux_module('geometry_msgs.msg', PoseStamped=Objet,
                                         PoseWithCovarianceStamped=Objet),
        'nav2_simple_commander': faux_module('nav2_simple_commander'),
        'nav2_simple_commander.robot_navigator': faux_module(
            'nav2_simple_commander.robot_navigator', BasicNavigator=FauxNavigateur,
            TaskResult=Resultat),
        'std_msgs': faux_module('std_msgs'),
        'std_msgs.msg': faux_module('std_msgs.msg', String=Objet),
        'argos_mission.cycle_de_vie': cycle,
    }
    for nom, module in faux.items():
        monkeypatch.setitem(sys.modules, nom, module)
    monkeypatch.delitem(sys.modules, 'argos_mission.repartiteur', raising=False)
    module = importlib.import_module('argos_mission.repartiteur')
    # Le paquet argos_mission garde ses sous-modules en attributs : on les remet aussi.
    import argos_mission
    monkeypatch.setattr(argos_mission, 'cycle_de_vie', cycle, raising=False)
    module.cycle_de_vie = cycle

    repartiteur = module.Repartiteur()
    repartiteur.cycle = cycle
    yield repartiteur
    sys.modules.pop('argos_mission.repartiteur', None)


def etats(robot):
    return robot.editeurs['/argos/etat'].messages


def journal(robot):
    return [json.loads(m)['niveau'] for m in robot.editeurs['/argos/journal'].messages]


def points_publies(robot):
    return json.loads(robot.editeurs['/argos/points'].messages[-1])


def arriver(robot, resultat=Resultat.SUCCEEDED):
    """Nav2 annonce la fin du trajet en cours."""
    assert robot.trajet_en_cours
    robot.resultat = resultat
    robot.fin_de_trajet()


# --- Liste de non-régression (essais/architecture/README.md) -------------------

def test_demarrage_en_veille(robot):
    assert etats(robot) == ['VEILLE']
    assert points_publies(robot) == {
        'points': robot.cartes.points, 'patrouille': ['alpha', 'beta'],
        'carte': 'test', 'cartes': ['autre', 'test']}


def test_appel_arrivee_retour_au_repos(robot):
    robot.traiter_commande('appel alpha')
    arriver(robot)
    assert etats(robot) == ['VEILLE', 'APPEL alpha', 'VEILLE']
    assert robot.appels == [('goToPose', 1.0, 2.0)]
    assert not robot.trajet_en_cours


def test_retour_a_la_base(robot):
    robot.traiter_commande('base')
    assert etats(robot)[-1] == 'APPEL base'


def test_veille_annule_le_trajet(robot):
    robot.traiter_commande('  Appel   ALPHA ')     # majuscules et espaces tolérés
    robot.traiter_commande('veille')
    assert etats(robot) == ['VEILLE', 'APPEL alpha', 'VEILLE']
    assert robot.appels[-1] == ('cancelTask',)
    assert not robot.trajet_en_cours


def test_nouvel_appel_remplace_le_precedent(robot):
    robot.traiter_commande('appel alpha')
    robot.traiter_commande('appel beta')
    assert etats(robot)[-1] == 'APPEL beta'
    assert robot.appels == [('goToPose', 1.0, 2.0), ('cancelTask',), ('goToPose', 3.0, 4.0)]


def test_patrouille_boucle_saute_et_echoue(robot):
    robot.traiter_commande('patrouille')
    arriver(robot)
    arriver(robot, Resultat.FAILED)                # beta inaccessible : sauté
    assert etats(robot)[-3:] == ['PATROUILLE alpha', 'PATROUILLE beta', 'PATROUILLE alpha']
    arriver(robot, Resultat.FAILED)                # puis alpha aussi : tous inaccessibles
    assert etats(robot)[-1] == 'ECHEC alpha'
    assert not robot.trajet_en_cours
    robot.traiter_commande('veille')
    assert etats(robot)[-1] == 'VEILLE'


def test_supprimer_un_point_de_la_patrouille_en_cours(robot):
    robot.traiter_commande('patrouille')
    robot.traiter_commande('supprimer alpha')
    assert etats(robot)[-1] == 'PATROUILLE alpha'  # le trajet en cours continue
    assert points_publies(robot)['patrouille'] == ['beta']
    robot.traiter_commande('supprimer beta')       # patrouille vide : arrêt propre
    assert etats(robot)[-1] == 'VEILLE'
    assert robot.appels[-1] == ('cancelTask',)


def test_destination_refusee(robot):
    robot.nav2_accepte = False
    robot.traiter_commande('appel alpha')
    assert etats(robot)[-1] == 'ECHEC alpha'
    assert journal(robot) == ['erreur']
    assert not robot.trajet_en_cours


def test_trajet_annule_depuis_rviz(robot):
    robot.traiter_commande('appel alpha')
    arriver(robot, Resultat.CANCELED)
    assert etats(robot)[-1] == 'VEILLE'


def test_changer_de_carte(robot):
    robot.traiter_commande('appel alpha')
    robot.traiter_commande('carte autre')
    assert etats(robot)[-1] == 'VEILLE'
    assert robot.appels[-3:] == [('cancelTask',), ('changeMap', 'autre.yaml'),
                                 ('clearAllCostmaps',)]
    assert points_publies(robot)['carte'] == 'autre'
    assert points_publies(robot)['points'] == {}
    robot.traiter_commande('carte inexistante')
    assert journal(robot)[-1] == 'avertissement'
    assert robot.cartes.actuelle == 'autre'


def test_cartographie_enregistrer(robot):
    robot.traiter_commande('appel alpha')
    robot.traiter_commande('cartographier')
    assert etats(robot)[-1] == 'CARTOGRAPHIE'
    assert ('cancelTask',) in robot.appels
    # Tant qu'elle dure, tout le reste est refusé, même « veille ».
    for commande in ('veille', 'appel alpha', 'carte autre', 'ajouter ici'):
        robot.traiter_commande(commande)
    assert etats(robot)[-1] == 'CARTOGRAPHIE'
    assert journal(robot) == ['avertissement'] * 4

    robot.traiter_commande('enregistrer etage_2')
    assert etats(robot)[-1] == 'VEILLE'
    assert robot.cycle.appels == ['demarrer', 'sauvegarder', 'arreter']
    assert robot.appels[-3:] == [('changeMap', 'etage_2.yaml'), ('clearAllCostmaps',),
                                 ('setInitialPose', 0.0, 0.0)]
    assert points_publies(robot)['carte'] == 'etage_2'
    assert list(points_publies(robot)['points']) == ['base']


def test_cartographie_abandonner(robot):
    robot.traiter_commande('cartographier')
    robot.traiter_commande('abandonner')
    assert etats(robot) == ['VEILLE', 'CARTOGRAPHIE', 'VEILLE']
    assert robot.cycle.appels == ['demarrer', 'arreter']
    assert robot.appels[-2:] == [('changeMap', 'test.yaml'), ('clearAllCostmaps',)]


def test_cartographie_impossible_a_demarrer(robot):
    robot.cycle.reussite = False
    robot.traiter_commande('cartographier')
    assert etats(robot) == ['VEILLE']
    assert journal(robot) == ['erreur']


def test_terminer_une_cartographie_qui_n_a_pas_commence(robot):
    robot.traiter_commande('enregistrer etage_2')
    robot.traiter_commande('abandonner')
    assert etats(robot) == ['VEILLE']
    assert journal(robot) == ['avertissement'] * 2
    assert robot.cycle.appels == []                # SLAM n'a pas été touché


def test_redemarrer_la_navigation(robot):
    robot.traiter_commande('redemarrer_navigation')    # robot pas encore placé
    assert journal(robot) == ['avertissement']
    assert robot.cycle.appels == []
    robot.pose_robot = Objet()
    robot.traiter_commande('appel alpha')
    robot.traiter_commande('redemarrer_navigation')
    assert etats(robot)[-1] == 'VEILLE'
    assert robot.cycle.appels == ['redemarrer_navigation']
    assert journal(robot)[-2:] == ['info', 'info']


def test_points_ajouter_et_refus(robot):
    robot.traiter_commande('ajouter chambre_101 2.5 -1')
    assert points_publies(robot)['points']['chambre_101'] == {'x': 2.5, 'y': -1.0, 'theta': 0.0}
    for commande in ('ajouter ici', 'ajouter chambre x y', 'ajouter', 'ajouter a 1',
                     'supprimer', 'supprimer inconnu', 'definir_patrouille alpha nulle_part',
                     'danser'):
        robot.traiter_commande(commande)
    assert journal(robot) == ['info'] + ['avertissement'] * 8
    assert points_publies(robot)['patrouille'] == ['alpha', 'beta']
    robot.traiter_commande('definir_patrouille')
    assert points_publies(robot)['patrouille'] == []
    assert etats(robot) == ['VEILLE']              # rien de tout cela ne fait bouger le robot
