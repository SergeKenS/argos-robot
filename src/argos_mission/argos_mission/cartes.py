"""Cartes et points nommés : lecture, modification, sauvegarde.

Une carte « X » = <dossier>/X.yaml (+ X.pgm) et ses points <dossier>/X_points.yaml.
Les coordonnées d'un point n'ont de sens que sur sa carte : un fichier par carte.

Ce fichier ne dépend pas de ROS : il se teste seul (test/test_cartes.py).
Une demande refusée lève ValueError avec le message à montrer à l'utilisateur ;
un fichier impossible à écrire lève OSError.
"""
import os
import re

import yaml

# Noms de points et de cartes autorisés : minuscules, chiffres et « _ » (ex. chambre_101).
FORMAT_NOM = re.compile(r'^[a-z0-9_]+$')

# En-tête réécrit à chaque sauvegarde du fichier de points.
EN_TETE_POINTS = """\
# Points nommés d'une carte (un fichier par carte).
#   x, y  : position en mètres dans le repère de la carte ; theta : orientation (radians)
# Fichier réécrit par le répartiteur quand on ajoute/supprime un point (page web).
"""

ORIGINE = {'x': 0.0, 'y': 0.0, 'theta': 0.0}


class Cartes:
    """La carte actuelle, ses points, son ordre de patrouille, et les autres cartes du dossier."""

    def __init__(self, chemin_points):
        """Part du fichier de points donné au lancement : <dossier>/<carte>_points.yaml."""
        self.dossier = os.path.dirname(chemin_points)
        self.actuelle = os.path.basename(chemin_points).removesuffix('_points.yaml')
        self.points = {}
        self.patrouille = []

    # --- Chemins et cartes disponibles -----------------------------------------

    @staticmethod
    def nom_valide(nom):
        return bool(FORMAT_NOM.match(nom))

    def chemin_carte(self, nom):
        return os.path.join(self.dossier, nom + '.yaml')

    def chemin_points(self, nom=None):
        return os.path.join(self.dossier, (nom or self.actuelle) + '_points.yaml')

    def existe(self, nom):
        return self.nom_valide(nom) and os.path.isfile(self.chemin_carte(nom))

    def lister(self):
        """Cartes disponibles : les <nom>.yaml du dossier, sauf les fichiers de points."""
        try:
            fichiers = os.listdir(self.dossier)
        except OSError:
            return [self.actuelle]
        return sorted(f.removesuffix('.yaml') for f in fichiers
                      if f.endswith('.yaml') and not f.endswith('_points.yaml'))

    def donnees(self):
        """Ce que la page web doit connaître (publié sur /argos/points)."""
        return {'points': self.points, 'patrouille': self.patrouille,
                'carte': self.actuelle, 'cartes': self.lister()}

    # --- Changer de carte --------------------------------------------------------

    def charger(self, nom=None):
        """Lit les points de la carte (aucun point si le fichier n'existe pas).

        Renvoie les noms de patrouille ignorés parce qu'ils ne sont pas des points.
        """
        if nom is not None:
            self.actuelle = nom
        contenu = {}
        if os.path.isfile(self.chemin_points()):
            with open(self.chemin_points()) as fichier:
                contenu = yaml.safe_load(fichier) or {}
        self.points = contenu.get('points') or {}
        patrouille = contenu.get('patrouille') or []
        self.patrouille = [p for p in patrouille if p in self.points]
        return [p for p in patrouille if p not in self.points]

    def creer(self, nom, base):
        """Nouvelle carte : un seul point, « base », et pas de patrouille."""
        self.actuelle = nom
        self.points = {'base': base}
        self.patrouille = []
        self.enregistrer()

    # --- Points ------------------------------------------------------------------

    def ajouter(self, nom, point):
        """Ajoute (ou remplace) un point {x, y, theta}."""
        if not self.nom_valide(nom):
            raise ValueError(f'Nom invalide : {nom} (minuscules, chiffres, _)')
        self.points[nom] = point
        self.enregistrer()

    def supprimer(self, nom):
        if nom not in self.points:
            raise ValueError(f'Point inconnu : {nom}')
        del self.points[nom]
        self.patrouille = [p for p in self.patrouille if p != nom]
        self.enregistrer()

    def definir_patrouille(self, noms):
        inconnus = [nom for nom in noms if nom not in self.points]
        if inconnus:
            raise ValueError(f'Points inconnus : {", ".join(inconnus)}')
        self.patrouille = list(noms)
        self.enregistrer()

    def enregistrer(self):
        """Réécrit le fichier de points de la carte actuelle."""
        contenu = {'points': self.points, 'patrouille': self.patrouille}
        with open(self.chemin_points(), 'w') as fichier:
            fichier.write(EN_TETE_POINTS)
            # default_flow_style=None : chaque point tient sur une ligne {x, y, theta}.
            yaml.safe_dump(contenu, fichier, default_flow_style=None,
                           sort_keys=False, allow_unicode=True)
