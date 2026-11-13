"""Fonctionnalités du robot : une par fichier, une seule active à la fois.

Une fonctionnalité = un comportement complet (ex. Service, Cartographie).
Quand aucune n'est active, le robot est en VEILLE : immobile, il attend.

Règles :
  1. Une fonctionnalité ne connaît pas les autres. Elle n'utilise que les
     outils du robot (voir « Outils du robot » dans repartiteur.py).
  2. Seul le répartiteur passe d'une fonctionnalité à l'autre, toujours en
     appelant sortir() sur celle qu'il quitte.
  3. Aucune fonctionnalité n'importe ROS : elles se testent sans robot ni
     simulation (test/test_fonctionnalites.py).
  4. Ce qui est lent ou lourd (vision, IA) tourne dans un nœud ROS séparé ;
     la fonctionnalité dirige, elle ne calcule pas.

Ajouter une fonctionnalité : un fichier ici, avec une classe qui hérite de
Fonctionnalite, et une ligne dans la liste FONCTIONNALITES de repartiteur.py.
"""


class Fonctionnalite:
    """Modèle commun : le répartiteur n'appelle que ces méthodes."""

    # Nom affiché dans les messages (ex. 'Service').
    nom = ''
    # Toutes les commandes que la fonctionnalité comprend (premier mot).
    commandes = ()
    # Parmi elles, celles qui la démarrent quand elle n'est pas active.
    entrees = ()
    # Si elle doit refuser toute autre commande tant qu'elle est active (même
    # « veille ») : le message de refus. Sinon None.
    exclusive = None

    def __init__(self, robot):
        """Reçoit le robot (le répartiteur) pour se servir de ses outils."""
        self.robot = robot

    def entrer(self):
        """Démarre. Renvoie False si c'est impossible (le robot reste en veille)."""
        return True

    def commande(self, mots):
        """Traite une de ses commandes, déjà découpée en mots."""

    def fin_de_tache(self, resultat):
        """Nav2 a fini le trajet demandé : 'reussi', 'echec' ou 'annule'."""

    def points_modifies(self):
        """Les points ou la patrouille viennent de changer."""

    def sortir(self):
        """S'arrête proprement. Peut être appelée même si tout est déjà arrêté."""

    def etat(self):
        """Texte publié sur /argos/etat, ou None si elle n'a plus rien à faire.

        None = le répartiteur la quitte et le robot repasse en VEILLE.
        """
        return None
