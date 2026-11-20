"""Rôles du robot (ses modes) : un par fichier, un seul actif à la fois.

Un rôle = un comportement complet (ex. Service, Cartographie).
Quand aucun n'est actif, le robot est en VEILLE : immobile, il attend.

Règles :
  1. Un rôle ne connaît pas les autres. Il n'utilise que les
     outils du robot (voir « Outils du robot » dans repartiteur.py).
  2. Seul le répartiteur passe d'un rôle à l'autre, toujours en
     appelant sortir() sur celui qu'il quitte.
  3. Aucun rôle n'importe ROS : ils se testent sans robot ni
     simulation (test/test_roles.py).
  4. Ce qui est lent ou lourd (vision, IA) tourne dans un nœud ROS séparé ;
     le rôle dirige, il ne calcule pas.

Ajouter un rôle : un fichier ici, avec une classe qui hérite de
Role, et une ligne dans la liste ROLES de repartiteur.py.
"""


class Role:
    """Modèle commun : le répartiteur n'appelle que ces méthodes."""

    # Nom affiché dans les messages (ex. 'Service').
    nom = ''
    # Toutes les commandes que le rôle comprend (premier mot).
    commandes = ()
    # Parmi elles, celles qui le démarrent quand il n'est pas actif.
    entrees = ()
    # S'il doit refuser toute autre commande tant qu'il est actif (même
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
        """S'arrête proprement. Peut être appelé même si tout est déjà arrêté."""

    def etat(self):
        """Texte publié sur /argos/etat, ou None s'il n'a plus rien à faire.

        None = le répartiteur le quitte et le robot repasse en VEILLE.
        """
        return None
