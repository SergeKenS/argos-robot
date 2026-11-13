"""Fonctionnalité Service : aller à un point sur appel, ou patrouiller.

États publiés (inchangés pour la page web) :
  « APPEL <point> »      : en route vers le point demandé
  « PATROUILLE <point> » : enchaîne en boucle les points de la patrouille
  « ECHEC <point> »      : le trajet a échoué, le robot attend une commande
Arrivé au point d'un appel, le robot repasse en VEILLE sur place.

Commandes : « appel <point> », « base » (= « appel base »), « patrouille ».
"""
from argos_mission.fonctionnalites import Fonctionnalite


class Service(Fonctionnalite):
    """Appels et patrouille."""

    nom = 'Service'
    commandes = ('appel', 'base', 'patrouille')
    entrees = commandes

    def __init__(self, robot):
        """Au départ : rien en cours."""
        super().__init__(robot)
        self.mode = None       # None, 'APPEL', 'PATROUILLE' ou 'ECHEC'
        self.cible = None      # point visé
        self.index = 0         # position dans la liste de patrouille
        self.echecs = 0        # échecs de suite, pour détecter une patrouille impossible

    def commande(self, mots):
        cartes = self.robot.cartes
        if mots[0] == 'base':
            # Raccourci : le point de retour s'appelle « base ».
            mots = ['appel', 'base']

        if mots[0] == 'appel':
            if len(mots) != 2:
                self.robot.signaler('avertissement', 'Appel : il manque le nom du point.')
            elif mots[1] not in cartes.points:
                self.robot.signaler('avertissement',
                                    f'Point inconnu : {mots[1]} (commande ignorée)')
            else:
                self.robot.annuler()
                self.mode = 'APPEL'
                self.lancer(mots[1])
        elif mots[0] == 'patrouille':
            if not cartes.patrouille:
                self.robot.signaler('avertissement',
                                    'Aucune patrouille définie (commande ignorée)')
            else:
                self.robot.annuler()
                self.mode = 'PATROUILLE'
                self.index = 0
                self.echecs = 0
                self.lancer(cartes.patrouille[0])

    def lancer(self, nom):
        self.cible = nom
        if not self.robot.aller_vers(nom):
            self.robot.signaler('erreur', 'Destination refusée : placer le robot, puis '
                                '« Redémarrer la navigation ».')
            self.mode = 'ECHEC'

    def fin_de_tache(self, resultat):
        patrouille = self.robot.cartes.patrouille
        if resultat == 'annule':
            # Annulée hors du répartiteur (ex. bouton Cancel de RViz).
            self.arreter()
        elif self.mode == 'APPEL':
            if resultat == 'reussi':
                self.robot.signaler('info', f'Arrivé à {self.cible}.')
                self.arreter()
            else:
                # On garde le nom du point dans l'état pour savoir lequel a échoué.
                self.mode = 'ECHEC'
        elif self.mode == 'PATROUILLE':
            if resultat == 'reussi':
                self.echecs = 0
            else:
                self.echecs += 1
                self.robot.signaler('avertissement', f'{self.cible} inaccessible, point suivant.')
            # Si aucun point n'est atteignable, on arrête au lieu de boucler sans fin.
            if self.echecs >= len(patrouille):
                self.mode = 'ECHEC'
                return
            self.index = (self.index + 1) % len(patrouille)
            self.lancer(patrouille[self.index])

    def points_modifies(self):
        # Si la patrouille en cours se retrouve vide, on s'arrête proprement.
        if self.mode == 'PATROUILLE' and not self.robot.cartes.patrouille:
            self.sortir()

    def arreter(self):
        self.mode = None
        self.cible = None

    def sortir(self):
        self.robot.annuler()
        self.arreter()

    def etat(self):
        if self.mode is None:
            return None
        return f'{self.mode} {self.cible}'
