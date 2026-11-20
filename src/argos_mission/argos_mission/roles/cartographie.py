"""Rôle Cartographie : dessiner une nouvelle carte (SLAM Toolbox).

État publié : « CARTOGRAPHIE ». Le robot se promène au pilotage manuel (page web).
Tant qu'elle est active, on ne peut que la terminer :
  « enregistrer <nom> » : sauvegarde la carte, y place un point « base » au
                          point de départ, et repasse en navigation dessus
  « abandonner »        : revient à la carte d'avant, sans rien sauvegarder
Elle démarre avec « cartographier ».

Le démarrage et l'arrêt de SLAM Toolbox sont des outils du robot (voir
cycle_de_vie.py) : ce fichier ne décide que de l'enchaînement.
"""
from argos_mission.roles import Role


class Cartographie(Role):
    """Création d'une carte."""

    nom = 'Cartographie'
    commandes = ('cartographier', 'enregistrer', 'abandonner')
    entrees = ('cartographier',)
    exclusive = 'Cartographie en cours : enregistrer ou abandonner.'

    def __init__(self, robot):
        """Au départ : pas de cartographie en cours."""
        super().__init__(robot)
        self.en_cours = False

    def entrer(self):
        if not self.robot.demarrer_slam():
            self.robot.signaler('erreur', 'Impossible de démarrer la cartographie.')
            return False
        self.en_cours = True
        return True

    def commande(self, mots):
        if mots[0] == 'enregistrer' and len(mots) == 2:
            self.enregistrer(mots[1])
        elif mots[0] == 'abandonner':
            self.abandonner()
        elif mots[0] != 'cartographier':
            self.robot.signaler('avertissement', self.exclusive)

    def enregistrer(self, nom):
        robot, cartes = self.robot, self.robot.cartes
        if not cartes.nom_valide(nom):
            robot.signaler('avertissement',
                           f'Nom de carte invalide : {nom} (minuscules, chiffres, _)')
            return
        if cartes.existe(nom):
            robot.signaler('avertissement', f'La carte {nom} existe déjà : choisir un autre nom.')
            return
        if not robot.sauvegarder_carte(nom):
            # On reste en cartographie : rien n'est perdu, on peut réessayer.
            robot.signaler('erreur', f'Échec de la sauvegarde de la carte {nom}.')
            return

        # Positions à retenir avant d'arrêter SLAM Toolbox.
        position, depart = robot.position_slam(), robot.depart_slam()
        if not robot.arreter_slam():
            robot.signaler('erreur', 'Retour à la navigation incomplet : si le robot '
                           'refuse les destinations, le relancer.')
        self.en_cours = False

        # Nouvelle carte, avec un point « base » là où la cartographie a commencé.
        try:
            cartes.creer(nom, depart)
        except OSError as erreur:
            robot.signaler('erreur', f"Impossible d'écrire les points de {nom} : {erreur}")
        if not robot.afficher_carte(nom):
            robot.publier_points()
            return
        # SLAM connaissait la position exacte du robot sur cette carte : on la
        # donne à la navigation, ainsi pas besoin de « Placer le robot ».
        robot.placer(position)
        robot.publier_points()
        robot.signaler('info', f'Carte {nom} enregistrée et chargée.')

    def abandonner(self):
        if not self.robot.arreter_slam():
            self.robot.signaler('erreur', 'Retour à la navigation incomplet : si le robot '
                                'refuse les destinations, le relancer.')
        self.en_cours = False
        # Recharger l'ancienne carte la republie (la page et Nav2 l'affichent à nouveau).
        self.robot.afficher_carte(self.robot.cartes.actuelle)
        self.robot.signaler('info', 'Cartographie abandonnée : replacer le robot sur la carte.')

    def sortir(self):
        if self.en_cours:
            self.abandonner()

    def etat(self):
        return 'CARTOGRAPHIE' if self.en_cours else None
