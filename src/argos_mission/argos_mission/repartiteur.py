"""Répartiteur : machine à états qui pilote le robot selon trois modes.

Modes (un seul actif à la fois) :
  VEILLE     : le robot ne fait rien (état de repos sûr, état au démarrage)
  APPEL      : va au point demandé, puis repasse en VEILLE sur place
  PATROUILLE : enchaîne en boucle les points de la liste « patrouille »
Plus un état de repos ECHEC : un appel a échoué, le robot attend une commande.

Commandes (texte sur /argos/commande) : « veille », « appel <point> », « patrouille »
État publié sur /argos/etat : « VEILLE », « APPEL alpha », « PATROUILLE beta »...

Usage :
  ros2 run argos_mission repartiteur --ros-args -p fichier_points:=<chemin du YAML>
"""
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from argos_mission.aller_a import creer_pose

# Arbre de comportement fourni par Nav2 : il ne recalcule le chemin que si le
# chemin actuel devient bloqué. Évite que le robot hésite entre deux chemins
# de coût presque égal (comportement par défaut : recalcul chaque seconde).
ARBRE_NAVIGATION = os.path.join(
    get_package_share_directory('nav2_bt_navigator'), 'behavior_trees',
    'navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml')


class Repartiteur(BasicNavigator):
    """Nœud unique : hérite de BasicNavigator pour envoyer les missions à Nav2.

    BasicNavigator fait tourner ROS lui-même à chaque appel (goToPose, cancelTask...).
    On ne doit donc JAMAIS l'appeler depuis une fonction de réception de message :
    la réception se contente de noter la commande, la boucle principale la traite.
    """

    def __init__(self):
        super().__init__(node_name='repartiteur')

        # Chemin du fichier de points, passé au lancement (un fichier par carte).
        chemin = self.declare_parameter('fichier_points', '').value
        with open(chemin) as fichier:
            contenu = yaml.safe_load(fichier)
        self.points = contenu['points']
        self.patrouille = contenu.get('patrouille', [])
        for nom in self.patrouille:
            if nom not in self.points:
                raise ValueError(f'Point de patrouille inconnu : {nom}')

        self.mode = 'VEILLE'
        self.cible = None              # point visé par la mission en cours
        self.index_patrouille = 0      # position dans la liste de patrouille
        self.echecs_consecutifs = 0    # pour détecter une patrouille impossible
        self.commande_en_attente = None

        # « TRANSIENT_LOCAL » : le dernier état reste disponible, même pour un
        # abonné qui arrive plus tard (ex. la page web ouverte après coup).
        qos_etat = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_etat = self.create_publisher(String, '/argos/etat', qos_etat)
        self.create_subscription(String, '/argos/commande', self.recevoir_commande, 10)

        self.publier_etat()
        self.get_logger().info(f'Prêt. Points : {", ".join(self.points)}')

    # --- Réception et publication -------------------------------------------

    def recevoir_commande(self, msg):
        """Appelée par ROS à chaque message : on note seulement la commande."""
        self.commande_en_attente = msg.data

    def publier_etat(self):
        texte = self.mode if self.cible is None else f'{self.mode} {self.cible}'
        self.pub_etat.publish(String(data=texte))
        self.get_logger().info(f'État : {texte}')

    # --- Boucle principale ----------------------------------------------------

    def boucle(self):
        while rclpy.ok():
            mission_en_cours = self.mode in ('APPEL', 'PATROUILLE')
            if mission_en_cours:
                # Laisse ROS travailler ~0,1 s et indique si Nav2 a terminé.
                terminee = self.isTaskComplete()
            else:
                # Pas de mission : on laisse simplement ROS recevoir les messages.
                rclpy.spin_once(self, timeout_sec=0.1)
                terminee = False

            # Une nouvelle commande est prioritaire sur la fin de mission.
            if self.commande_en_attente is not None:
                commande = self.commande_en_attente
                self.commande_en_attente = None
                self.traiter_commande(commande)
            elif terminee:
                self.fin_de_mission()

    def traiter_commande(self, texte):
        mots = texte.strip().lower().split()
        if not mots:
            return
        action = mots[0]

        if action == 'veille':
            self.annuler_mission()
            self.passer_au_repos('VEILLE')
        elif action == 'appel' and len(mots) == 2:
            if mots[1] not in self.points:
                self.get_logger().warn(f'Point inconnu : {mots[1]} (commande ignorée)')
                return
            self.annuler_mission()
            self.mode = 'APPEL'
            self.lancer_vers(mots[1])
        elif action == 'patrouille':
            if not self.patrouille:
                self.get_logger().warn('Aucune patrouille définie (commande ignorée)')
                return
            self.annuler_mission()
            self.mode = 'PATROUILLE'
            self.index_patrouille = 0
            self.echecs_consecutifs = 0
            self.lancer_vers(self.patrouille[0])
        else:
            self.get_logger().warn(f'Commande inconnue : « {texte} »')

    def fin_de_mission(self):
        """Nav2 a terminé la mission en cours : on décide de la suite."""
        resultat = self.getResult()

        if resultat == TaskResult.CANCELED:
            # Annulée hors du répartiteur (ex. bouton Cancel de RViz).
            self.passer_au_repos('VEILLE')
        elif self.mode == 'APPEL':
            if resultat == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Arrivé à {self.cible}.')
                self.passer_au_repos('VEILLE')
            else:
                # On garde le nom du point dans l'état pour savoir lequel a échoué.
                self.mode = 'ECHEC'
                self.publier_etat()
        elif self.mode == 'PATROUILLE':
            if resultat == TaskResult.SUCCEEDED:
                self.echecs_consecutifs = 0
            else:
                self.echecs_consecutifs += 1
                self.get_logger().warn(f'{self.cible} inaccessible, point suivant.')
            # Si aucun point n'est atteignable, on arrête au lieu de boucler sans fin.
            if self.echecs_consecutifs >= len(self.patrouille):
                self.mode = 'ECHEC'
                self.publier_etat()
                return
            self.index_patrouille = (self.index_patrouille + 1) % len(self.patrouille)
            self.lancer_vers(self.patrouille[self.index_patrouille])

    # --- Actions élémentaires ---------------------------------------------------

    def lancer_vers(self, nom):
        self.cible = nom
        if self.goToPose(creer_pose(self, self.points[nom]), behavior_tree=ARBRE_NAVIGATION):
            self.publier_etat()
        else:
            self.get_logger().error('Destination refusée : Nav2 est-il actif ?')
            self.mode = 'ECHEC'
            self.publier_etat()

    def annuler_mission(self):
        if self.mode in ('APPEL', 'PATROUILLE'):
            self.cancelTask()

    def passer_au_repos(self, mode):
        self.mode = mode
        self.cible = None
        self.publier_etat()


def main():
    # Comme dans aller_a : Ctrl+C doit lever une KeyboardInterrupt pour
    # qu'on puisse annuler la mission avant de quitter.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        repartiteur = Repartiteur()
    except (OSError, KeyError, ValueError) as erreur:
        print(f'Fichier de points invalide : {erreur}')
        rclpy.shutdown()
        sys.exit(1)

    try:
        repartiteur.boucle()
    except KeyboardInterrupt:
        repartiteur.get_logger().warn('Ctrl+C : annulation et arrêt.')
        repartiteur.annuler_mission()

    repartiteur.destroyNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
