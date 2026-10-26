"""Répartiteur : machine à états qui pilote le robot selon trois modes.

Modes (un seul actif à la fois) :
  VEILLE     : le robot ne fait rien (état de repos sûr, état au démarrage)
  APPEL      : va au point demandé, puis repasse en VEILLE sur place
  PATROUILLE : enchaîne en boucle les points de la liste « patrouille »
Plus un état de repos ECHEC : un appel a échoué, le robot attend une commande.

Commandes (texte sur /argos/commande) :
  « veille », « appel <point> », « patrouille », « base » (= « appel base »)
  « ajouter <nom> »            : enregistre la position actuelle du robot
  « ajouter <nom> <x> <y> »    : enregistre un point choisi sur la carte
  « supprimer <nom> »
  « definir_patrouille <nom> <nom>... » : ordre de patrouille (vide = aucune)
  « carte <nom> »              : charge maps/<nom>.yaml et ses points maps/<nom>_points.yaml
État publié sur /argos/etat : « VEILLE », « APPEL alpha », « PATROUILLE beta »...
Points, carte actuelle et cartes disponibles publiés sur /argos/points (JSON).

Usage :
  ros2 run argos_mission repartiteur --ros-args -p fichier_points:=<chemin du YAML>
"""
import json
import math
import os
import re
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
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

# Noms de points autorisés : minuscules, chiffres et « _ » (ex. chambre_101).
FORMAT_NOM = re.compile(r'^[a-z0-9_]+$')

# En-tête réécrit à chaque sauvegarde du fichier de points.
EN_TETE_POINTS = """\
# Points nommés d'une carte (un fichier par carte).
#   x, y  : position en mètres dans le repère de la carte ; theta : orientation (radians)
# Fichier réécrit par le répartiteur quand on ajoute/supprime un point (page web).
"""


class Repartiteur(BasicNavigator):
    """Nœud unique : hérite de BasicNavigator pour envoyer les missions à Nav2.

    BasicNavigator fait tourner ROS lui-même à chaque appel (goToPose, cancelTask...).
    On ne doit donc JAMAIS l'appeler depuis une fonction de réception de message :
    la réception se contente de noter la commande, la boucle principale la traite.
    """

    def __init__(self):
        super().__init__(node_name='repartiteur')

        # Chemin du fichier de points, passé au lancement (un fichier par carte).
        # Il est rangé à côté de sa carte : maps/<carte>_points.yaml.
        self.chemin_points = self.declare_parameter('fichier_points', '').value
        self.dossier_cartes = os.path.dirname(self.chemin_points)
        self.carte = os.path.basename(self.chemin_points).removesuffix('_points.yaml')
        self.charger_points()

        self.mode = 'VEILLE'
        self.cible = None              # point visé par la mission en cours
        self.index_patrouille = 0      # position dans la liste de patrouille
        self.echecs_consecutifs = 0    # pour détecter une patrouille impossible
        self.commande_en_attente = None
        self.pose_robot = None         # dernière position estimée par AMCL

        # « TRANSIENT_LOCAL » : le dernier message reste disponible, même pour un
        # abonné qui arrive plus tard (ex. la page web ouverte après coup).
        qos_dernier = QoSProfile(depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_etat = self.create_publisher(String, '/argos/etat', qos_dernier)
        self.pub_points = self.create_publisher(String, '/argos/points', qos_dernier)
        self.create_subscription(String, '/argos/commande', self.recevoir_commande, 10)
        # AMCL publie sa position avec la même QoS (voir robot_navigator.py).
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                 self.recevoir_pose, qos_dernier)

        self.publier_etat()
        self.publier_points()
        self.get_logger().info(f'Prêt. Points : {", ".join(self.points)}')

    # --- Réception et publication -------------------------------------------

    def recevoir_commande(self, msg):
        """Appelée par ROS à chaque message : on note seulement la commande."""
        self.commande_en_attente = msg.data

    def recevoir_pose(self, msg):
        """Mémorise la dernière position du robot (pour « ajouter <nom> »)."""
        self.pose_robot = msg.pose.pose

    def publier_points(self):
        donnees = {'points': self.points, 'patrouille': self.patrouille,
                   'carte': self.carte, 'cartes': self.lister_cartes()}
        self.pub_points.publish(String(data=json.dumps(donnees)))

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
        if action == 'base':
            # Raccourci : « base » = « appel base » (le point de base s'appelle « base »).
            action, mots = 'appel', ['appel', 'base']

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
        elif action == 'ajouter' and len(mots) in (2, 4):
            self.ajouter_point(mots[1], mots[2:])
        elif action == 'supprimer' and len(mots) == 2:
            self.supprimer_point(mots[1])
        elif action == 'definir_patrouille':
            self.definir_patrouille(mots[1:])
        elif action == 'carte' and len(mots) == 2:
            self.changer_carte(mots[1])
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

    # --- Cartes ----------------------------------------------------------------

    def charger_points(self):
        """Lit les points de la carte actuelle (aucun point si le fichier n'existe pas)."""
        contenu = {}
        if os.path.isfile(self.chemin_points):
            with open(self.chemin_points) as fichier:
                contenu = yaml.safe_load(fichier) or {}
        self.points = contenu.get('points') or {}
        patrouille = contenu.get('patrouille') or []
        self.patrouille = [nom for nom in patrouille if nom in self.points]
        if len(self.patrouille) != len(patrouille):
            self.get_logger().warn('Points de patrouille inconnus ignorés.')

    def lister_cartes(self):
        """Cartes disponibles : les <nom>.yaml du dossier, sauf les fichiers de points."""
        try:
            fichiers = os.listdir(self.dossier_cartes)
        except OSError:
            return [self.carte]
        return sorted(f.removesuffix('.yaml') for f in fichiers
                      if f.endswith('.yaml') and not f.endswith('_points.yaml'))

    def changer_carte(self, nom):
        fichier_carte = os.path.join(self.dossier_cartes, nom + '.yaml')
        if not FORMAT_NOM.match(nom) or not os.path.isfile(fichier_carte):
            self.get_logger().warn(f'Carte introuvable : {fichier_carte}')
            return
        self.annuler_mission()
        self.passer_au_repos('VEILLE')
        # Service « map_server/load_map » de Nav2, appelé via BasicNavigator.
        # Bloque jusqu'à la réponse (quelques dixièmes de seconde).
        self.changeMap(fichier_carte)
        # Les obstacles vus sur l'ancienne carte n'ont plus de sens sur la nouvelle.
        self.clearAllCostmaps()
        self.carte = nom
        self.chemin_points = os.path.join(self.dossier_cartes, nom + '_points.yaml')
        self.charger_points()
        self.publier_points()
        self.get_logger().info(f'Carte {nom} chargée : replacer le robot sur la carte.')

    # --- Gestion des points (page web) ------------------------------------------

    def ajouter_point(self, nom, coordonnees):
        """Ajoute (ou remplace) un point : position du robot, ou x y donnés."""
        if not FORMAT_NOM.match(nom):
            self.get_logger().warn(f'Nom invalide : {nom} (minuscules, chiffres, _)')
            return
        if coordonnees:
            # Point choisi sur la carte : orientation 0 par défaut.
            try:
                x, y = (float(valeur) for valeur in coordonnees)
            except ValueError:
                self.get_logger().warn(f'Coordonnées invalides : {coordonnees}')
                return
            theta = 0.0
        else:
            # Position actuelle du robot, telle qu'estimée par AMCL.
            if self.pose_robot is None:
                self.get_logger().warn('Position du robot inconnue : faire « 2D Pose Estimate »')
                return
            x = self.pose_robot.position.x
            y = self.pose_robot.position.y
            # Inverse de creer_pose : quaternion (rotation autour de z) → angle.
            q = self.pose_robot.orientation
            theta = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)

        self.points[nom] = {'x': round(x, 2), 'y': round(y, 2), 'theta': round(theta, 2)}
        self.enregistrer_points()
        self.get_logger().info(f'Point {nom} enregistré : x={x:.2f} y={y:.2f}')

    def supprimer_point(self, nom):
        if nom not in self.points:
            self.get_logger().warn(f'Point inconnu : {nom}')
            return
        del self.points[nom]
        if nom in self.patrouille:
            self.patrouille = [p for p in self.patrouille if p != nom]
            self.verifier_patrouille()
        self.enregistrer_points()
        self.get_logger().info(f'Point {nom} supprimé.')

    def definir_patrouille(self, noms):
        inconnus = [nom for nom in noms if nom not in self.points]
        if inconnus:
            self.get_logger().warn(f'Points inconnus : {", ".join(inconnus)}')
            return
        self.patrouille = noms
        self.verifier_patrouille()
        self.enregistrer_points()
        self.get_logger().info(f'Patrouille : {" → ".join(noms) or "aucune"}')

    def verifier_patrouille(self):
        """Si la patrouille en cours se retrouve vide, on s'arrête proprement."""
        if self.mode == 'PATROUILLE' and not self.patrouille:
            self.annuler_mission()
            self.passer_au_repos('VEILLE')

    def enregistrer_points(self):
        """Réécrit le fichier de points, puis informe la page web."""
        contenu = {'points': self.points, 'patrouille': self.patrouille}
        try:
            with open(self.chemin_points, 'w') as fichier:
                fichier.write(EN_TETE_POINTS)
                # default_flow_style=None : chaque point tient sur une ligne {x, y, theta}.
                yaml.safe_dump(contenu, fichier, default_flow_style=None,
                               sort_keys=False, allow_unicode=True)
        except OSError as erreur:
            self.get_logger().error(f"Impossible d'écrire {self.chemin_points} : {erreur}")
        self.publier_points()


def main():
    # Comme dans aller_a : Ctrl+C doit lever une KeyboardInterrupt pour
    # qu'on puisse annuler la mission avant de quitter.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        repartiteur = Repartiteur()
    except (OSError, KeyError, ValueError, yaml.YAMLError) as erreur:
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
