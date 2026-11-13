"""Répartiteur : reçoit les commandes et les confie à la bonne fonctionnalité.

Une seule fonctionnalité active à la fois (dossier fonctionnalites/) :
  Service      : « appel <point> », « base », « patrouille »
  Cartographie : « cartographier », puis « enregistrer <nom> » ou « abandonner »
Aucune active = VEILLE : le robot ne fait rien (état de repos sûr, état au démarrage).

Commandes communes, traitées ici (texte sur /argos/commande) :
  « veille »                   : arrête la fonctionnalité active
  « ajouter <nom> »            : enregistre la position actuelle du robot
  « ajouter <nom> <x> <y> »    : enregistre un point choisi sur la carte
  « supprimer <nom> »
  « definir_patrouille <nom> <nom>... » : ordre de patrouille (vide = aucune)
  « carte <nom> »              : charge maps/<nom>.yaml et ses points maps/<nom>_points.yaml
  « redemarrer_navigation »    : relance Nav2 si son démarrage a avorté
Toute autre commande va à la fonctionnalité qui la comprend.

Publié :
  /argos/etat    : « VEILLE », « APPEL alpha », « PATROUILLE beta », « ECHEC alpha »,
                   « CARTOGRAPHIE »
  /argos/points  : points, patrouille, carte actuelle, cartes disponibles (JSON)
  /argos/journal : messages pour l'utilisateur (JSON : niveau, texte)

Usage :
  ros2 run argos_mission repartiteur --ros-args -p fichier_points:=<chemin du YAML>
"""
import json
import math
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from argos_mission import cycle_de_vie
from argos_mission.cartes import Cartes, ORIGINE
from argos_mission.fonctionnalites.cartographie import Cartographie
from argos_mission.fonctionnalites.service import Service

# Les fonctionnalités du robot. En ajouter une = une ligne ici.
FONCTIONNALITES = [Service, Cartographie]

# Arbre de comportement fourni par Nav2 : il ne recalcule le chemin que si le
# chemin actuel devient bloqué. Évite que le robot hésite entre deux chemins
# de coût presque égal (comportement par défaut : recalcul chaque seconde).
ARBRE_NAVIGATION = os.path.join(
    get_package_share_directory('nav2_bt_navigator'), 'behavior_trees',
    'navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml')


def pose_vers_point(pose):
    """Pose ROS → point {x, y, theta} ; sans pose connue, l'origine de la carte."""
    if pose is None:
        return dict(ORIGINE)
    q = pose.orientation
    theta = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
    return {'x': round(pose.position.x, 2), 'y': round(pose.position.y, 2),
            'theta': round(theta, 2)}


class Repartiteur(BasicNavigator):
    """Nœud unique : hérite de BasicNavigator pour envoyer les trajets à Nav2.

    BasicNavigator fait tourner ROS lui-même à chaque appel (goToPose, cancelTask...).
    On ne doit donc JAMAIS l'appeler depuis une fonction de réception de message :
    la réception se contente de noter la commande, la boucle principale la traite.
    """

    def __init__(self):
        super().__init__(node_name='repartiteur')
        # Messages pour l'utilisateur, affichés dans le journal de la page web.
        self.pub_journal = self.create_publisher(String, '/argos/journal', 10)

        # Fichier de points passé au lancement : maps/<carte>_points.yaml.
        self.cartes = Cartes(self.declare_parameter('fichier_points', '').value)
        if self.cartes.charger():
            self.signaler('avertissement', 'Points de patrouille inconnus ignorés.')

        self.fonctionnalites = [classe(self) for classe in FONCTIONNALITES]
        self.active = None             # fonctionnalité active (None = VEILLE)
        self.trajet_en_cours = False   # Nav2 conduit le robot vers un point
        self.etat_publie = None
        self.commande_en_attente = None
        self.pose_robot = None         # dernière position estimée par AMCL
        self.pose_slam = None          # dernière position estimée par SLAM Toolbox
        self.pose_depart_slam = None   # position au début de la cartographie (→ base)

        # Commandes communes : premier mot → (fonction, nombres de mots acceptés après lui).
        self.outils = {
            'ajouter': (self.ajouter_point, (1, 3)),
            'supprimer': (self.supprimer_point, (1,)),
            'definir_patrouille': (self.definir_patrouille, None),
            'carte': (self.changer_carte, (1,)),
            'redemarrer_navigation': (self.redemarrer_navigation, None),
        }

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
        # Pendant la cartographie, SLAM Toolbox publie la position du robot sur « pose ».
        self.create_subscription(PoseWithCovarianceStamped, 'pose', self.recevoir_pose_slam, 10)
        # Clients des services de cycle de vie créés tout de suite (voir cycle_de_vie.client).
        cycle_de_vie.preparer(self)

        self.publier_etat()
        self.publier_points()
        self.get_logger().info(f'Prêt. Points : {", ".join(self.cartes.points)}')

    # --- Réception ------------------------------------------------------------

    def recevoir_commande(self, msg):
        """Appelée par ROS à chaque message : on note seulement la commande."""
        self.commande_en_attente = msg.data

    def recevoir_pose(self, msg):
        """Mémorise la dernière position du robot (pour « ajouter <nom> »)."""
        self.pose_robot = msg.pose.pose

    def recevoir_pose_slam(self, msg):
        """Position sur la carte en cours de création ; la première devient la base."""
        # SLAM Toolbox ne publie que pendant la cartographie (remis à None au départ).
        self.pose_slam = msg.pose.pose
        if self.pose_depart_slam is None:
            self.pose_depart_slam = msg.pose.pose

    # --- Boucle principale et aiguillage ---------------------------------------

    def boucle(self):
        while rclpy.ok():
            if self.trajet_en_cours:
                # Laisse ROS travailler ~0,1 s et indique si Nav2 a terminé.
                termine = self.isTaskComplete()
            else:
                # Pas de trajet : on laisse simplement ROS recevoir les messages.
                rclpy.spin_once(self, timeout_sec=0.1)
                termine = False

            # Une nouvelle commande est prioritaire sur la fin du trajet.
            if self.commande_en_attente is not None:
                commande = self.commande_en_attente
                self.commande_en_attente = None
                self.traiter_commande(commande)
            elif termine:
                self.fin_de_trajet()

    def traiter_commande(self, texte):
        mots = texte.strip().lower().split()
        if not mots:
            return
        action = mots[0]
        active = self.active

        if active is not None and active.exclusive and action not in active.commandes:
            self.signaler('avertissement', active.exclusive)
        elif action == 'veille':
            self.basculer(None)
        elif action in self.outils:
            outil, tailles = self.outils[action]
            if tailles is not None and len(mots) - 1 not in tailles:
                self.signaler('avertissement', f'Commande incomplète : « {texte} »')
            else:
                outil(mots[1:])
        else:
            cible = next((f for f in self.fonctionnalites if action in f.commandes), None)
            if cible is None:
                self.signaler('avertissement', f'Commande inconnue : « {texte} »')
            elif cible is not active and action not in cible.entrees:
                self.signaler('avertissement',
                              f'« {action} » refusée : pas de {cible.nom} en cours.')
            elif self.basculer(cible):
                cible.commande(mots)
        self.faire_le_point()

    def fin_de_trajet(self):
        """Nav2 a terminé le trajet : la fonctionnalité active décide de la suite."""
        self.trajet_en_cours = False
        resultat = {TaskResult.SUCCEEDED: 'reussi',
                    TaskResult.CANCELED: 'annule'}.get(self.getResult(), 'echec')
        if self.active is not None:
            self.active.fin_de_tache(resultat)
        self.faire_le_point()

    def basculer(self, nouvelle):
        """Quitte la fonctionnalité active et entre dans la nouvelle (None = veille)."""
        if nouvelle is self.active:
            return True
        if self.active is not None:
            self.active.sortir()
        self.active = None
        if nouvelle is not None and not nouvelle.entrer():
            return False
        self.active = nouvelle
        return True

    def faire_le_point(self):
        """Après chaque action : retour en veille si la fonctionnalité a fini, puis état."""
        if self.active is not None and self.active.etat() is None:
            self.active.sortir()
            self.active = None
        self.publier_etat()

    def publier_etat(self):
        texte = 'VEILLE' if self.active is None else self.active.etat()
        if texte != self.etat_publie:
            self.etat_publie = texte
            self.pub_etat.publish(String(data=texte))
            self.get_logger().info(f'État : {texte}')

    # --- Outils du robot (les seuls que les fonctionnalités utilisent) ----------
    # self.cartes en fait partie : points, patrouille, cartes disponibles.

    def signaler(self, niveau, texte):
        """Écrit un message dans le terminal ET dans le journal de la page web.

        niveau : 'info', 'avertissement' ou 'erreur'.
        """
        # Un appel distinct par niveau : rclpy refuse qu'une même ligne de code
        # écrive tantôt en info, tantôt en erreur (« Logger severity cannot be changed »).
        if niveau == 'erreur':
            self.get_logger().error(texte)
        elif niveau == 'avertissement':
            self.get_logger().warn(texte)
        else:
            self.get_logger().info(texte)
        self.pub_journal.publish(String(data=json.dumps({'niveau': niveau, 'texte': texte})))

    def aller_vers(self, nom):
        """Demande à Nav2 de conduire le robot au point nommé. False si Nav2 refuse."""
        pose = self.creer_pose(self.cartes.points[nom])
        self.trajet_en_cours = bool(self.goToPose(pose, behavior_tree=ARBRE_NAVIGATION))
        return self.trajet_en_cours

    def annuler(self):
        """Arrête le trajet en cours, s'il y en a un."""
        if self.trajet_en_cours:
            self.cancelTask()
            self.trajet_en_cours = False

    def publier_points(self):
        self.pub_points.publish(String(data=json.dumps(self.cartes.donnees())))

    def afficher_carte(self, nom):
        """Donne la carte « nom » à Nav2 (service map_server/load_map, via BasicNavigator).

        Renvoie False si map_server ne répond pas. Sans cette vérification,
        changeMap attendrait sans fin et le robot ne répondrait plus à rien.
        """
        if not self.change_maps_srv.wait_for_service(timeout_sec=10.0):
            self.signaler('erreur', f'Carte {nom} non chargée : map_server ne répond pas. '
                          'Relancer le robot.')
            return False
        # Bloque jusqu'à la réponse (quelques dixièmes de seconde).
        self.changeMap(self.cartes.chemin_carte(nom))
        # Les obstacles vus sur l'ancienne carte n'ont plus de sens sur la nouvelle.
        self.clearAllCostmaps()
        return True

    def placer(self, point):
        """Indique à la navigation où se trouve le robot sur la carte."""
        self.setInitialPose(self.creer_pose(point))

    def demarrer_slam(self):
        self.pose_slam = None
        self.pose_depart_slam = None
        return cycle_de_vie.demarrer(self)

    def arreter_slam(self):
        return cycle_de_vie.arreter(self)

    def sauvegarder_carte(self, nom):
        """Enregistre la carte en cours de création : <dossier>/<nom>.yaml et .pgm."""
        return cycle_de_vie.sauvegarder(self, os.path.join(self.cartes.dossier, nom))

    def position_slam(self):
        return pose_vers_point(self.pose_slam)

    def depart_slam(self):
        return pose_vers_point(self.pose_depart_slam)

    def creer_pose(self, point):
        """Convertit un point {x, y, theta} en message PoseStamped dans le repère de la carte."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(point['x'])
        pose.pose.position.y = float(point['y'])
        # ROS exprime l'orientation en quaternion ; pour une simple rotation
        # autour de l'axe vertical, seuls z et w sont non nuls.
        pose.pose.orientation.z = math.sin(point['theta'] / 2)
        pose.pose.orientation.w = math.cos(point['theta'] / 2)
        return pose

    # --- Commandes communes ------------------------------------------------------

    def modifier_points(self, action, message):
        """Applique une modification des points, prévient l'utilisateur et la page."""
        try:
            action()
            self.signaler('info', message)
        except ValueError as refus:
            self.signaler('avertissement', str(refus))
            return
        except OSError as erreur:
            self.signaler('erreur',
                          f"Impossible d'écrire {self.cartes.chemin_points()} : {erreur}")
        self.publier_points()
        if self.active is not None:
            self.active.points_modifies()

    def ajouter_point(self, arguments):
        """Ajoute (ou remplace) un point : position du robot, ou x y donnés."""
        nom, coordonnees = arguments[0], arguments[1:]
        if coordonnees:
            # Point choisi sur la carte : orientation 0 par défaut.
            try:
                x, y = (float(valeur) for valeur in coordonnees)
            except ValueError:
                self.signaler('avertissement', f'Coordonnées invalides : {coordonnees}')
                return
            point = {'x': round(x, 2), 'y': round(y, 2), 'theta': 0.0}
        else:
            # Position actuelle du robot, telle qu'estimée par AMCL.
            if self.pose_robot is None:
                self.signaler('avertissement',
                              'Position du robot inconnue : le placer sur la carte.')
                return
            point = pose_vers_point(self.pose_robot)
        self.modifier_points(lambda: self.cartes.ajouter(nom, point),
                             f'Point {nom} enregistré : x={point["x"]} y={point["y"]}')

    def supprimer_point(self, arguments):
        self.modifier_points(lambda: self.cartes.supprimer(arguments[0]),
                             f'Point {arguments[0]} supprimé.')

    def definir_patrouille(self, noms):
        self.modifier_points(lambda: self.cartes.definir_patrouille(noms),
                             f'Patrouille : {" → ".join(noms) or "aucune"}')

    def changer_carte(self, arguments):
        nom = arguments[0]
        if not self.cartes.existe(nom):
            self.signaler('avertissement', f'Carte introuvable : {nom}')
            return
        self.basculer(None)
        if not self.afficher_carte(nom):
            return
        if self.cartes.charger(nom):
            self.signaler('avertissement', 'Points de patrouille inconnus ignorés.')
        self.publier_points()
        self.signaler('info', f'Carte {nom} chargée : replacer le robot sur la carte.')

    def redemarrer_navigation(self, arguments):
        """Relance les serveurs de Nav2 si leur démarrage a avorté (voir cycle_de_vie.py)."""
        # Leur activation attend que le robot soit placé : inutile d'essayer avant.
        if self.pose_robot is None:
            self.signaler('avertissement', "Placer d'abord le robot sur la carte.")
            return
        self.basculer(None)
        self.signaler('info', 'Redémarrage de la navigation...')
        if cycle_de_vie.redemarrer_navigation(self):
            self.signaler('info', 'Navigation prête.')
        else:
            self.signaler('erreur', 'Redémarrage impossible : relancer le robot.')


def main():
    # Ctrl+C doit lever une KeyboardInterrupt (et non arrêter ROS aussitôt) pour
    # qu'on puisse annuler le trajet avant de quitter.
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
        repartiteur.annuler()

    repartiteur.destroyNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
