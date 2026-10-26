"""Répartiteur : machine à états qui pilote le robot selon trois modes.

Modes (un seul actif à la fois) :
  VEILLE     : le robot ne fait rien (état de repos sûr, état au démarrage)
  APPEL      : va au point demandé, puis repasse en VEILLE sur place
  PATROUILLE : enchaîne en boucle les points de la liste « patrouille »
Plus un état de repos ECHEC : un appel a échoué, le robot attend une commande.
Et un état CARTOGRAPHIE : on dessine une nouvelle carte (pilotage manuel seulement).

Commandes (texte sur /argos/commande) :
  « veille », « appel <point> », « patrouille », « base » (= « appel base »)
  « ajouter <nom> »            : enregistre la position actuelle du robot
  « ajouter <nom> <x> <y> »    : enregistre un point choisi sur la carte
  « supprimer <nom> »
  « definir_patrouille <nom> <nom>... » : ordre de patrouille (vide = aucune)
  « carte <nom> »              : charge maps/<nom>.yaml et ses points maps/<nom>_points.yaml
  « redemarrer_navigation »    : relance Nav2 si son démarrage a avorté
  « cartographier »            : passe en CARTOGRAPHIE (SLAM Toolbox)
  « enregistrer <nom> »        : sauvegarde la nouvelle carte et repasse en navigation dessus
  « abandonner »               : quitte la cartographie sans sauvegarder
État publié sur /argos/etat : « VEILLE », « APPEL alpha », « PATROUILLE beta »...
Points, carte actuelle et cartes disponibles publiés sur /argos/points (JSON).
Messages pour l'utilisateur publiés sur /argos/journal (JSON : niveau, texte).

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

from argos_mission import cartographie
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
        # Messages pour l'utilisateur, affichés dans le journal de la page web.
        self.pub_journal = self.create_publisher(String, '/argos/journal', 10)

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
        self.pose_slam = None          # dernière position estimée par SLAM Toolbox
        self.depart_carto = None       # position au début de la cartographie (→ base)

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

    def recevoir_pose_slam(self, msg):
        """Position sur la carte en cours de création ; la première devient la base."""
        # SLAM Toolbox ne publie que pendant la cartographie (remis à None au départ).
        self.pose_slam = msg.pose.pose
        if self.depart_carto is None:
            self.depart_carto = msg.pose.pose

    def publier_points(self):
        donnees = {'points': self.points, 'patrouille': self.patrouille,
                   'carte': self.carte, 'cartes': self.lister_cartes()}
        self.pub_points.publish(String(data=json.dumps(donnees)))

    def signaler(self, niveau, texte):
        """Écrit un message dans le terminal ET dans le journal de la page web.

        niveau : 'info', 'avertissement' ou 'erreur'.
        """
        ecrire = {'info': self.get_logger().info, 'avertissement': self.get_logger().warn,
                  'erreur': self.get_logger().error}[niveau]
        ecrire(texte)
        self.pub_journal.publish(String(data=json.dumps({'niveau': niveau, 'texte': texte})))

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

        # En cartographie, on n'accepte que de la terminer (pilotage manuel à part).
        if self.mode == 'CARTOGRAPHIE':
            if action == 'enregistrer' and len(mots) == 2:
                self.enregistrer_carte(mots[1])
            elif action == 'abandonner':
                self.abandonner_cartographie()
            else:
                self.signaler('avertissement',
                              'Cartographie en cours : enregistrer ou abandonner.')
            return

        if action == 'veille':
            self.annuler_mission()
            self.passer_au_repos('VEILLE')
        elif action == 'appel' and len(mots) == 2:
            if mots[1] not in self.points:
                self.signaler('avertissement', f'Point inconnu : {mots[1]} (commande ignorée)')
                return
            self.annuler_mission()
            self.mode = 'APPEL'
            self.lancer_vers(mots[1])
        elif action == 'patrouille':
            if not self.patrouille:
                self.signaler('avertissement', 'Aucune patrouille définie (commande ignorée)')
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
        elif action == 'cartographier':
            self.demarrer_cartographie()
        elif action == 'redemarrer_navigation':
            self.redemarrer_navigation()
        else:
            self.signaler('avertissement', f'Commande inconnue : « {texte} »')

    def fin_de_mission(self):
        """Nav2 a terminé la mission en cours : on décide de la suite."""
        resultat = self.getResult()

        if resultat == TaskResult.CANCELED:
            # Annulée hors du répartiteur (ex. bouton Cancel de RViz).
            self.passer_au_repos('VEILLE')
        elif self.mode == 'APPEL':
            if resultat == TaskResult.SUCCEEDED:
                self.signaler('info', f'Arrivé à {self.cible}.')
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
                self.signaler('avertissement', f'{self.cible} inaccessible, point suivant.')
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
            self.signaler('erreur', 'Destination refusée : placer le robot, puis '
                          '« Redémarrer la navigation ».')
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
            self.signaler('avertissement', 'Points de patrouille inconnus ignorés.')

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
            self.signaler('avertissement', f'Carte introuvable : {fichier_carte}')
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
        self.signaler('info', f'Carte {nom} chargée : replacer le robot sur la carte.')

    def redemarrer_navigation(self):
        """Relance les serveurs de Nav2 si leur démarrage a avorté (voir cartographie.py)."""
        # Leur activation attend que le robot soit placé : inutile d'essayer avant.
        if self.pose_robot is None:
            self.signaler('avertissement', "Placer d'abord le robot sur la carte.")
            return
        self.annuler_mission()
        self.passer_au_repos('VEILLE')
        self.signaler('info', 'Redémarrage de la navigation...')
        if cartographie.redemarrer_navigation(self):
            self.signaler('info', 'Navigation prête.')
        else:
            self.signaler('erreur', 'Redémarrage impossible : relancer le robot.')

    # --- Création d'une carte (voir cartographie.py) ---------------------------

    def demarrer_cartographie(self):
        self.annuler_mission()
        self.passer_au_repos('VEILLE')
        self.pose_slam = None
        self.depart_carto = None
        if cartographie.demarrer(self):
            self.mode = 'CARTOGRAPHIE'
            self.publier_etat()
        else:
            self.signaler('erreur', 'Impossible de démarrer la cartographie.')

    def enregistrer_carte(self, nom):
        fichier = os.path.join(self.dossier_cartes, nom)
        if not FORMAT_NOM.match(nom):
            self.signaler('avertissement',
                          f'Nom de carte invalide : {nom} (minuscules, chiffres, _)')
            return
        if os.path.exists(fichier + '.yaml'):
            self.signaler('avertissement', f'La carte {nom} existe déjà : choisir un autre nom.')
            return
        if not cartographie.sauvegarder(self, fichier):
            # On reste en cartographie : rien n'est perdu, on peut réessayer.
            self.signaler('erreur', f'Échec de la sauvegarde de {fichier}.')
            return

        # Positions à retenir avant d'arrêter SLAM Toolbox.
        pose_actuelle, depart = self.pose_slam, self.depart_carto
        cartographie.arreter(self)

        # Nouvelle carte, avec un point « base » là où la cartographie a commencé.
        self.carte = nom
        self.chemin_points = fichier + '_points.yaml'
        self.points = {'base': self.pose_vers_point(depart)}
        self.patrouille = []
        self.enregistrer_points()
        self.changeMap(fichier + '.yaml')
        self.clearAllCostmaps()

        # SLAM connaissait la position exacte du robot sur cette carte : on la donne
        # à AMCL, ainsi pas besoin de « Placer le robot ».
        self.setInitialPose(creer_pose(self, self.pose_vers_point(pose_actuelle)))
        self.passer_au_repos('VEILLE')
        self.signaler('info', f'Carte {nom} enregistrée et chargée.')

    def abandonner_cartographie(self):
        cartographie.arreter(self)
        # Recharger l'ancienne carte la republie (la page et Nav2 l'affichent à nouveau).
        self.changeMap(os.path.join(self.dossier_cartes, self.carte + '.yaml'))
        self.clearAllCostmaps()
        self.passer_au_repos('VEILLE')
        self.signaler('info', 'Cartographie abandonnée : replacer le robot sur la carte.')

    @staticmethod
    def pose_vers_point(pose):
        """Pose ROS → point {x, y, theta} ; sans pose connue, l'origine de la carte."""
        if pose is None:
            return {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        q = pose.orientation
        theta = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
        return {'x': round(pose.position.x, 2), 'y': round(pose.position.y, 2),
                'theta': round(theta, 2)}

    # --- Gestion des points (page web) ------------------------------------------

    def ajouter_point(self, nom, coordonnees):
        """Ajoute (ou remplace) un point : position du robot, ou x y donnés."""
        if not FORMAT_NOM.match(nom):
            self.signaler('avertissement', f'Nom invalide : {nom} (minuscules, chiffres, _)')
            return
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
            point = self.pose_vers_point(self.pose_robot)

        self.points[nom] = point
        self.enregistrer_points()
        self.signaler('info', f'Point {nom} enregistré : x={point["x"]} y={point["y"]}')

    def supprimer_point(self, nom):
        if nom not in self.points:
            self.signaler('avertissement', f'Point inconnu : {nom}')
            return
        del self.points[nom]
        if nom in self.patrouille:
            self.patrouille = [p for p in self.patrouille if p != nom]
            self.verifier_patrouille()
        self.enregistrer_points()
        self.signaler('info', f'Point {nom} supprimé.')

    def definir_patrouille(self, noms):
        inconnus = [nom for nom in noms if nom not in self.points]
        if inconnus:
            self.signaler('avertissement', f'Points inconnus : {", ".join(inconnus)}')
            return
        self.patrouille = noms
        self.verifier_patrouille()
        self.enregistrer_points()
        self.signaler('info', f'Patrouille : {" → ".join(noms) or "aucune"}')

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
            self.signaler('erreur', f"Impossible d'écrire {self.chemin_points} : {erreur}")
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
