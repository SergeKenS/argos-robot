"""Envoie le robot vers un point nommé défini dans config/points.yaml.

Usage : ros2 run argos_mission aller_a <nom_du_point>
"""
import math
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.signals import SignalHandlerOptions


def charger_points():
    """Lit points.yaml (installé avec le paquet) et renvoie {nom: {x, y, theta}}."""
    chemin = os.path.join(get_package_share_directory('argos_mission'), 'config', 'points.yaml')
    with open(chemin) as fichier:
        return yaml.safe_load(fichier)['points']


def creer_pose(navigateur, point):
    """Convertit un point {x, y, theta} en message PoseStamped dans le repère de la carte."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigateur.get_clock().now().to_msg()
    pose.pose.position.x = float(point['x'])
    pose.pose.position.y = float(point['y'])
    # ROS exprime l'orientation en quaternion ; pour une simple rotation
    # autour de l'axe vertical, seuls z et w sont non nuls.
    pose.pose.orientation.z = math.sin(point['theta'] / 2)
    pose.pose.orientation.w = math.cos(point['theta'] / 2)
    return pose


def main():
    # Vérifications avant de démarrer ROS : un nom est-il donné, et existe-t-il ?
    # En cas d'erreur, on sort avec le code 1 pour qu'un programme appelant le sache.
    if len(sys.argv) < 2:
        print('Usage : ros2 run argos_mission aller_a <nom_du_point>')
        sys.exit(1)
    nom = sys.argv[1]
    points = charger_points()
    if nom not in points:
        print(f'Point inconnu : {nom}. Points disponibles : {", ".join(points)}')
        sys.exit(1)

    # Par défaut, ROS intercepte Ctrl+C et s'arrête aussitôt : on ne pourrait
    # plus annuler la navigation. On désactive ce comportement pour que Ctrl+C
    # lève une KeyboardInterrupt, que l'on traite plus bas.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    # BasicNavigator (paquet nav2_simple_commander) encapsule l'action
    # NavigateToPose de Nav2 : on n'a pas à écrire le client d'action nous-mêmes.
    navigateur = BasicNavigator()
    code_sortie = 1

    try:
        if not navigateur.goToPose(creer_pose(navigateur, points[nom])):
            navigateur.get_logger().error('Destination refusée : Nav2 est-il actif ?')
        else:
            navigateur.get_logger().info(f'En route vers {nom}...')
            # isTaskComplete() laisse ROS traiter les messages pendant ~0,1 s
            # puis indique si la navigation est terminée (réussie ou non).
            while not navigateur.isTaskComplete():
                pass

            resultat = navigateur.getResult()
            if resultat == TaskResult.SUCCEEDED:
                navigateur.get_logger().info(f'Arrivé à {nom}.')
                code_sortie = 0
            elif resultat == TaskResult.CANCELED:
                navigateur.get_logger().warn('Navigation annulée.')
            else:
                navigateur.get_logger().error(
                    f"Échec vers {nom}. Le robot est-il localisé (2D Pose Estimate) ?")
    except KeyboardInterrupt:
        # Sans cette annulation, Nav2 continuerait à conduire le robot
        # vers la destination même après l'arrêt du script.
        navigateur.get_logger().warn('Ctrl+C : annulation de la navigation.')
        navigateur.cancelTask()

    navigateur.destroyNode()
    rclpy.shutdown()
    sys.exit(code_sortie)


if __name__ == '__main__':
    main()
