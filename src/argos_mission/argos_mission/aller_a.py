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


def charger_points():
    """Lit points.yaml (installé dans le dossier share du paquet) et renvoie {nom: {x, y, theta}}."""
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
    if len(sys.argv) < 2:
        print('Usage : ros2 run argos_mission aller_a <nom_du_point>')
        return
    nom = sys.argv[1]
    points = charger_points()
    if nom not in points:
        print(f'Point inconnu : {nom}. Points disponibles : {", ".join(points)}')
        return

    rclpy.init()
    # BasicNavigator (paquet nav2_simple_commander) encapsule l'action
    # NavigateToPose de Nav2 : on n'a pas à écrire le client d'action nous-mêmes.
    navigateur = BasicNavigator()

    if not navigateur.goToPose(creer_pose(navigateur, points[nom])):
        navigateur.get_logger().error(
            'Destination refusée par Nav2. As-tu fait « 2D Pose Estimate » dans RViz ?')
    else:
        navigateur.get_logger().info(f'En route vers {nom}...')
        # isTaskComplete() laisse ROS traiter les messages pendant ~0,1 s
        # puis indique si la navigation est terminée (réussie ou non).
        while not navigateur.isTaskComplete():
            pass

        resultat = navigateur.getResult()
        if resultat == TaskResult.SUCCEEDED:
            navigateur.get_logger().info(f'Arrivé à {nom}.')
        elif resultat == TaskResult.CANCELED:
            navigateur.get_logger().warn('Navigation annulée.')
        else:
            navigateur.get_logger().error(f'Échec : le robot n\'a pas pu atteindre {nom}.')

    navigateur.destroyNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
