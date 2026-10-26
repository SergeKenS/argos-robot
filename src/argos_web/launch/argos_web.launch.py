"""Démarre la page web du robot.

  - rosbridge (port 9090) : pont WebSocket entre la page et les topics ROS
  - serveur HTTP de Python (port 8080) : sert les fichiers du dossier web/

Usage : ros2 launch argos_web argos_web.launch.py
Puis ouvrir http://localhost:8080 (ou http://<adresse du robot>:8080).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    dossier_web = os.path.join(get_package_share_directory('argos_web'), 'web')

    # Fichier de lancement fourni par rosbridge_server (on ne le réécrit pas).
    rosbridge = IncludeLaunchDescription(AnyLaunchDescriptionSource(os.path.join(
        get_package_share_directory('rosbridge_server'), 'launch',
        'rosbridge_websocket_launch.xml')))

    # Serveur web intégré à Python : suffisant pour quelques fichiers statiques.
    serveur_http = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8080', '--directory', dossier_web],
        output='screen')

    return LaunchDescription([rosbridge, serveur_http])
