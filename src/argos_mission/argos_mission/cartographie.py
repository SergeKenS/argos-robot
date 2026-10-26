"""Bascule entre navigation (carte enregistrée) et cartographie (nouvelle carte).

Les deux ne peuvent pas tourner en même temps : chacun publierait sa propre idée
de la position du robot sur la carte (transformation map → odom).
  Navigation   : map_server + amcl, gérés par lifecycle_manager_localization (Nav2)
  Cartographie : slam_toolbox, démarré « à l'arrêt » par argos.launch.py
On passe de l'un à l'autre avec les services standards de cycle de vie de ROS 2,
sans rien relancer.

Ces fonctions font tourner ROS en attendant les réponses : comme pour BasicNavigator,
on les appelle depuis la boucle principale du répartiteur, jamais depuis une réception
de message.
"""
import subprocess

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.srv import ManageLifecycleNodes


def appeler_service(noeud, type_service, nom, requete, delai=10.0):
    """Appelle un service et attend la réponse (None si absent ou trop lent)."""
    client = noeud.create_client(type_service, nom)
    try:
        if not client.wait_for_service(timeout_sec=delai):
            noeud.get_logger().error(f'Service indisponible : {nom}')
            return None
        futur = client.call_async(requete)
        rclpy.spin_until_future_complete(noeud, futur, timeout_sec=delai)
        return futur.result()
    finally:
        noeud.destroy_client(client)


def localisation(noeud, commande):
    """Arrête (RESET) ou redémarre (STARTUP) map_server + amcl.

    RESET plutôt qu'une simple pause (PAUSE) : en pause, map_server garderait sa
    dernière carte prête à être renvoyée à tout nouvel abonné, et map_saver_cli
    risquerait d'enregistrer l'ancienne carte au lieu de la nouvelle.
    """
    reponse = appeler_service(noeud, ManageLifecycleNodes,
                              'lifecycle_manager_localization/manage_nodes',
                              ManageLifecycleNodes.Request(command=commande))
    return reponse is not None and reponse.success


def slam(noeud, *transitions):
    """Applique des transitions de cycle de vie à slam_toolbox, dans l'ordre."""
    for transition in transitions:
        requete = ChangeState.Request()
        requete.transition.id = transition
        reponse = appeler_service(noeud, ChangeState, 'slam_toolbox/change_state', requete)
        if reponse is None or not reponse.success:
            noeud.get_logger().error(f'slam_toolbox : transition {transition} refusée')
            return False
    return True


def demarrer(noeud):
    """Navigation → cartographie. En cas d'échec, on revient à la navigation."""
    if not localisation(noeud, ManageLifecycleNodes.Request.RESET):
        return False
    # CONFIGURE crée une carte vierge, ACTIVATE lance la cartographie.
    if slam(noeud, Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
        return True
    localisation(noeud, ManageLifecycleNodes.Request.STARTUP)
    return False


def arreter(noeud):
    """Cartographie → navigation. CLEANUP efface la carte gardée en mémoire par SLAM.

    Au redémarrage, map_server recharge la carte du lancement : l'appelant doit
    ensuite charger la bonne carte (changeMap) et redonner la position du robot.
    """
    slam_arrete = slam(noeud, Transition.TRANSITION_DEACTIVATE, Transition.TRANSITION_CLEANUP)
    return localisation(noeud, ManageLifecycleNodes.Request.STARTUP) and slam_arrete


def sauvegarder(noeud, chemin_sans_extension):
    """Enregistre la carte publiée sur /map : crée <chemin>.pgm et <chemin>.yaml.

    Même outil qu'à l'étape 2 (map_saver_cli de Nav2), lancé comme une commande.
    Son délai d'attente de la carte est de 2 s par défaut : trop court quand tout
    le robot tourne (le nouveau programme met du temps à « trouver » les autres).
    On le porte à 10 s.
    """
    commande = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', chemin_sans_extension,
                '--ros-args', '-p', 'save_map_timeout:=10.0']
    try:
        resultat = subprocess.run(commande, capture_output=True, text=True, timeout=60)
    except subprocess.TimeoutExpired:
        noeud.get_logger().error('map_saver_cli : pas de réponse après 60 s')
        return False
    if resultat.returncode != 0:
        # Dernières lignes affichées par map_saver_cli, pour comprendre l'échec.
        lignes = (resultat.stdout + resultat.stderr).strip().splitlines()[-5:]
        noeud.get_logger().error('map_saver_cli : ' + ' | '.join(lignes))
        return False
    return True
