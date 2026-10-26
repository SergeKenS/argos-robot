"""Bascule entre navigation (carte enregistrée) et cartographie (nouvelle carte).

Les deux ne peuvent pas tourner en même temps : chacun publierait sa propre idée
de la position du robot sur la carte (transformation map → odom).
  Navigation   : map_server + amcl, gérés par lifecycle_manager_localization (Nav2)
  Cartographie : slam_toolbox, démarré « à l'arrêt » par argos.launch.py
On passe de l'un à l'autre avec les services standards de cycle de vie de ROS 2,
sans rien relancer.

Le même mécanisme sert à redémarrer la navigation si son lancement a avorté
(voir redemarrer_navigation).

Ces fonctions font tourner ROS en attendant les réponses : comme pour BasicNavigator,
on les appelle depuis la boucle principale du répartiteur, jamais depuis une réception
de message.
"""
import subprocess

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes

# Serveurs de navigation, dans l'ordre de démarrage (mêmes noms que argos.launch.py).
SERVEURS_NAVIGATION = ['controller_server', 'smoother_server', 'planner_server',
                       'behavior_server', 'bt_navigator', 'velocity_smoother',
                       'collision_monitor']

# Activer un serveur qui a une carte de coûts attend que le robot soit placé
# (initial_transform_timeout = 300 s dans nav2_params.yaml) : on laisse un peu plus.
DELAI_ACTIVATION = 330.0


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


def changer_etat(noeud, nom, *transitions, delai=10.0):
    """Applique des transitions de cycle de vie au nœud « nom », dans l'ordre."""
    for transition in transitions:
        requete = ChangeState.Request()
        requete.transition.id = transition
        reponse = appeler_service(noeud, ChangeState, f'{nom}/change_state', requete, delai)
        if reponse is None or not reponse.success:
            noeud.get_logger().error(f'{nom} : transition {transition} refusée')
            return False
    return True


def slam(noeud, *transitions):
    """Applique des transitions de cycle de vie à slam_toolbox, dans l'ordre."""
    return changer_etat(noeud, 'slam_toolbox', *transitions)


def redemarrer_navigation(noeud):
    """Remet en marche les serveurs de navigation restés arrêtés.

    Cas typique : le robot n'a pas été placé dans les 5 min après le lancement.
    Nav2 a alors abandonné son démarrage à mi-chemin : certains serveurs sont
    actifs, d'autres seulement configurés. On reprend serveur par serveur :
    configurer ce qui ne l'est pas, activer ce qui ne l'est pas.
    (Un RESET du gestionnaire de Nav2 risquerait d'échouer : il s'arrête dès qu'il
    ne peut pas désactiver un serveur, ce qui arrive si ce serveur n'est pas actif.)
    """
    for nom in SERVEURS_NAVIGATION:
        reponse = appeler_service(noeud, GetState, f'{nom}/get_state', GetState.Request())
        if reponse is None:
            return False
        etat = reponse.current_state.id
        if etat == State.PRIMARY_STATE_UNCONFIGURED:
            if not changer_etat(noeud, nom, Transition.TRANSITION_CONFIGURE):
                return False
            etat = State.PRIMARY_STATE_INACTIVE
        if etat == State.PRIMARY_STATE_INACTIVE:
            if not changer_etat(noeud, nom, Transition.TRANSITION_ACTIVATE,
                                delai=DELAI_ACTIVATION):
                return False
        elif etat != State.PRIMARY_STATE_ACTIVE:
            noeud.get_logger().error(f'{nom} : état {reponse.current_state.label}, '
                                     'relancer le robot')
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
