"""Démarre tout le robot ARGOS en une commande (simulation pour l'instant).

  ros2 launch argos_mission argos.launch.py
  ros2 launch argos_mission argos.launch.py carte:=arene rviz:=false gazebo_gui:=false

Contenu (repris des fichiers de lancement de nav2_bringup, branche jazzy, réduits) :
  1. Simulation : Gazebo (monde entrepôt) + TurtleBot3 (à remplacer par les pilotes sur le vrai robot)
  2. Localisation : map_server + AMCL (localization_launch.py de Nav2, inclus tel quel)
     + SLAM Toolbox à l'arrêt, démarré seulement pour créer une carte
  3. Navigation : seulement les serveurs utiles, tous dans un seul processus.
     Retirés par rapport à Nav2 : docking_server, route_server, waypoint_follower.
  4. Répartiteur (argos_mission) + page web (argos_web)
  5. RViz (facultatif)

Une carte « X » = maps/X.yaml (+ X.pgm) et ses points maps/X_points.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, Node, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile

# Serveurs de navigation gardés, démarrés dans cet ordre par le gestionnaire de cycle de vie.
SERVEURS_NAVIGATION = [
    ('nav2_controller', 'nav2_controller::ControllerServer', 'controller_server'),
    ('nav2_smoother', 'nav2_smoother::SmootherServer', 'smoother_server'),
    ('nav2_planner', 'nav2_planner::PlannerServer', 'planner_server'),
    ('nav2_behaviors', 'behavior_server::BehaviorServer', 'behavior_server'),
    ('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator'),
    ('nav2_velocity_smoother', 'nav2_velocity_smoother::VelocitySmoother',
     'velocity_smoother'),
    ('nav2_collision_monitor', 'nav2_collision_monitor::CollisionMonitor',
     'collision_monitor'),
]


def generate_launch_description():
    dossier_bringup = get_package_share_directory('nav2_bringup')
    dossier_sim = get_package_share_directory('nav2_minimal_tb3_sim')
    dossier_argos = get_package_share_directory('argos_mission')

    # --- Arguments de lancement -------------------------------------------------
    carte = LaunchConfiguration('carte')
    dossier_cartes = LaunchConfiguration('dossier_cartes')
    rviz = LaunchConfiguration('rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    arguments = [
        DeclareLaunchArgument('carte', default_value='arene',
                              description='Nom de la carte (fichier maps/<carte>.yaml)'),
        # Le dépôt est cloné au même endroit sur le PC et sur le Pi.
        DeclareLaunchArgument('dossier_cartes',
                              default_value=os.path.expanduser('~/Documents/Projet/argos_ws/maps'),
                              description='Dossier des cartes et des fichiers de points'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Ouvrir RViz (outil de dev, la page web suffit)'),
        DeclareLaunchArgument('gazebo_gui', default_value='true',
                              description="Afficher la fenêtre 3D de Gazebo"),
    ]
    fichier_carte = PathJoinSubstitution([dossier_cartes, [carte, '.yaml']])
    fichier_points = PathJoinSubstitution([dossier_cartes, [carte, '_points.yaml']])
    params = os.path.join(dossier_argos, 'config', 'nav2_params.yaml')
    params_nav2 = ParameterFile(params, allow_substs=True)

    # --- 1. Simulation (repris de tb3_simulation_launch.py) ---------------------
    # Monde : entrepôt d'environ 27 x 45 m (worlds/warehouse.sdf de ce paquet).
    monde = os.path.join(dossier_argos, 'worlds', 'warehouse.sdf')
    gazebo = ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', monde], output='screen')
    fenetre_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        condition=IfCondition(gazebo_gui),
        launch_arguments={'gz_args': '-v4 -g '}.items())
    # Le robot apparaît au point (0, 0) de l'entrepôt (zone libre), qui devient
    # l'origine des cartes créées dans ce monde.
    robot_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(dossier_sim, 'launch', 'spawn_tb3.launch.py')),
        launch_arguments={
            'use_sim_time': 'true', 'robot_name': 'turtlebot3_waffle',
            'robot_sdf': os.path.join(dossier_sim, 'urdf', 'gz_waffle.sdf.xacro'),
            'x_pose': '0.00', 'y_pose': '0.00', 'z_pose': '0.01',
            'roll': '0.00', 'pitch': '0.00', 'yaw': '0.00',
        }.items())
    with open(os.path.join(dossier_sim, 'urdf', 'turtlebot3_waffle.urdf')) as fichier:
        description_robot = fichier.read()
    etat_robot = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': description_robot}],
        output='screen')

    # --- 2 et 3. Nav2 dans un seul processus (« composition ») ------------------
    conteneur = Node(
        package='rclcpp_components', executable='component_container_isolated',
        name='nav2_container', parameters=[params_nav2, {'autostart': True}],
        output='screen')
    localisation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(dossier_bringup, 'launch',
                                                   'localization_launch.py')),
        launch_arguments={
            'map': fichier_carte, 'use_sim_time': 'true', 'autostart': 'true',
            'params_file': params, 'use_composition': 'True',
            'container_name': 'nav2_container',
        }.items())
    # SLAM Toolbox (cartographie) : lancé « à l'arrêt » (autostart false). Le
    # répartiteur le démarre seulement quand on crée une carte (voir cartographie.py).
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={'autostart': 'false', 'use_sim_time': 'true'}.items())

    # Les serveurs qui publient des vitesses passent par le lisseur de vitesse.
    vers_lisseur = [('cmd_vel', 'cmd_vel_nav')]
    navigation = GroupAction([
        SetParameter('use_sim_time', True),
        LoadComposableNodes(target_container='/nav2_container', composable_node_descriptions=[
            ComposableNode(
                package=paquet, plugin=plugin, name=nom, parameters=[params_nav2],
                remappings=vers_lisseur if nom in (
                    'controller_server', 'behavior_server', 'velocity_smoother') else [])
            for paquet, plugin, nom in SERVEURS_NAVIGATION
        ] + [
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'autostart': True,
                             'node_names': [nom for _, _, nom in SERVEURS_NAVIGATION]}]),
        ]),
    ])

    # --- 4. Répartiteur et page web ---------------------------------------------
    repartiteur = Node(
        package='argos_mission', executable='repartiteur', output='screen',
        parameters=[{'fichier_points': fichier_points, 'use_sim_time': True}])
    page_web = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('argos_web'), 'launch', 'argos_web.launch.py')))

    # --- 5. RViz (facultatif) ---------------------------------------------------
    outil_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(dossier_bringup, 'launch',
                                                   'rviz_launch.py')),
        condition=IfCondition(rviz),
        launch_arguments={'use_sim_time': 'true'}.items())

    return LaunchDescription(arguments + [
        robot_sim, gazebo, fenetre_gazebo, etat_robot,
        conteneur, localisation, slam, navigation,
        repartiteur, page_web, outil_rviz,
    ])
