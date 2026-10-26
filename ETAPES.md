# Étapes réalisées

Chaque terminal commence par :
```
source /opt/ros/jazzy/setup.bash
```

## Installation (une seule fois)
```
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-navigation2 ros-jazzy-key-teleop
```

## Étape 1 : Piloter le robot au clavier ✔

Terminal 1, simulation (Gazebo + RViz) :
```
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```
Terminal 2, clavier (flèches, `q` pour quitter) :
```
ros2 run key_teleop key_teleop --ros-args -r key_vel:=cmd_vel -p twist_stamped_enabled:=false
```

## Étape 2 : Cartographier (SLAM) ✔

Terminal 1 :
```
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False slam:=True
```
Terminal 2 : le clavier, comme à l'étape 1. Faire le tour de l'arène.

Terminal 3, sauvegarder la carte :
```
mkdir -p ~/Documents/Projet/argos_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/Documents/Projet/argos_ws/maps/arene
```
Résultat : `arene.pgm` (image) + `arene.yaml` (description).

## Étape 3 : Naviguer sur la carte ✔

```
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False map:=$HOME/Documents/Projet/argos_ws/maps/arene.yaml
```
Dans RViz :
1. **2D Pose Estimate** → placer le robot. **À faire dans les 60 s**, sinon la
   navigation abandonne (relancer).
2. **Nav2 Goal** → cliquer-glisser la destination.

Ce qui calcule le trajet :
- **NavFn** (Dijkstra) : le chemin le plus court sur la carte.
- **MPPI** : la conduite (marche arrière, orientation finale).

## Étape 4 : Aller à un point nommé ✔

Paquet `argos_mission` (dans `src/`) :
- `config/points.yaml` : nom → x, y, theta (relevés avec RViz → **Publish Point**
  + `ros2 topic echo /clicked_point`)
- `argos_mission/aller_a.py` : lit le YAML, envoie le goal à Nav2 via
  `BasicNavigator` (nav2_simple_commander)

Compiler (après chaque `git pull`) :
```
cd ~/Documents/Projet/argos_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```
Lancer (simulation de l'étape 3 démarrée, 2D Pose Estimate fait) :
```
ros2 run argos_mission aller_a alpha
```

## Étape 5 : Répartiteur à 3 modes ✔

- `maps/arene_points.yaml` : points + ordre de patrouille (un fichier par carte)
- `argos_mission/repartiteur.py` : machine à états VEILLE / APPEL / PATROUILLE
  (+ état ECHEC). Ne recalcule le chemin que s'il devient bloqué.
- `config/nav2_params.yaml` : copie des paramètres Nav2, marge d'obstacle
  `inflation_radius` 0.70 → **0.25** (jamais sous 0.22, le rayon du robot)

Terminal 1, simulation avec nos paramètres :
```
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False map:=$HOME/Documents/Projet/argos_ws/maps/arene.yaml params_file:=$HOME/Documents/Projet/argos_ws/src/argos_mission/config/nav2_params.yaml
```
Terminal 2, répartiteur :
```
ros2 run argos_mission repartiteur --ros-args -p fichier_points:=$HOME/Documents/Projet/argos_ws/maps/arene_points.yaml
```
Terminal 3, commandes (`appel <point>`, `patrouille`, `veille`) :
```
ros2 topic pub --once /argos/commande std_msgs/String "data: 'appel beta'"
```

Régler la marge en direct (sans relancer) :
```
ros2 param set /global_costmap/global_costmap inflation_layer.inflation_radius 0.25
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.25
```
Robot qui tourne sur place = manœuvre de secours de Nav2 (compteur « Recoveries »
dans RViz) : le contrôleur n'a pas trouvé comment avancer.

## Optimisation pour le Raspberry Pi 5 (en cours)

Allègements dans `config/nav2_params.yaml` (lignes marquées « ARGOS ») :
- MPPI : 1000 trajectoires (au lieu de 2000), model_dt 0.1 + time_steps 28
  (model_dt doit être >= 1/fréquence, sinon le contrôleur refuse de démarrer), 10 Hz (au lieu de 20 ; ne pas
  descendre plus bas), visualisation désactivée (la doc MPPI la déconseille hors débogage)
- Carte locale : `obstacle_layer` 2D au lieu de `voxel_layer` 3D (lidar 2D)
- AMCL : 200 à 1000 particules (au lieu de 500 à 2000)

À venir : fichier de lancement réduit (sans docking, routes, suivi de points).

Mesurer sur le Pi : `ros2 topic hz /cmd_vel_nav`, `htop`, `vcgencmd measure_temp`,
`vcgencmd get_throttled` (≠ 0x0 = le Pi ralentit à cause de la chaleur).

### Pistes notées (non appliquées)

- **Contrôleur Regulated Pure Pursuit au lieu de MPPI.** Observé : « Optimizer fail
  to compute path » dans les passages étroits → manœuvres de secours → nouveau chemin.
  RPP suit le chemin tracé, calcule beaucoup moins (bon pour le Pi), mais contourne
  moins bien un obstacle imprévu. Changement : un bloc dans `config/nav2_params.yaml`.
  À appliquer seulement si le besoin se confirme.

## Étape 6 : Page web ✔

Paquet `argos_web` (dans `src/`) :
- `web/index.html` : la page (carte, mode, points, réglages, journal)
- `web/roslib.min.js` : bibliothèque roslib.js 1.4.1 (copie locale, marche sans Internet)
- `launch/argos_web.launch.py` : démarre rosbridge (port 9090) + serveur HTTP (port 8080)

Le répartiteur publie aussi `/argos/points` et accepte : `base`, `ajouter <nom>`,
`ajouter <nom> <x> <y>`, `supprimer <nom>`, `definir_patrouille <nom>...`.
Point **`base`** = point de retour (bouton « Retour à la base »).

Première fois seulement (sinon rosdep n'installe rien, en silence) :
```
sudo rosdep init
rosdep update
```
Installer et compiler (après chaque `git pull`) :
```
cd ~/Documents/Projet/argos_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```
Lancer : simulation (terminal 1) + répartiteur (terminal 2) comme à l'étape 5, puis
terminal 3 :
```
ros2 launch argos_web argos_web.launch.py
```
Ouvrir **http://localhost:8080** (depuis un téléphone sur le même réseau :
`http://<adresse IP du PC>:8080`).

⚠ Aucune protection : toute personne sur le réseau peut piloter le robot. À traiter
avant un usage réel.
