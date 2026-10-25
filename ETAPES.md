# Étapes réalisées

Chaque terminal commence par :
```
source /opt/ros/jazzy/setup.bash
```

## Installation (une seule fois)
```
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-navigation2 ros-jazzy-key-teleop
```

## Étape 1 — Piloter le robot au clavier ✔

Terminal 1 — simulation (Gazebo + RViz) :
```
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```
Terminal 2 — clavier (flèches, `q` pour quitter) :
```
ros2 run key_teleop key_teleop --ros-args -r key_vel:=cmd_vel -p twist_stamped_enabled:=false
```

## Étape 2 — Cartographier (SLAM) ✔

Terminal 1 :
```
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False slam:=True
```
Terminal 2 : le clavier, comme à l'étape 1. Faire le tour de l'arène.

Terminal 3 — sauvegarder la carte :
```
mkdir -p ~/Documents/Projet/argos_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/Documents/Projet/argos_ws/maps/arene
```
Résultat : `arene.pgm` (image) + `arene.yaml` (description).

## Étape 3 — Naviguer sur la carte ✔

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

## Étape 4 — Aller à un point nommé ✔

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
