# Robot de service autonome

Un robot qui se déplace seul dans un bâtiment de type hôpital.
Quand on appelle depuis une chambre, il s'y rend sans aide.

Construit avec ROS 2 Jazzy et Nav2. La première phase se fait entièrement
en simulation (Gazebo), avant tout achat de matériel.

## Ce que fait le robot

Il a trois modes. Un seul est actif à la fois :

- **APPEL** : il va au point demandé et attend sur place.
- **PATROUILLE** : il fait une boucle entre des points définis.
- **VEILLE** : il ne bouge pas.

Les points (ex. `chambre_101`) sont définis dans un fichier YAML.

## Livrables de fin de projet

1. **Un robot réel** : 2 roues motrices, LiDAR 2D, IMU.
2. **Une carte** du bâtiment, faite par le robot lui-même.
3. **La navigation autonome** vers n'importe quel point nommé.
4. **Les trois modes** APPEL, PATROUILLE et VEILLE.
5. **Une application web** pour appeler le robot et changer de mode.
6. **Un paquet ROS 2 installable** sur une autre machine avec `rosdep`.

La phase 0 (simulation) livre les points 2 à 6, sans matériel.
