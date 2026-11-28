# Essai du LiDAR RPLIDAR C1 (2026-11-28)

But : faire marcher le LiDAR seul sur le Pi, avant de le fixer sur le robot.
Pour cet essai, il était posé sur une table, à côté du robot.

## Matériel

- RPLIDAR C1 (Slamtec) : 5 000 mesures par seconde, 8 à 12 tours/s (10 par défaut),
  0,72° entre deux mesures à 10 tours/s, portée 0,05 à 12 m (6 m sur du noir),
  précision ±30 mm, résolution 15 mm
  ([fiche Slamtec](https://www.slamtec.com/en/C1/Spec)).
- Liaison USB par puce CP2102N (10c4:ea60), 460 800 bauds. Elle a un numéro de série,
  donc un nom fixe sans règle udev :
  `/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_2426acec1c5ef01189c75c401045c30f-if00-port0`
  (les deux cartes CH340, moteurs et tourelle, n'ont pas de numéro de série : nos
  programmes les trouvent par leur identifiant USB).

## Installation du pilote (sur le Pi, une seule fois)

Pilote officiel de Slamtec, compilé dans un espace de travail à part, hors du dépôt :
```
mkdir -p ~/pilotes_ws/src && cd ~/pilotes_ws/src
git clone --depth 1 https://github.com/Slamtec/sllidar_ros2.git
cd ~/pilotes_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
```
(20 s de compilation sur le Pi 5.)

## Lancer

```
source ~/pilotes_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_2426acec1c5ef01189c75c401045c30f-if00-port0
```
Voir le scan sur le PC Ubuntu : `rviz2`, repère fixe `laser`, affichage LaserScan sur
`/scan` (fiabilité « Best Effort »).

## Résultats

| Étape | Résultat |
|---|---|
| 1. Branchement | Reconnu au premier démarrage, nom fixe (voir plus haut) |
| 2. Pilote | `/scan` à 10,0 tours/s (écart 1 ms), 720 cases par tour, 86 % valides |
| 3. Affichage | Scan visible dans RViz sur le PC ; reçu par Wi-Fi à 9,7 tours/s |
| 4. Précision | Tablette posée à 1,00 m : vue à 1,026 m, 1 mm de variation sur 92 mesures (écart probablement dû au point de départ de la mesure au mètre : bord ou centre du LiDAR) |
| 4. Orientation | La **flèche** du LiDAR correspond à **180°** dans ROS (0° = côté câble). Sur le robot : déclarer le LiDAR tourné d'un demi-tour |
| 4. Zone cachée | Côté câble, obstacle fixe à 7,8 cm sur environ 50° : sans doute le câble ou son adaptateur |

À savoir :
- Les 720 cases sont un découpage du pilote : le C1 fait environ 500 mesures par tour
  à 10 tours/s. Les trous isolés (une case) viennent de là.
- Un tour ne donne qu'une **tranche** de la pièce, à la hauteur du LiDAR. La carte
  se construit en roulant (SLAM), en assemblant des milliers de tranches.
- Posé sur une table encombrée, le LiDAR voit surtout les objets de la table : des
  traits séparés, à cause des sauts de distance entre objets proches et murs.
- La taille des points dans RViz ne change que l'affichage. La finesse de la carte
  est la taille des cases du SLAM : garder 5 cm (2,5 cm au plus fin utile, vu la
  précision de ±3 cm).

## Problèmes rencontrés

1. **Le pilote installé par apt ne démarre pas la mesure.**
   - Symptôme : `ros-jazzy-rplidar-ros` (2.1.0) lit le numéro de série et la santé du
     LiDAR, puis « Cannot start scan: '80008002' ». Pas de fichier de lancement C1.
   - Cause : sa bibliothèque Slamtec (SDK 1.12) est trop ancienne pour le C1.
   - Solution : pilote officiel `sllidar_ros2` compilé depuis ses sources (voir
     Installation) ; le paquet apt est retiré, pour n'avoir qu'un pilote.
2. **La tablette n'apparaissait pas à 1 m.**
   - Cause : la personne qui prenait la photo était juste derrière le LiDAR (mesure
     à 8 cm). Faire les mesures en s'éloignant d'au moins 1 m.

## Reste à faire (avec le montage définitif)

1. Fixer le LiDAR sur le robot : plan du faisceau dégagé sur 360° (tourelle,
   montants, câbles), hauteur de 15 à 25 cm, câble vers le bas, flèche vers l'avant.
2. Mesurer sa position par rapport au milieu de l'axe des roues (avant ou arrière,
   hauteur), pour la déclarer à ROS.
3. Essai de cartographie : moteurs, LiDAR, SLAM Toolbox, conduite lente depuis la
   page, carte visible dans RViz, puis sauvegardée.
4. Alimentation : Pi, LiDAR et moteurs ensemble sur batterie, sans redémarrage du Pi.
