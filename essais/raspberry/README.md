# Essais : Raspberry Pi 5

Essai hors du projet ROS : installer et comprendre le Raspberry Pi 5 8 Go seul,
avant de le mettre sur le robot.

## Matériel

- Raspberry Pi 5 8 Go, boîtier HighPi Pro 5S, Active Cooler (PiShop.ca)
- Active Cooler : piloté par le système selon la température, arrêté sous 50 °C.
  Sans système (pas de microSD), il ne tourne pas : normal.
- microSD Raspberry Pi 128 Go A2
- Pas d'écran ni de câble micro-HDMI : tout se fait depuis le PC (SSH)
- Alimentation de bureau : chargeur USB-C 5 V 3 A du téléphone

## Plan

1. ✔ Installer Ubuntu Server 24.04 sans lecteur microSD (voir « Installation »)
2. ✔ Premier démarrage et connexion SSH depuis le PC
3. ✔ Mises à jour du système
4. ✔ Installer ROS 2 Jazzy (`ros-jazzy-ros-base` + `ros-dev-tools`, rosdep)
5. ✔ Échange ROS 2 entre le PC et le Pi
6. Brancher le LiDAR C1 sur le Pi, voir /scan dans RViz sur le PC
   (après l'essai d'alimentation : les ports USB sont limités à 600 mA en 3 A)

## Installation (2026-11-13)

Aucun PC n'avait de lecteur microSD (le PC Ubuntu a une fente SD pleine taille,
mais pas d'adaptateur). Contournement, sans rien acheter :

1. Sur le PC Windows, Ubuntu Server 24.04.5 pour Raspberry Pi
   (`ubuntu-24.04.5-preinstalled-server-arm64+raspi.img.xz`, somme de contrôle
   vérifiée) écrit sur une clé USB SanDisk 32 Go, puis relu pour vérification.
2. Premier démarrage préparé sur la partition `system-boot` de la clé
   (fichiers lus par cloud-init) :
   - `user-data` : nom `argos-pi`, utilisateur `serge`, connexion par clé SSH
     seulement (pas de mot de passe, donc `sudo` sans mot de passe), `avahi-daemon`
     pour joindre le Pi par `argos-pi.local`
   - `network-config` : Wi-Fi de la maison, pays `CA`
3. Le Pi démarre sur la clé (sans microSD), rejoint le Wi-Fi en 70 s.
4. Depuis le Pi, la même image est écrite sur la microSD insérée à chaud, avec les
   mêmes `user-data` et `network-config`.
5. Le Pi éteint, la clé retirée : il démarre sur la microSD. ✔
   La clé USB garde un Ubuntu de secours prêt à démarrer.

Connexion depuis le PC Windows : `ssh -i ~/.ssh/id_ed25519_argos serge@argos-pi.local`

## Mesures

- Premier démarrage : Ubuntu 24.04.5 LTS, noyau 6.8.0-1064-raspi, 7,8 Go de mémoire
  (7,4 Go libres), 54 °C au repos.
- Sur la microSD : partition agrandie seule à 115 Go (108 Go libres), démarrage et
  connexion Wi-Fi en moins de 2 min, 55,6 °C au repos, `usb_max_current_enable=0`.
- Adresse donnée par la box : 192.168.2.77 (clé), puis 192.168.2.78 (microSD).
  Elle peut changer : utiliser `argos-pi.local`.

## ROS 2 sur le Pi (2026-11-13)

Même méthode qu'`INSTALLATION.md`, mais `ros-jazzy-ros-base` au lieu de `desktop` :
pas d'écran, donc ni RViz ni outils graphiques (RViz tourne sur le PC). Script
`~/argos_claude/installer_ros.sh` sur le Pi : 6 minutes, puis un redémarrage.
201 paquets ROS, `colcon` et `rosdep` présents, `source /opt/ros/jazzy/setup.bash`
ajouté au `~/.bashrc`.

Vérification : `ros2 topic pub /essai_pi ...` sur le Pi, reçu par
`ros2 topic echo /essai_pi` sur le Pi **et sur le PC Ubuntu** par le Wi-Fi, sans
réglage (ROS_DOMAIN_ID 0 des deux côtés).
Note : `ros2 run demo_nodes_cpp talker` n'existe pas en `ros-base` (paquet de `desktop`).

Attention pour la suite : sur le même réseau et le même ROS_DOMAIN_ID, le Pi et la
simulation du PC voient les topics l'un de l'autre (deux `/cmd_vel`, deux `/scan`...).
Quand le robot réel et la simulation tourneront en même temps, donner un
ROS_DOMAIN_ID différent à la simulation.

## Paquets pour l'application (2026-11-20)

But : faire tourner sur le Pi le répartiteur et la page web, pour le rôle Manuel,
sans attendre le LiDAR.

Choix : **pas** le paquet complet `ros-jazzy-navigation2`. Il dépend de
`nav2_rviz_plugins`, donc de RViz et de Qt, inutiles sur un Pi sans écran.
`ros-jazzy-nav2-bringup` est pire (722 paquets, Gazebo compris). On installe
seulement ce que le code importe :
```
sudo apt install ros-jazzy-nav2-simple-commander ros-jazzy-rosbridge-server ros-jazzy-nav2-bt-navigator
```
(3 + 9 + 31 paquets, aucun graphique). `nav2_bt_navigator` est lu par
`repartiteur.py` au démarrage (arbre de comportement) et servira à la phase 3.
Le reste de Nav2 (contrôleur, planificateur, AMCL...) et SLAM Toolbox : avec le LiDAR.

Compilation : `colcon build --symlink-install --packages-select argos_base argos_mission argos_web`.
Vérifié : 49 tests passent sur le Pi ; le répartiteur démarre sans Nav2 (VEILLE,
points lus) ; la page répond (ports 8080 et 9090).
Mesure au repos : rosbridge 80 Mo de mémoire, 14 % d'un cœur au démarrage ;
serveur HTTP 18 Mo.

Veille : rosbridge (Python) est connu pour être lent avec de gros messages. Règle
retenue : la vidéo ne passe **pas** par rosbridge (flux MJPEG direct du nœud
caméra), rosbridge ne transporte que des petits messages (commandes, état,
journal). Remplaçant possible si besoin, compatible avec roslibjs : RWS
(C++, https://github.com/v-kiniv/rws). foxglove_bridge est plus rapide mais ne
parle pas le protocole de roslibjs.

Piège : `pkill -f "ros2 launch argos_web"` lancé dans une commande SSH qui contient
ce même texte tue aussi la session SSH. Arrêter par numéro de processus (`kill <pid>`).

## État

2026-11-13 : étapes 1 à 5 faites. Le Pi tourne sur la microSD, joignable en SSH,
ROS 2 Jazzy installé et vu depuis le PC. Étape 6 (LiDAR) après l'essai
d'alimentation (convertisseur 12 V vers 5 V reçu).

## Problèmes rencontrés

1. **Le Pi ne démarrait pas sur la clé USB.**
   - Symptôme : après 10 minutes, aucun appareil Raspberry Pi sur le réseau.
   - Diagnostic : tous les appareils du réseau essayés en SSH, aucun n'est le Pi ;
     puis lecture du livre blanc Raspberry Pi sur l'alimentation du Pi 5.
   - Cause : sans alimentation 5 A, le Pi 5 désactive le démarrage sur USB.
     Le chargeur du téléphone donne 3 A.
   - Solution : `usb_max_current_enable=1` (section `[pi5]` de `config.txt` sur la
     clé). Pas remis sur la microSD : sans cette ligne, les ports USB restent limités
     à 600 mA, ce qui protège le Pi tant qu'il est alimenté en 3 A.
   - Source : livre blanc « USB Power delivery on Raspberry Pi 5 » (RP-009856).

## À retenir pour le robot

- Alimentation : le convertisseur 12 V vers 5 V 5 A du robot lèvera la limite de
  600 mA des ports USB. Le LiDAR et la caméra y seront branchés : à vérifier avec
  `vcgencmd get_throttled` une fois sur la batterie.
- Le même premier démarrage (cloud-init) permet de réinstaller le Pi en 10 minutes,
  sans écran.
