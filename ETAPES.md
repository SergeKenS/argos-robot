# Feuille de route

Mise à jour le 2026-11-28. Principe : un composant, un essai, une mesure. Peu de
fonctions nouvelles avant que le robot réel roule. Architecture : `ARCHITECTURE.md`.

| Phase | Contenu | État |
|---|---|---|
| 0. Simulation | Étapes 1 à 6 plus bas : clavier, SLAM, Nav2, points, répartiteur, page web | Fait |
| 1. Base logicielle | Répartiteur en rôles (ex-fonctionnalités), tests automatiques, application web, rôle Manuel | Validé en simulation (2026-11-13) ; application faite (2026-11-20) ; reste 2, 3 et 4 |
| 2. Essais par composant | Tourelle, Pi 5, LiDAR, moteurs, alimentation | Tourelle, Pi, moteurs ✔ ; LiDAR seul ✔ (2026-11-28) ; reste : montage définitif, puis cartographie |
| 3. Robot réel | Assemblage, pilotes, refaire les étapes 1 à 6 sur le vrai robot | À venir |
| 4. Tête | Tourelle sur le robot, nœud « regard », rôle Suivi | À venir |
| 5. Supervision | Claude comme superviseur, rôle Observation | À venir |

## Phase 1 : base logicielle

1. **Valider la réorganisation en simulation** ✔ (2026-11-13) : 25 vérifications sur
   25, scénario automatique `essais/architecture/non_regression.py`. Reste à faire à
   la main : Ctrl+C pendant un trajet. Branche `revue-architecture`, à fusionner
   dans `main` quand l'utilisateur l'a vue.
2. **Lancement sans simulation** : `argos.launch.py` lance toujours Gazebo et
   `use_sim_time`. Ajouter un argument `simulation:=false` (ou un second fichier)
   avant la phase 3. Réussi = Nav2 démarre sans Gazebo, en attente des capteurs.
3. **Protéger la page** (mot de passe, rosbridge limité au réseau local) : avant
   tout usage hors de la maison, et avant d'ouvrir un accès au superviseur.
4. **Rôle Manuel** (conduire le corps et la tête depuis la page, sans LiDAR).
   - ✔ Application (2026-11-20) : `src/argos_web/web/` découpée (thème, onglets par
     rôle, menu, panneau Manuel pour PC, téléphone couché et debout). Validée par
     l'utilisateur sur son téléphone, avec un banc d'essai provisoire
     (`essais/appli/`, résultats et problèmes résolus dans son README).
   - ✔ `pont_moteurs` : vitesse maximale 0,70 m/s, rampe d'accélération (0,4 m/s²),
     roue libre quand les ordres cessent, freinage sur ordre 0 ou sens opposé.
   - Reste, dans l'ordre (arborescence dans `ARCHITECTURE.md`) : les organes dans le
     répartiteur ; `roles/manuel.py` et ses tests (un test par bouton) ; les nœuds
     `tourelle` et `camera` (la caméra ne filme que si quelqu'un regarde) ;
     `robot.launch.py`. Le banc d'essai sert de référence, puis il est supprimé.
   - À savoir : sous Windows, `argos-pi.local` met 11 s à répondre ; ouvrir la page
     par l'adresse IP (http://192.168.2.78:8080). Pour la phase 3 : aligner le
     lisseur de vitesse de Nav2 (2,5 m/s², réglage TurtleBot) sur la rampe du robot.

## Phase 2 : essais par composant (dossier `essais/`)

| Essai | Reste à faire | Réussi quand |
|---|---|---|
| Tourelle (`essais/tourelle/`) | ✔ Suivi validé sur le Pi et sur batterie (essai 4) ; fixée sur le châssis, objet vert, image en direct (`--flux`) le 2026-11-14 | Suivi validé à la main, mesures dans le README ✔ |
| Pi 5 (`essais/raspberry/`) | ✔ Ubuntu Server 24.04, SSH par clé, ROS 2 Jazzy, échange avec le PC ; ✔ répartiteur et page web compilés et démarrés sur le Pi (2026-11-20, Nav2 partiel sans RViz) | `ros2 topic list` du Pi vu depuis le PC ✔ |
| LiDAR C1 (`essais/lidar/`) | ✔ Seul sur table (2026-11-28) : pilote officiel `sllidar_ros2` (celui d'apt ne gère pas le C1), 10 tours/s, 1 m vu à 2,6 cm près, flèche = 180° ; reste : fixation sur le robot, essai de cartographie, alimentation avec le reste | `/scan` affiché dans RViz sur le PC ✔ |
| Moteurs et encodeurs (`essais/moteurs/`) | Roues en l'air ✔ (protocole, vitesse à 3 %, nœud `pont_moteurs`, entraxe, rodage) ; au sol : marche avant ✔, rotation sur place ✔ (problème 7 : les roues partaient 0,25 à 0,9 s après l'ordre, essais trop courts ; régulateur de la carte réglé, départ en 0,1 s ; tour complet juste à 2 à 3 %) ; rampe, roue libre et freinage ✔ (2026-11-20, `essais/appli/`) | Robot sur cales : `cmd_vel` obéi ✔, `/odom` juste à 5 % sur 1 m (au sol) |
| Tête et corps (`essais/suivi_corps/`) | Liaison roues ✔, compensation ✔ (objet à 14 px du centre en médiane, corps à 60 °/s), suivi en rotation ✔ (objet à 50-100 °/s : 34 px, tourelle jamais en butée), rejoindre et suivre à 40 cm ✔ (0,50 m/s en avant et en arrière), recherche de l'objet perdu ✔ premier essai (retrouvé en 11,5 s) ; reste : répéter la recherche 10 fois | Le robot cherche, suit et rejoint l'objet vert, de façon fluide |
| Alimentation (`essais/alimentation/`) | Pi sur batterie ✔, tourelle ✔, moteurs 20 min à fond ✔ (Pi jamais sous 5,016 V) ; reste : LiDAR, câblage définitif (fusible, interrupteur, Wago), arrêt d'urgence, autonomie | Pi + LiDAR + moteurs sans redémarrage du Pi ; l'arrêt d'urgence coupe les moteurs |

**Décision moteurs (prise le 2026-11-13).** La carte du châssis Yahboom,
**YB-ESF01 V2.0**, pilote les moteurs : STM32F103 avec son propre programme, quatre
pilotes de moteur AT8236, quatre entrées d'encodeur, liaison USB-C (CH340) vers le Pi,
alimentation 5 à 12 V (13 V au maximum ; batterie 12,6 V chargée : OK). Un nœud ROS 2
sur le Pi traduira `cmd_vel` en vitesses de roues et les encodeurs en `/odom`. L'ESP32
reste dédié à la tourelle. La carte d'extension MSPM0 du kit ne sert pas.
Raison : `cmd_vel` et `/odom` avec le moins de code à nous (pilotes et programme de
la carte déjà faits et réglés pour ces moteurs). À vérifier dans `essais/moteurs/` :
le protocole exact (non trouvé dans la doc officielle), la précision de l'odométrie,
le temps de réaction. `MATERIEL.md` (estimation de départ, ESP32 + linorobot2) n'est
pas modifié.
Leçon de l'essai tourelle, toujours valable : aucune fonction bloquante dans la
boucle d'un microcontrôleur, et ne pas inonder la carte d'ordres.

## Phase 3 : robot réel

1. Assemblage, description du robot (URDF : roues, LiDAR, dimensions réelles),
   rayon du robot dans `nav2_params.yaml` (aujourd'hui celui du TurtleBot3).
2. Pilotes : base roulante, `rplidar_ros`, IMU.
3. Refaire les étapes 1 à 6 sur le vrai robot, dans l'ordre : clavier, SLAM, Nav2,
   répartiteur, page web. Mesures sur le Pi : `ros2 topic hz /cmd_vel_nav`
   (10 Hz tenus), `htop`, `vcgencmd measure_temp`.
4. Réussi = les livrables du `README.md`, dans une vraie pièce.

## Phase 4 : tête (tourelle et Suivi)

1. Nœud « regard » sur le Pi : reprend `essais/tourelle/suivi_fluide.py` (vision,
   direction de la cible). La tourelle garde son croquis `suivi_fluide.ino` et son
   protocole texte, qui marche : micro-ROS seulement s'il apporte quelque chose.
2. Rôle Suivi (`roles/suivi.py`) : détails dans « Pistes notées ».
3. Image à la demande sur la page (une photo, pas forcément un flux).

## Phase 5 : supervision

Le robot reste autonome ; le superviseur est un utilisateur de plus.

1. Accès : SSH vers le Pi, page protégée.
2. Tâches de Claude, par séances : surveiller une patrouille (journal, une photo de
   temps en temps), identifier un objet inconnu, régler et mesurer, développer.
3. Ce qui revient souvent devient un rôle (ex. Observation) et n'a plus
   besoin de lui.

Idées pour après (cas d'usage, interprète de texte, évolution de l'architecture) :
`PISTES.md`. Rien n'y est décidé.

Limite connue : il décide en secondes, pas en millisecondes. Jamais de pilotage
direct des moteurs par le superviseur : il donne des objectifs, Nav2 conduit.

# Étapes réalisées

Chaque terminal commence par :
```
source /opt/ros/jazzy/setup.bash
```

## Installation (une seule fois)

Voir `INSTALLATION.md` : PC neuf ou réinstallé (Ubuntu, ROS 2, Gazebo, projet).

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
- `argos_mission/aller_a.py` (OBSOLÈTE depuis le répartiteur) : lit le YAML, envoie le goal à Nav2 via
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
- `argos_mission/repartiteur.py` : états VEILLE / APPEL / PATROUILLE (+ ECHEC).
  Ne recalcule le chemin que s'il devient bloqué. Depuis le 2026-11-13, la logique
  est rangée par rôle (`roles/service.py`), voir `ARCHITECTURE.md`.
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
- **Exploration automatique** : le robot cartographie seul les zones inconnues
  (paquets du type `m-explore`, compatibilité Jazzy non vérifiée). Reportée : ajouter
  des fonctions seulement quand le robot réel marche, pour garder le débogage simple.
- **Démarrage automatique de la navigation** au premier placement du robot
  (navigation lancée avec `autostart: false`, puis STARTUP par le répartiteur) : plus
  de délai de 5 min ni de bouton « Redémarrer la navigation » à presser dans ce cas.
- **Vision : webcam USB** (~30-50 $, pilote ROS 2 `v4l2_camera`) + modèle d'IA déjà
  entraîné sur COCO (~80 objets : pomme, personne, tasse...). Quelques images/s sur le
  processeur du Pi (non mesuré). Suivi d'objet lent possible : direction donnée par la
  caméra, distance par le LiDAR. Plus rapide plus tard : accélérateur (Hailo, OAK-D).
  Optimisations retenues (à mesurer une par une sur le Pi, avec `htop`) : modèle
  « nano », image réduite (ex. 320 × 320), quantification INT8 + moteur ARM (NCNN ou
  TFLite), détection espacée + traceur OpenCV entre deux, suivi de couleur si suffisant.
  Nœud de vision démarré seulement quand on en a besoin (futur mode SUIVI).
  Périmètre décidé : reconnaître des objets, suivre un objet ou une personne (tourelle
  2 axes, servos pilotés par l'ESP32 ou l'Arduino, alimentés par un 5 V séparé), image
  en direct sur la page web (`web_video_server`, port 8080 par défaut : à changer, déjà
  pris par la page). Écarté : reconnaissance de visages. Plus tard : détection de chute.
  Mode SUIVI, précisé le 2026-11-10 après l'essai `essais/tourelle/` : la couleur à
  suivre se choisit dans une palette de carreaux sur la page, envoyée par son nom
  (commande « suivre rouge »). Chaque nom correspond à une plage HSV rangée dans un
  YAML, mesurée sur de vrais objets sous l'éclairage du bâtiment (la couleur à l'écran
  n'est pas celle que voit la caméra). Peu de couleurs bien séparées (ex. rouge, vert,
  bleu, jaune) ; éviter orange, rose, marron, proches de la peau. Déplacement par Nav2 (arbre « follow point » :
  but mis à jour en continu, arrêt à 1 m), jamais de vitesses envoyées directement aux
  moteurs. Tourelle : garder le croquis et le protocole texte de l'essai 4
  (`suivi_fluide.ino`), qui acceptent 500 ordres/s. L'ancienne limite de 25 ordres/s
  venait de la bibliothèque ESP32Servo, pas de l'ESP32 (voir `essais/tourelle/README.md`). Vérifier qu'il
  reste 2 broches libres pour les servos sur la carte du robot. Ordre : couleur, puis
  personne (modèle COCO), visage seulement si la décision change.

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

## Lancement en une commande ✔

`src/argos_mission/launch/argos.launch.py` démarre tout : simulation, Nav2 réduit
(sans docking, routes, suivi de points, dans un seul processus), répartiteur, page web (RViz en option).
```
ros2 launch argos_mission argos.launch.py
```
Options : `carte:=arene` (fichiers `maps/<carte>.yaml` et `maps/<carte>_points.yaml`),
`rviz:=true` (RViz, outil de développement), `gazebo_gui:=false`.
Placer le robot depuis la page web (bouton **Placer le robot sur la carte** : appuyer
sur sa position, glisser vers où il regarde). Délai : 5 min (`initial_transform_timeout`, 60 s par défaut dans Nav2).

## Page web : placer le robot ✔, changer de carte ✔

- **Placer le robot sur la carte** : remplace « 2D Pose Estimate » de RViz.
- **Charger cette carte** : liste des `maps/<nom>.yaml`. Le robot s'arrête, la carte
  et ses points (`maps/<nom>_points.yaml`, créé vide s'il n'existe pas) sont chargés,
  puis il faut replacer le robot. Commande équivalente : `carte <nom>`.
  Au prochain lancement, c'est la carte `carte:=` (par défaut `arene`) qui est chargée.

## Cartographie depuis la page web

1. **Pilotage manuel** ✔ : carte « Pilotage manuel » de la page, maintenir
   ▲ ▼ ◀ ▶ (ou les flèches du clavier). Seulement en veille. Vitesses : 0,5 m/s et
   0,8 rad/s. Publie sur `/cmd_vel_nav`, donc passe par `collision_monitor` (le robot
   ralentit puis s'arrête devant un obstacle). Bouton relâché ou page figée : arrêt.
2. **Nouvelle carte** ✔ : bouton « Nouvelle carte » (commande `cartographier`).
   La localisation (map_server + AMCL) est arrêtée, SLAM Toolbox démarré, sans relancer.
   Promener le robot avec le pilotage manuel, la carte se dessine sur la page.
3. **Enregistrer** (commande `enregistrer <nom>`) : `maps/<nom>.yaml` + `.pgm`, un point
   `base` au point de départ, retour en navigation, robot replacé automatiquement.
   **Abandonner** (commande `abandonner`) : retour à la carte d'avant, replacer le robot.

Note : les cartes créées depuis la page n'existent que sur la machine du robot tant
qu'on ne les a pas envoyées sur GitHub (`git add maps && git commit && git push`).

## Messages du robot sur la page ✔

Le répartiteur publie ses messages sur `/argos/journal` (point enregistré, commande
refusée, échec de sauvegarde...). Ils s'affichent dans le **Journal** de la page,
préfixés « Robot : » (orange = avertissement, rouge = erreur).

## Redémarrer la navigation ✔

Si le robot n'a pas été placé dans les 5 min, Nav2 abandonne son démarrage et refuse
toute destination. Sans rien relancer : placer le robot, puis bouton **Redémarrer la
navigation** (Réglages), ou commande `redemarrer_navigation`. Le répartiteur configure
puis active un par un les serveurs de navigation restés arrêtés.

## Monde plus grand : entrepôt (à tester)

La carte `arene` (monde `tb3_sandbox`) est donc obsolète.
`tb3_sandbox` est trop petit pour une grande carte (la carte ne peut pas dépasser le
monde simulé). Monde retenu : `src/argos_mission/worlds/warehouse.sdf`, entrepôt
d'environ 27 x 45 m copié de turtlebot4_simulator (jazzy), avec les plugins LiDAR et
IMU ajoutés. Internet requis au premier lancement (modèles Gazebo Fuel).
Voir le monde seul (sans robot) : `gz sim src/argos_mission/worlds/warehouse.sdf`.
`argos.launch.py` lance maintenant ce monde (robot en (0, 0)) à la place de
`tb3_sandbox`. La carte `arene` ne correspond plus : en créer une nouvelle depuis la
page (« Nouvelle carte », puis « Enregistrer »), puis la lancer avec `carte:=<nom>`.
Testé 2026-11-06 : RTF 99-100 % une fois le pilote NVIDIA installé (INSTALLATION.md, 1b),
pilotage manuel à 0,5 m/s confirmé.
