# Robot de service autonome

Un robot qui se déplace seul dans un bâtiment de type hôpital.
Quand on appelle depuis une chambre, il s'y rend sans aide.

Construit avec ROS 2 Jazzy et Nav2. La première phase s'est faite entièrement
en simulation (Gazebo). Le matériel est en cours de réception et d'essai.

Où lire quoi :
- `ETAPES.md` : la feuille de route, puis les commandes de chaque étape réalisée
- `ARCHITECTURE.md` : comment le code est organisé, comment ajouter une fonctionnalité
- `INSTALLATION.md` : installer le projet sur un PC neuf
- `MATERIEL.md` (estimation de départ) et `Spec_Materiel/Cout.md` (achats réels)
- `essais/` : chaque composant essayé seul, avec ses mesures et ses problèmes résolus
  (tourelle, Raspberry Pi, alimentation, moteurs)

Paquets ROS 2 (`src/`) : `argos_mission` (missions, lancement), `argos_web` (page
web), `argos_base` (pilotes du robot réel).

## Ce que fait le robot

Il a trois modes. Un seul est actif à la fois :

- **APPEL** : il va au point demandé et attend sur place.
- **PATROUILLE** : il fait une boucle entre des points définis.
- **VEILLE** : il ne bouge pas.

Les points (ex. `chambre_101`) sont définis dans un fichier YAML.

## Livrables de fin de projet

1. **Un robot réel** : 2 roues motrices, LiDAR 2D, IMU, Raspberry Pi 5 (navigation, page web)
   et un microcontrôleur pour les moteurs.
2. **Une carte** du bâtiment, faite par le robot lui-même.
3. **La navigation autonome** vers n'importe quel point nommé.
4. **Les trois modes** APPEL, PATROUILLE et VEILLE.
5. **Une application web** pour appeler le robot et changer de mode.
6. **Un paquet ROS 2 installable** sur une autre machine avec `rosdep`.

La phase 0 (simulation) a livré les points 2 à 6, sans matériel.

Ensuite (voir la feuille de route) : une tête mobile avec caméra pour suivre un
objet, et un superviseur (une personne ou une IA) qui peut donner des objectifs au
robot sans que celui-ci en dépende.
