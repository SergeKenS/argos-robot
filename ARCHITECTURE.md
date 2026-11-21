# Architecture

Réorganisation du 2026-11-13 : 39 tests automatiques sans ROS, et validée en
simulation le même jour (liste de non-régression à la fin, 25 vérifications sur 25).

But : ajouter un rôle (Manuel, Suivi, Observation...) sans toucher aux autres.

## Trois niveaux

Chaque niveau fonctionne sans celui du dessus.

```
3. Superviseur (une personne sur la page web, ou Claude par séances)
      donne des objectifs, regarde le journal et la caméra
            |  /argos/commande              ^  /argos/etat, /argos/journal, /argos/points
            v                               |
2. Raspberry Pi 5 : autonomie
      Répartiteur -> rôle actif -> Nav2, SLAM Toolbox
      plus tard : nœuds « regard » (tourelle, vision) et « mémoire »
            |  cmd_vel, cibles de la tourelle   ^  odométrie, positions
            v                                   |
1. Microcontrôleur : temps réel
      moteurs, encodeurs, servos (boucles à 50 Hz ou plus, jamais bloquantes)
```

Le robot ne dépend pas du superviseur : sans page ouverte ni séance, il finit
sa mission et reste en veille.

## Vocabulaire

- **Rôle** : un mode du robot, un comportement complet. Un seul actif à la fois.
  Aujourd'hui : Service (appel, patrouille) et Cartographie. Appelé « fonctionnalité »
  jusqu'au 2026-11-20.
- **Veille** : aucun rôle actif, robot immobile. État au démarrage.
- **Outils du robot** : ce que le répartiteur offre aux rôles (aller à un
  point, annuler, signaler, cartes et points, SLAM). Écrits une fois.

## Paquets

| Paquet | Rôle | Où il tourne |
|---|---|---|
| `argos_mission` | Répartiteur et rôles, lancement (`argos.launch.py`), paramètres Nav2 | PC (simulation), Pi (robot réel) |
| `argos_web` | Page web (rosbridge + serveur HTTP) | PC, Pi |
| `argos_base` | Pilotes du robot réel : `pont_moteurs` (carte moteurs Yahboom : `cmd_vel` vers les roues, encodeurs vers `/odom` et `odom -> base_link`, chien de garde) | Pi seulement |

`argos_base` est le niveau 1 vu par ROS : il remplace, sur le robot réel, ce que
Gazebo fournit en simulation (`cmd_vel` et `/odom`). Même règle que pour le
répartiteur : la logique sans ROS (`carte_yahboom.py`, `cinematique.py`) est testée
seule (`python3 -m pytest src/argos_base/test/`). Mesures et réglages :
`essais/moteurs/README.md`.

## Arborescence cible (décidée le 2026-11-20)

✔ = existe, ★ = à créer pour le rôle Manuel, ○ = plus tard. Mise à jour le 2026-11-20. Chaque fichier est
créé à sa place, au moment où on l'écrit : pas de fichier vide en attente.

```
src/
├── argos_base/              pilotes du vrai robot : tout ce qui parle au matériel
│   ├── argos_base/
│   │   ├── pont_moteurs.py      ✔ nœud : roues (rampe, roue libre, freinage)
│   │   ├── carte_yahboom.py     ✔ protocole de la carte moteurs, sans ROS
│   │   ├── cinematique.py       ✔ calculs, sans ROS
│   │   ├── tourelle.py          ★ nœud : tête
│   │   ├── liaison_tourelle.py  ★ protocole de l'ESP32, sans ROS
│   │   └── camera.py            ★ nœud : image diffusée à la page
│   ├── config/   base.yaml ✔, tourelle.yaml ★
│   └── test/     tests sans ROS
│
├── argos_mission/           autonomie
│   ├── argos_mission/
│   │   ├── repartiteur.py       ✔ aiguillage
│   │   ├── organes.py           ★ qui tient les roues, qui tient la tête
│   │   ├── cartes.py, cycle_de_vie.py  ✔
│   │   ├── roles/               les modes du robot
│   │   │   ├── service.py, cartographie.py  ✔
│   │   │   ├── manuel.py        ★
│   │   │   └── suivi.py, observation.py     ○
│   │   └── competences/         ○ (avec le LiDAR)
│   ├── launch/   argos.launch.py ✔ (simulation), robot.launch.py ★ (vrai robot)
│   └── test/
│
└── argos_web/               l'application (téléphone, tablette, PC)
    └── web/                 servi tel quel, sans étape de compilation
        ├── index.html           ✔ squelette de la page
        ├── style.css            ✔ apparence (thème jeu vidéo, PC, téléphone)
        ├── app.js               ✔ connexion au robot, état, onglets, menu, journal
        ├── panneaux/            ✔ un fichier par rôle : service.js, cartographie.js, manuel.js
        ├── outils/              ✔ carte.js, croix.js (boutons et clavier), joystick.js
        └── polices/             ✔ copies locales (la page marche sans Internet)

firmware/tourelle/           ★ programme de l'ESP32 (repris de essais/tourelle/)
```

Les fichiers marqués « OBSOLÈTE » seront supprimés au nettoyage.

## Fichiers (`src/argos_mission/argos_mission/`)

| Fichier | Ce qu'il fait | Dépend de ROS |
|---|---|---|
| `repartiteur.py` | Nœud ROS : reçoit les commandes, les aiguille, publie l'état. Fournit les outils du robot | oui |
| `roles/__init__.py` | Le modèle commun `Role` et les règles | non |
| `roles/service.py` | Appel, base, patrouille, échec | non |
| `roles/cartographie.py` | Nouvelle carte : enregistrer ou abandonner | non |
| `cartes.py` | Cartes, points nommés, patrouille, fichiers YAML | non |
| `cycle_de_vie.py` | Démarrer et arrêter Nav2 et SLAM Toolbox (ancien `cartographie.py`) | oui |

Ce qui ne dépend pas de ROS se teste sur n'importe quel PC, en une seconde :
```
python3 -m pytest src/argos_mission/test/test_roles.py src/argos_mission/test/test_repartiteur.py
```
(`test_repartiteur.py` exécute le vrai `repartiteur.py` avec un faux ROS.)

## Contrat d'un rôle

Une classe qui hérite de `Role` :

| Élément | Rôle |
|---|---|
| `commandes` | Les commandes qu'elle comprend (premier mot) |
| `entrees` | Celles qui la démarrent quand elle n'est pas active |
| `exclusive` | Message de refus si elle bloque tout le reste tant qu'elle dure (Cartographie), sinon `None` |
| `entrer()` | Démarrer. `False` si impossible : le robot reste en veille |
| `commande(mots)` | Traiter une de ses commandes |
| `fin_de_tache(resultat)` | Nav2 a fini le trajet : `'reussi'`, `'echec'` ou `'annule'` |
| `points_modifies()` | Les points ou la patrouille ont changé |
| `sortir()` | S'arrêter proprement |
| `etat()` | Texte publié sur `/argos/etat`. `None` = elle a fini, retour en veille |

Règles :
1. Un rôle ne connaît pas les autres et n'importe pas ROS. Il
   n'utilise que les outils du robot.
2. Seul le répartiteur passe de l'une à l'autre, toujours par `sortir()`.
3. Les réceptions de messages notent, la boucle principale agit (imposé par
   BasicNavigator).
4. Ce qui est lent ou lourd (vision, IA) tourne dans un nœud ROS séparé, démarré
   au besoin. Le rôle dirige, il ne calcule pas.
5. Ajouter un rôle = un fichier + une ligne dans `ROLES`
   (`repartiteur.py`) + ses tests. Plus un panneau dans la page si elle a des
   boutons.

## Aiguillage d'une commande

1. Un rôle exclusif est actif et la commande n'est pas à lui : refus.
2. `veille` : quitter le rôle actif.
3. Commande commune (`ajouter`, `supprimer`, `definir_patrouille`, `carte`,
   `redemarrer_navigation`) : outil du répartiteur.
4. Sinon, le rôle qui comprend la commande. S'il n'est pas actif :
   on quitte l'active et on entre dans celle-ci, à condition que la commande soit
   une de ses `entrees`.
5. Après chaque action : si le rôle actif n'a plus d'état, retour en
   veille ; puis l'état est publié s'il a changé.

Les commandes et le format de `/argos/etat` sont **les mêmes qu'avant** : la page
web n'a pas été modifiée.

## Décisions revues (et pourquoi)

Par rapport à la proposition du 2026-11-12 et au code d'avant :

- **Pas de commande `activer <rôle>`.** Une commande démarre elle-même son
  rôle (`appel alpha` démarre Service). Un seul message au lieu de deux,
  et rien ne change pour la page ni pour un autre client.
- **`/argos/etat` garde son format.** Le changer obligeait à modifier la page et le
  répartiteur d'un coup, sans pouvoir tester l'un sans l'autre.
- **Les rôles n'importent pas ROS.** Avant, toute la logique vivait dans
  un nœud de 487 lignes, testable seulement en simulation. C'était la dette la
  plus coûteuse : chaque changement demandait de relancer Gazebo.
- **`ECHEC` appartient à Service**, pas au robot entier. Il reste publié tel quel.
- **Les points et la patrouille restent des outils communs** (pas un réglage de
  Service) : la page les modifie en veille, et Suivi ou Observation en auront besoin.
- **`aller_a.py` n'est plus utilisé** par le répartiteur (voir Fichiers obsolètes).

Points faibles connus, non traités ici (voir la feuille de route dans `ETAPES.md`) :
- `argos.launch.py` lance toujours la simulation : pas encore de lancement pour le
  robot réel (il devra démarrer `pont_moteurs` au lieu de Gazebo).
- La page et rosbridge n'ont aucune protection. La page parle directement à Nav2
  pour le pilotage manuel, le placement et la marge.
- La liste des serveurs de navigation existe en double (`argos.launch.py` et
  `cycle_de_vie.py`).
- Les réglages (ex. marge de sécurité) ne sont pas sauvegardés.

## Brancher la suite

- **Suivi** : `roles/suivi.py` + un nœud « regard » (caméra, tourelle,
  repris de `essais/tourelle/suivi_fluide.py`). Le rôle reçoit la
  direction de la cible par un topic et demande les trajets avec les outils du robot.
- **Observation** : parcourt des points, demande au nœud « regard » de balayer, au
  nœud « mémoire » de noter ce qui est vu.
- **Superviseur** : aucun code dans le répartiteur. Il lui faut seulement un
  accès (SSH), une image à la demande et une page protégée.
- **Sélecteur et panneaux par rôle dans la page** : à faire avec le troisième
  rôle (Manuel). Principe retenu : chaque panneau
  marqué `data-role`, une règle CSS n'affiche que celui de l'active
  (généralise `body.cartographie .hors-carto`).

## Fichiers obsolètes

Marqués « OBSOLÈTE » en tête de fichier. À supprimer lors du nettoyage, pas avant :

| Fichier | Remplacé par |
|---|---|
| `src/argos_mission/argos_mission/aller_a.py` (+ commande `aller_a`) | Le répartiteur (`appel <point>`) |
| `src/argos_mission/config/points.yaml` | `maps/<carte>_points.yaml` |
| `essais/tourelle/balayage/`, `suivi_simule/` | Essais 1 et 2, sans suite |
| `essais/tourelle/suivi_camera/`, `suivi_rouge.py` | `suivi_fluide/` et `suivi_fluide.py`, une fois l'essai 4 validé (ils servent encore de référence au banc de mesure) |
| `maps/arene.*` | Carte du petit monde `tb3_sandbox` : ne correspond plus à l'entrepôt simulé |

## Liste de non-régression (simulation)

Les tests automatiques couvrent la logique. La simulation, elle seule, montre le
vrai Nav2 et le vrai SLAM. Scénario automatique, sur simulation lancée avec la carte
`entrepot` et le robot non placé :
```
python3 essais/architecture/non_regression.py      # toute la liste ci-dessous
python3 essais/architecture/carto_repetee.py 5     # 5 cartographies de suite
```
Il sauvegarde les points de la carte avant et les remet après.

Validé le 2026-11-13 (simulation neuve, entrepôt) :
- [x] Appel vers un point, arrivée, retour en veille
- [x] Retour à la base
- [x] Patrouille : boucle, point inaccessible sauté, tous inaccessibles = échec
- [x] Supprimer un point de la patrouille en cours
- [x] Destination refusée (robot non placé) : échec + message dans le journal
- [x] Changer de carte
- [x] Cartographie : enregistrer, puis abandonner (6 fois de suite)
- [x] Redémarrer la navigation (seulement le cas « tout est déjà actif »)
- [ ] Ctrl+C pendant un trajet : trajet annulé (non couvert par le scénario)

En cas de problème : `git checkout main` revient au répartiteur d'avant.

## Problèmes rencontrés pendant la validation

Tous deux existaient déjà dans l'ancien répartiteur : la validation automatique,
répétée, les a fait sortir.

1. **Le robot restait sans carte après une cartographie.**
   - Symptôme : après « enregistrer », plus aucune réponse du répartiteur ; le
     journal répétait « change map service not available ».
   - Diagnostic : SLAM bien arrêté, mais aucune trace du redémarrage de la
     localisation. Appelé à la main, ce redémarrage marchait aussitôt.
   - Cause probable : la demande s'est perdue. `cycle_de_vie.py` créait un client
     de service neuf à chaque appel, et ROS 2 peut perdre la première demande d'un
     client tout juste créé. Panne intermittente (1 fois sur 3 passages).
   - Solution : clients créés au démarrage et gardés ; map_server vérifié avant de
     charger une carte, avec un message d'erreur au lieu d'une attente sans fin.
2. **Un appel trop tôt figeait le répartiteur.**
   - Symptôme : « 'NavigateToPose' action server not available, waiting... » en
     boucle, plus aucune commande traitée.
   - Cause : `goToPose` (BasicNavigator) attend le serveur de navigation sans limite.
   - Solution : le répartiteur vérifie sa présence (5 s au plus) et refuse sinon.
   - Révélé par un défaut de l'outil de test : la simulation précédente mal arrêtée
     a privé la nouvelle de Nav2. Leçon : vérifier que tout est arrêté avant de relancer.

Leçon commune : aucune attente sans limite dans le répartiteur. Toute attente d'un
autre nœud a un délai et un message d'erreur.

---

# Évolution prévue (PAS EN VIGUEUR)

Proposition du 2026-11-15, acceptée comme direction. **Rien de ce qui suit n'est
codé. Tout ce qui précède reste la référence** tant que cette partie n'a pas été
déplacée plus haut.

Quand : après la phase 3 (appel et patrouille validés sur le robot réel). Pas avant.

## Pourquoi

Aujourd'hui, un seul rôle est actif. Combiner deux comportements
(patrouiller en observant) oblige à en écrire une troisième. Défaut connu des
machines à états : elles deviennent fragiles en grossissant.

## Principe : organes, compétences, rôles

| Niveau | Définition | Exemples |
|---|---|---|
| **Organe** | Ce qui ne se partage pas. Un seul maître à la fois | roues, tête |
| **Compétence** | Brique réutilisable. Déclare les organes qu'elle prend et ce qu'il lui faut pour démarrer. Finit par « réussi » ou « échec, raison » | aller à un point (roues), balayer (tête), suivre une cible (tête + roues) |
| **Rôle** | L'état du robot, un seul actif, affiché par la page. Une recette de compétences écrite en données | Appel, Patrouille, Ronde d'observation, Suivi |

Deux façons de combiner, et seulement deux : **en même temps** (organes différents)
et **l'une après l'autre**. Une recette qui donne un organe à deux compétences en
même temps est refusée au chargement, par un test automatique.

Correspondance avec aujourd'hui : outils du robot → compétences ; répartiteur →
aiguillage, plus l'arbitrage des organes. Le pas 1 (organes) commence avec le
rôle Manuel (décision du 2026-11-20).

## Ce qui ne change pas

- Les trois niveaux, et le robot qui ne dépend jamais du superviseur.
- La sécurité (chien de garde, arrêt devant un obstacle) reste sous les
  compétences : toujours active, jamais désactivable par une recette.
- Sans ROS, testable sans robot : vrai aussi pour les compétences et les recettes.
- Les commandes et le format de `/argos/etat` : la page n'a pas à changer.

## Place du superviseur

Un utilisateur du répartiteur, comme la page. Deux usages :
- **Commander** : il choisit un rôle ou une compétence dans la liste ; le
  répartiteur refuse ce que l'état du robot n'autorise pas.
- **Observer** : il lit le journal et les mesures après coup, propose un réglage,
  qui passe par l'utilisateur avant d'aller sur le robot.

Ce qu'il apprend ne reste que si on l'écrit (paramètres, tests, documentation).

## Migration en trois pas, chacun testé

1. Les organes dans le répartiteur (qui tient les roues, qui tient la tête).
2. Les compétences extraites de Service ; « suivre » et « chercher » reprises de
   `essais/suivi_corps/`.
3. Les recettes ; Service devient les rôles Appel et Patrouille.

## Modèles et défauts connus

- **WPILib** (robots de compétition FRC) : le plus proche. Défauts : une
  composition garde tous ses organes pendant toute sa durée ; l'intérieur d'une
  composition est moins visible au débogage.
- **ros2_control** : un seul maître par interface matérielle.
- **Open-RMF** : tâches décrites en données. **SkiROS2** : compétences composées.
- **PX4** : modèle actuel (un mode à la fois, l'humain reprend toujours la main).
- Arbres de comportement : possibles plus tard si les recettes ne suffisent plus ;
  plus lourds à apprendre et à déboguer.
