# Architecture

Réorganisation du 2026-11-13. Le code est écrit et couvert par 36 tests
automatiques sans ROS. **Pas encore validé en simulation** : voir la liste de
non-régression à la fin.

But : ajouter une fonctionnalité (Suivi, Observation...) sans toucher aux autres.

## Trois niveaux

Chaque niveau fonctionne sans celui du dessus.

```
3. Superviseur (une personne sur la page web, ou Claude par séances)
      donne des objectifs, regarde le journal et la caméra
            |  /argos/commande              ^  /argos/etat, /argos/journal, /argos/points
            v                               |
2. Raspberry Pi 5 : autonomie
      Répartiteur -> fonctionnalité active -> Nav2, SLAM Toolbox
      plus tard : nœuds « regard » (tourelle, vision) et « mémoire »
            |  cmd_vel, cibles de la tourelle   ^  odométrie, positions
            v                                   |
1. Microcontrôleur : temps réel
      moteurs, encodeurs, servos (boucles à 50 Hz ou plus, jamais bloquantes)
```

Le robot ne dépend pas du superviseur : sans page ouverte ni séance, il finit
sa mission et reste en veille.

## Vocabulaire

- **Fonctionnalité** : un comportement complet du robot. Une seule active à la fois.
  Aujourd'hui : Service (appel, patrouille) et Cartographie.
- **Veille** : aucune fonctionnalité active, robot immobile. État au démarrage.
- **Outils du robot** : ce que le répartiteur offre aux fonctionnalités (aller à un
  point, annuler, signaler, cartes et points, SLAM). Écrits une fois.

## Fichiers (`src/argos_mission/argos_mission/`)

| Fichier | Rôle | Dépend de ROS |
|---|---|---|
| `repartiteur.py` | Nœud ROS : reçoit les commandes, les aiguille, publie l'état. Fournit les outils du robot | oui |
| `fonctionnalites/__init__.py` | Le modèle commun `Fonctionnalite` et les règles | non |
| `fonctionnalites/service.py` | Appel, base, patrouille, échec | non |
| `fonctionnalites/cartographie.py` | Nouvelle carte : enregistrer ou abandonner | non |
| `cartes.py` | Cartes, points nommés, patrouille, fichiers YAML | non |
| `cycle_de_vie.py` | Démarrer et arrêter Nav2 et SLAM Toolbox (ancien `cartographie.py`) | oui |

Ce qui ne dépend pas de ROS se teste sur n'importe quel PC, en une seconde :
```
python3 -m pytest src/argos_mission/test/test_fonctionnalites.py src/argos_mission/test/test_repartiteur.py
```
(`test_repartiteur.py` exécute le vrai `repartiteur.py` avec un faux ROS.)

## Contrat d'une fonctionnalité

Une classe qui hérite de `Fonctionnalite` :

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
1. Une fonctionnalité ne connaît pas les autres et n'importe pas ROS. Elle
   n'utilise que les outils du robot.
2. Seul le répartiteur passe de l'une à l'autre, toujours par `sortir()`.
3. Les réceptions de messages notent, la boucle principale agit (imposé par
   BasicNavigator).
4. Ce qui est lent ou lourd (vision, IA) tourne dans un nœud ROS séparé, démarré
   au besoin. La fonctionnalité dirige, elle ne calcule pas.
5. Ajouter une fonctionnalité = un fichier + une ligne dans `FONCTIONNALITES`
   (`repartiteur.py`) + ses tests. Plus un panneau dans la page si elle a des
   boutons.

## Aiguillage d'une commande

1. Une fonctionnalité exclusive est active et la commande n'est pas à elle : refus.
2. `veille` : quitter la fonctionnalité active.
3. Commande commune (`ajouter`, `supprimer`, `definir_patrouille`, `carte`,
   `redemarrer_navigation`) : outil du répartiteur.
4. Sinon, la fonctionnalité qui comprend la commande. Si elle n'est pas active :
   on quitte l'active et on entre dans celle-ci, à condition que la commande soit
   une de ses `entrees`.
5. Après chaque action : si la fonctionnalité active n'a plus d'état, retour en
   veille ; puis l'état est publié s'il a changé.

Les commandes et le format de `/argos/etat` sont **les mêmes qu'avant** : la page
web n'a pas été modifiée.

## Décisions revues (et pourquoi)

Par rapport à la proposition du 2026-11-12 et au code d'avant :

- **Pas de commande `activer <fonctionnalité>`.** Une commande démarre elle-même sa
  fonctionnalité (`appel alpha` démarre Service). Un seul message au lieu de deux,
  et rien ne change pour la page ni pour un autre client.
- **`/argos/etat` garde son format.** Le changer obligeait à modifier la page et le
  répartiteur d'un coup, sans pouvoir tester l'un sans l'autre.
- **Les fonctionnalités n'importent pas ROS.** Avant, toute la logique vivait dans
  un nœud de 487 lignes, testable seulement en simulation. C'était la dette la
  plus coûteuse : chaque changement demandait de relancer Gazebo.
- **`ECHEC` appartient à Service**, pas au robot entier. Il reste publié tel quel.
- **Les points et la patrouille restent des outils communs** (pas un réglage de
  Service) : la page les modifie en veille, et Suivi ou Observation en auront besoin.
- **`aller_a.py` n'est plus utilisé** par le répartiteur (voir Fichiers obsolètes).

Points faibles connus, non traités ici (voir la feuille de route dans `ETAPES.md`) :
- `argos.launch.py` lance toujours la simulation : pas encore de lancement pour le
  robot réel.
- La page et rosbridge n'ont aucune protection. La page parle directement à Nav2
  pour le pilotage manuel, le placement et la marge.
- La liste des serveurs de navigation existe en double (`argos.launch.py` et
  `cycle_de_vie.py`).
- Les réglages (ex. marge de sécurité) ne sont pas sauvegardés.

## Brancher la suite

- **Suivi** : `fonctionnalites/suivi.py` + un nœud « regard » (caméra, tourelle,
  repris de `essais/tourelle/suivi_fluide.py`). La fonctionnalité reçoit la
  direction de la cible par un topic et demande les trajets avec les outils du robot.
- **Observation** : parcourt des points, demande au nœud « regard » de balayer, au
  nœud « mémoire » de noter ce qui est vu.
- **Superviseur** : aucun code dans le répartiteur. Il lui faut seulement un
  accès (SSH), une image à la demande et une page protégée.
- **Sélecteur et panneaux par fonctionnalité dans la page** : à faire quand une
  troisième fonctionnalité arrive, pas avant. Principe retenu : chaque panneau
  marqué `data-fonctionnalite`, une règle CSS n'affiche que celui de l'active
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

Les tests automatiques couvrent déjà chaque ligne côté logique. À refaire en
simulation, parce qu'eux seuls montrent le vrai Nav2 et le vrai SLAM :

- [ ] Appel vers un point, arrivée, retour en veille
- [ ] Retour à la base
- [ ] Patrouille : boucle, point inaccessible sauté, tous inaccessibles = échec
- [ ] Supprimer un point de la patrouille en cours
- [ ] Destination refusée (robot non placé) : échec + message dans le journal
- [ ] Changer de carte
- [ ] Cartographie : enregistrer, puis abandonner
- [ ] Redémarrer la navigation
- [ ] Ctrl+C pendant un trajet : trajet annulé

En cas de problème : `git checkout main` revient au répartiteur d'avant.
