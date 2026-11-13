# Architecture : fonctionnalités modulaires

Proposition du 2026-11-12, issue d'une discussion. Rien n'est codé ni testé.

But : ajouter une fonctionnalité (ex. Observation) sans toucher aux autres, et une
page web qui n'affiche que les réglages de la fonctionnalité active.

## Vocabulaire

- **Fonctionnalité** : un paquet complet, comportement du robot + panneau de la page.
  Une seule active à la fois. Ex. : Service, Cartographie, plus tard Observation.
- **État** : ce que fait la fonctionnalité en ce moment. Ex. : Service est en
  attente, en appel, en patrouille ou en échec.
- **Veille** : aucune fonctionnalité active, robot immobile. État au démarrage, et
  passage obligé entre deux fonctionnalités.
- **Outils communs** : ce qui sert à plusieurs fonctionnalités (aller à un point,
  points, cartes, journal). Écrits une fois, jamais recopiés.

## Vue d'ensemble

```
Page web
  zone commune : carte, placer le robot, cartes, points, pilotage, marge, journal
  sélecteur    : Veille | Service | Cartographie | ...
  panneau      : celui de la fonctionnalité active, et lui seul
      |                                   ^
      | /argos/commande                   | /argos/etat, /argos/journal, /argos/points
      v                                   |
Répartiteur (repartiteur.py)
  aiguillage : commande commune -> outils communs, sinon -> fonctionnalité active
  bascule    : sortir() de l'active, Veille, entrer() dans la nouvelle

  [ service ]   [ cartographie ]   [ observation (futur) ]

  outils communs : aller_vers, annuler, points, cartes, signaler
      |
      v
Nav2, SLAM Toolbox ; plus tard : nœud « regard » (tourelle), nœud « mémoire » (IA)
```

## Ce que devient l'existant

| Aujourd'hui | Demain |
|---|---|
| `veille` | Commande commune : sortir de la fonctionnalité active, robot immobile |
| `appel <point>`, `base`, `patrouille` | Fonctionnalité Service |
| État ECHEC (commun à tout le robot) | État de Service : `SERVICE ECHEC chambre_101` |
| `definir_patrouille` | Réglage de Service (toujours rangé dans `<carte>_points.yaml`) |
| `cartographier`, `enregistrer`, `abandonner` | Fonctionnalité Cartographie (`cartographie.py` existe déjà) |
| `ajouter`, `supprimer` | Outils communs (points) |
| `carte <nom>`, `redemarrer_navigation` | Outils communs |
| Placer le robot, pilotage manuel, marge | Inchangés : la page parle directement à ROS |

## Contrat d'une fonctionnalité

Un fichier Python par fonctionnalité, dans `argos_mission/fonctionnalites/` (nom à
décider). Chacun définit une classe, créée une fois par le répartiteur, avec les
mêmes méthodes :

| Méthode | Rôle |
|---|---|
| `entrer()` | Démarrer (ex. Cartographie lance SLAM). Renvoie False si impossible |
| `commande(mots)` | Traiter une commande pendant qu'elle est active |
| `fin_de_tache(resultat)` | Nav2 a terminé ce qu'elle avait lancé |
| `sortir()` | S'arrêter proprement (annuler Nav2, arrêter SLAM...). Toujours appelée avant de quitter |
| `etat()` | Texte de son état, ex. `APPEL chambre_101` |

Plus deux listes :
- `outils_bloques` : les outils communs refusés pendant qu'elle est active. Ex. :
  Cartographie bloque les points et le changement de carte, comme aujourd'hui.
- `reglages` : ses réglages modifiables depuis la page (voir Réglages).

Chaque classe reçoit le répartiteur à sa création et appelle ses outils communs
(`robot.aller_vers(...)`, `robot.signaler(...)`). Même principe que `cartographie.py`
aujourd'hui, dont les fonctions reçoivent `noeud`.

Règles :
1. Une fonctionnalité ne connaît pas les autres. Elle n'utilise que les outils communs.
2. Seul le répartiteur passe d'une fonctionnalité à l'autre, toujours par `sortir()`.
3. Les réceptions de messages notent, la boucle principale agit (règle actuelle,
   imposée par BasicNavigator).
4. Ce qui est lent ou lourd (vision, appel à une IA) tourne dans un nœud ROS séparé,
   démarré seulement quand une fonctionnalité en a besoin. La fonctionnalité lui parle
   par topics : elle dirige, elle ne calcule pas. Sinon la boucle principale se
   bloque et le robot ne répond plus.
5. Ajouter une fonctionnalité = un fichier Python + une ligne pour l'inscrire dans le
   répartiteur + un panneau dans la page. Rien d'autre ne change.

## Messages

`/argos/commande` :
- communes : `veille`, `activer <fonctionnalité>`, `carte <nom>`, `ajouter ...`,
  `supprimer <nom>`, `redemarrer_navigation`, `regler <nom> <valeur>`
- toute autre commande va à la fonctionnalité active (ex. Service : `appel <point>`,
  `patrouille`, `definir_patrouille ...`). Commande d'une fonctionnalité inactive :
  refusée, avec un message dans le journal.

`/argos/etat` : premier mot = la fonctionnalité, la suite = son état.
Ex. : `VEILLE`, `SERVICE ATTENTE`, `SERVICE APPEL chambre_101`, `CARTOGRAPHIE`.

`/argos/reglages` : valeurs actuelles des réglages (JSON, dernier message gardé,
comme `/argos/points`).

## Réglages

Ce qu'un utilisateur peut vouloir changer se règle sur la page, sans toucher au code.
Les paramètres techniques (MPPI, AMCL...) restent dans les YAML de `config/`.

- Chaque fonctionnalité déclare ses réglages : nom, valeur par défaut, valeurs
  permises. Ex. Service : `apres_mission` = `attente` (défaut) ou `veille`.
- C'est le répartiteur qui vérifie : valeur non permise = refusée, avec la raison
  dans le journal. La page peut griser une option, mais ne décide pas : demain, un
  autre client (ex. le nœud langage naturel) pourra envoyer des réglages.
- Rangés dans un YAML : ils survivent au redémarrage.
- Peu de réglages, ajoutés quand le besoin apparaît : chaque réglage multiplie les
  cas à tester.

Exception actuelle : la marge de sécurité. La page la vérifie seule (minimum 0,22 m)
et l'envoie directement à Nav2. Elle n'est pas sauvegardée : retour à 0,25 m au
redémarrage (valeur de `nav2_params.yaml`). À faire passer par le répartiteur le
moment venu.

## Page web

- Une seule page (`index.html`), comme aujourd'hui.
- Chaque panneau propre à une fonctionnalité est marqué `data-fonctionnalite="SERVICE"`.
  Une seule règle CSS cache tous ces panneaux sauf celui de la fonctionnalité active.
  Elle généralise le cas actuel de la cartographie (`body.cartographie .hors-carto`).
- Le sélecteur envoie `activer <nom>` ; l'écran ne change qu'une fois l'état confirmé
  par le robot sur `/argos/etat` (principe actuel, gardé).
- Plus tard, si la page devient trop longue (923 lignes aujourd'hui) : un fichier
  JavaScript par panneau.

## Exemple : brancher Observation plus tard

- `fonctionnalites/observation.py` : parcourt des points (outil `aller_vers`), demande
  au nœud « regard » de balayer à chaque arrêt, et au nœud « mémoire » d'enregistrer
  ce qui est vu avec la position du robot.
- Nœuds « regard » (tourelle) et « mémoire » (IA) : séparés (règle 4).
- Un panneau « Observation » dans la page.
- Service, Cartographie et le reste du répartiteur : inchangés.

Le futur mode SUIVI (voir `ETAPES.md`) se branche de la même façon.

## Deux chemins

**A. Réorganiser (recommandé)** : petites étapes, chacune testée en simulation avant
la suivante.
1. Sortir Service de `repartiteur.py` vers son propre fichier. Commandes et format de
   `/argos/etat` inchangés : la page ne bouge pas, tout doit marcher comme avant.
2. Même chose pour Cartographie.
3. Nouveau format de `/argos/etat`, commande `activer`, sélecteur et panneaux dans la page.
4. Seulement ensuite : une nouvelle fonctionnalité.

**B. Remplacer** : écrire un nouveau répartiteur à côté, basculer quand il fait tout
ce que fait l'ancien. Plus propre sur le papier, mais deux versions à maintenir, et le
risque de perdre des problèmes déjà résolus dans le code actuel (cycle de vie de
Nav2, délai de map_saver_cli, placement du robot après cartographie...).

Ce qui augmente la difficulté : les deux chemins touchent du code validé. D'où la
liste ci-dessous, à repasser en simulation après chaque étape.

## Liste de non-régression

- [ ] Appel vers un point, arrivée, retour au repos
- [ ] Retour à la base
- [ ] Patrouille : boucle, point inaccessible sauté, tous inaccessibles = échec
- [ ] Supprimer un point de la patrouille en cours
- [ ] Destination refusée (robot non placé) : échec + message dans le journal
- [ ] Changer de carte
- [ ] Cartographie : enregistrer, puis abandonner
- [ ] Redémarrer la navigation
- [ ] Ctrl+C pendant une mission : mission annulée

## Décisions à prendre

1. Noms : « fonctionnalité » (et sur la page : « mode » ?), Service, Observation,
   dossier `fonctionnalites/`, commandes `activer` et `regler`, topic `/argos/reglages`.
2. Valeur par défaut de `apres_mission` (Service) : rester en attente (proposé :
   prêt pour le prochain appel) ou repasser en Veille ? Et fichier des réglages : un
   seul pour le robot, ou un par carte comme les points ?
3. Retour à la base : commande commune, utilisable depuis n'importe quelle
   fonctionnalité (va à la base, puis Veille), ou seulement un appel dans Service ?
4. Chemin A ou B.
