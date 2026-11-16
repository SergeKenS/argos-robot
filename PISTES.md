# Pistes pour la suite

Idées discutées le 2026-11-15. **Rien n'est décidé ni codé.** Le plan en vigueur
reste la feuille de route de `ETAPES.md` et l'architecture de `ARCHITECTURE.md`.

Règle : rien de ce fichier ne commence avant que l'appel et la patrouille marchent
sur le robot réel (phase 3).

## Cas d'usage

Ce qui distingue Argos des autres robots Nav2, c'est sa tête. Ces cas d'usage
s'en servent. Du plus simple au plus ambitieux :

| # | Cas d'usage | Ce que fait le robot | S'appuie sur |
|---|---|---|---|
| 1 | « Suis-moi », puis « retourne à ta base » | Suit une personne, puis revient seul | Suivi (`essais/suivi_corps/`) + Nav2 |
| 2 | Ronde qui regarde | À chaque point de patrouille, la tête balaie la pièce ; la page signale ce qui a changé (porte ouverte, objet au sol, personne présente) | Patrouille + tête |
| 3 | « Trouve l'objet » | Visite les pièces une à une, cherche avec la tête, revient dire où il est | Recherche de l'objet perdu, étendue au bâtiment |
| 4 | Regard qui anticipe | La tête regarde vers où le robot va tourner, ou vers la personne qui lui parle | Tête seule ; coût faible, fort effet en démonstration |
| 5 | Objectifs écrits en phrases | « Va voir si la chambre 102 est libre » devient des étapes : aller, regarder, répondre | Interprète de texte (plus bas) |

Ordre conseillé : 1 et 2 d'abord (ils réutilisent presque tout l'existant), 4 comme
finition, puis 3 et 5.

Les cas 1 et 2 combinent deux comportements. C'est la raison de l'évolution
« organes, compétences, rôles » décrite à la fin de `ARCHITECTURE.md`.

## Interprète de texte

On écrit une phrase, le robot la traduit en commandes qui existent déjà.

```
texte tapé dans la page ─> interprète ─> commandes existantes ─> répartiteur ─> robot
                               ^
                               └─ état actuel du robot + commandes permises
```

- **Un champ de texte sur la page**, à côté des boutons. Pas de voix.
- **L'interprète** est un nœud à part. Il reçoit la phrase, l'état du robot et la
  liste des commandes permises à cet instant. Il répond par une ou plusieurs
  commandes existantes (`appel chambre_101`, `patrouille`, `veille`...).
- **Ce n'est pas un état du robot.** C'est un utilisateur de plus, comme une
  personne devant les boutons. Il ne peut rien faire de plus qu'un bouton.
- **Le répartiteur ne change pas** : il refuse déjà ce que l'état courant n'autorise
  pas.
- **La page affiche ce qui a été compris** (« Compris : appel chambre_101 »), et le
  refus s'il y en a un.
- **Une phrase qui demande plusieurs choses** : les commandes sont envoyées l'une
  après l'autre, chacune attend la fin de la précédente.
- **Tests sans robot** : phrase donnée, commande attendue.

Deux choix à faire le moment venu :

1. **Où tourne le modèle d'IA.** En ligne : comprend très bien, peu de code, mais
   demande Internet et se paie à l'usage. Sur le Pi : marche sans réseau, mais un
   petit modèle comprend moins bien et charge le processeur (à mesurer).
2. **Confirmer ou non** avant un déplacement. Conseil : confirmer au début.

Ce qui augmente la difficulté : un modèle d'IA peut mal comprendre. D'où la liste
fermée de commandes, l'affichage de ce qui est compris et la confirmation.
