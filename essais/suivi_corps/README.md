# Essais : tête et corps ensemble

But : que la tourelle (tête) et les deux roues (corps) forment un seul système qui
cherche, suit et rejoint un objet vert, de façon fluide. Essai hors du projet ROS,
comme `essais/tourelle/` : il servira de base au nœud « regard » et à la
fonctionnalité Suivi quand le LiDAR sera là.

## Principe

- La tête est rapide (30 corrections par seconde, recentrage en 0,3 s), le corps est
  lent (0,2 à 0,3 s pour changer de vitesse) : la tête vise, le corps suit la tête.
- La direction de l'objet est estimée par rapport au sol : angle de la tourelle +
  effet des mouvements du corps, mesuré par les encodeurs 175 fois par seconde. Dès
  que le corps pivote, la tourelle contre-tourne, sans attendre l'image.
- Le corps est commandé en continu, avec une accélération limitée : pas de seuils
  « je pars, je m'arrête ».

## Étapes

| Étape | Contenu | État |
|---|---|---|
| 1 | `base.py` : ordres lissés aux roues, lacet daté par les encodeurs | ✔ |
| 2 | Compensation : la tête garde l'objet au centre quand le corps pivote | ✔ |
| 3 | Alignement : le corps suit la tête (rotation sur place) | ✔ |
| 4 | Distance : le robot avance et s'arrête à 40 cm de l'objet (`--essai rejoindre`) | ✔ ; vitesses essayées en ligne droite, à confirmer objet en mouvement |
| 5 | Recherche : tête seule, puis tour du corps, puis arrêt « perdu » | ✔ premier essai (une recherche réussie, à répéter) |

## Fichiers

Sur le Pi, garder la même disposition que le dépôt (`essais/tourelle/` à côté de
`essais/suivi_corps/`) : les scripts importent `suivi_fluide.py`. Arrêter
`pont_moteurs` avant (un seul programme à la fois sur la carte moteurs).

- `base.py` : liaison avec les roues. `python3 base.py` : aller-retour en rotation,
  consigne contre mesure.
- `mesure_retard.py` : retard entre la caméra et les encodeurs.
- `etalonnage.py` : distance de l'objet d'après sa taille dans l'image (le robot
  recule de 25 cm et se sert de ses encodeurs comme mètre ruban).
- `etalonnage_gain.py` : de combien la tête doit tourner quand le corps tourne,
  selon la distance de l'objet.
- `tete_corps.py` : le programme principal. `--essai pivot --comparer` : le corps
  va et vient, la tête compense, puis le même essai sans compensation.
  `--essai suivi --duree 60` : le corps pivote pour suivre l'objet.
  `--journal fichier.csv` : une ligne par image ; `--flux` : image en direct (8081).
- `analyse.py` : lit un journal de pivot et sépare erreur de gain et erreur de retard.
- `bilan_suivi.py` : bilan d'un journal de suivi (écart selon la vitesse de l'objet,
  agitation du corps, butées de la tourelle).
- `rejouer_estimateur.py`, `simuler_corps.py` : rejouent un journal sur le PC pour
  comparer des réglages sans refaire l'essai (problèmes 4 et 5).

## Mesures (2026-11-14, objet vert à 30 cm, carrelage)

- `base.py`, rotation à ±86 °/s : les roues suivent après 0,25 s, dépassement au
  départ (123 °/s pendant 0,1 s), puis ±10 °/s autour de la consigne ; arrêt et
  inversion en 0,15 s. 175 relevés de lacet par seconde.
- Retard caméra par rapport aux encodeurs : **0,03 s** (4 mesures, 0,025 à 0,04).
  Pour la tourelle c'est 0,08 s : l'ESP32 annonce une position que le servo n'a pas
  encore atteinte. Le lacet prédit le glissement du décor à 0,8 px près par image.
- Distance = **11,0 / rayon** (m, px), droite ajustée à 1,4 cm près sur 198 relevés.
- Un degré de rotation du corps fait glisser l'objet de **8,18 + 0,067 x rayon**
  pixels (10,5 px à 31 cm, 9,5 px à 54 cm) : la caméra est en avant de l'axe des
  roues, elle se déplace de côté quand le corps pivote.
- Compensation, corps en va-et-vient (inversion chaque seconde), objet immobile :

  | Rotation du corps | Écart médian | 9 fois sur 10 sous | Sans compensation (médian / 9 sur 10) |
  |---|---|---|---|
  | 30 °/s | 5 px | 23 px | |
  | 60 °/s | 14 px | 35 px | 46 px / 122 px |
  | 90 °/s | 40 px | 68 px | |

  Objet vu sur 239 images sur 239 dans tous les cas. 100 px = 13° environ.

- Alignement, objet posé à 24° sur le côté (bord de l'image à 35 cm) : corps aligné
  à 5° près en 0,4 s, aucune inversion, puis robot immobile, objet à 0 px du centre.
- Suivi en rotation, objet déplacé à la main autour du robot (40 à 50 cm, 30 à
  40 °/s en médiane, pointes à 100 °/s et plus, changements de sens fréquents) :

  | | Essai 1 | Essai 2 | Essai 3 (objet au bout d'un fil) |
  |---|---|---|---|
  | Changement | départ | nouvel estimateur de vitesse | nouvelle loi du corps, images douteuses ignorées |
  | Écart médian, objet à 20-50 °/s | 49 px | 30 px | 24 px |
  | Écart médian, objet à 50-100 °/s | 97 px | 39 px | 34 px |
  | Retard équivalent de la tête | 253 ms | 90 ms | 64 ms |
  | Tourelle en butée | 4,5 % | 4,8 % | 0 % |
  | Inversions du corps par 10 s | 6,7 | 12,3 | 6,8 |
  | Agitation de l'ordre au corps | 144 °/s² | 295 °/s² | 187 °/s² |

  Objet jamais perdu plus de 0,2 s dans les trois essais, une fois vu.
- Rejoindre et suivre (`--essai rejoindre`, 40 s, objet promené au bout d'un fil) :
  objet vu sur 1191 images sur 1213, plus longue perte 0,07 s ; écart au centre de
  l'image 8 px en médiane (9 fois sur 10 sous 37 px) ; 4,4 m roulés dont un
  demi-tour de 200° ; 6 inversions avant/arrière. Limite : à 0,25 m/s au plus, le
  robot se laisse distancer quand l'objet s'éloigne aussi vite (objet à 0,97 m
  pour 0,40 m visés), et à 0,15 m/s en arrière il recule trop lentement (objet
  venu à 0,16 m).

- Vitesses (`--essai vitesses --couples ...`), objet immobile, le robot recule seul
  jusqu'à 1,0 m puis revient à 0,40 m ; dépassement mesuré par les encodeurs :

  | Avant / arrière demandés | Vitesse atteinte | Dépassement | Arrêt à | Écart image (médian / 9 sur 10) | Images perdues |
  |---|---|---|---|---|---|
  | 0,25 / 0,15 m/s | 0,29 / 0,18 | 0 cm / 3 cm | 0,42 m | 3 px / 6 px | 0 |
  | 0,35 / 0,25 m/s | 0,39 / 0,28 | 0 cm / 3 cm | 0,42 m | 3 px / 6 px | 0 |
  | 0,50 / 0,35 m/s | 0,52 / 0,38 | 0 cm / 0 cm | 0,42 m | 3 px / 9 px | 0 |
  | 0,70 / 0,50 m/s | 0,54 / 0,50 | 0 cm / 2 cm | 0,42 m | 4 px / 10 px | 0 |

  Rien ne se dégrade en ligne droite. Sur 60 cm de trajet, le robot n'a pas le
  temps de dépasser 0,54 m/s (accélération 0,6 m/s², freinage 0,4 m/s²) : 0,70 m/s
  n'est donc pas validé. Couple retenu pour la suite : 0,50 / 0,35 m/s, à confirmer
  avec un objet en mouvement.

- Essai complet (`--essai rejoindre --duree 60`, 0,50 m/s, recherche active, objet
  promené, approché, caché, 2026-11-15) : 5,8 m roulés ; objet à 9 px du centre en
  médiane tant qu'il est vu ; recul jusqu'à 0,49 m/s quand l'objet approche (objet
  à moins de 30 cm pendant 8,3 s, au plus près 12 cm) ; première perte : retrouvé
  en 11,5 s (6 s de tête, puis 4 s de tour du corps) ; seconde perte : 6 s de tête,
  un tour complet en 9,4 s, puis arrêt « perdu ».

## Reste à faire

1. Vitesses : 0,50 m/s en avant et en arrière sont dans `VITESSE_SUIVI` et
   `VITESSE_RECUL` (fait).
2. Recherche : répéter 10 fois « objet caché puis posé derrière le robot »
   (critère : retrouvé 9 fois sur 10 en moins de 15 s ; une seule mesure pour
   l'instant, 11,5 s). Vérifier la reprise après les 30 s de pause.
   Limite connue : le robot suit la plus grande tache verte compacte ; un autre
   objet vert dans la pièce peut être pris pour la cible (vu dans une chambre :
   un objet vert dans un placard). Pendant une recherche, il faut 3 images de
   suite pour dire « retrouvé ».
3. Revérifier l'alignement avec un objet à 40° sur le côté (critère : ±5° en moins
   de 3 s), et l'essai de pivot après les changements d'estimateur.
4. Plus tard, avec le LiDAR : passage en nœuds ROS 2 (« regard », « suivi »),
   aiguillage de `cmd_vel` devant `collision_monitor`, fonctionnalité Suivi.

## Problèmes rencontrés

1. **La compensation était 10 % trop forte.**
   - Symptôme : écart médian de 27 px à 60 °/s, quel que soit le réglage de l'avance.
   - Diagnostic : journal image par image, puis `analyse.py` : par rapport au sol,
     un objet immobile ne doit pas bouger. Ce qui reste se sépare en erreur de
     gain (proportionnelle à la rotation) et erreur de retard (proportionnelle à
     la vitesse). Résultat : retard -3 ms, gain +10 %.
   - Cause : le premier étalonnage se faisait tourelle immobile, l'objet
     traversant l'image jusqu'aux bords, où l'objectif déforme. En suivi, l'objet
     reste au centre.
   - Solution : `etalonnage_gain.py`, qui mesure le gain pendant que la tête suit.
   - Leçon : étalonner dans les conditions d'utilisation.
2. **La tourelle arrivait en butée pendant l'essai.**
   - Symptôme : médiane correcte (21 px) mais 9 fois sur 10 sous 107 px seulement.
   - Diagnostic : dans le journal, la cible de la tourelle était à sa limite (150°)
     sur 33 % des images ; hors butée : 8 px de médiane.
   - Cause : le va-et-vient imposé au corps partait d'un seul côté (0 à +36° au
     lieu de ±18°) : erreur de phase dans le script d'essai.
   - Solution : va-et-vient centré. Sur le robot final, c'est le corps qui ramène
     la tourelle vers son centre (étape 3).
3. **Objet non détecté sur les premières images.** La caméra règle sa luminosité
   pendant environ une demi-seconde après son ouverture : attendre 1 s avant de mesurer.

4. **La tête traînait derrière un objet rapide.**
   - Symptôme (essai 1) : 97 px d'écart quand l'objet va de 50 à 100 °/s, soit
     l'équivalent de 0,25 s de retard. L'utilisateur, lui, trouvait le suivi correct :
     c'est le journal qui a montré le défaut.
   - Cause : l'estimateur repris de `suivi_fluide.py` remet la vitesse de l'objet à
     zéro dès qu'il bouge de plus de 2° entre deux images. Réglé pour un objet
     lent (31 °/s au banc), il ne prédit plus rien quand l'objet accélère.
   - Méthode : `rejouer_estimateur.py` rejoue l'enregistrement et compare des
     variantes, sans redemander l'essai. Erreur de prédiction : 49 px avec
     l'ancien, 58 px sans aucune vitesse, 26 px avec la règle retenue.
   - Solution : confiance dans la mesure qui grandit avec la surprise (prudent
     quand l'objet est calme, réactif quand il accélère).
5. **Le corps tressautait** (essai 2).
   - Symptôme : 12 inversions de sens par 10 s, même objet presque immobile.
   - Causes : la main cachait une partie de l'objet (rayon vu de 25, puis 8, puis
     16 px, centre décalé), et le corps recevait en plus la vitesse estimée de
     l'objet, bruitée par ces images.
   - Méthode : `simuler_corps.py`, un modèle simple du corps rejoué sur les
     enregistrements. Il retrouve les essais réels (4,4 % de butée contre 4,8 %
     mesurés, 70 inversions contre 68), donc on peut lui faire confiance pour
     comparer 14 lois de commande.
   - Solution : images douteuses ignorées (objet vu à moins de 60 % de sa taille
     habituelle) ; le corps réagit à l'angle de la tourelle et à sa variation,
     plus à la vitesse estimée de l'objet. Prévu par la simulation : butée 0,1 %,
     agitation réduite d'un tiers. Mesuré à l'essai 3 : butée 0 %, agitation -37 %.
   - Leçon : enregistrer chaque essai. Un enregistrement rejoué remplace dix
     essais, et la personne qui déplace l'objet n'a pas à recommencer.

6. **Le robot ne reculait pas assez devant un objet qui fonce sur lui** (essai à
   0,50 / 0,35 m/s, 8,1 m roulés, objet promené à la main).
   - Symptôme : objet venu à 7 cm de la caméra, puis perdu à 35 s (sorti de l'image
     par le côté, tourelle en butée à 45°).
   - Diagnostic (journal) : deux moments différents. À 6 s, le robot reculait déjà
     à sa vitesse maximale (0,33 m/s) : l'objet allait plus vite que lui. À 35 s,
     l'objet était à 11 cm ET sur le côté : la règle « objet de côté, on pivote
     avant d'avancer » freinait aussi le recul, le robot restait sur place, et à
     cette distance le moindre déplacement de l'objet fait 100 px par image.
   - Solution : le recul n'est plus freiné quand l'objet est de côté ; gain de
     recul doublé (2,5) ; vitesse de recul portée à 0,50 m/s (essayée en ligne
     droite : 2 cm de dépassement). NON ENCORE REVÉRIFIÉ avec l'objet en mouvement.
   - Reste vrai : à moins de 15 cm, un objet rapide sortira toujours du champ.
     C'est le rôle de la recherche (étape 5).

## À retenir pour le robot

- Carte moteurs : régulateur réglé sur `$MPID:0.8,0.6,0.5#` (voir
  `essais/moteurs/README.md`, problème 7). Sans ce réglage, rien de ceci n'est fluide.
- Les constantes (distance, gain) valent pour cet objet vert et cette position de
  la tourelle : à refaire si l'un des deux change.
