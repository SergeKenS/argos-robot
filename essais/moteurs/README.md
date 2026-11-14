# Essais : moteurs et encodeurs

Essai hors du projet ROS : faire tourner les roues du châssis Yahboom depuis le
Raspberry Pi, et lire les encodeurs, avant d'écrire le nœud ROS 2.

## Matériel

- Châssis Yahboom 2 roues motrices, moteurs 520 à encodeur (12 V, réduction 1:40,
  300 tr/min), roues de 65 mm, 2 roulettes folles.
- Carte moteurs **Yahboom YB-ESF01 V2.0** (retenue le 2026-11-13, voir `ETAPES.md`) :
  - STM32F103 avec le programme de Yahboom
  - 4 pilotes de moteur AT8236 (Motor 1 à 4) ; par moteur : M+, M-, GND, 3V3, HA, HB
  - liaison avec le Pi : USB-C (puce CH340), ou port série (TX2/RX2) ou I2C
  - alimentation 5 à 12 V par bornier à vis, **jamais plus de 13 V**
- Carte d'extension MSPM0 du kit : non utilisée (carte d'apprentissage). Son
  interrupteur ON/OFF et ses borniers IN/OUT pourraient servir d'interrupteur général.

## Consignes de Yahboom (carte livrée)

1. Moteurs non Yahboom : respecter l'ordre des fils de la carte, sinon elle grille.
2. Serrer les vis de la carte avant de mettre sous tension (court-circuit sinon).
3. Poser la carte sur une surface non conductrice.
4. Pas de batterie sans carte de protection (type batterie de modélisme).
5. Entrée 5 à 12 V ; plus de 13 V interdit.

## Câblage (2026-11-13)

- Moteurs sur **Motor 3** et **Motor 4** (connecteurs 6 broches, avec encodeur).
  Motor 1 et Motor 2 libres.
- **Motor 3 = roue droite, Motor 4 = roue gauche** (montés en miroir). Vérifié en
  faisant tourner les moteurs : pour **avancer**, `$spd:0,0,+v,-v#`. Le compteur de
  la roue gauche est donc aussi à inverser. L'avant du robot est celui choisi par
  l'utilisateur le 2026-11-13 : le garder pour le LiDAR et la caméra.
- Alimentation : bornier « DC5V-12V », batterie par épissure commune avec le
  convertisseur du Pi (essai sur table seulement, voir `essais/alimentation/`).
- USB-C de la carte sur le Pi : `/dev/ttyUSB1` (CH340, identifiant `1a86:7522` ;
  l'ESP32 de la tourelle est `1a86:7523`). Le numéro ttyUSB peut changer : utiliser
  `/dev/serial/by-path/...` dans le code.

## Protocole (vérifié sur la carte, programme `Motor_Version:1.7.3`)

Liaison série 115 200 bauds, commandes texte `$...#`. Ouvrir le port avec DTR et RTS
au repos. Une commande inconnue reçoit `fail,Command not found!` : l'exploration
est donc sans risque, tant qu'on n'envoie pas de vitesse non nulle.

| Commande | Réponse | Vérifié |
|---|---|---|
| `$read_flash#` | configuration : type, zone morte, impulsions, réduction, roue, PID | oui |
| `$upload:1,0,0#` | `$MAll:a,b,c,d#` en continu : compteurs totaux des 4 encodeurs | oui |
| `$upload:0,1,0#` | `$MTEP:a,b,c,d#` : impulsions par période | oui |
| `$upload:0,0,1#` | `$MSPD:a,b,c,d#` : vitesse de chaque roue (unité à confirmer) | oui |
| `$upload:0,0,0#` | arrête les envois | oui |
| `$mphase:40#` | `Set mphase:OK!` : réduction du moteur | oui |
| `$spd:a,b,c,d#` | aucune réponse : consigne de vitesse en **mm/s**, asservie par la carte | oui |
| `$pwm:a,b,c,d#` | aucune réponse : commande directe ; `$pwm:0,0,0,0#` = arrêt rapide | à 0 seulement |
| `$read#`, `$speed:...#` | `fail,Command not found!` | oui |

Configuration lue à la livraison : `Motor_type:1` (520), `Dead_Zone:1600`,
`Pulse_Line:11`, `Pulse_Phase:30`, `wheel_diameter:67`, PID P 0,8 / I 0,06 / D 0,5.

## Plan

1. ✔ Câblage vérifié sur photo
2. ✔ Carte sur USB-C du Pi : elle se présente seule (`Motor_Version:1.7.3`)
3. ✔ Protocole de lecture et de configuration (tableau ci-dessus).
4. ✔ Roues en l'air : sens, précision de vitesse, temps de réponse, arrêt (Mesures)
5. ✔ `Pulse_Phase:40` survit à une coupure d'alimentation (batterie débranchée 10 s)
6. Au sol : 1 m mesuré au mètre contre l'odométrie (5 % attendu), distance d'arrêt
   avec le poids du robot
7. ✔ Nœud ROS 2 `pont_moteurs` (paquet `src/argos_base`), essayé roues en l'air.
   Entraxe mesuré : 0,119 m. Reste : essai au sol (étape 6).
4. Mesures : sens de rotation, impulsions par tour de roue, vitesse obtenue contre
   vitesse demandée, temps de réaction, ordres par seconde acceptés sans retard
5. Odométrie : 1 m au sol mesuré au mètre, contre l'odométrie (5 % attendu)

## Sources

- Page produit : https://www.yahboom.store/products/quad-md-module
- Tutoriel série : https://www.yahboom.net/public/upload/upload-html/1740817239/4%20channel%20encoder%20motor%20driver%20module-USART.html
- Article tiers (pont ROS 2) : https://dev.to/shaifurcodes/building-a-ros-2-hardware-bridge-for-yahboom-520-motor-drivers-without-serial-conflicts-cmb

## Mesures (2026-11-13)

Roues tournées à la main, compteurs `$MAll` enregistrés (aucun moteur alimenté) :
- Motor 3 : -1 767 puis -1 852 impulsions pour un tour de roue vers l'avant.
- Motor 4 : +2 434 (environ 1,4 tour, nombre de tours exact à confirmer).
- Théorie : 11 impulsions par tour moteur x 4 (deux signaux, chaque front) x 40
  (réduction) = **1 760 par tour de roue**. En 1:30, ce serait 1 320.
- Les deux moteurs comptent en sens opposés pour un même sens du robot (montés en
  miroir) : à compenser dans le nœud ROS 2.
- Roue mesurée à la règle : **67 mm** (la facture annonçait 65 mm ; la carte
  était déjà réglée sur 67).
- Entraxe (milieu à milieu des roues) : écartement intérieur 92 mm, extérieur
  146 mm, donc **(92 + 146) / 2 = 119 mm** (largeur d'un pneu : 27 mm). Mis dans
  `config/base.yaml`. La valeur provisoire de 170 mm aurait fait sous-estimer les
  rotations de 30 % (82° annoncés pour environ 117° réels).

Roues en l'air, les deux roues ensemble en marche avant (vitesse réelle calculée
par les encodeurs sur les 2 dernières secondes de 4 s) :

| Consigne | Droite | Gauche | 90 % atteint en |
|---|---|---|---|
| +100 mm/s | 100 mm/s | 100 mm/s | 1,4 s / 1,1 s |
| +200 mm/s | 204 mm/s | 201 mm/s | 1,1 s / 0,9 s |
| +300 mm/s | 303 mm/s | 302 mm/s | 0,5 s / 0,3 s |
| -200 mm/s | -203 mm/s | -206 mm/s | 0,2 s / 0,2 s |

- Précision à vitesse établie : **3 % au plus**, roues égales entre elles. L'unité de
  `$spd` est bien le mm/s. Au premier essai de 2 s, la roue gauche paraissait plus
  lente (65 mm/s) : elle accélérait encore.
- Départ arrêté lent (1,1 à 1,4 s à 100 mm/s) : l'asservissement de la carte monte
  progressivement. Plus rapide d'une vitesse à une autre.
- Arrêt, roues en l'air (sans le poids du robot, à refaire au sol) :

  | Ordre d'arrêt | Depuis 200 mm/s | Depuis 300 mm/s |
  |---|---|---|
  | `$spd:0,0,0,0#` (asservi) | 60 à 80 mm, roues immobiles après 1,8 s | 115 à 127 mm, 2,5 s |
  | `$pwm:0,0,0,0#` (direct) | 14 à 23 mm, 0,2 s | 35 à 59 mm, 0,4 s |

Nœud `pont_moteurs` sur le Pi, roues en l'air, entraxe provisoire 0,17 m :

| Essai | Résultat |
|---|---|
| `/odom` et transformation `odom -> base_link` | 20 Hz exactement |
| `cmd_vel` 0,1 m/s pendant 3 s | vitesse mesurée 0,100 m/s, odométrie +0,318 m (3 s + arrêt) |
| Un seul `cmd_vel`, puis plus rien (chien de garde) | roues arrêtées 0,8 s après l'ordre |
| Rotation 0,5 rad/s pendant 3 s | +82° (86° attendus ; dépend de l'entraxe, provisoire) |
| Nœud coupé en plein mouvement | roues immobiles |

Lancer le nœud sur le Pi :
```
source ~/Documents/Projet/argos_ws/install/setup.bash
ros2 run argos_base pont_moteurs --ros-args --params-file ~/Documents/Projet/argos_ws/install/argos_base/share/argos_base/config/base.yaml
```

## Problèmes rencontrés

1. **L'article trouvé en ligne ne correspond pas à ce programme.**
   - Symptôme : `$read#` et `$speed:...#` reçoivent `fail,Command not found!`.
   - Cause : article tiers écrit pour une autre version du programme de la carte ;
     la doc officielle Yahboom n'est lisible que dans un navigateur.
   - Solution : explorer la carte elle-même, en commençant par des commandes qui ne
     font rien tourner (lecture, rapports), puis des vitesses nulles.
2. **La carte était réglée pour des moteurs 1:30, les nôtres sont en 1:40.**
   - Symptôme : un tour de roue donne environ 1 760 impulsions, pas 1 320.
   - Conséquence évitée : la carte aurait calculé des vitesses et des distances
     fausses d'un tiers (33 %).
   - Solution : `$mphase:40#` (réponse `Set mphase:OK!`, relu dans `$read_flash#`).
     Vérifié : le réglage est gardé après une coupure d'alimentation (mémoire flash).
3. **L'arrêt asservi est lent, et la carte n'a pas de chien de garde.**
   - Symptôme : après `$spd:0,0,0,0#`, les roues tournent encore jusqu'à 2,5 s et
     6 à 13 cm. Et une seule consigne suffit : les roues ont tourné 5 s sans autre
     ordre, la carte garde la dernière vitesse tant qu'on ne lui dit rien.
   - Risque : si le Pi plante pendant un trajet, le robot continue tout droit.
   - Solution (pour le nœud ROS 2) : arrêter par `$pwm:0,0,0,0#` (0,2 à 0,4 s,
     3 fois plus court) ; chien de garde dans le nœud (arrêt si aucun `cmd_vel`
     depuis 0,5 s) ; et un arrêt d'urgence matériel qui coupe l'alimentation des
     moteurs, indépendant du logiciel.
   - Fait dans `pont_moteurs` : arrêt par `$pwm:0`, chien de garde 0,5 s (mesuré :
     roues arrêtées 0,8 s après le dernier ordre), arrêt à la fermeture du nœud.
4. **Le nœud ne publiait aucune odométrie** (premier essai sur le Pi).
   - Symptôme : nœud démarré, mais rien sur `/odom`.
   - Diagnostic : la classe `CarteYahboom` seule, sur le Pi, ne recevait aucun
     compteur ; une seconde demande `$upload:1,0,0#` les faisait arriver.
   - Cause : la première commande, écrite juste après l'ouverture du port, se perd.
     Les scripts d'essai attendaient 0,5 s avant d'écrire, pas le nœud.
   - Solution : pause de 0,5 s après l'ouverture, et nouvelle demande si aucun
     compteur n'arrive pendant 1 s (couvre aussi un redémarrage de la carte).
     Test automatique ajouté.
5. **Point dur une fois par tour de roue : moteurs neufs raides, réglé par un rodage.**
   - **Symptôme** (2026-11-13, essai de 5 m roues en l'air) : l'utilisateur voit la
     roue gauche ralentir toujours au même endroit du tour, et le sent aussi en la
     tournant à la main, moteurs coupés. Le cap du robot oscillait de -12° à +10°.
   - **Diagnostic** :
     - Mesure de la vitesse de chaque roue selon sa position dans le tour
       (encodeurs, environ 250 relevés/s, 150 mm/s, 15 s) : roue gauche environ
       -40 % dans un secteur précis, roue droite environ -25 %. La moyenne sur 4 s
       (environ 3 tours) cachait le défaut : c'est l'œil qui l'a trouvé.
     - Une fois par tour de roue (et non 40 fois, la réduction) et présent moteur
       coupé : mécanique, côté roue ou sortie du réducteur. La roue ne frotte sur rien.
     - Pleine vitesse : jusqu'à 700 mm/s, les deux roues tiennent la consigne à 2 % ;
       à fond, la gauche plafonne plus bas (842 contre 910 mm/s).
     - Moteurs inversés sur la carte : le défaut suit le moteur gauche (38 % ; droit
       17 %). Ce n'est pas la carte.
   - **Fausse piste** : conclusion « moteur gauche défectueux », message d'échange
     rédigé pour le vendeur. Un rodage court (3 min dans chaque sens à fond) ne
     changeait rien de mesurable (39 % avant, 49 % après, dans la dispersion des
     mesures : 38 à 49 %).
   - **Cause réelle** : moteurs neufs et raides, le gauche plus que le droit. Idée
     de l'utilisateur : « les moteurs sont neufs, encore crispés ».
   - **Solution** : rodage long. Deux essais de 30 s par roue, puis 20 min à vitesse
     maximale, sens alterné toutes les 2 min, pause de 3 s à chaque changement de
     sens (éviter le pic de courant d'une inversion brutale). Arrêts automatiques
     prévus (tension du Pi sous 4,8 V, carte muette).
   - **Résultat** :

     | Point dur dans le tour, 150 mm/s | Avant | Après le rodage de 20 min |
     |---|---|---|
     | Roue gauche | 38 à 49 % | 0 à 5 % |
     | Roue droite | 17 à 29 % | 3 à 8 % |

     Contrôle juste après, 30 s par sens : les deux roues tiennent 150 mm/s
     exactement, avec 0 à 8 % de variation. Les quelques pourcents restants sont à
     la limite de la mesure (dispersion de quelques pourcents d'une mesure à l'autre).
     Estimation de l'utilisateur : problème réglé à 95 %, le reste se fera à l'usage.
   - **Reste** : à fond, la roue gauche est 12 % plus lente en marche avant qu'en
     arrière (793 contre 892 mm/s ; droite symétrique, 903 et 904). Sans effet aux
     vitesses d'usage (0,3 à 0,5 m/s). À suivre : cette mesure montre l'avancée du
     rodage. Échange du moteur : plus nécessaire si la mesure à froid confirme.
   - **Alimentation pendant les rodages** : Pi entre 5,016 et 5,079 V (6 min puis
     20 min à pleine vitesse), aucun manque de courant.
   - **Leçons** :
     - Sur des moteurs à réducteur neufs, roder environ 20 min avant de conclure à
       un défaut. 6 min ne suffisaient pas.
     - Une moyenne sur plusieurs tours cache un défaut qui revient à chaque tour :
       mesurer selon la position dans le tour.
     - L'observation humaine (voir, toucher) a trouvé ce que les chiffres moyens
       cachaient, et l'intuition « moteurs neufs » a corrigé une conclusion fausse.
6. **Deux nœuds `pont_moteurs` restés en marche, en cachette.**
   - Symptôme : le script de rodage s'arrête dès le départ sur « device reports
     readiness to read but returned no data (device disconnected or multiple
     access on port?) ».
   - Diagnostic : alimentation stable, aucune déconnexion USB dans le journal du
     noyau ; `sudo fuser -v /dev/ttyUSB1` montre deux `pont_moteurs` qui occupent
     le port.
   - Cause : pour arrêter le nœud, le signal était envoyé à `ros2 run`, qui l'avait
     lancé. `ros2 run` s'arrêtait, mais pas le nœud (un processus enfant). La
     vérification regardait aussi `ros2 run` : elle annonçait « nœud arrêté » à tort.
   - Solution : viser le processus du nœud lui-même
     (`pkill -TERM -f "[l]ib/argos_base/pont_moteurs"` ; les crochets empêchent la
     commande de se viser elle-même), puis vérifier que le port est libre
     (`sudo fuser /dev/ttyUSB1`).
   - Pour le robot : ces nœuds sans surveillance gardaient les moteurs arrêtés
     (chien de garde), mais un nœud oublié peut bloquer le suivant. `ros2 launch`
     arrête bien ses propres nœuds ; le piège est surtout dans les essais à la main.
   - Impact : faible à 0,3 à 0,5 m/s (vitesse prévue en intérieur), compensé par
     l'asservissement ; léger zigzag, corrigé par Nav2 avec le LiDAR.
