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
| `$spd:a,b,c,d#` | aucune réponse (reconnue) : consigne de vitesse, unité à mesurer | à 0 seulement |
| `$pwm:a,b,c,d#` | aucune réponse (reconnue) : commande directe du moteur | à 0 seulement |
| `$read#`, `$speed:...#` | `fail,Command not found!` | oui |

Configuration lue à la livraison : `Motor_type:1` (520), `Dead_Zone:1600`,
`Pulse_Line:11`, `Pulse_Phase:30`, `wheel_diameter:67`, PID P 0,8 / I 0,06 / D 0,5.

## Plan

1. ✔ Câblage vérifié sur photo
2. ✔ Carte sur USB-C du Pi : elle se présente seule (`Motor_Version:1.7.3`)
3. ✔ Protocole de lecture et de configuration (tableau ci-dessus).
   À venir, **roues en l'air** : premiers ordres de vitesse.
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
     À vérifier : le réglage survit-il à une coupure d'alimentation ?
