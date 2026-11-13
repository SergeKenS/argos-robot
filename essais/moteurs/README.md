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

## Plan

1. Câblage vérifié sur photo : moteurs gauche et droit, encodeurs, alimentation
2. Carte sur USB-C du Pi : port série vu (`/dev/ttyUSB*`), ce qu'elle envoie seule
3. Protocole : type de moteur, vitesse, lecture des encodeurs (doc Yahboom ; un
   article tiers parle de `$mtype:x#` et de 4 vitesses séparées par des virgules :
   **non confirmé**). **Roues en l'air** pour les premiers essais.
4. Mesures : sens de rotation, impulsions par tour de roue, vitesse obtenue contre
   vitesse demandée, temps de réaction, ordres par seconde acceptés sans retard
5. Odométrie : 1 m au sol mesuré au mètre, contre l'odométrie (5 % attendu)

## Sources

- Page produit : https://www.yahboom.store/products/quad-md-module
- Tutoriel série : https://www.yahboom.net/public/upload/upload-html/1740817239/4%20channel%20encoder%20motor%20driver%20module-USART.html
- Article tiers (pont ROS 2) : https://dev.to/shaifurcodes/building-a-ros-2-hardware-bridge-for-yahboom-520-motor-drivers-without-serial-conflicts-cmb

## Problèmes rencontrés

(symptôme, diagnostic, cause, solution)
