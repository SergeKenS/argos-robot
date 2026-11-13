# Essais : alimentation

Essai hors du projet ROS : alimenter le Raspberry Pi 5 (puis le LiDAR et la caméra)
depuis la batterie du robot, avant de tout assembler.

## Matériel

- Batterie du châssis Yahboom : 3 cellules Li-ion, 11,1 V nominal, 12,6 V chargée,
  coupure à 8,1 V, 4 400 mAh (environ 49 Wh), 6 A en continu, 10 A au maximum.
  Sortie : prise en T (femelle). Charge : prise ronde DC 4,0 x 1,7 mm.
- Câble prise en T mâle avec fils rouge et noir (fourni avec le châssis).
- Convertisseur 12 V vers 5 V : F35J5V5A2S-C (AirheadStorm), entrée 12/24 V
  (8 à 32 V d'après l'annonce), sortie 5 V 5 A max, fils nus en entrée, prise USB-C
  mâle en sortie.

## Plan

1. ✔ Tension de la batterie et polarité du câble en T
2. Relier le câble en T au convertisseur (Wago 221 ou domino, rouge avec rouge)
3. Tension de sortie à vide, avec un testeur USB-C : 5,0 à 5,25 V attendus.
   Au-dessus de 5,5 V : ne pas brancher le Pi.
4. Pi sur le convertisseur : tension en charge (au-dessus de 4,9 V au démarrage),
   courant consommé, `vcgencmd pmic_read_adc EXT5V_V`, `vcgencmd get_throttled` (0x0)
5. Le convertisseur annonce-t-il 5 A au Pi (USB Power Delivery) ? Sinon, le Pi
   limite ses ports USB à 600 mA : régler `PSU_MAX_CURRENT=5000` dans son chargeur
   de démarrage, seulement si l'étape 4 montre qu'il tient vraiment 5 A.
6. Autonomie : batterie chargée à fond, Pi en charge, jusqu'à la coupure

## Mesures (2026-11-13)

- Batterie : 11,73 V (chargée en partie, environ aux deux tiers).
- Câble en T mâle branché sur la batterie : +11,7 V entre le bout rouge et le bout
  noir. Rouge = +, polarité correcte.

## Règles de sécurité

- La batterie peut fournir 10 A : jamais de fils tordus ensemble ni de ruban
  adhésif. Brins bien torsadés, aucun brin qui dépasse.
- Câble en T débranché de la batterie dès qu'on ne mesure plus : sinon ses bouts
  dénudés sont sous tension.
- Jamais le Pi sur le convertisseur avant d'avoir mesuré 5 V à sa sortie.

## Problèmes rencontrés

(symptôme, diagnostic, cause, solution)
