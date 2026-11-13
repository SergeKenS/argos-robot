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
2. ✔ Relier le câble en T au convertisseur (essai : raccords au ruban isolant,
   chacun séparé ; pour le robot : Wago 221 ou soudure)
3. ✔ Tension de sortie, mesurée sur un ESP32 (voir Mesures)
4. ✔ Pi sur le convertisseur : tension au repos et sous effort, `get_throttled`
5. Le convertisseur n'annonce pas 5 A au Pi (pas d'USB Power Delivery) : le Pi
   limite ses ports USB à 600 mA. Régler `PSU_MAX_CURRENT=5000` dans son chargeur
   de démarrage au moment du LiDAR, et le valider par une mesure avec LiDAR, caméra
   et processeur chargés ensemble (la vraie charge du robot).
6. Autonomie : batterie chargée à fond, Pi en charge, jusqu'à la coupure

## Mesures (2026-11-13)

- Batterie : 11,73 V (chargée en partie, environ aux deux tiers).
- Câble en T mâle branché sur la batterie : +11,7 V entre le bout rouge et le bout
  noir. Rouge = +, polarité correcte.
- Sortie du convertisseur, sans testeur USB-C : convertisseur branché sur un ESP32
  DevKit, multimètre entre VIN et GND de l'ESP32 : 4,8 V, soit environ 5,1 V en
  sortie (la diode de l'ESP32 retire environ 0,3 V). L'ESP32 et ses deux servos
  ont démarré normalement. Astuce : la broche VIN d'une carte de développement
  remplace un testeur USB pour vérifier une source 5 V (l'ESP32 qui s'allume ne
  prouve rien seul : son régulateur supporterait aussi 12 V).
- Raspberry Pi 5, comparaison des deux alimentations
  (`vcgencmd pmic_read_adc EXT5V_V`, `vcgencmd get_throttled`) :

  | | Chargeur de téléphone 3 A | Batterie + convertisseur |
  |---|---|---|
  | Tension au repos | 4,97 V | 5,06 V |
  | Tension min, 4 cœurs à 100 % pendant 60 s | non mesuré | 5,00 V |
  | Manque de courant (`get_throttled`) | 0x0 | 0x0 |
  | Alimentation détectée par le Pi | 3 000 mA | 3 000 mA (pas de PD) |
  | Température après l'effort | | 63,9 °C |

  Le processeur seul ne tire qu'une partie des 5 A : la limite de 5 A n'est donc pas
  encore prouvée.

## Règles de sécurité

- La batterie peut fournir 10 A : jamais de fils tordus ensemble ni de ruban
  adhésif. Brins bien torsadés, aucun brin qui dépasse.
- Câble en T débranché de la batterie dès qu'on ne mesure plus : sinon ses bouts
  dénudés sont sous tension.
- Jamais le Pi sur le convertisseur avant d'avoir mesuré 5 V à sa sortie.

## Problèmes rencontrés

(symptôme, diagnostic, cause, solution)
