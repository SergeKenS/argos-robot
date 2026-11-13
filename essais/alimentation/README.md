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
   limitait ses ports USB à 600 mA. ✔ `PSU_MAX_CURRENT=5000` réglé (2026-11-13) :
   le Pi voit 5 000 mA, `usb_max_current_enable=1` (1,6 A pour l'USB).
   Commande : `sudo rpi-eeprom-config --apply <fichier>` puis redémarrage ; ancienne
   configuration gardée dans `~/argos_claude/eeprom_avant_psu.txt` sur le Pi.
   ✔ Validé avec la tourelle (voir Mesures) ; à refaire avec le LiDAR.
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
- Charge réelle, après `PSU_MAX_CURRENT=5000` : tourelle sur le Pi (caméra USB +
  ESP32 + deux servos alimentés par l'USB du Pi), suivi de la balle tenue à la main
  pendant 5 min, avec 10 recherches en 8. Tension relevée chaque seconde
  (309 relevés) : **min 4,999 V, moyenne 5,050 V, max 5,070 V**, aucun manque de
  courant (`get_throttled` 0x0), aucune coupure USB dans le journal du noyau.

## Règles de sécurité

- La batterie peut fournir 10 A : jamais de fils tordus ensemble ni de ruban
  adhésif. Brins bien torsadés, aucun brin qui dépasse.
- Câble en T débranché de la batterie dès qu'on ne mesure plus : sinon ses bouts
  dénudés sont sous tension.
- Jamais le Pi sur le convertisseur avant d'avoir mesuré 5 V à sa sortie.

## Problèmes rencontrés

1. **Ports USB du Pi limités à 600 mA sur le convertisseur.**
   - Symptôme : sur le convertisseur 5 V 5 A, le Pi annonçait une alimentation de
     3 000 mA seulement (`/proc/device-tree/chosen/power/max_current`) et
     `usb_max_current_enable=0`. Il limitait donc l'ensemble de ses ports USB à
     600 mA. Repéré par la mesure, avant toute panne.
   - Pourquoi c'est bloquant : la tourelle (caméra, ESP32, deux servos) dépasse
     600 mA dès que les servos bougent (un servo SG90 bloqué tire environ 650 mA),
     et le LiDAR s'y ajoutera. Au-delà, le Pi coupe ses ports USB : la caméra et la
     tourelle s'éteindraient en plein suivi.
   - Cause : le Pi 5 ne sait qu'une alimentation fournit 5 A que si elle le lui
     annonce par USB Power Delivery. Le convertisseur (F35J5V5A2S-C) ne le fait pas :
     le Pi le traite comme une alimentation de 3 A, comme un chargeur de téléphone.
   - Solution : `PSU_MAX_CURRENT=5000` dans la configuration du chargeur de démarrage
     (`sudo rpi-eeprom-config --apply <fichier>`, puis redémarrage). Le Pi voit alors
     5 000 mA, active `usb_max_current_enable=1` et accorde 1,6 A à ses ports USB.
     Justifié parce que le convertisseur est donné pour 5 A, et vérifié en charge
     réelle : tourelle en mouvement pendant 5 min, tension jamais sous 4,999 V,
     aucune coupure. À revérifier quand le LiDAR s'ajoutera.
   - À savoir : ce réglage dit au Pi de faire confiance à l'alimentation. Sur une
     alimentation qui ne tient pas vraiment 5 A, il ne faut pas l'utiliser. Retour
     arrière : configuration d'origine gardée dans `~/argos_claude/eeprom_avant_psu.txt`.
   - Source : livre blanc Raspberry Pi « USB Power delivery on Raspberry Pi 5 »
     (RP-009856).
