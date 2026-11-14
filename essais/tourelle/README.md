# Essais : tourelle caméra

Essai hors du projet ROS : comprendre la tourelle Yahboom 2DOF (2 servos YB-SG90
et caméra USB 2 MP) pilotée par un ESP32, avant de l'intégrer au robot.

## Branchement

ESP32 DevKit (WROOM-32), puce USB CH340, port COM3 sur ce PC.
Servos alimentés par le 5 V de l'USB du PC (suffisant pour les 2 SG90).
Caméra de la tourelle branchée au PC : vue par Windows comme « HD USB Camera »
(la webcam intégrée est « USB2.0 HD UVC WebCam »).
Piège : le numéro OpenCV des caméras peut changer quand on rebranche. Repère :
la caméra de la tourelle accepte 1920 x 1080, la webcam s'arrête à 1280 x 720.

- fils bruns → GND
- fils rouges → VIN
- orange du servo du bas (gauche/droite) → D18
- orange du servo du haut (haut/bas) → D19

## Installé (PC Windows, 2026-11-10)

- Arduino IDE 2 (les pilotes Arduino et Adafruit proposés ne servent pas à l'ESP32)
- URL des cartes : `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
- Carte « esp32 by Espressif Systems » 3.3.12
- Bibliothèque ESP32Servo 3.2.1
- Pilote CH340 : déjà présent

Dans l'IDE : carte « ESP32 Dev Module », port COM3, série à 115200 bauds.

## Paramètres de la tourelle (mesurés)

- Centre du bas : 100°, centre du haut : 30° (caméra droit devant)
- Débattement : 45° de part et d'autre du centre, dans 0° à 180°
- Sens : angle du bas qui augmente = caméra vers la gauche, angle du haut qui
  augmente = caméra vers le haut
- Servos : 50 Hz, impulsion de 500 à 2400 µs
- Une consigne toutes les 20 ms (rythme du servo) : mouvement plus régulier

## Fichiers

Les essais 1 à 3 (`balayage/`, `suivi_simule/`, `suivi_camera/`, `suivi_rouge.py`)
sont marqués OBSOLÈTE : l'essai 4 les remplace (validé le 2026-11-13), ils peuvent
être supprimés au nettoyage. `suivi_camera` et `suivi_rouge.py` servent encore de
point de comparaison à `banc_mesure/banc.py essai3`.

- `balayage/` : essai 1, balayage lent de chaque servo (branchement, centres)
- `suivi_simule/` : essai 2, suivi d'un objet virtuel, réactivité réglée par `GAIN`
- `suivi_camera/` : essai 3, côté ESP32 : reçoit "bas,haut" par l'USB (115200
  bauds) et applique les angles, dans les limites de la monture
- `suivi_rouge.py` : essai 3, côté PC : garde un objet rouge au centre de
  l'image et envoie les angles à l'ESP32. Lancer : `python suivi_rouge.py`,
  quitter avec q. Affiche chaque seconde un bilan dans le terminal.
  Méthode « regarder puis bouger » : corrige tout l'écart d'un coup, puis
  attend 0,3 s que l'image montre le résultat.
  Objet perdu 1,5 s : balayage gauche-droite à deux hauteurs, retour au centre,
  pause de 5 s, et on recommence tant qu'il n'est pas retrouvé.
  Ignore les taches rouges fines (fil de servo) : elles remplissent mal leur cercle.
- `mesure_servo/` : mesure la durée de `servo.write()` (cause de la limite de
  25 ordres par seconde, voir problème 2)
- `suivi_fluide/` : essai 4, côté ESP32 : signal des servos par MCPWM au lieu
  d'ESP32Servo, lecture série sans attente, boucle à 50 Hz qui amène la tourelle
  à la cible avec vitesse et accélération limitées, rattrapage du jeu des servos.
  Reçoit "bas,haut" (décimales permises), renvoie sa position "P,temps_ms,bas,haut".
  Réglages par le port série : "V,300" (°/s), "A,2000" (°/s²), "G,15",
  "J,2.4,2.1" (jeu des servos, degrés).
- `suivi_fluide.py` : essai 4, côté PC ou Pi : corrige à chaque image (30 par
  seconde) au lieu de « regarder puis bouger ». Retrouve où visait la tourelle
  quand l'image a été prise (retard mesuré 0,08 s, positions annoncées par
  l'ESP32), estime la vitesse de l'objet pour viser un peu en avant. Recherche
  en forme de 8 quand l'objet est perdu. Sur le Pi :
  `python3 suivi_fluide.py --camera 0 --sans-fenetre` (image dans /tmp/tourelle.jpg).
- `banc_mesure/` : outils de mesure (balle immobile, consigne déplacée dans
  l'image) : `banc.py fluide` ou `banc.py essai3`, `jeu2.py` (jeu des servos),
  `retard.py` (retard de l'image), `flasher.ps1` (compiler et téléverser).

## Mesures (2026-11-12, essai 4)

- `servo.write()` d'ESP32Servo : 30 µs si le servo n'a rien reçu depuis 40 ms,
  sinon il bloque jusqu'à 40 ms (2 périodes du signal 50 Hz)
- ESP32Servo règle le signal sur 10 bits : 1 cran = 19,5 µs = 1,85°. La
  tourelle ne pouvait donc se placer que tous les 1,85° (12 à 16 px)
- `suivi_fluide.ino` : pas de 1 µs (0,1°). 500 ordres par seconde pendant 8 s,
  puis réaction au dernier ordre en 7 ms : plus de file d'attente
- Saut de 30° avec le profil (300 °/s, 2000 °/s²) : 0,48 s, aucun dépassement
- Avec la balle comme repère (lumière du jour) : l'image montre la position
  annoncée par l'ESP32 environ 0,08 s plus tôt ; 7,6 px par degré (bas) et 8,8
  (haut) ; jeu des servos 2,5° (bas) et 2,2° (haut), ramené à environ 0,1° par le
  rattrapage ; les servos plafonnent vers 380 °/s.
- Banc d'essai, balle immobile, même détection (essai 3 puis essai 4 sans
  prédiction) : écart au repos 11,9 px puis 0,9 px ; saut de 10° rattrapé sous
  5 px en 0,64 s puis 0,34 s ; cible mobile à 31 °/s, écart 41 px puis 24 px.

## Mesures (2026-11-10)

- Caméra de la tourelle : numéro 1, en 640 x 480, 30 images/s
- 1° de servo fait glisser l'image de 6,4 px (bas) et 8,7 px (haut)
- Retard : l'image commence à bouger 0,11 s après l'ordre, position finale vers 0,25 s
- Couleur de la balle (HSV OpenCV) : teinte 174 à 177, saturation 173 à 200,
  luminosité 57 à 100
- Résultat final, objet immobile : vu 100 % du temps pendant 32 s, écart
  toujours sous 22 px. Le servo du haut garde 2 à 3° de jeu (poids de la caméra)
- Recherche testée (objet masqué 12 s) : balayage, pause, 2e balayage, objet
  retrouvé au bord de l'image puis recentré en moins de 1 s

## Sur le Raspberry Pi (2026-11-13)

`python3 suivi_fluide.py --camera 0 --sans-fenetre`, la vision tourne sur le Pi
(architecture du robot final), l'ESP32 et la caméra branchés en USB sur le Pi, le
tout sur la batterie du robot et le convertisseur 12 V vers 5 V.
- Balle tenue à la main pendant 5 min : 30 images analysées par seconde en
  moyenne (27 au plus bas), écart au centre de quelques pixels à l'arrêt.
- 10 pertes de la balle (cachée exprès) : recherche en 8 lancée, balle retrouvée 9 fois.
- Processeur : 45 % d'un cœur (environ 11 % du Pi), 53 °C.
- Alimentation : voir `essais/alimentation/` (tension jamais sous 4,999 V).

## Problèmes rencontrés

Les deux qui bloquaient tout (la tourelle tournait « au hasard ») :

1. **La tourelle n'était pas fixée.**
   - Symptôme : marche quelques secondes, puis la visée dérive.
   - Diagnostic : tourelle commandée immobile, toute l'image glissait quand
     même ; même consigne envoyée deux fois, visée différente de 2 à 15°.
   - Cause : servo du bas simplement posé sur la table ; en tournant, tout le
     bloc pivotait, puis le câble USB de la caméra le tirait.
   - Solution : servo du bas fixé, câbles avec du mou. Écart ramené à moins de 1,4°.

2. **Les ordres s'accumulaient dans l'ESP32.**
   - Symptôme : parfait les premières secondes, puis la tourelle exécute
     n'importe quoi et part en butée.
   - Diagnostic : chronométrage des réponses de l'ESP32 à 30 ordres par
     seconde : retard qui grandit de 0,2 s par seconde (2 s au bout de 10 s).
   - Cause : l'ESP32 (suivi_camera.ino) traite environ 25 ordres par seconde ;
     le surplus attend dans sa file, la tourelle exécute des ordres périmés.
   - Solution : le PC n'envoie un ordre que quand l'angle change.
   - Pourquoi si long à trouver : les tests courts (3 à 5 s) ne le montrent
     pas, seul un test long l'a révélé.
   - Cause profonde (2026-11-12) : ni le port série ni la puissance de
     l'ESP32. Lu dans le pilote PWM de l'ESP32 (LEDC), puis mesuré avec
     `mesure_servo.ino` : `servo.write()` attend que la consigne précédente
     soit passée dans le signal, soit jusqu'à 40 ms à 50 Hz. D'où 25 ordres
     par seconde au plus.
   - Vraie solution : `suivi_fluide.ino` (MCPWM, changement de consigne
     immédiat, lecture série sans attente).
   - Leçon pour le robot : jamais de code bloquant dans la boucle de
     l'ESP32. Ce n'était pas un manque de puissance : une seule fonction
     qui attend suffit à tout ralentir.

3. **Les servos ont du jeu** (trouvé le 2026-11-12).
   - Symptôme : la tourelle s'arrête 2 à 3° à côté de la cible, et la visée
     dérive toute seule à l'arrêt (0,8° en 10 s).
   - Diagnostic : balle immobile comme repère, escalier de pas de 0,25°
     montants puis descendants. Au changement de sens, le servo ne bouge pas
     tant que la consigne n'a pas reculé de 2,5° (bas) ou 2,2° (haut).
   - Cause : limite physique des SG90 (zone morte et jeu des engrenages).
   - Solution : `suivi_fluide.ino` ajoute ce recul lui-même à chaque
     changement de sens. Écart entre les deux sens : de 1,35° à 0,1° environ.
     Il reste des pas d'environ 0,4° : d'où la zone morte de 3 px côté PC.

3. **Sur le Pi, le suivi lancé en arrière-plan ne s'arrêtait pas** (2026-11-13).
   - Symptôme : `kill -INT` (l'équivalent de Ctrl+C) sans effet, la tourelle
     continuait de suivre.
   - Cause : un programme lancé en tâche de fond (`&`) dans un script ignore Ctrl+C,
     et il hérite de ce réglage. Le `except KeyboardInterrupt` n'était jamais atteint.
   - Solution : `suivi_fluide.py` traite aussi SIGTERM (envoyé par `kill` ou `pkill`)
     comme un Ctrl+C : la tourelle revient au centre et le port série se ferme.
     Vérifié : lancement en arrière-plan, `pkill -TERM`, arrêt propre en moins de 3 s.
   - Pour le robot : tout nœud lancé sans terminal doit savoir s'arrêter sur SIGTERM
     (c'est ce qu'envoie `ros2 launch` à l'arrêt).

Autres pièges, plus simples :

- Numéro de caméra : OpenCV regardait la webcam du portable (la tourelle
  bougeait sans que l'image change). Le numéro peut changer au rebranchement.
- Réglages devinés (sens des servos, angle de vue) au lieu d'être mesurés :
  corrections fausses. Leçon : mesurer d'abord.
- Retard de l'image (0,25 s) : corriger un peu à chaque image fait osciller la
  tourelle dès qu'on accélère. Solution : « regarder puis bouger ».

## À améliorer plus tard

- La main est parfois prise pour l'objet (peau sous éclairage chaud). Seuils
  resserrés dans `suivi_fluide.py` (saturation min 140, teinte 168 à 8) ; à
  revérifier sous l'éclairage du bâtiment. Le mode SUIVI prévoit des couleurs
  mesurées sur place (`ETAPES.md`).

## À retenir pour le robot

- La caméra USB ne va pas sur l'ESP32 : la vision tourne sur le Pi (validé),
  l'ESP32 n'applique que les angles.
- Servos SG90 : 6 V au maximum, jamais sur la batterie du robot (12 V). Ils sont
  alimentés par l'USB du Pi à travers l'ESP32, d'où le réglage
  `PSU_MAX_CURRENT=5000` du Pi (voir `essais/alimentation/`).

## État

Essai 4 validé le 2026-11-13 sur le Raspberry Pi, sur la batterie du robot :
balle tenue à la main pendant 5 min, 30 images analysées par seconde, recherche
en 8 déclenchée 10 fois, balle retrouvée 9 fois (voir « Sur le Raspberry Pi »).

Essai 3 validé le 2026-11-10 : suivi d'un objet rouge tenu à la main.
Installé : Python 3.14, `python -m pip install opencv-python pyserial`
(OpenCV 5.0.0, pyserial 3.5, numpy 2.5.4).
Un seul programme à la fois sur COM3 : fermer le moniteur série d'Arduino
avant de lancer le script Python.
