# Matériel à acheter

Robot différentiel : 2 roues motrices + roulette folle, LiDAR 2D, IMU,
Raspberry Pi 5 (navigation, page web) et ESP32 (moteurs).

**Prix = estimations à vérifier avant d'acheter** (relevés le 2026-10-26, les prix
du Raspberry Pi ont augmenté plusieurs fois en 2026). Les prix en USD sont les
prix officiels ; en dollars canadiens, compter environ ×1,4 plus la livraison.

## Calcul

| Article | Qté | Rôle | Prix estimé |
|---|---|---|---|
| Raspberry Pi 5 **4 Go** | 1 | Nav2, répartiteur, page web | 110 USD (8 Go : 175 USD, ~245 $ CAD vu sur PiShop.ca / Newark) |
| Refroidisseur actif officiel (Active Cooler) | 1 | Sinon le Pi ralentit en chauffant | ~5 USD |
| Carte microSD 64 Go (A2, « endurance ») | 1 | Ubuntu 24.04 + ROS 2 | ~20 $ CAD |

## Électronique de commande

| Article | Qté | Rôle | Prix estimé |
|---|---|---|---|
| **Waveshare General Driver for Robots** | 1 | ESP32 + pilote moteurs (TB6612FNG) + IMU 9 axes (QMI8658C + AK09918C) sur une seule carte | 27,99 USD |

Pourquoi cette carte : le firmware `linorobot2_hardware` la gère déjà (`gendrv`), et
elle remplace 3 achats (ESP32, pilote moteur, IMU) et beaucoup de câblage.
Ton ESP32 DevKit actuel n'est alors pas utilisé.

## Base roulante

| Article | Qté | Rôle | Prix estimé |
|---|---|---|---|
| Moteur **JGB37-520 12 V à encodeur Hall** (6 fils), ~150-200 tr/min | 2 | Roues motrices, l'encodeur mesure la rotation | ~15-25 USD chacun (prix peu fiables, à comparer) |
| Support moteur pour moteur 37 mm | 2 | Fixer les moteurs | ~5 $ CAD chacun |
| Roue 65-85 mm, moyeu 6 mm | 2 | | ~15-25 $ CAD la paire |
| Roulette folle (bille) | 1-2 | Équilibre | ~5-10 $ CAD |
| Plaque de châssis ~250 mm + entretoises M3 (2 étages) | 1 | Porter le tout | ~30 $ CAD (ou découpe au fab lab) |

## Perception

| Article | Qté | Rôle | Prix estimé |
|---|---|---|---|
| **LiDAR Slamtec RPLIDAR C1** (12 m, 360°) | 1 | Carte et localisation | ~80 USD |

Pilote ROS 2 officiel de Slamtec (`rplidar_ros`, branche `ros2`) : il contient bien
`rplidar_c1_launch.py`. Le paquet `apt` de Jazzy ne l'a peut-être pas encore :
prévoir de le compiler depuis les sources dans `src/`.

L'IMU est déjà sur la carte Waveshare.

## Énergie et sécurité

| Article | Qté | Rôle | Prix estimé |
|---|---|---|---|
| Batterie **3S** (11,1 V) Li-ion ou LiPo, 2 200-5 000 mAh + chargeur adapté | 1 | Alimente tout ; la carte Waveshare accepte 7-13 V | ~50-90 $ CAD |
| Convertisseur 12 V → **5 V 5 A** | 1 | Alimentation dédiée du Pi 5 (il exige 5 V / 5 A) | ~15-25 $ CAD |
| **Bouton d'arrêt d'urgence** (coup de poing) | 1 | Coupe l'alimentation des moteurs | ~10-15 $ CAD |
| Interrupteur, fusible, connecteurs XT60, fils | 1 lot | | ~20-25 $ CAD |

## Total estimé

**Environ 550 à 650 $ CAD**, selon le Pi (4 ou 8 Go) et la livraison.

## À vérifier avant de commander

- **Moteurs : 6 fils** (2 moteur + 4 encodeur). Un moteur à 2 fils n'a pas d'encodeur.
- **Brochage des moteurs** contre le connecteur PH2.0 6 broches de la carte Waveshare :
  l'ordre des fils peut différer, prévoir de refaire les câbles si besoin.
- **Courant des moteurs** : le TB6612FNG supporte peu de courant. Choisir des moteurs
  modestes et éviter de les bloquer (moteur forcé à l'arrêt).
- **Version du LiDAR** : bien le **C1**, pas un autre modèle.

## Plus tard (pas maintenant)

- Capteurs de vide orientés vers le sol (détecter un escalier qui descend : le
  LiDAR 2D ne le voit pas).
- SSD NVMe à la place de la microSD (plus fiable sur la durée).
- Caméra et accélérateur d'IA.

## Sources

- [Waveshare General Driver for Robots](https://www.waveshare.com/product/general-driver-for-robots.htm) et [sa documentation](https://www.waveshare.com/wiki/General_Driver_for_Robots)
- [linorobot2_hardware (cartes, IMU et pilotes gérés)](https://github.com/linorobot/linorobot2_hardware)
- [Pilote rplidar_ros de Slamtec, branche ros2](https://github.com/Slamtec/rplidar_ros/tree/ros2/launch)
- [Prix Raspberry Pi 2026 (Hackster)](https://www.hackster.io/news/raspberry-pi-hikes-prices-by-up-to-150-as-the-ai-bubble-hits-hard-confirms-raspberry-pi-6-plans-711d4384a4dd), [PiShop.ca](https://www.pishop.ca/product/raspberry-pi-5-8gb/), [Newark Canada](https://canada.newark.com/raspberry-pi/rpi5-8gb-single/raspberry-pi-5-model-b-8gb-2-4ghz/dp/81AK1348)
- [RPLIDAR C1 chez Waveshare](https://www.waveshare.com/product/rplidar-c1.htm)
