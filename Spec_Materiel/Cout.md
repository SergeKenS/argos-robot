# Coûts réels

Achats réels, comparés à l'estimation de référence `MATERIEL.md` (non modifiée),
pour documenter la stratégie d'achat. Montants en dollars canadiens.
Factures : dossier `Facture/`.

## Achats

États : Commandé, Reçu, Testé.

| Date | Article | Vendeur | Prix | Livraison | Taxes | Total | État |
|---|---|---|---|---|---|---|---|
| ? | Raspberry Pi 5 8 Go (250,00) + boîtier HighPi Pro 5S, sans ventilateur (14,45) | PiShop.ca (commande 267909) | 264,45 | 13,92 | 41,68 | 320,05 | Commandé |
| 2026-11-02 | Active Cooler (7,15) + microSD Raspberry Pi 128 Go A2 (52,95), ajoutés à la commande 267909 | PiShop.ca (commande 267970) | 60,10 | 0,00 (code add-to-order-fs) | 9,00 | 69,10 | Commandé |
| 2026-11-01 | Châssis Yahboom 2WD avec batterie : 2 moteurs 520 à encodeur (12 V, 1:40, 300 tr/min), cadre aluminium, roues 65 mm, 2 roulettes, batterie 12 V 4 400 mAh + chargeur 12 V 2 A, carte MSPM0 + carte d'extension + module pilote 4 moteurs à encodeur | Amazon.ca, Yahboom (commande 701-2265577-0645842) | 119,00 | 0,00 | 0,00 | 119,00 | Commandé |
| 2026-11-01 | Tourelle Yahboom 2DOF Basic : caméra USB 2 MP, 2 servos YB-SG90, lumière LED | Amazon.ca, Yahboom (même commande) | 54,63 | 20,00 | 0,00 | 74,63 | Commandé |
| 2026-11-02 | LiDAR Slamtec RPLIDAR C1 (avec adaptateur USB) | Amazon.ca, Waveshare (commande 701-0972199-2561864) | 93,99 | 0,00 (gratuite) | 14,08 | 108,07 | Commandé |
| 2026-11-02 | Convertisseur 12 V vers 5 V 5 A USB-C, AirheadStorm (entrée 8-32 V) | Amazon.ca, Sengying (commande 701-3582176-9138631) | 11,74 (12,49 moins promotion 0,75) | 2,29 | 0,00 | 14,03 | Commandé |
| | **Total** | | | | | **704,88** | |

## Comparaison avec l'estimation

Les prix de `MATERIEL.md` sont hors taxes et hors livraison ; la colonne « Réel » est
le total payé. Taxes de la commande 267970 réparties au prorata (14,975 %).

| Poste de `MATERIEL.md` | Estimé | Réel | Remarque |
|---|---|---|---|
| Pi 5 8 Go + refroidisseur | ~252 | 328,27 | Pi seul : 250,00 hors taxes et livraison (estimé ~245). Boîtier HighPi Pro 5S en plus (14,45), sans ventilateur, d'où l'Active Cooler (7,15, soit 8,22 taxes comprises) |
| Carte microSD | ~20 (64 Go) | 60,88 | 128 Go A2 officielle Raspberry Pi (52,95 hors taxes) |
| Base roulante + batterie 3S et chargeur | ~150 à 235 | 119,00 | Kit complet moins cher que les pièces séparées. Son module pilote 4 moteurs à encodeur remplace la Waveshare (~39 $ évités, à confirmer à la réception) |
| LiDAR RPLIDAR C1 | ~80 USD (~112) | 108,07 | 93,99 hors taxes, livraison gratuite |
| Convertisseur 12 V vers 5 V 5 A | ~15 à 25 | 14,03 | |
| Caméra + tourelle | non estimé (« Plus tard ») | 74,63 | Ajout |
