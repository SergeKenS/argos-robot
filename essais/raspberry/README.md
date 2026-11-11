# Essais : Raspberry Pi 5

Essai hors du projet ROS : installer et comprendre le Raspberry Pi 5 8 Go seul,
avant de le mettre sur le robot.

## Matériel

- Raspberry Pi 5 8 Go, boîtier HighPi Pro 5S, Active Cooler (PiShop.ca)
- Active Cooler : piloté par le système selon la température, arrêté sous 50 °C.
  Sans système (pas de microSD), il ne tourne pas : normal.
- microSD Raspberry Pi 128 Go A2
- Pas d'écran ni de câble micro-HDMI : tout se fait depuis le PC (SSH)

## Plan

1. Préparer la microSD depuis le PC (Raspberry Pi Imager, Ubuntu 24.04)
2. Premier démarrage et connexion SSH depuis le PC
3. Bases Linux sur le Pi : mises à jour, température, mémoire
4. Installer ROS 2 Jazzy
5. Échange ROS 2 entre le PC et le Pi
6. Brancher le LiDAR C1 sur le Pi, voir /scan dans RViz sur le PC

## Installé (PC Windows, 2026-11-10)

- Raspberry Pi Imager 2.0.8 : `winget install --id RaspberryPiFoundation.RaspberryPiImager -e`

## Problèmes rencontrés

(symptôme, diagnostic, cause, solution)

## État

2026-11-10 : Pi reçu. Étape 1 bloquée : pas de lecteur microSD sur le PC (à acheter).
Alimentation de bureau : chargeur USB-C 5 V 3 A du téléphone (le Pi 5 démarre,
mais limite le courant de ses ports USB à 600 mA).
