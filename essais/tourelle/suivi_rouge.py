"""Garde un objet rouge au centre de l'image de la caméra de la tourelle.

Le PC voit l'objet et envoie les angles à l'ESP32 (suivi_camera.ino) par l'USB.
Méthode « regarder puis bouger » : l'image arrive en retard, donc on corrige
tout l'écart d'un coup, puis on attend de voir le résultat avant de recorriger.
Objet perdu : la tourelle balaie gauche-droite à deux hauteurs, revient au
centre, attend 5 s, et recommence tant que l'objet n'est pas retrouvé.

Usage : python suivi_rouge.py   (fermer d'abord le moniteur série d'Arduino)
Quitter : touche q dans la fenêtre de l'image.
"""
import time

import cv2
import numpy as np
import serial
from serial.tools import list_ports

# Caméra de la tourelle pour OpenCV (le numéro peut changer quand on rebranche)
CAMERA = 1
LARGEUR, HAUTEUR = 640, 480

# Tourelle : centres et débattement (les mêmes que dans suivi_camera.ino)
CENTRE_BAS, CENTRE_HAUT = 100, 30
AMPLITUDE = 45

# Mesuré : quand l'angle augmente de 1°, l'image glisse de 6,4 px vers la
# droite (servo du bas) ou de 8,7 px vers le bas (servo du haut)
PIXELS_PAR_DEGRE_BAS = 6.4
PIXELS_PAR_DEGRE_HAUT = 8.7

# Attente après une correction, le temps que l'image montre le résultat (s)
ATTENTE = 0.3
# Écart toléré autour du centre, en pixels
ZONE_MORTE = 15
# Objet perdu depuis ce temps (s) : la tourelle part à sa recherche
PERTE = 1.5

# Recherche : angles du servo du haut pour les deux passages (bas, puis haut)
HAUTEURS_RECHERCHE = (10, 50)
# Vitesse de balayage (degrés par seconde) : assez lente pour que l'image,
# en retard de 0,25 s, montre encore l'objet quand on le découvre
VITESSE_RECHERCHE = 60
# Un ordre toutes les 0,1 s pendant le balayage (l'ESP32 en traite environ 25 par seconde)
PAS_RECHERCHE = 0.1
# Pause au centre entre deux balayages (s)
PAUSE_RECHERCHE = 5

# Rouge en HSV : teinte 0 à 10 ou 170 à 180, assez saturé et lumineux
ROUGE_1 = (np.array([0, 120, 70]), np.array([10, 255, 255]))
ROUGE_2 = (np.array([170, 120, 70]), np.array([180, 255, 255]))
# Taille minimale en pixels, et part minimale du cercle englobant remplie :
# un fil ou une trace fine remplit mal son cercle et est ignoré
AIRE_MIN = 300
REMPLISSAGE_MIN = 0.3


def trouver_objet(image):
    """Renvoie (x, y, rayon) du plus grand objet rouge compact, ou None."""
    hsv = cv2.cvtColor(cv2.GaussianBlur(image, (11, 11), 0), cv2.COLOR_BGR2HSV)
    masque = cv2.inRange(hsv, *ROUGE_1) | cv2.inRange(hsv, *ROUGE_2)
    masque = cv2.morphologyEx(masque, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(masque, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    meilleur = None
    for contour in contours:
        aire = cv2.contourArea(contour)
        if aire < AIRE_MIN or (meilleur is not None and aire <= meilleur[0]):
            continue
        (x, y), rayon = cv2.minEnclosingCircle(contour)
        if aire / (np.pi * rayon ** 2) >= REMPLISSAGE_MIN:
            meilleur = (aire, int(x), int(y), int(rayon))
    return None if meilleur is None else meilleur[1:]


def limiter(angle, centre):
    """Garde un angle dans le débattement autour du centre et dans 0° à 180°."""
    return min(max(angle, max(centre - AMPLITUDE, 0)), min(centre + AMPLITUDE, 180))


def parcours_recherche():
    """Liste des angles (bas, haut) du balayage, un point par ordre envoyé.

    De gauche à droite en bas, de droite à gauche en haut, puis retour au centre.
    """
    pas = round(VITESSE_RECHERCHE * PAS_RECHERCHE)  # degrés entre deux ordres
    gauche_droite = range(limiter(0, CENTRE_BAS), limiter(180, CENTRE_BAS) + 1, pas)
    points = [(bas, HAUTEURS_RECHERCHE[0]) for bas in gauche_droite]
    points += [(bas, HAUTEURS_RECHERCHE[1]) for bas in reversed(gauche_droite)]
    points.append((CENTRE_BAS, CENTRE_HAUT))
    return points


def main():
    # L'ESP32 se reconnaît à sa puce USB CH340 (identifiant 1A86)
    port = next((p.device for p in list_ports.comports() if p.vid == 0x1A86), None)
    if port is None:
        raise SystemExit('ESP32 introuvable : est-il branché ?')
    esp32 = serial.Serial(port, 115200, timeout=0)
    time.sleep(2)  # l'ouverture du port redémarre l'ESP32

    camera = cv2.VideoCapture(CAMERA)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, LARGEUR)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, HAUTEUR)
    ok, image = camera.read()
    if not ok:
        raise SystemExit(f'Caméra {CAMERA} : aucune image (occupée par un autre programme ?)')
    if image.shape[:2] != (HAUTEUR, LARGEUR):
        # Les pixels par degré ont été mesurés dans cette taille d'image
        raise SystemExit(f'Image en {image.shape[1]} x {image.shape[0]} au lieu de {LARGEUR} x {HAUTEUR}')

    bas, haut = CENTRE_BAS, CENTRE_HAUT
    envoye = None
    prochaine_correction = 0
    derniere_vue = time.time()
    # Recherche : position dans le parcours (None = pas de balayage en cours)
    parcours = parcours_recherche()
    etape, prochaine_etape, fin_pause = None, 0, 0
    # Bilan affiché chaque seconde dans le terminal
    bilan, vues, images = time.time(), 0, 0

    while True:
        ok, image = camera.read()
        if not ok:
            break
        maintenant = time.time()
        objet = trouver_objet(image)

        if objet is not None:
            x, y, rayon = objet
            if maintenant - derniere_vue > PERTE:
                print('Objet retrouvé')
            if etape is not None:
                # Trouvé pendant le balayage : la tourelle s'arrête, et on laisse
                # l'image (en retard) la rattraper avant de corriger
                etape = None
                prochaine_correction = maintenant + ATTENTE
            derniere_vue = maintenant
            fin_pause = 0
            ecart_x, ecart_y = x - LARGEUR // 2, y - HAUTEUR // 2
            # Pendant l'attente, l'image ne montre pas encore la dernière correction
            if maintenant >= prochaine_correction and max(abs(ecart_x), abs(ecart_y)) > ZONE_MORTE:
                # L'image doit glisser de -ecart pixels, soit -ecart / pixels_par_degre degrés
                if abs(ecart_x) > ZONE_MORTE:
                    bas = limiter(bas - ecart_x / PIXELS_PAR_DEGRE_BAS, CENTRE_BAS)
                if abs(ecart_y) > ZONE_MORTE:
                    haut = limiter(haut - ecart_y / PIXELS_PAR_DEGRE_HAUT, CENTRE_HAUT)
                prochaine_correction = maintenant + ATTENTE
            cv2.circle(image, (x, y), rayon, (0, 255, 0), 2)
        elif maintenant - derniere_vue > PERTE:
            if etape is None and maintenant >= fin_pause:
                print('Objet perdu : recherche')
                etape, prochaine_etape = 0, maintenant
            if etape is not None and maintenant >= prochaine_etape:
                bas, haut = parcours[etape]
                etape += 1
                prochaine_etape = maintenant + PAS_RECHERCHE
                if etape == len(parcours):
                    # Balayage fini, tourelle au centre : pause avant le suivant
                    etape = None
                    fin_pause = maintenant + PAUSE_RECHERCHE

        # N'envoyer que les changements : l'ESP32 traite environ 25 ordres par
        # seconde (mesuré). À 30 par seconde, les ordres s'accumulent et la
        # tourelle exécute des ordres de plus en plus anciens.
        commande = (round(bas), round(haut))
        if commande != envoye:
            esp32.write(f'{commande[0]},{commande[1]}\n'.encode())
            envoye = commande
        esp32.reset_input_buffer()  # les réponses de l'ESP32 ne servent pas ici

        vues, images = vues + (objet is not None), images + 1
        if maintenant - bilan >= 1:
            ecart = f'écart {ecart_x:+4d} {ecart_y:+4d} px' if objet else 'objet non vu'
            print(f'vu {vues:2d}/{images} images | {ecart} | bas {round(bas)} haut {round(haut)}')
            bilan, vues, images = maintenant, 0, 0

        cv2.drawMarker(image, (LARGEUR // 2, HAUTEUR // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
        cv2.imshow('Camera tourelle', image)
        touche = cv2.waitKey(1) & 0xFF
        if touche in (ord('q'), 27):
            break

    # Remet la tourelle au centre avant de quitter
    esp32.write(f'{CENTRE_BAS},{CENTRE_HAUT}\n'.encode())
    esp32.flush()
    esp32.close()
    camera.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
