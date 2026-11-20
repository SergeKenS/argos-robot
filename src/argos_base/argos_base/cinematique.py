"""Calculs d'un robot à deux roues motrices : vitesses des roues et odométrie.

Ne dépend pas de ROS : se teste seul (test/test_cinematique.py).
Unités : mètres, secondes, radians. Repère du robot : x vers l'avant, y vers la
gauche, rotation positive vers la gauche (sens antihoraire vu de dessus).
"""
import math


def vitesses_roues(lineaire, angulaire, entraxe):
    """Vitesse du robot (m/s, rad/s) → vitesses des roues (gauche, droite) en m/s."""
    return (lineaire - angulaire * entraxe / 2,
            lineaire + angulaire * entraxe / 2)


def limiter(gauche, droite, vitesse_max):
    """Ramène la roue la plus rapide à vitesse_max, en gardant le rapport entre les deux.

    Garder le rapport conserve la courbe voulue : le robot va moins vite, mais
    tourne autant.
    """
    plus_rapide = max(abs(gauche), abs(droite))
    if plus_rapide <= vitesse_max:
        return gauche, droite
    facteur = vitesse_max / plus_rapide
    return gauche * facteur, droite * facteur


def approcher(actuelle, voulue, acceleration, freinage, duree):
    """Fait évoluer une vitesse vers la vitesse voulue, sans à-coup.

    Pendant « duree » (s), la vitesse change d'au plus acceleration x duree quand
    elle s'éloigne de zéro (le robot accélère), et d'au plus freinage x duree
    quand elle s'en rapproche (le robot ralentit ou change de sens). Freiner
    plus fort qu'on n'accélère : l'arrêt reste court.
    """
    if voulue * actuelle >= 0 and abs(voulue) > abs(actuelle):
        pas = acceleration * duree
    else:
        pas = freinage * duree
    if abs(voulue - actuelle) <= pas:
        return voulue
    return actuelle + math.copysign(pas, voulue - actuelle)


class Odometrie:
    """Position estimée du robot à partir des compteurs des roues.

    metres_par_impulsion : distance parcourue par la roue pour une impulsion
    (périmètre de la roue / impulsions par tour).
    """

    def __init__(self, metres_par_impulsion, entraxe):
        """Départ en (0, 0), orientation 0."""
        self.metres_par_impulsion = metres_par_impulsion
        self.entraxe = entraxe
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.lineaire = 0.0    # vitesse actuelle (m/s)
        self.angulaire = 0.0   # vitesse de rotation actuelle (rad/s)
        self.precedent = None  # (heure, compteur gauche, compteur droite)

    def mettre_a_jour(self, heure, gauche, droite):
        """Intègre les compteurs (impulsions, déjà dans le bon sens) reçus à « heure » (s)."""
        if self.precedent is None:
            self.precedent = (heure, gauche, droite)
            return
        heure_avant, gauche_avant, droite_avant = self.precedent
        duree = heure - heure_avant
        if duree <= 0:
            return
        self.precedent = (heure, gauche, droite)

        distance_gauche = (gauche - gauche_avant) * self.metres_par_impulsion
        distance_droite = (droite - droite_avant) * self.metres_par_impulsion
        distance = (distance_gauche + distance_droite) / 2
        rotation = (distance_droite - distance_gauche) / self.entraxe

        # Le robot a avancé en suivant la direction moyenne pendant ce court instant
        direction = self.theta + rotation / 2
        self.x += distance * math.cos(direction)
        self.y += distance * math.sin(direction)
        self.theta = math.atan2(math.sin(self.theta + rotation), math.cos(self.theta + rotation))
        self.lineaire = distance / duree
        self.angulaire = rotation / duree
