"""Tests des calculs de roues et d'odométrie, sans ROS ni robot.

Lancer : python3 -m pytest src/argos_base/test/
"""
import math

from argos_base.cinematique import approcher, limiter, Odometrie, vitesses_roues
import pytest

ENTRAXE = 0.2
PAR_IMPULSION = math.pi * 0.067 / 1760   # m par impulsion (roue 67 mm, 1 760 par tour)


def test_vitesses_roues():
    assert vitesses_roues(0.2, 0.0, ENTRAXE) == (0.2, 0.2)          # tout droit
    g, d = vitesses_roues(0.0, 1.0, ENTRAXE)                         # sur place, vers la gauche
    assert (g, d) == pytest.approx((-0.1, 0.1))
    g, d = vitesses_roues(0.2, -1.0, ENTRAXE)                        # courbe à droite
    assert g > d > 0


def test_limiter_garde_le_rapport():
    assert limiter(0.1, 0.2, 0.3) == (0.1, 0.2)                      # déjà sous la limite
    g, d = limiter(0.3, 0.6, 0.3)
    assert (g, d) == pytest.approx((0.15, 0.3))
    assert g / d == pytest.approx(0.5)                               # même courbe
    assert limiter(-0.6, 0.6, 0.3) == pytest.approx((-0.3, 0.3))


def test_approcher_accelere_progressivement():
    v = 0.0
    for _ in range(10):                      # 10 pas de 0,05 s à 0,4 m/s²
        v = approcher(v, 0.7, 0.4, 1.0, 0.05)
    assert v == pytest.approx(0.2)           # 0,4 x 0,5 s
    for _ in range(100):
        v = approcher(v, 0.7, 0.4, 1.0, 0.05)
    assert v == 0.7                          # atteint, sans dépasser


def test_approcher_freine_plus_fort():
    assert approcher(0.7, 0.0, 0.4, 1.0, 0.05) == pytest.approx(0.65)
    assert approcher(-0.7, 0.0, 0.4, 1.0, 0.05) == pytest.approx(-0.65)
    # Changement de sens : on freine jusqu'à zéro, puis on accélère dans l'autre sens
    assert approcher(0.02, -0.5, 0.4, 1.0, 0.05) == pytest.approx(-0.03)
    assert approcher(0.0, -0.5, 0.4, 1.0, 0.05) == pytest.approx(-0.02)


def impulsions(metres):
    return round(metres / PAR_IMPULSION)


def test_odometrie_ligne_droite():
    o = Odometrie(PAR_IMPULSION, ENTRAXE)
    o.mettre_a_jour(0.0, 0, 0)            # premier relevé : point de départ
    n = impulsions(1.0)
    o.mettre_a_jour(5.0, n, n)
    assert (o.x, o.y, o.theta) == pytest.approx((1.0, 0.0, 0.0), abs=1e-3)
    assert o.lineaire == pytest.approx(0.2, abs=1e-3)
    assert o.angulaire == pytest.approx(0.0)


def test_odometrie_demi_tour_sur_place():
    o = Odometrie(PAR_IMPULSION, ENTRAXE)
    o.mettre_a_jour(0.0, 0, 0)
    arc = math.pi / 2 * ENTRAXE / 2       # chaque roue parcourt cet arc pour 90°
    for pas in range(1, 101):             # 180° en 100 petits pas
        n = impulsions(2 * arc * pas / 100)
        o.mettre_a_jour(pas * 0.05, -n, n)
    assert abs(o.theta) == pytest.approx(math.pi, abs=0.01)
    assert (o.x, o.y) == pytest.approx((0.0, 0.0), abs=1e-3)


def test_odometrie_quart_de_cercle():
    # Virage à gauche de rayon 0,5 m : après 90°, le robot est en (0,5 ; 0,5)
    o = Odometrie(PAR_IMPULSION, ENTRAXE)
    o.mettre_a_jour(0.0, 0, 0)
    rayon, pas_total = 0.5, 200
    for pas in range(1, pas_total + 1):
        angle = math.pi / 2 * pas / pas_total
        o.mettre_a_jour(pas * 0.05, impulsions(angle * (rayon - ENTRAXE / 2)),
                        impulsions(angle * (rayon + ENTRAXE / 2)))
    assert (o.x, o.y, o.theta) == pytest.approx((0.5, 0.5, math.pi / 2), abs=5e-3)


def test_odometrie_ignore_une_heure_qui_ne_bouge_pas():
    o = Odometrie(PAR_IMPULSION, ENTRAXE)
    o.mettre_a_jour(1.0, 0, 0)
    o.mettre_a_jour(1.0, 500, 500)        # même heure : pas de division par zéro
    assert o.x == 0.0
