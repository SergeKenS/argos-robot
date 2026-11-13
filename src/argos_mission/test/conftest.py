"""Réglage commun des tests : rend le paquet argos_mission importable.

Sans ce fichier, pytest ne trouve argos_mission que s'il est lancé depuis
src/argos_mission. Avec lui, la commande marche depuis n'importe où, par exemple
depuis la racine du workspace :
  python3 -m pytest src/argos_mission/test/test_fonctionnalites.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
