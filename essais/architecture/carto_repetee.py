"""Enchaîne N cartographies (enregistrer puis abandonner) pour traquer une panne intermittente.

Panne visée (2026-11-13) : après « enregistrer », la localisation ne redémarrait
pas et le répartiteur restait bloqué. À lancer après non_regression.py :
  python3 essais/architecture/carto_repetee.py 5
"""
import os
import sys

import rclpy

from non_regression import CARTE, CARTES, Essai


def main():
    tours = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    rclpy.init()
    e = Essai()
    e.attendre(lambda: e.etat() is not None, 30)
    crees = []
    try:
        for i in range(tours):
            nom = f'claude_repet_{i}'
            crees.append(nom)
            e.envoyer('cartographier')
            e.attendre(lambda: e.etat() == 'CARTOGRAPHIE', 30)
            e.tourner(3)
            e.envoyer(f'enregistrer {nom}')
            ok = e.attendre(lambda: e.etat() == 'VEILLE'
                            and any('enregistrée' in t for _, t in e.journal[-3:]), 90)
            e.verifier(f'Tour {i + 1} : enregistrer', ok and e.points.get('carte') == nom)
            if not ok:
                break
            e.envoyer('cartographier')
            e.attendre(lambda: e.etat() == 'CARTOGRAPHIE', 30)
            e.envoyer('abandonner')
            ok = e.attendre(lambda: e.etat() == 'VEILLE'
                            and any('abandonnée' in t for _, t in e.journal[-3:]), 60)
            e.verifier(f'Tour {i + 1} : abandonner', ok)
            if not ok:
                break
        e.envoyer(f'carte {CARTE}')
        e.tourner(3)
        e.placer(0.0, 0.0)
    finally:
        for nom in crees:
            for extension in ('.yaml', '.pgm', '_points.yaml'):
                chemin = os.path.join(CARTES, nom + extension)
                if os.path.exists(chemin):
                    os.remove(chemin)
        e.noter('Cartes d\'essai effacées.')
    print('\n=== Bilan ===')
    for nom, ok in e.resultats:
        print(f'{"RÉUSSI" if ok else "ÉCHOUÉ"}  {nom}')
    rclpy.shutdown()
    sys.exit(0 if e.resultats and all(ok for _, ok in e.resultats) else 1)


if __name__ == '__main__':
    main()
