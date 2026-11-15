"""Analyse d'un journal de tete_corps.py (option --journal), objet immobile.

Si le modèle est juste, « angle de la tourelle + ego - écart dans l'image » ne
bouge pas quand le corps bouge. On cherche ce qui reste : une erreur de gain
(proportionnelle à l'ego) ou de retard (proportionnelle à sa vitesse).

Usage : python3 analyse.py journal.csv
"""
import sys

import numpy as np

PX = 7.6   # px par degré de servo (axe bas)
d = np.genfromtxt(sys.argv[1], delimiter=',', names=True)
d = d[d['vu'] == 1]
t = d['heure'] - d['heure'][0]
ecart = (d['x'] - 320) / PX
sol = d['bas'] - ecart + d['ego']          # direction de l'objet par rapport au sol
ego = d['ego'] - d['ego'].mean()
vitesse = np.gradient(d['ego'], d['heure'])
bouge = np.abs(vitesse) > 5
print(f'{len(d)} images, ego de {d["ego"].min():+.1f} à {d["ego"].max():+.1f} deg servo, '
      f'vitesse jusqu\'à {np.abs(vitesse).max():.0f} deg/s')
print(f'Direction au sol : écart type {sol.std():.2f} deg ({sol.std() * PX:.0f} px) ; '
      f'écart dans l\'image : {np.abs(ecart * PX)[bouge].mean():.0f} px en moyenne quand le corps bouge')
A = np.column_stack([np.ones(len(d)), ego, vitesse])
coef, *_ = np.linalg.lstsq(A, sol, rcond=None)
reste = sol - A @ coef
print(f'Erreur de gain : {coef[1] * 100:+.1f} % ; erreur de retard : {coef[2] * 1000:+.0f} ms ; '
      f'reste inexpliqué : {reste.std():.2f} deg ({reste.std() * PX:.0f} px)')
# L'écart dans l'image lui-même : retard de la tourelle sur sa cible
A = np.column_stack([np.ones(len(d)), ego, vitesse])
coef, *_ = np.linalg.lstsq(A, ecart, rcond=None)
print(f'Écart dans l\'image : {coef[1] * 100:+.1f} % de l\'ego, {coef[2] * 1000:+.0f} ms de sa vitesse, '
      f'reste {(ecart - A @ coef).std() * PX:.0f} px')
suivi = d['cible_bas'] - d['bas']
print(f'Tourelle derrière sa cible : {np.abs(suivi)[bouge].mean():.1f} deg en moyenne, maximum {np.abs(suivi).max():.1f} deg')
