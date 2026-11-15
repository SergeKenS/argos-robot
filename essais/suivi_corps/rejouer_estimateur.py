"""Rejoue un journal de suivi pour comparer des estimateurs de vitesse de l'objet :
erreur entre ce que la tête aurait visé et où l'objet était vraiment un instant après.

Usage : python3 rejouer_estimateur.py journal.csv
"""
import sys
import numpy as np
d = np.genfromtxt(sys.argv[1], delimiter=',', names=True)
d = d[(d['vu'] == 1)]
t = d['heure'] - d['heure'][0]
d = d[t > 29]; t = d['heure']
mes = d['bas'] - (d['x'] - 320) / 7.6 + d['ego']     # direction mesuree au sol (deg servo)

def filtre(mode, alpha=0.6, beta=0.15, saut=2.0, beta_max=0.5, pente=0.1):
    dirs, vits = [], []
    di, v, tp = mes[0], 0.0, t[0]
    for ti, m in zip(t, mes):
        dt = ti - tp; tp = ti
        if dt <= 0 or dt > 0.2:
            di, v = m, 0.0
        else:
            prev = di + v * dt; s = m - prev
            if mode == 'ancien':
                if abs(s) > saut: di, v = m, 0.0
                else: di = prev + alpha * s; v += beta * s / dt
            else:
                b = min(beta_max, beta + pente * abs(s))
                a = min(1.0, alpha + 0.2 * abs(s))
                di = prev + a * s; v += b * s / dt
        dirs.append(di); vits.append(v)
    return np.array(dirs), np.array(vits)

def erreur(dirs, vits, tau):
    # ce que la tete viserait a t (dir + v*tau) contre la mesure reelle a t + tau
    futur = np.interp(t + tau, t, mes)
    e = (dirs + vits * tau - futur)[t + tau < t[-1]]
    return np.median(np.abs(e)) * 7.6, np.percentile(np.abs(e), 90) * 7.6

print('erreur de prediction en px (mediane / 9 sur 10), pour une avance de 0,15 s et 0,22 s')
for nom, args in (('ancien (remise a zero au saut)', dict(mode='ancien')),
                  ('sans vitesse du tout', None),
                  ('nouveau beta 0.15 +0.1/deg', dict(mode='n')),
                  ('nouveau beta 0.25 +0.1/deg', dict(mode='n', beta=0.25)),
                  ('nouveau beta 0.15 +0.2/deg', dict(mode='n', pente=0.2)),
                  ('nouveau beta 0.3 fixe', dict(mode='n', beta=0.3, pente=0)),
                  ('nouveau beta 0.5 fixe', dict(mode='n', beta=0.5, pente=0)),
                  ('nouveau beta max 0.8', dict(mode='n', beta=0.2, pente=0.2, beta_max=0.8))):
    if args is None:
        dirs, vits = filtre('ancien'); vits = vits * 0
    else:
        dirs, vits = filtre(**args)
    r = [erreur(dirs, vits, tau) for tau in (0.15, 0.22)]
    print(f'  {nom:32s} {r[0][0]:4.0f} / {r[0][1]:4.0f}    {r[1][0]:4.0f} / {r[1][1]:4.0f}   bruit vitesse {np.std(np.diff(vits)):.1f}')
