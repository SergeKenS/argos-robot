"""Simule le corps du robot sur un journal de tete_corps.py (objet déplacé à la main),
pour comparer des lois de commande du corps sans refaire l'essai.

Le corps simulé : retard pur de 0,17 s, puis réponse en 0,08 s, accélération limitée
comme dans base.py. Vérifié sur deux essais réels (tourelle en butée 4,4 % simulé
contre 4,8 % mesuré, 70 inversions contre 68).

Usage : python3 simuler_corps.py journal1.csv [journal2.csv ...]
"""
import sys
import numpy as np

def charger(nom):
    d = np.genfromtxt(nom, delimiter=',', names=True)
    vu = d['vu'] == 1
    g = 1.33
    t = d['heure'] - d['heure'][0]
    # direction de l'objet au sol, en degres reels, et vitesse estimee en ligne (bruitee)
    obj = np.interp(t, t[vu], (d['bas'] - (d['x'] - 320) / 7.6 + d['ego'])[vu]) / g
    return t, obj, d['direction'] / g, d['vitesse'] / g

def simuler(t, obj, direction, vitesse, loi, zone_calme=1.5, zone_depart=5.0, rot_max=120, acc=458):
    dt = 1 / 30
    lacet = obj[0] - 105 / 1.33
    w = 0.0; applique = 0.0; repos = True; etat = {}
    file = [0.0] * 5
    exc, ordres = [], []
    for i in range(len(t)):
        e = direction[i] - 105 / 1.33 - lacet
        if repos and abs(e) >= zone_depart: repos = False
        if not repos and abs(e) < zone_calme and abs(applique) < 8: repos = True
        ordre = 0.0 if repos else float(np.clip(loi(e, vitesse[i], w, etat, dt), -rot_max, rot_max))
        applique += float(np.clip(ordre - applique, -acc * dt, acc * dt))
        file.append(applique); cible = file.pop(0)
        w += min(1.0, dt / 0.08) * (cible - w)
        lacet += w * dt
        exc.append((obj[i] - 105 / 1.33 - lacet) * 1.33)
        ordres.append(applique)
    exc, ordres = np.array(exc), np.array(ordres)
    sg = np.sign(ordres[np.abs(ordres) > 3])
    return (np.percentile(np.abs(exc), 50), np.percentile(np.abs(exc), 90), np.abs(exc).max(),
            np.mean(np.abs(exc) > 45) * 100, np.sqrt(np.mean(np.gradient(ordres, 1 / 30) ** 2)), int(np.sum(sg[1:] != sg[:-1])))

def p(k): return lambda e, v, w, s, dt: k * e
def cubique(k, e0): return lambda e, v, w, s, dt: k * e * (1 + (e / e0) ** 2)
def lisse(k, tau):
    def f(e, v, w, s, dt):
        s['o'] = s.get('o', 0.0) + min(1, dt / tau) * (k * e - s.get('o', 0.0)); return s['o']
    return f
def pd(k, kd, tau):
    def f(e, v, w, s, dt):
        # derivee filtree de l'angle de la tourelle
        ef = s.get('ef', e); nf = ef + min(1, dt / tau) * (e - ef); s['d'] = (nf - ef) / dt; s['ef'] = nf
        return k * e + kd * s['d']
    return f
def pv(k, kv, tau):
    def f(e, v, w, s, dt):
        s['v'] = s.get('v', 0.0) + min(1, dt / tau) * (v - s.get('v', 0.0)); return k * e + kv * s['v']
    return f

lois = [('P 2.0', p(2.0)), ('P 2.5', p(2.5)), ('P 3.0', p(3.0)), ('P 3.5', p(3.5)), ('P 4.5', p(4.5)),
        ('cubique 2.0, 25 deg', cubique(2.0, 25)), ('cubique 1.5, 20 deg', cubique(1.5, 20)), ('cubique 2.5, 30 deg', cubique(2.5, 30)),
        ('P 3.0 ordre lisse 0.1', lisse(3.0, 0.1)),
        ('PD 2.5 + 0.15', pd(2.5, 0.15, 0.1)), ('PD 2.5 + 0.3', pd(2.5, 0.3, 0.1)), ('PD 3.0 + 0.3', pd(3.0, 0.3, 0.15)),
        ('P 2.5 + 0.5 vitesse lissee 0.15', pv(2.5, 0.5, 0.15)), ('P 2.5 + 1.0 vitesse lissee 0.15', pv(2.5, 1.0, 0.15))]
tous = [charger(n) for n in sys.argv[1:]]
print('pour chaque essai : tourelle 9 sur 10 / max (deg servo), % en butee, agitation, inversions')
for titre, loi in lois:
    print(f'  {titre:34s}', '   |   '.join('%3.0f / %3.0f  %4.1f %%  %4.0f  %3d' % simuler(*d, loi)[1:] for d in tous))
