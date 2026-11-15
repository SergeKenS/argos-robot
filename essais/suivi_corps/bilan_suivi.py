"""Bilan d'un journal de suivi (tete_corps.py --essai suivi --journal) : écart de
l'objet au centre selon sa vitesse, agitation du corps, butées de la tourelle.

Usage : python3 bilan_suivi.py journal1.csv [journal2.csv ...]
"""
import sys
import numpy as np
for nom in sys.argv[1:]:
    d = np.genfromtxt(nom, delimiter=",", names=True)
    t = d["heure"] - d["heure"][0]
    vu = d["vu"] == 1
    ecart = np.where(vu, d["x"] - 320, np.nan)
    gain = (8.18 + 0.067 * np.where(vu & (d["rayon"] > 12), d["rayon"], 25)) / 7.6
    sol = d["bas"] - ecart / 7.6 + d["ego"]
    sol = np.interp(d["heure"], d["heure"][vu], sol[vu])
    vs = np.convolve(np.gradient(sol, d["heure"]), np.ones(5) / 5, mode="same") / gain
    w = np.gradient(d["lacet"], d["heure"])
    ordre = np.degrees(d["ordre_rotation"])
    m = np.convolve(np.abs(vs), np.ones(30) / 30, mode="same") > 3
    p = np.diff(np.concatenate([[0], (~vu).astype(int), [0]])); longues = (np.where(p == -1)[0] - np.where(p == 1)[0]) / 30
    sg = np.sign(ordre[m][np.abs(ordre[m]) > 3]); inv = int(np.sum(sg[1:] != sg[:-1]))
    print(nom, ": objet en mouvement %.0f s ; non vu %d images, plus longue perte %.2f s ; distance mediane %.2f m" % (m.sum() / 30, (~vu).sum(), longues.max() if len(longues) else 0, 11.0 / np.median(d["rayon"][vu])))
    print("   vitesse objet deg/s : mediane %.0f p90 %.0f max %.0f | corps : mediane %.0f p90 %.0f max %.0f" % (*np.percentile(np.abs(vs[m]), [50, 90, 100]), *np.percentile(np.abs(w[m]), [50, 90, 100])))
    print("   tourelle hors centre (deg servo) : mediane %.0f p90 %.0f ; en butee %.1f %% ; %.1f inversions du corps par 10 s ; agitation de l ordre %.0f deg/s2" % (*np.percentile(np.abs(d["bas"][m] - 105), [50, 90]), 100 * np.mean(np.abs(d["cible_bas"][m] - 105) > 44.5), inv / (m.sum() / 300), np.sqrt(np.mean(np.gradient(ordre[m], 1 / 30) ** 2))))
    for lo, hi in ((0, 20), (20, 50), (50, 100), (100, 999)):
        s = m & vu & (np.abs(vs) >= lo) & (np.abs(vs) < hi)
        if s.sum() > 5: print("   objet a %3d-%3d deg/s : %4d images, ecart median %3.0f px, 9 sur 10 sous %3.0f px" % (lo, hi, s.sum(), np.nanmedian(np.abs(ecart[s])), np.nanpercentile(np.abs(ecart[s]), 90)))
    s = m & vu; c = np.polyfit(vs[s], ecart[s], 1)
    print("   retard equivalent de la tete : %.0f ms ; ecart median hors mouvement %.0f px" % (abs(c[0]) / (7.6 / 1.35) * 1000, np.nanmedian(np.abs(ecart[~m])) if (~m).sum() > 5 else -1))
