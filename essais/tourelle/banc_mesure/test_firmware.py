"""Tests de suivi_fluide.ino sans caméra : profil de mouvement et inondation d'ordres."""
import math
import time

import serial
from serial.tools import list_ports

port = next((p.device for p in list_ports.comports() if p.vid == 0x1A86), None)
esp32 = serial.Serial(port, 115200, timeout=0)
tampon = b''


def lire(duree):
    """Lit pendant `duree` s ; renvoie [(heure PC, ligne)]."""
    global tampon
    lignes = []
    fin = time.perf_counter() + duree
    while time.perf_counter() < fin:
        tampon += esp32.read(4096)
        while b'\n' in tampon:
            brut, tampon = tampon.split(b'\n', 1)
            lignes.append((time.perf_counter(), brut.decode(errors='replace').strip()))
        time.sleep(0.001)
    return lignes


def positions(lignes):
    """Extrait [(heure PC, ms ESP32, bas, haut)] des lignes de position."""
    resultat = []
    for heure, texte in lignes:
        morceaux = texte.split(',')
        if morceaux[0] == 'P' and len(morceaux) == 4:
            try:
                resultat.append((heure, int(morceaux[1]), float(morceaux[2]), float(morceaux[3])))
            except ValueError:
                pass
    return resultat


# Démarrage : l'ouverture du port redémarre l'ESP32
for _, texte in lire(2.5):
    if not texte.startswith(('ets', 'rst', 'conf', 'clk', 'mode', 'load', 'ho ', 'entry')):
        print('ESP32 :', texte)

# Test 1 : un saut de 30° sur le bas, 10° sur le haut, avec décimales
print('\nTest 1 : saut 100,30 -> 130.5,40.25')
depart = time.perf_counter()
esp32.write(b'130.5,40.25\n')
pos = positions(lire(1.5))
if pos:
    print(f'  première position reçue {1000 * (pos[0][0] - depart):.0f} ms après l\'ordre')
    print(f'  durée du mouvement : {pos[-1][1] - pos[0][1]} ms, {len(pos)} positions')
    print(f'  arrivée : bas {pos[-1][2]}, haut {pos[-1][3]}')
    ecarts = [b[1] - a[1] for a, b in zip(pos, pos[1:])]
    print(f'  écart entre positions (ms ESP32) : min {min(ecarts)}, max {max(ecarts)}')
    vitesses = [(b[2] - a[2]) / ((b[1] - a[1]) / 1000) for a, b in zip(pos, pos[1:])]
    accels = [(v2 - v1) / 0.02 for v1, v2 in zip(vitesses, vitesses[1:])]
    print(f'  vitesse max du bas : {max(vitesses):.0f} °/s, accélération max : {max(accels):.0f}, freinage max : {min(accels):.0f} °/s²')
    print(f'  dépassement du bas : {max(p[2] for p in pos) - 130.5:.2f}°')
    print('  profil du bas (un point sur 3) :', ' '.join(f'{p[2]:.1f}' for p in pos[::3]))

# Test 2 : inondation. Cible en sinus (±15°, 0,5 Hz) à plusieurs cadences,
# puis un saut : la réponse au saut doit rester immédiate.
for cadence in (30, 100, 500):
    esp32.write(b'100,30\n')
    lire(1.0)
    print(f'\nTest 2 : {cadence} ordres par seconde pendant 8 s')
    debut = time.perf_counter()
    envoyes = 0
    recu = []
    while (t := time.perf_counter() - debut) < 8:
        if t >= envoyes / cadence:
            bas = 100 + 15 * math.sin(2 * math.pi * 0.5 * t)
            esp32.write(f'{bas:.2f},30\n'.encode())
            envoyes += 1
        recu += lire(0.0005)
    # Saut final : retard entre l'ordre et le premier mouvement vers le haut
    lire(0.3)
    haut_avant = 30.0
    saut = time.perf_counter()
    esp32.write(b'100,45\n')
    pos = positions(lire(1.0))
    reaction = next((p[0] for p in pos if p[3] > haut_avant + 0.01), None)
    sinus = positions(recu)
    amplitude = (max(p[2] for p in sinus) - min(p[2] for p in sinus)) / 2 if sinus else 0
    print(f'  {envoyes} ordres envoyés, {len(sinus)} positions reçues, amplitude suivie ±{amplitude:.1f}°')
    if reaction:
        print(f'  réaction au saut final : {1000 * (reaction - saut):.0f} ms après l\'ordre')
    else:
        print('  PAS DE RÉACTION au saut final')

esp32.write(b'100,30\n')
lire(1.0)
esp32.close()
