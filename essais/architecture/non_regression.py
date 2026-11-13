"""Passe la liste de non-régression d'ARCHITECTURE.md sur la simulation en marche.

À lancer sur la machine de la simulation, robot non placé, carte « entrepot » :
  ros2 launch argos_mission argos.launch.py carte:=entrepot gazebo_gui:=false
  python3 essais/architecture/non_regression.py

Le script parle au répartiteur comme la page web (/argos/commande) et vérifie
ce qu'il publie (/argos/etat, /argos/journal). Il modifie les points de la carte
pendant l'essai : il les sauvegarde avant et les remet à la fin.
Ctrl+C du répartiteur : non couvert ici (il faut arrêter le processus), voir la fin.
"""
import json
import math
import os
import shutil
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

CARTES = os.path.expanduser('~/Documents/Projet/argos_ws/maps')
CARTE = 'entrepot'
NOUVELLE = 'claude_essai'          # carte créée puis effacée par l'essai
LOIN = '60 60'                     # point hors de la carte : inaccessible


class Essai:
    """Envoie les commandes, note les états et le journal, vérifie les attentes."""

    def __init__(self):
        """Branche le nœud d'essai sur les topics du répartiteur."""
        self.noeud = rclpy.create_node('essai_non_regression')
        dernier = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.etats, self.journal, self.points = [], [], {}
        self.pose = None
        self.noeud.create_subscription(String, '/argos/etat', self.recevoir_etat, dernier)
        self.noeud.create_subscription(String, '/argos/journal', self.recevoir_journal, 50)
        self.noeud.create_subscription(String, '/argos/points', self.recevoir_points, dernier)
        self.noeud.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                       self.recevoir_pose, dernier)
        self.pub = self.noeud.create_publisher(String, '/argos/commande', 10)
        self.pub_pose = self.noeud.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.debut = time.monotonic()
        self.resultats = []

    # --- Réception ---
    def recevoir_etat(self, msg):
        self.etats.append(msg.data)
        self.noter(f'état  {msg.data}')

    def recevoir_journal(self, msg):
        d = json.loads(msg.data)
        self.journal.append((d['niveau'], d['texte']))
        self.noter(f'journal [{d["niveau"]}] {d["texte"]}')

    def recevoir_points(self, msg):
        self.points = json.loads(msg.data)

    def recevoir_pose(self, msg):
        self.pose = msg.pose.pose.position

    # --- Outils ---
    def noter(self, texte):
        print(f'{time.monotonic() - self.debut:7.1f} s  {texte}', flush=True)

    def tourner(self, duree):
        fin = time.monotonic() + duree
        while time.monotonic() < fin:
            rclpy.spin_once(self.noeud, timeout_sec=0.05)

    def attendre(self, condition, delai):
        fin = time.monotonic() + delai
        while time.monotonic() < fin:
            rclpy.spin_once(self.noeud, timeout_sec=0.05)
            if condition():
                return True
        return False

    def etat(self):
        return self.etats[-1] if self.etats else None

    def envoyer(self, texte):
        self.noter(f'>>> {texte}')
        nb_journal = len(self.journal)
        self.pub.publish(String(data=texte))
        # Le répartiteur ne garde que la dernière commande reçue : on lui laisse le
        # temps de traiter celle-ci avant la suivante.
        self.tourner(1.5)
        return self.journal[nb_journal:]

    def verifier(self, nom, ok, detail=''):
        self.resultats.append((nom, ok))
        self.noter(f'{"RÉUSSI" if ok else "ÉCHOUÉ"} : {nom} {detail}')

    def placer(self, x, y):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        for _ in range(3):
            self.pub_pose.publish(msg)
            self.tourner(0.5)

    def distance(self, nom):
        p = self.points['points'][nom]
        return math.hypot(self.pose.x - p['x'], self.pose.y - p['y'])


def scenario(e):
    e.attendre(lambda: e.etat() is not None and e.points, 30)
    e.verifier('Démarrage en VEILLE sur la carte entrepot',
               e.etat() == 'VEILLE' and e.points.get('carte') == CARTE)

    # 5. Destination refusée : robot non placé, Nav2 pas encore actif.
    nouveaux = e.envoyer('appel omega')
    e.verifier('Destination refusée (robot non placé) : échec + message',
               e.etat() == 'ECHEC omega' and any(n == 'erreur' for n, _ in nouveaux))
    e.envoyer('veille')

    # Placement au point de départ de l'entrepôt, puis attente de Nav2.
    e.placer(0.0, 0.0)
    e.attendre(lambda: e.pose is not None, 20)
    e.noter('Robot placé ; attente de l\'activation de Nav2 (60 s au plus)')
    e.tourner(15)

    # 8. Redémarrer la navigation (ici, tout est déjà actif : doit le constater).
    nouveaux = e.envoyer('redemarrer_navigation')
    e.attendre(lambda: any('prête' in t or 'impossible' in t for _, t in e.journal[-3:]), 400)
    e.verifier('Redémarrer la navigation', any('Navigation prête' in t for _, t in e.journal[-3:]))

    # 1. Appel, arrivée, retour au repos.
    e.envoyer('appel omega')
    e.verifier('Appel accepté', e.etat() == 'APPEL omega')
    arrive = e.attendre(lambda: e.etat() == 'VEILLE', 120)
    e.tourner(2)
    e.verifier('Appel : arrivée puis VEILLE', arrive and e.distance('omega') < 0.5,
               f'(à {e.distance("omega"):.2f} m du point)')

    # 2. Retour à la base.
    e.envoyer('base')
    e.verifier('Base : en route', e.etat() == 'APPEL base')
    arrive = e.attendre(lambda: e.etat() == 'VEILLE', 120)
    e.tourner(2)
    e.verifier('Base : arrivée', arrive and e.distance('base') < 0.5,
               f'(à {e.distance("base"):.2f} m)')

    # 3. Patrouille : boucle, point inaccessible sauté.
    e.envoyer(f'ajouter loin_claude {LOIN}')
    e.envoyer('definir_patrouille omega loin_claude base')
    e.envoyer('patrouille')
    vus = lambda: [s for s in e.etats if s.startswith('PATROUILLE')]
    e.attendre(lambda: vus().count('PATROUILLE omega') >= 2, 300)
    e.verifier('Patrouille : boucle et point inaccessible sauté',
               vus()[-1] == 'PATROUILLE omega' and 'PATROUILLE loin_claude' in vus()
               and any('loin_claude inaccessible' in t for _, t in e.journal),
               f'({" > ".join(s.split()[1] for s in vus())})')

    # 4. Supprimer un point de la patrouille en cours, puis la vider.
    e.envoyer('supprimer loin_claude')
    e.verifier('Supprimer un point de la patrouille en cours',
               e.etat().startswith('PATROUILLE') and e.points['patrouille'] == ['omega', 'base'])
    e.envoyer('definir_patrouille')
    e.verifier('Patrouille vidée : arrêt en VEILLE', e.etat() == 'VEILLE')

    # 3 bis. Patrouille dont tous les points sont inaccessibles.
    e.envoyer(f'ajouter loin_a {LOIN}')
    e.envoyer('ajouter loin_b -60 -60')
    e.envoyer('definir_patrouille loin_a loin_b')
    e.envoyer('patrouille')
    e.attendre(lambda: (e.etat() or '').startswith('ECHEC'), 120)
    e.verifier('Patrouille : tous inaccessibles = ECHEC', (e.etat() or '').startswith('ECHEC'),
               f'({e.etat()})')
    e.envoyer('veille')

    # 7. Cartographie : enregistrer, puis abandonner.
    e.envoyer('cartographier')
    e.attendre(lambda: e.etat() == 'CARTOGRAPHIE', 30)
    nouveaux = e.envoyer('appel base')
    e.verifier('Cartographie : démarrée, autres commandes refusées',
               e.etat() == 'CARTOGRAPHIE' and any(n == 'avertissement' for n, _ in nouveaux))
    e.tourner(5)
    e.envoyer(f'enregistrer {NOUVELLE}')
    e.attendre(lambda: e.etat() == 'VEILLE', 60)
    e.tourner(3)
    e.verifier('Cartographie : enregistrer',
               e.etat() == 'VEILLE' and e.points.get('carte') == NOUVELLE
               and os.path.isfile(os.path.join(CARTES, NOUVELLE + '.pgm')))
    e.envoyer('cartographier')
    e.attendre(lambda: e.etat() == 'CARTOGRAPHIE', 30)
    e.envoyer('abandonner')
    # L'état et le journal sont deux topics : l'ordre d'arrivée n'est pas garanti.
    e.attendre(lambda: e.etat() == 'VEILLE'
               and any('abandonnée' in t for _, t in e.journal[-3:]), 30)
    e.verifier('Cartographie : abandonner', e.etat() == 'VEILLE'
               and e.points.get('carte') == NOUVELLE
               and any('abandonnée' in t for _, t in e.journal[-3:]))

    # 6. Changer de carte (retour sur entrepot), puis replacer le robot.
    e.envoyer(f'carte {CARTE}')
    e.tourner(3)
    e.verifier('Changer de carte', e.points.get('carte') == CARTE
               and 'omega' in e.points.get('points', {}))
    e.placer(0.0, 0.0)


def main():
    rclpy.init()
    fichier = os.path.join(CARTES, CARTE + '_points.yaml')
    sauvegarde = fichier + '.avant_essai'
    shutil.copy2(fichier, sauvegarde)
    e = Essai()
    try:
        scenario(e)
    finally:
        shutil.copy2(sauvegarde, fichier)
        os.remove(sauvegarde)
        for extension in ('.yaml', '.pgm', '_points.yaml'):
            chemin = os.path.join(CARTES, NOUVELLE + extension)
            if os.path.exists(chemin):
                os.remove(chemin)
        e.noter('Points de la carte remis comme avant ; carte d\'essai effacée.')
    print('\n=== Bilan ===')
    for nom, ok in e.resultats:
        print(f'{"RÉUSSI" if ok else "ÉCHOUÉ"}  {nom}')
    rclpy.shutdown()
    sys.exit(0 if all(ok for _, ok in e.resultats) else 1)


if __name__ == '__main__':
    main()
